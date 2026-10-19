// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// HealthMonitor — pure C++ helper class embedded in the PackML manager.
//
// Monitors NodeHeartbeat messages from Equipment Modules and fires PackML
// actions (Hold / Suspend / Abort) when new health events are detected.
//
// Design principles:
//   - Transition-based: only fires on NEW or ESCALATING events, not on repeats.
//   - De-escalation suppressed: ABORT → HOLD does NOT re-fire.
//   - WARN is logged but never invokes fire_action (manager observes, not acts).
//   - Timeout fires ABORT exactly once; recovery resets the flag.
//   - Gate blocks transitions from STOPPED if any required node is unhealthy
//     or timed out. MANUAL mode bypasses ERROR but not TIMEOUT.
//
// Usage pattern (matches TransitionGuard embedding in SMNode_new):
//
//   Inside the SMNode_new constructor:
//   health_monitor_ = std::make_unique<HealthMonitor>(
//     [this](int32_t action) { fire_health_action(action); });
//   health_monitor_->register_required_node("motor_driver");
//
//   Wire a ROS timer to call check_timeouts() periodically, OR
//   construct with check_interval_ms > 0 for a built-in thread.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <packml_msgs/msg/node_health.hpp>
#include <packml_msgs/msg/node_heartbeat.hpp>

/// Alarm event fired by HealthMonitor for every health-state change on a PackML Node
/// (Equipment Module).  All transitions — including WARN and clears — produce an AlarmEvent.
/// Flat and self-contained (mirrors Alarm.msg) — no field is reread from NodeHealth.
struct AlarmEvent
{
  bool trigger{true};              ///< true = alarm raised/updated, false = cleared
  std::string node_name;
  int32_t severity{0};             ///< WARN..ABORT (NodeHealth::action levels); on clear, the
                                    ///< severity the alarm had before it cleared.
  int32_t error_code{0};           ///< user-defined condition id; 0 = unspecified, not a sentinel.
  bool is_timeout{false};          ///< true for a manager-synthesized heartbeat-timeout alarm,
                                    ///< and for the event that clears one.
  std::string message;             ///< empty on clear.
  std::string instance_id;         ///< per-location/per-unit id, or empty if not instanced;
                                    ///< always empty for a timeout alarm (see check_timeouts()).
};

/// Pure C++ health monitor for PackML Equipment Modules.
///
/// Thread-safe: on_heartbeat(), check_timeouts(), and can_transition_from_stopped()
/// may be called from any thread.
class HealthMonitor
{
public:
  using NodeHeartbeat = packml_msgs::msg::NodeHeartbeat;
  using NodeHealth = packml_msgs::msg::NodeHealth;

  /// Outcome of processing one heartbeat (see on_heartbeat()).
  ///   ACCEPTED      — normal, applied to state.
  ///   DROPPED_STALE — sequence <= last seen (duplicate/out-of-order); ignored.
  ///   RESTART       — sequence collapsed to ~1 after a high value; re-baselined.
  enum class HeartbeatResult { ACCEPTED, DROPPED_STALE, RESTART };

  /// @param fire_action   Called with NodeHealth::HOLD / SUSPEND / ABORT when a
  ///   new health event fires (new, escalating, or timeout). WARN events are
  ///   observed but do NOT invoke this callback.
  /// @param check_interval_ms  Period of the internal timeout-check thread.
  ///   Pass 0 (default) to disable the internal thread; call check_timeouts()
  ///   manually (e.g., from a ROS wall timer).  Pass a positive value for tests
  ///   or embedded use where an external timer is not available.
  /// @param on_alarm  Optional callback fired for every alarm state change,
  ///   including WARN events and alarm clears.  Designed for alarm consumers.
  explicit HealthMonitor(
    std::function<void(int32_t)> fire_action,
    uint32_t check_interval_ms = 0,
    std::function<void(const AlarmEvent &)> on_alarm = nullptr)
  : fire_action_(std::move(fire_action)),
    on_alarm_(std::move(on_alarm))
  {
    if (check_interval_ms > 0) {
      running_.store(true);
      const uint32_t interval = check_interval_ms;
      check_thread_ = std::thread(
        [this, interval]() {
          while (running_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
            check_timeouts();
          }
        });
    }
  }

  ~HealthMonitor()
  {
    running_.store(false);
    if (check_thread_.joinable()) {
      check_thread_.join();
    }
  }

  // Non-copyable, non-movable (owns a std::thread).
  HealthMonitor(const HealthMonitor &) = delete;
  HealthMonitor & operator=(const HealthMonitor &) = delete;

  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------

  /// Register a node as *required* for the health gate check.
  ///
  /// @param name                  ROS node name — must match NodeHeartbeat::node_name.
  /// @param timeout_factor        timeout = interval_ms × factor.  0 → use global.
  /// @param startup_interval_ms   Interval assumed *before* the first heartbeat
  ///   arrives from this node.  Timeout starts counting from registration time.
  ///   Default 100 ms keeps unit tests fast; production callers pass a value
  ///   matching the node's expected heartbeat_interval_ms.
  void register_required_node(
    const std::string & name,
    double timeout_factor = 0.0,
    uint32_t startup_interval_ms = 100)
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    auto & node = nodes_[name];
    node.is_required = true;
    node.timeout_factor = timeout_factor;
    node.interval_ms = startup_interval_ms;
    node.last_stamp = std::chrono::steady_clock::now();
    // last_status stays UNKNOWN and ever_seen stays false until first heartbeat.
  }

  /// Override the global timeout factor (default: 3.0).
  /// timeout = node.interval_ms × effective_factor(node)
  void set_global_timeout_factor(double factor)
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    global_timeout_factor_ = factor;
  }

  /// Reset the timed-out flag for all required nodes that are still silent.
  /// Call this when the machine returns to STOPPED (e.g. after operator CLEAR)
  /// so that still-silent nodes immediately trigger ABORT again on the next
  /// check_timeouts() cycle instead of staying stuck in STOPPED.
  void rearm_timed_out_nodes()
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    for (auto & [name, node] : nodes_) {
      if (node.timed_out) {
        node.timed_out = false;
        // last_stamp is NOT reset: the elapsed time remains large so
        // check_timeouts() fires ABORT again on its very next cycle.
      }
    }
  }

  // -------------------------------------------------------------------------
  // Runtime operations
  // -------------------------------------------------------------------------

  /// Process an incoming heartbeat from an Equipment Module.  Thread-safe.
  /// Callbacks (fire_action / on_alarm) are invoked *after* the lock is released.
  /// Returns how the heartbeat was handled (see HeartbeatResult).
  HeartbeatResult on_heartbeat(const NodeHeartbeat & msg)
  {
    int32_t action_to_fire = NodeHealth::NONE;   // NONE = nothing to fire
    std::optional<AlarmEvent> alarm;
    HeartbeatResult result = HeartbeatResult::ACCEPTED;

    {
      std::lock_guard<std::mutex> lk(nodes_mutex_);
      // Any node on the topic is tracked and may drive an action, registered or not. That
      // asymmetry with the rest of this class is deliberate: "required" marks the nodes whose
      // ABSENCE is mission-critical, so it gates the timeout sweep and the STOPPED gate, where
      // silence has to mean something. An action is the opposite case -- a node actively
      // reporting a fault it can see -- and refusing to act on one because the node was left out
      // of required_nodes would discard the report, not contain it.
      auto & node = nodes_[msg.node_name];

      // --- Sequence-number checks --------------------------------------
      // seq == 0 means "unsequenced" (legacy/tests) — skip the checks.
      // Real nodes increment from 1 via HeartbeatState::next_sequence().
      //
      // A backward/equal sequence is ambiguous by magnitude alone — it could be a
      // node restart (counter reset), a duplicate/out-of-order packet, or a second
      // "zombie" publisher on the same node_name. We disambiguate by OBSERVED
      // ABSENCE, not by how large the drop is: a real restart means the process was
      // gone, so the manager saw a gap (the node timed out, or has been silent for
      // at least its own timeout). A zombie/reorder coexists with the live node, so
      // the real stream keeps last_stamp fresh — no gap — and we drop it.
      const uint64_t seq = msg.sequence_number;
      if (seq != 0) {
        if (node.last_sequence >= 0 &&
            static_cast<int64_t>(seq) <= node.last_sequence)
        {
          const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - node.last_stamp).count();
          const bool counter_reset = static_cast<int64_t>(seq) < node.last_sequence;
          const bool was_absent =
            node.timed_out ||
            (elapsed_ms >= 0 &&
             static_cast<uint64_t>(elapsed_ms) >= effective_timeout_ms(node));
          if (counter_reset && was_absent) {
            result = HeartbeatResult::RESTART;  // accept + re-baseline below
          } else {
            // Duplicate (seq==last), or a backward jump with no observed gap
            // (out-of-order / duplicate "zombie" publisher) — drop, keep state.
            return HeartbeatResult::DROPPED_STALE;
          }
        }
        node.last_sequence = static_cast<int64_t>(seq);
      }

      // Learn (or update) the heartbeat interval, clamped to the max expected
      // so a node cannot evade timeout detection by advertising a long interval.
      if (msg.heartbeat_interval_ms > 0) {
        uint32_t iv = msg.heartbeat_interval_ms;
        if (max_interval_ms_ > 0 && iv > max_interval_ms_) {
          iv = max_interval_ms_;
        }
        node.interval_ms = iv;
      }

      // Reset timeout tracking.  A heartbeat means the node is alive, so any
      // active timeout alarm is implicitly over; a later timeout re-raises.
      node.last_stamp = std::chrono::steady_clock::now();
      node.timed_out = false;
      node.ever_seen = true;
      node.timeout_alarm_active = false;

      const int32_t prev_action = node.last_action;
      const int32_t prev_alarm_id = node.last_alarm_id;
      const std::string prev_instance_id = node.last_instance_id;
      action_to_fire = process_action_transition(node, msg.health.action, msg.health.status);
      alarm = compute_alarm(node, msg.node_name, prev_action, prev_alarm_id, prev_instance_id, msg);
    }

    // Fire callbacks outside the lock (avoids holding nodes_mutex_ across changeState).
    if (action_to_fire != NodeHealth::NONE && fire_action_) {
      fire_action_(action_to_fire);
    }
    if (alarm && on_alarm_) {
      on_alarm_(*alarm);
    }
    return result;
  }

  /// Check all tracked required nodes for heartbeat timeout.
  ///
  /// Called automatically when check_interval_ms > 0.  In manual mode
  /// (check_interval_ms == 0) the caller is responsible for periodic invocation,
  /// e.g., from a ROS wall timer.
  /// Thread-safe.
  void check_timeouts()
  {
    bool any_timeout = false;
    std::vector<AlarmEvent> alarms;
    {
      std::lock_guard<std::mutex> lk(nodes_mutex_);
      const auto now = std::chrono::steady_clock::now();

      for (auto & [name, node] : nodes_) {
        if (!node.is_required) {
          continue;
        }
        if (node.timed_out) {
          continue;  // Already fired — wait for heartbeat resume before re-arming.
        }

        const uint32_t timeout_ms = effective_timeout_ms(node);
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - node.last_stamp).count();

        // Compare in int64 (no uint32 truncation): timeout_ms <= UINT32_MAX fits.
        if (elapsed_ms >= static_cast<int64_t>(timeout_ms)) {
          const bool never_seen = !node.ever_seen;
          const bool first_raise = !node.timeout_alarm_active;
          node.timed_out = true;
          node.last_action = NodeHealth::ABORT;
          node.last_status = NodeHealth::ERROR;
          node.timeout_alarm_active = true;
          node.current_alarm_is_timeout = true;
          any_timeout = true;  // ABORT re-fires every cycle (incl. after re-arm); fire_action is idempotent.

          // Raise the alarm only on the false->true edge: re-arm re-fires
          // ABORT but must not spam duplicate "raise" alarms with no clear between.
          if (on_alarm_ && first_raise) {
            AlarmEvent ev;
            ev.trigger    = true;
            ev.node_name  = name;
            ev.severity   = NodeHealth::ABORT;
            ev.error_code = 0;
            ev.is_timeout = true;
            ev.message    = never_seen
              ? "Required node never reported a heartbeat — check the node name and that it is running"
              : "Heartbeat timeout — node stopped responding";
            // A heartbeat timeout is node-level, not per-location — never instanced.
            node.last_alarm_id = 0;
            node.last_instance_id.clear();
            alarms.push_back(std::move(ev));
          }
        }
      }
    }

    // Fire callbacks outside the lock. ABORT is a single machine-level reaction,
    // so fire it once regardless of how many nodes timed out this tick (the
    // per-node detail is in the alarms); fire_packml_action is idempotent anyway.
    if (any_timeout && fire_action_) {
      fire_action_(NodeHealth::ABORT);
    }
    if (on_alarm_) {
      for (const auto & ev : alarms) {
        on_alarm_(ev);
      }
    }
  }

  /// Returns true when all *required* nodes are healthy and a transition from
  /// STOPPED is allowed.
  ///
  /// Blocking conditions:
  ///   - Node never seen (UNKNOWN / no heartbeat yet)     → always blocks
  ///   - Node timed out                                   → always blocks
  ///   - Node last_action > WARN (HOLD / SUSPEND / ABORT) → blocks unless manual_mode
  ///
  /// @param manual_mode_active  If true, ERROR/DEGRADED states are bypassed.
  ///   TIMEOUT is non-bypassable — always blocks regardless of mode.
  bool can_transition_from_stopped(bool manual_mode_active = false) const
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    const auto now = std::chrono::steady_clock::now();
    for (const auto & [name, node] : nodes_) {
      if (gate_block_reason(node, manual_mode_active, now) != nullptr) {
        return false;
      }
    }
    return true;
  }

  /// Upper bound (ms) the manager will trust for a node's advertised interval.
  /// The per-node interval is clamped to this so a node cannot disable its own
  /// liveness check by advertising a huge interval.  0 = unbounded (default).
  void set_max_expected_interval_ms(uint32_t max_interval_ms)
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    max_interval_ms_ = max_interval_ms;
  }

  /// Human-readable list of *required* nodes currently blocking the gate, with
  /// the reason for each (timed out / never seen / unhealthy).  Empty if open.
  std::string gate_block_summary(bool manual_mode_active = false) const
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    const auto now = std::chrono::steady_clock::now();
    std::string s;
    for (const auto & [name, node] : nodes_) {
      const char * reason = gate_block_reason(node, manual_mode_active, now);
      if (reason) {
        if (!s.empty()) {
          s += ", ";
        }
        s += name + " (" + reason;
        // Diagnostics only: include the node's last reported health status
        // when it is noteworthy.  Does NOT affect the gate decision.
        if (node.last_status == NodeHealth::ERROR || node.last_status == NodeHealth::DEGRADED) {
          s += std::string(", status=") + health_status_name(node.last_status);
        }
        s += ")";
      }
    }
    return s;
  }

  /// Required nodes that have not yet reported any heartbeat.
  std::vector<std::string> unseen_required_nodes() const
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    std::vector<std::string> v;
    for (const auto & [name, node] : nodes_) {
      if (node.is_required && !node.ever_seen) {
        v.push_back(name);
      }
    }
    return v;
  }

  /// Tri-state heartbeat-liveness query for one node, for use by a completion-tracking
  /// consumer (e.g. CompletionTracker) that wants to cross-check a dedicated
  /// completion signal against liveness data.
  ///
  /// @return true  — node is registered as required and its heartbeat is current.
  ///         false — node is registered as required and is timed out, has never
  ///                 sent a heartbeat, or has a stale heartbeat, as determined by
  ///                 liveness_block_reason().
  ///         nullopt — node is NOT registered as required (e.g. listed in a
  ///                 caller's own coordinated-node list but not in this
  ///                 monitor's required_nodes) — the manager cannot
  ///                 cross-check liveness for a node it isn't monitoring.
  ///
  /// This query does not inspect `last_action`. A current heartbeat returns true
  /// even when the node reports an actionable error. gate_block_reason() applies
  /// the actionable-error policy and the manual-mode bypass separately.
  std::optional<bool> is_node_healthy(const std::string & name) const
  {
    std::lock_guard<std::mutex> lk(nodes_mutex_);
    auto it = nodes_.find(name);
    if (it == nodes_.end() || !it->second.is_required) {
      return std::nullopt;
    }
    const auto now = std::chrono::steady_clock::now();
    return liveness_block_reason(it->second, now) == nullptr;
  }

private:
  // -----------------------------------------------------------------------
  // Per-node tracked state
  // -----------------------------------------------------------------------

  struct TrackedNode
  {
    bool is_required{false};
    double timeout_factor{0.0};          // 0 → use global_timeout_factor_

    int32_t last_action{NodeHealth::NONE};
    int32_t last_status{NodeHealth::UNKNOWN};
    int32_t last_alarm_id{0};            // error_code of last fired alarm
    std::string last_instance_id;        // instance_id of last fired alarm

    std::chrono::steady_clock::time_point last_stamp{std::chrono::steady_clock::now()};
    uint32_t interval_ms{100};           // learned from heartbeat or startup default

    int64_t last_sequence{-1};           // last accepted sequence_number; -1 = none yet

    bool timed_out{false};
    bool ever_seen{false};
    bool timeout_alarm_active{false};    // dedup timeout RAISE alarms across re-arm cycles
    bool current_alarm_is_timeout{false};  // true while the active alarm is manager-synthesized
                                            // (from a timeout) rather than node-reported; carried
                                            // onto the event that eventually clears it.
  };

  // -----------------------------------------------------------------------
  // Internals
  // -----------------------------------------------------------------------

  std::function<void(int32_t)> fire_action_;
  std::function<void(const AlarmEvent &)> on_alarm_;
  mutable std::mutex nodes_mutex_;
  std::map<std::string, TrackedNode> nodes_;
  double global_timeout_factor_{3.0};
  uint32_t max_interval_ms_{0};          // 0 = unbounded; see set_max_expected_interval_ms()

  std::thread check_thread_;
  std::atomic<bool> running_{false};

  /// Evaluate one heartbeat action against the node's current tracked state.
  /// Must be called with nodes_mutex_ held.
  ///
  /// Transition rules (in order of evaluation):
  ///   1. CLEARED  : action==NONE OR status==HEALTHY → reset, no fire
  ///   2. SAME     : same action already active      → suppress
  ///   3. DE-ESCALATION : new_action < last_action   → update, suppress
  ///   4. WARN     : new_action == WARN              → log only, update, no fire
  ///   5. NEW / ESCALATION (HOLD/SUSPEND/ABORT)      → fire
  /// Returns the action to fire (HOLD/SUSPEND/ABORT), or NodeHealth::NONE for
  /// "no fire".  Updates tracked state.  Must be called with nodes_mutex_ held; the
  /// caller invokes fire_action_ after releasing the lock.
  int32_t process_action_transition(
    TrackedNode & node,
    int32_t new_action,
    int32_t new_status)
  {
    // 1. Event cleared: node reports healthy again.
    if (new_action == NodeHealth::NONE || new_status == NodeHealth::HEALTHY) {
      node.last_action = NodeHealth::NONE;
      node.last_status = new_status;
      return NodeHealth::NONE;  // recovery is handled by the state machine externally.
    }

    // 2. Same action still active — suppress repeat firing.
    if (new_action == node.last_action) {
      node.last_status = new_status;
      return NodeHealth::NONE;
    }

    // 3. De-escalation while still in error (e.g., ABORT → HOLD after partial recovery).
    //    Update tracked action so a subsequent re-escalation can be detected,
    //    but do NOT fire again.
    if (new_action < node.last_action && node.last_action != NodeHealth::NONE) {
      node.last_action = new_action;
      node.last_status = new_status;
      return NodeHealth::NONE;
    }

    // 4. WARN — manager observes but does not act (WARN is not a blocking condition).
    if (new_action == NodeHealth::WARN) {
      node.last_action = NodeHealth::WARN;
      node.last_status = new_status;
      return NodeHealth::NONE;
    }

    // 5. New event or escalation (HOLD, SUSPEND, ABORT) — fire.
    node.last_action = new_action;
    node.last_status = new_status;
    return new_action;
  }

  /// Compute the effective timeout for a node.  Must be called with nodes_mutex_ held.
  uint32_t effective_timeout_ms(const TrackedNode & node) const
  {
    const double factor =
      (node.timeout_factor > 0.0) ? node.timeout_factor : global_timeout_factor_;
    // Saturate instead of truncating: interval_ms * factor can exceed UINT32_MAX
    // for an absurd advertised interval, and a uint32 wrap would yield a tiny
    // timeout (spurious premature ABORT). Cap at UINT32_MAX (~49.7 days).
    const double t = static_cast<double>(node.interval_ms) * factor;
    return (t >= static_cast<double>(std::numeric_limits<uint32_t>::max()))
      ? std::numeric_limits<uint32_t>::max()
      : static_cast<uint32_t>(t);
  }

  /// Reason a required node fails the heartbeat-liveness check, or nullptr if it
  /// passes. Detects an overdue heartbeat inline instead of relying only on the
  /// cached timed_out flag. Does not inspect last_action. Must be called with
  /// nodes_mutex_ held.
  const char * liveness_block_reason(
    const TrackedNode & node,
    std::chrono::steady_clock::time_point now) const
  {
    if (!node.is_required) {
      return nullptr;
    }
    if (node.timed_out) {
      return "timed out";   // Non-bypassable — already flagged by check_timeouts().
    }
    if (!node.ever_seen) {
      return "never seen";  // Non-bypassable — node not yet heard from.
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - node.last_stamp).count();
    if (elapsed_ms >= static_cast<int64_t>(effective_timeout_ms(node))) {
      // Distinct from "timed out": the heartbeat is overdue but check_timeouts()
      // has not yet run to formally flag it (so no ABORT has fired for it yet).
      return "heartbeat stale";   // Non-bypassable — beat overdue, timeout not yet confirmed.
    }
    return nullptr;
  }

  /// Reason a required node blocks the STOPPED gate, or nullptr if it does not.
  /// Combines heartbeat liveness with actionable-error policy. The manual-mode
  /// bypass applies only to actionable errors. Shared by can_transition_from_stopped()
  /// and gate_block_summary(). Must be called with nodes_mutex_ held.
  const char * gate_block_reason(
    const TrackedNode & node,
    bool manual_mode_active,
    std::chrono::steady_clock::time_point now) const
  {
    if (const char * reason = liveness_block_reason(node, now)) {
      return reason;
    }
    if (node.is_required && !manual_mode_active && node.last_action > NodeHealth::WARN) {
      return "unhealthy";   // Actionable error — bypassed only in manual mode.
    }
    return nullptr;
  }

  /// Human-readable name for a NodeHealth status constant (diagnostics only).
  static const char * health_status_name(int32_t status)
  {
    switch (status) {
      case NodeHealth::HEALTHY:  return "HEALTHY";
      case NodeHealth::DEGRADED: return "DEGRADED";
      case NodeHealth::ERROR:    return "ERROR";
      default:                   return "UNKNOWN";
    }
  }

  /// Returns the AlarmEvent to fire (raise/update/clear), or nullopt for none.
  /// Updates tracked state.  Must be called with nodes_mutex_ held; the caller invokes
  /// on_alarm_ after releasing the lock.  Also fires for WARN (unlike actions).
  std::optional<AlarmEvent> compute_alarm(
    TrackedNode & node,
    const std::string & node_name,
    int32_t prev_action,
    int32_t prev_alarm_id,
    const std::string & prev_instance_id,
    const NodeHeartbeat & msg)
  {
    if (!on_alarm_) {
      return std::nullopt;
    }
    const int32_t cur_action = node.last_action;
    const int32_t cur_id     = msg.health.error_code;
    const std::string & cur_instance_id = msg.health.instance_id;

    const bool was_alarmed = (prev_action != NodeHealth::NONE);
    const bool is_alarmed  = (cur_action  != NodeHealth::NONE);

    if (is_alarmed) {
      // Fire if new alarm, severity changed, error code changed, or — for the same
      // action/code — a different instance (e.g. a second, simultaneous E-stop
      // location) rather than a repeat of the one already alarmed.
      if (!was_alarmed || prev_action != cur_action || prev_alarm_id != cur_id ||
        prev_instance_id != cur_instance_id)
      {
        AlarmEvent ev;
        ev.trigger    = true;
        ev.node_name  = node_name;
        ev.severity   = cur_action;
        ev.error_code = cur_id;
        ev.is_timeout = false;  // any heartbeat-driven raise is node-reported, not synthesized
        ev.message    = msg.health.message;
        ev.instance_id = cur_instance_id;
        node.last_alarm_id = cur_id;
        node.last_instance_id = cur_instance_id;
        node.current_alarm_is_timeout = false;
        return ev;
      }
    } else if (was_alarmed) {
      // Alarm cleared. severity/error_code/instance_id preserve what the alarm was, not
      // a neutral value — a consumer sees "the HOLD alarm (id 101) cleared", not "a 0
      // alarm cleared".
      AlarmEvent ev;
      ev.trigger    = false;
      ev.node_name  = node_name;
      ev.severity   = prev_action;
      ev.error_code = prev_alarm_id;
      ev.is_timeout = node.current_alarm_is_timeout;
      ev.instance_id = prev_instance_id;
      node.current_alarm_is_timeout = false;
      node.last_alarm_id = 0;
      node.last_instance_id.clear();
      return ev;
    }
    return std::nullopt;
  }
};
