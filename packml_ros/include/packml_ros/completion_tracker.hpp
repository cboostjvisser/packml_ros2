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
// CompletionTracker — pure C++ helper class embedded in the PackML manager,
// structurally parallel to (but independent of) HealthMonitor.
//
// Aggregates the per-node results of the ~/packml_state_transition ACTION (see
// packml_msgs/action/StateTransition.action) so a StateMachine::setStateOperation-bound
// function can block until every fanned-out node has reported that it finished (or failed)
// the current acting state, with an optional cross-check against a health predicate (see
// HealthMonitor::is_node_healthy()) so an unhealthy node aborts the wait instead of riding
// out the full timeout. Correlation to "the current cycle" rides the action's own goal id --
// no separate generation/sequence field is needed on the wire.

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

#include <rclcpp_action/types.hpp>

/// Thread-safe aggregator for one fan-out round of the state-transition action.
///
/// begin_round() / on_goal_accepted() / on_goal_rejected() / on_result() may be called from
/// any thread (in practice, the ROS executor thread, via the action client's callbacks).
/// wait_for_all() is intended to be called from the setStateOperation-bound function, which
/// runs on the QtConcurrent worker pool ActingState::operation() dispatches onto — a thread
/// distinct from both the ROS executor thread and the Qt state-machine thread.
class CompletionTracker
{
public:
  enum class WaitResult { COMPLETE, TIMEOUT, FAILED, ABORTED_BY_HEALTH, SHUTDOWN, INTERRUPTED };

  /// @param node_healthy  Tri-state health query (see HealthMonitor::is_node_healthy()).
  ///   nullopt (not registered as a required node) is treated as health-blind for that
  ///   node — no interrupt fires for it; the completion timeout is the only backstop.
  ///   Pass nullptr to disable the health cross-check entirely.
  explicit CompletionTracker(
    std::function<std::optional<bool>(const std::string &)> node_healthy = nullptr)
  : node_healthy_(std::move(node_healthy))
  {}

  /// Start a new fan-out round for exactly these node names, discarding all prior-round
  /// tracking. Called once, synchronously, from the manager's fan-out (on_state_changed, the
  /// Qt thread) BEFORE any goal for this round is sent, so a node whose acceptance/result
  /// callback fires unusually fast can never race ahead of its own registration.
  ///
  /// RETURNS THIS ROUND'S ID, which every subsequent mutator for this round must be given.
  /// Rebuilding nodes_ cannot separate rounds by itself -- every round rebuilds the same keys
  /// from the same immutable client map, so a dead round's late callback still finds a live entry
  /// to write into. REQUIRED on every mutator rather than defaulted, so it cannot be omitted.
  [[nodiscard]] uint64_t begin_round(const std::vector<std::string> & node_names)
  {
    std::lock_guard<std::mutex> lk(round_mutex_);
    ++round_;
    nodes_.clear();
    for (const auto & name : node_names) {
      nodes_[name];  // default-construct: no goal id yet, no result yet
    }
    return round_;
  }

  /// The round id begin_round() most recently handed out. Lets a caller holding a
  /// longer-lived object (the manager's ClientFanout, whose lifetime is deliberately
  /// different from a round's) ask "am I still the live round?" before acting.
  uint64_t current_round() const
  {
    std::lock_guard<std::mutex> lk(round_mutex_);
    return round_;
  }

  /// Bind the goal id the action client just received acceptance for, for `name`, in the
  /// CURRENT round. A later result tagged with a DIFFERENT goal id (a stale round's leftover,
  /// or a goal this tracker never registered) is therefore never mistaken for an answer to
  /// this one.
  void on_goal_accepted(
    const std::string & name, const rclcpp_action::GoalUUID & goal_id, uint64_t round)
  {
    std::lock_guard<std::mutex> lk(round_mutex_);
    if (round != round_) {
      return;  // a dead round's acceptance -- must not satisfy the live one
    }
    auto it = nodes_.find(name);
    if (it == nodes_.end()) {
      return;  // this round has already been superseded
    }
    it->second.goal_id = goal_id;
    it->second.has_goal_id = true;
    round_progress_cv_.notify_all();
  }

  /// Record that a goal was rejected outright at the ROS admission level. Not expected in
  /// normal operation (handle_goal always accepts; business-logic rejection surfaces as a
  /// failed RESULT, not a rejected goal) but handled defensively so a misbehaving/incompatible
  /// server still fails the wait fast rather than riding out the full timeout.
  /// `round` is what makes this safe to call from a ClientFanout's expiry path: a fan-out
  /// deliberately outlives a round (its own 5s deadline vs. the round's much longer completion
  /// budget), so without the round check an expired OLD fan-out's on_client_failed poisons the
  /// CURRENT round.
  void on_goal_rejected(const std::string & name, const std::string & message, uint64_t round)
  {
    std::lock_guard<std::mutex> lk(round_mutex_);
    if (round != round_) {
      return;  // a dead round's rejection -- must not fail the live one
    }
    auto it = nodes_.find(name);
    if (it == nodes_.end()) {
      return;
    }
    it->second.has_goal_id = true;  // no real id, but accounted for
    it->second.has_result = true;
    it->second.success = false;
    it->second.message = message;
    round_progress_cv_.notify_all();
  }

  /// Feed one action result. Called from the action client's result callback on the ROS
  /// executor thread — MUST stay non-blocking: the manager runs a single-threaded executor,
  /// so a blocking callback here would starve delivery of every other node's result,
  /// deadlocking the very aggregation it feeds (the same hazard as HealthMonitor's heartbeat
  /// callback).
  void on_result(
    const std::string & name, const rclcpp_action::GoalUUID & goal_id,
    bool success, int32_t error_code, const std::string & message, uint64_t round)
  {
    {
      std::lock_guard<std::mutex> lk(round_mutex_);
      // The goal-id check below is already sound for results (a result carries the id of the
      // goal it answers), so the round check is redundant here. It is applied anyway so all
      // three mutators share one identity rule rather than each having its own.
      if (round != round_) {
        return;  // a dead round's leftover result — discard
      }
      auto it = nodes_.find(name);
      if (it == nodes_.end() || !it->second.has_goal_id || it->second.goal_id != goal_id) {
        return;  // stale round's leftover result — discard
      }
      it->second.has_result = true;
      it->second.success = success;
      it->second.error_code = error_code;
      it->second.message = message;
    }
    round_progress_cv_.notify_all();
  }

  /// Blocks the calling thread until every node registered by the current round's
  /// begin_round() has a matching result, one fails, a node goes unhealthy, shutdown is
  /// requested, `stop_token` requests a stop, or `timeout` elapses.
  ///
  /// `stop_token` is how a bound StateMachine::setInterruptibleStateOperation() function
  /// reacts to being exited early (see ActingState::onExit()): an operator's HOLD/SUSPEND/
  /// ABORT/STOP accepted while this wait is still in flight requests stop on the SAME token
  /// this call was given, waking it immediately instead of riding out `timeout`. Defaults to
  /// a token with no associated source, which never requests stop.
  /// `health_cross_check`: pass false to make this wait ignore node health entirely. Intended
  /// for ABORTING, and the caller is the only party that knows which coordinated state this wait
  /// belongs to (this class deliberately does not). ABORTING is already the machine's terminal
  /// response to an unhealthy node, so cross-checking health there asks the fault to authorise
  /// its own handling: the node that caused the abort is still unhealthy while ABORTING runs, the
  /// wait fails, and there is no state left to escalate to. Every other coordinated state
  /// genuinely wants the cross-check, because for them "a required node just died" means this
  /// transition cannot complete and ABORTING is a real escalation target.
  WaitResult wait_for_all(
    std::chrono::milliseconds timeout, std::stop_token stop_token = {},
    bool health_cross_check = true)
  {
    std::unique_lock<std::mutex> lk(round_mutex_);
    // Bridges the token into the condition variable, which has no native stop_token support:
    // wakes an in-flight wait on interrupt rather than at its next natural wakeup. The predicate
    // still re-checks stop_requested(), so this is a wakeup hint, not the interrupt detection.
    std::stop_callback wake_on_interrupt(stop_token, [this]() {round_progress_cv_.notify_all();});

    bool failed = false;
    bool unhealthy = false;
    const bool predicate_satisfied = round_progress_cv_.wait_for(
      lk, timeout,
      [this, &failed, &unhealthy, &stop_token, health_cross_check]() {
        if (shutdown_.load()) {
          return true;
        }
        if (stop_token.stop_requested()) {
          return true;
        }
        // EVERY node is examined before concluding "still waiting": returning at the first node
        // without a result would hide a FAILURE recorded by any node sorting after it in this
        // std::map, making the outcome depend on node names. The two early exits below are
        // deliberate -- a failure and an unhealthy unanswered node both end the wait outright.
        bool still_waiting = false;
        for (const auto & [name, entry] : nodes_) {
          // ORDER IS LOAD-BEARING: a node's own answer is checked BEFORE its health. Health
          // predicts whether a node will still answer, so it is only meaningful for one that has
          // NOT answered yet; a node that reported success and then went unhealthy is the next
          // coordinated state's problem, not grounds for failing a completed one.
          if (entry.has_result) {
            if (!entry.success) {
              failed = true;
              return true;
            }
            continue;  // answered successfully -- done, regardless of health
          }

          // No answer yet. An unhealthy node is one that will most likely never answer, so
          // give up now rather than riding out the full timeout.
          if (node_healthy_ && health_cross_check) {
            const auto healthy = node_healthy_(name);
            if (healthy.has_value() && !*healthy) {
              unhealthy = true;
              return true;
            }
          }
          still_waiting = true;  // no answer and not known-unhealthy -- keep scanning the rest
        }
        return !still_waiting;  // true only when every node in this round answered success
      });
    if (shutdown_.load()) {
      return WaitResult::SHUTDOWN;
    }
    if (stop_token.stop_requested()) {
      return WaitResult::INTERRUPTED;
    }
    if (!predicate_satisfied) {
      return WaitResult::TIMEOUT;
    }
    if (unhealthy) {
      return WaitResult::ABORTED_BY_HEALTH;
    }
    if (failed) {
      return WaitResult::FAILED;
    }
    return WaitResult::COMPLETE;
  }

  /// Node names in the CURRENT round with no result recorded yet — used by the caller (after
  /// a TIMEOUT) to know which outstanding goals are still worth cancelling.
  std::vector<std::string> pending_nodes() const
  {
    std::lock_guard<std::mutex> lk(round_mutex_);
    std::vector<std::string> pending;
    for (const auto & [name, entry] : nodes_) {
      if (!entry.has_result) {
        pending.push_back(name);
      }
    }
    return pending;
  }

  /// Wake any in-flight wait_for_all() immediately. Call once, at manager shutdown, BEFORE
  /// the owning StateMachine is torn down (StateMachine::drainActingStates() has no internal
  /// timeout — see PackmlManagerInterface::shutdown()) so an in-flight wait does not block
  /// process teardown for up to state_complete_timeout_ms.
  void request_shutdown()
  {
    // Taken under round_mutex_ even though shutdown_ is atomic, for the same reason
    // notify_health_change() does: an atomic store stops a TORN read, not a LOST WAKEUP. A waiter
    // that has evaluated the predicate but not yet parked on round_progress_cv_ would miss a
    // notify issued in that gap and sleep out the whole state_complete_timeout_ms. Publishing the
    // flag under the predicate's own mutex closes it.
    {
      std::lock_guard<std::mutex> lk(round_mutex_);
      shutdown_.store(true);
    }
    round_progress_cv_.notify_all();
  }

  /// Call from HealthMonitor's fire_action path so a health-triggered abort during an
  /// in-flight wait doesn't have to wait for the condition variable's next natural wakeup.
  void notify_health_change()
  {
    std::lock_guard<std::mutex> lk(round_mutex_);
    round_progress_cv_.notify_all();
  }

private:
  struct Entry
  {
    rclcpp_action::GoalUUID goal_id{};
    bool has_goal_id{false};
    bool has_result{false};
    bool success{false};
    int32_t error_code{0};
    std::string message;
  };

  /// Guards this round's tracking state -- round_ and nodes_ -- and is the lock
  /// round_progress_cv_ waits on, so it also publishes shutdown_ (see request_shutdown()).
  mutable std::mutex round_mutex_;
  /// Signals anything that could satisfy or end an in-flight wait_for_all(): a goal accepted or
  /// rejected, a result recorded, a health change, shutdown.
  std::condition_variable round_progress_cv_;
  std::map<std::string, Entry> nodes_;
  /// Monotonic round identity. Guarded by round_mutex_ (not atomic) so a mutator's round check
  /// and its write to nodes_ are one critical section -- an atomic would let a round change land
  /// between the check and the write.
  uint64_t round_{0};
  std::atomic<bool> shutdown_{false};
  std::function<std::optional<bool>(const std::string &)> node_healthy_;
};
