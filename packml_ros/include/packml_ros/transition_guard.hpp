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
// Shared protocol logic for PackML managed nodes, used by both rclcpp (C++) and
// rclpy (Python, via pybind11) nodes.
//
// Two focused classes cover the two distinct concerns:
//   - TransitionGuard  : validates and tracks state/mode transition requests
//   - HeartbeatState   : sequence counter, interval, node name, active flag
//
// PackmlNodeProtocol composes both as named sub-objects so call sites are explicit:
//   protocol_.transitions.request_state(state)
//   protocol_.heartbeat.next_sequence()

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <packml_sm/common.hpp>
#include "packml_ros/ros_names.hpp"

namespace packml_ros {

class TransitionGuard;

/// Result of a state or mode transition request.
struct TransitionResult
{
  bool accepted{false};      ///< Request was accepted (may still carry a warning in `error`).
  bool already_there{false}; ///< Node was already in the requested state/mode - no-op success.
  std::string error;         ///< Non-empty if there was a warning or rejection reason.
};

// ---------------------------------------------------------------------------
/// Move-only handle to one armed, in-flight state transition.  Dropping it disarms the guard.
///
/// The arm exists to refuse a SECOND state transition while this node is working on one, so its
/// lifetime is exactly "this node is working on it" -- which is a scope, and therefore a
/// destructor.  Releasing it on whichever status echo reports a DIFFERENT state fails in both
/// directions: a request ending without the state changing (the node refuses it, the deferred wait
/// times out, the goal is cancelled) leaves the arm set, and a node adopting exactly the state it
/// was asked for reports no change at all, so its own COMPLETED request stays armed and rejects
/// the next one -- aborting a legitimate goal, failing the manager's coordinated wait, and
/// collapsing a running machine to ABORTING.
///
/// Release is keyed on the arm's own id, not on its target state.  A superseded arm and a fresh
/// arm for the same target are different arms, so a late release can never disarm somebody
/// else's transition.
///
/// The token holds SHARED ownership of the guard's state, not a pointer to the guard, so a token
/// can safely outlive the TransitionGuard it came from.  That is not a hypothetical: a detached
/// deferred-completion thread can still hold one while its node is torn down, and Python's
/// interpreter shutdown destroys the guard and the token in whatever order it likes.  With a raw
/// back-pointer, releasing then locks a destroyed mutex -- which reliably hung the Python suite at
/// exit.
// ---------------------------------------------------------------------------
class InFlightToken
{
public:
  InFlightToken() = default;
  ~InFlightToken() { release(); }

  InFlightToken(InFlightToken && other) noexcept
  : core_(std::move(other.core_)),
    arm_id_(std::exchange(other.arm_id_, 0)) {}

  InFlightToken & operator=(InFlightToken && other) noexcept
  {
    if (this != &other) {
      release();
      core_ = std::move(other.core_);
      arm_id_ = std::exchange(other.arm_id_, 0);
    }
    return *this;
  }

  InFlightToken(const InFlightToken &) = delete;
  InFlightToken & operator=(const InFlightToken &) = delete;

  /// Disarm now rather than at scope exit.  Idempotent, and safe on an empty or moved-from
  /// token.  Bound to Python's __exit__, which has no destructor to lean on.
  void release() noexcept;

  /// True while this token still holds an arm.
  bool armed() const noexcept { return nullptr != core_; }

private:
  friend class TransitionGuard;
  struct Core;
  InFlightToken(std::shared_ptr<Core> core, uint64_t arm_id) noexcept
  : core_(std::move(core)), arm_id_(arm_id) {}

  std::shared_ptr<Core> core_;
  uint64_t arm_id_{0};
};

/// A decided state request together with the arm it created.  Move-only, because the arm is.
/// `arm` is disarmed when the decision was already_there or a rejection -- neither holds the
/// guard, because neither is going to do any work that a second request must wait behind.
struct ArmedTransition
{
  TransitionResult result;
  InFlightToken arm;
};

// ---------------------------------------------------------------------------
/// Validates and tracks state/mode transition requests from the PackML manager.
///
/// Thread-safe: a single internal mutex guards all state, so the transition
/// service callbacks and the status-update callback may run on different
/// executor threads without racing.
// ---------------------------------------------------------------------------
class TransitionGuard
{
public:
  TransitionGuard();
  ~TransitionGuard();
  TransitionGuard(const TransitionGuard &) = delete;
  TransitionGuard & operator=(const TransitionGuard &) = delete;

  /// Decide a state request AT GOAL ADMISSION and hold the decision -- and the arm it creates --
  /// until that same goal's execution claims it with claim_state().
  ///
  /// The decision, above all whether this node is already_there, is taken here and never
  /// recomputed, because admission is the only point where its input is clean.  already_there is
  /// a function of current_state_, and current_state_ is poisoned by this very goal's own status
  /// echo: the manager publishes it as soon as the SendGoal response lands, which is after this
  /// call returns but can be before the goal executes.  Deciding later would mean inferring "an
  /// echo must have raced me" from a snapshot, and an inference has false negatives where a stored
  /// answer has none.
  TransitionResult admit_state(packml_sm::State target);

  /// Take the decision admit_state() parked for `target`, together with its arm.
  ///
  /// Falls back to deciding now -- loudly -- if nothing was admitted for this target, so a
  /// caller that never had an admission step still works.  That fallback re-opens the racing-echo
  /// hole for its one goal, which is why it says so rather than passing silently.
  ArmedTransition claim_state(packml_sm::State target);

  /// admit_state() immediately followed by claim_state(), for callers with no admission/execution
  /// split -- direct users of the guard, and tests.
  ArmedTransition request_state(packml_sm::State target);

  TransitionResult request_mode(packml_sm::ModeType target);

  /// Called when the manager publishes a status update.
  /// Clears in-flight switching flags and returns true if anything changed.
  bool on_status_update(packml_sm::State state, packml_sm::ModeType mode);

  packml_sm::State    current_state()      const;
  packml_sm::ModeType current_mode()       const;
  bool                is_switching_state() const;
  bool                is_switching_mode()  const;

private:
  friend class InFlightToken;

  /// Decide a state request against the current view.  Caller holds core_->guard_mutex.
  TransitionResult decide_state(packml_sm::State target);

  // Every field lives here, behind one mutex, in a separately-allocated block that outstanding
  // InFlightTokens co-own.  The indirection buys exactly one thing: a token may outlive the
  // guard, and releasing must stay well-defined when it does.
  std::shared_ptr<InFlightToken::Core> core_;
};

// ---------------------------------------------------------------------------
/// Heartbeat sequencing and publishing-control state.
///
/// Owns the sequence counter, publishing interval, node name, and the active
/// flag used to simulate silent/crashed nodes in tests.  All methods are
/// trivial and defined inline.  Uses std::atomic for fields shared across threads.
// ---------------------------------------------------------------------------
class HeartbeatState
{
public:
  /// Call once during node startup, before the first timer tick or post_event().
  void init(std::string node_name, uint32_t interval_ms)
  {
    node_name_   = std::move(node_name);
    interval_ms_ = interval_ms;
  }

  /// Atomically increment and return the next sequence number.  sequence_ starts
  /// at 0 and this pre-increments, so the first heartbeat is 1.  0 is reserved to
  /// mean "unsequenced" (see NodeHeartbeat.msg and HealthMonitor's sequence checks).
  uint64_t next_sequence() { return ++sequence_; }

  uint32_t           interval_ms() const { return interval_ms_; }
  const std::string& node_name()   const { return node_name_; }

  /// Pause or resume publishing.  When paused the timer ticks but nothing is
  /// sent, simulating a crashed or silent node for the HealthMonitor.
  void set_active(bool active) { active_.store(active); }
  bool is_active()       const { return active_.load(); }

  // --- Event latch (sticky post_event health) -----------------------------
  // When a node calls post_event() with an actionable health, the node interface
  // latches it here so the periodic publisher repeats that state instead of
  // calling get_health_status() — preventing fault/heal flapping when the getter
  // is not also updated. Cleared by posting a healthy/NONE event (clear_latch()).
  void set_latch(
    int32_t status, int32_t action, int32_t error_code, std::string message,
    std::string instance_id = "")
  {
    std::lock_guard<std::mutex> lk(latch_mutex_);
    latch_active_     = true;
    latch_status_     = status;
    latch_action_     = action;
    latch_error_code_ = error_code;
    latch_message_    = std::move(message);
    latch_instance_id_ = std::move(instance_id);
  }
  void clear_latch()
  {
    std::lock_guard<std::mutex> lk(latch_mutex_);
    latch_active_ = false;
  }

  /// Atomic snapshot of the whole latch under one lock — the ONLY way to read the
  /// latch, so a concurrent set_latch()/clear_latch() cannot splice a torn
  /// (mixed-field) read. Used by both the C++ and Python periodic publishers.
  struct LatchSnapshot
  {
    bool        active{false};
    int32_t     status{0};
    int32_t     action{0};
    int32_t     error_code{0};
    std::string message;
    std::string instance_id;
  };
  LatchSnapshot latch_snapshot() const
  {
    std::lock_guard<std::mutex> lk(latch_mutex_);
    return LatchSnapshot{
      latch_active_, latch_status_, latch_action_, latch_error_code_, latch_message_,
      latch_instance_id_};
  }

private:
  std::string           node_name_;
  uint32_t              interval_ms_{1000};
  std::atomic<uint64_t> sequence_{0};
  std::atomic<bool>     active_{true};

  mutable std::mutex latch_mutex_;
  bool        latch_active_{false};
  int32_t     latch_status_{0};
  int32_t     latch_action_{0};
  int32_t     latch_error_code_{0};
  std::string latch_message_;
  std::string latch_instance_id_;
};

// ---------------------------------------------------------------------------
/// Umbrella that composes both protocol concerns for a PackML managed node.
///
/// Usage:
///   PackmlNodeProtocol protocol_;
///   protocol_.transitions.request_state(state);
///   protocol_.heartbeat.next_sequence();
///
/// Note: the state-completion signal (see PackmlNodeInterface::on_deferred_work() /
/// defers_completion()) is carried by the ~/packml_state_transition ACTION's own result and
/// goal id -- it needs no shared pure-logic class here, unlike transitions/heartbeat. The
/// deferred-completion wait itself is plumbing (a condition_variable in C++, a
/// threading.Condition in Python) that differs enough between the two languages' action-server
/// implementations that it is hand-written on each side rather than shared through pybind11.
// ---------------------------------------------------------------------------
struct PackmlNodeProtocol
{
  TransitionGuard transitions;
  HeartbeatState  heartbeat;
};


}  // namespace packml_ros
