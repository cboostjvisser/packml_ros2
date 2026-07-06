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
#include <string>
#include <packml_sm/common.hpp>
#include "packml_ros/ros_names.hpp"

namespace packml_ros {

/// Result of a state or mode transition request.
struct TransitionResult
{
  bool accepted{false};      ///< Request was accepted (may still carry a warning in `error`).
  bool already_there{false}; ///< Node was already in the requested state/mode - no-op success.
  std::string error;         ///< Non-empty if there was a warning or rejection reason.
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
  TransitionResult request_state(packml_sm::State target);
  TransitionResult request_mode(packml_sm::ModeType target);

  /// Called when the manager publishes a status update.
  /// Clears in-flight switching flags and returns true if anything changed.
  bool on_status_update(packml_sm::State state, packml_sm::ModeType mode);

  packml_sm::State    current_state()      const { std::lock_guard<std::mutex> lk(state_mutex_); return current_state_; }
  packml_sm::ModeType current_mode()       const { std::lock_guard<std::mutex> lk(state_mutex_); return current_mode_; }
  bool                is_switching_state() const { std::lock_guard<std::mutex> lk(state_mutex_); return waiting_for_state_; }
  bool                is_switching_mode()  const { std::lock_guard<std::mutex> lk(state_mutex_); return waiting_for_mode_; }

private:
  mutable std::mutex  state_mutex_;   // guards every field below
  packml_sm::State    current_state_{packml_sm::State::UNDEFINED};
  packml_sm::ModeType current_mode_{0};
  packml_sm::State    switching_state_{packml_sm::State::UNDEFINED};
  packml_sm::ModeType switching_mode_{0};
  bool waiting_for_state_{false};
  bool waiting_for_mode_{false};
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
  void set_latch(int32_t status, int32_t action, int32_t error_code, std::string message)
  {
    std::lock_guard<std::mutex> lk(latch_mutex_);
    latch_active_     = true;
    latch_status_     = status;
    latch_action_     = action;
    latch_error_code_ = error_code;
    latch_message_    = std::move(message);
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
  };
  LatchSnapshot latch_snapshot() const
  {
    std::lock_guard<std::mutex> lk(latch_mutex_);
    return LatchSnapshot{latch_active_, latch_status_, latch_action_, latch_error_code_, latch_message_};
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
};

// ---------------------------------------------------------------------------
/// Umbrella that composes both protocol concerns for a PackML managed node.
///
/// Usage:
///   PackmlNodeProtocol protocol_;
///   protocol_.transitions.request_state(state);
///   protocol_.heartbeat.next_sequence();
// ---------------------------------------------------------------------------
struct PackmlNodeProtocol
{
  TransitionGuard transitions;
  HeartbeatState        heartbeat;
};


}  // namespace packml_ros
