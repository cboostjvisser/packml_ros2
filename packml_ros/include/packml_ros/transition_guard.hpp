// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// ROS-independent, C++/Python(pybind11) shared code, client protocol logic for PackML managed nodes.

#pragma once

#include <string>
#include <packml_sm/common.hpp>

namespace packml_ros {

// ─── Canonical topic / service names ─────────────────────────────────
// These are used by both C++ PackmlNodeInterface and Python PackmlNode.
static constexpr auto kStateTransitionService = "packml_state_transition";
static constexpr auto kModeTransitionService  = "packml_mode_transition";
static constexpr auto kStatusTopic            = "packml_status";

/// Result of a state or mode transition request.
struct TransitionResult
{
  /// Whether the request was accepted (may still carry a warning in `error`).
  bool accepted{false};
  /// True if the node was already in the requested state/mode (no-op success).
  bool already_there{false};
  /// Non-empty if there was a warning or rejection reason.
  std::string error;
};

/// ROS-independent client protocol logic for PackML managed nodes.
///
/// Tracks current state/mode and validates incoming transition requests from
/// the manager.  Shared between C++ PackmlNodeInterface and pybind11 Python
/// bindings — single source of truth for the coordination protocol.
class TransitionGuard
{
public:
  /// Request a state transition.  Returns whether it was accepted and any
  /// warnings/errors.
  TransitionResult request_state(packml_sm::State target);

  /// Request a mode transition.
  TransitionResult request_mode(packml_sm::ModeType target);

  /// Called when the manager publishes a status update.
  /// Updates internal state/mode and clears switching flags.
  /// Returns true if state or mode actually changed.
  bool on_status_update(packml_sm::State state, packml_sm::ModeType mode);

  /// Current confirmed state.
  packml_sm::State current_state() const { return current_state_; }

  /// Current confirmed mode.
  packml_sm::ModeType current_mode() const { return current_mode_; }

  /// True if a state transition is in-flight (requested but not yet confirmed).
  bool is_switching_state() const { return waiting_for_state_; }

  /// True if a mode transition is in-flight.
  bool is_switching_mode() const { return waiting_for_mode_; }

private:
  packml_sm::State current_state_{packml_sm::State::UNDEFINED};
  packml_sm::ModeType current_mode_{0};
  packml_sm::State switching_state_{packml_sm::State::UNDEFINED};
  packml_sm::ModeType switching_mode_{0};
  bool waiting_for_state_{false};
  bool waiting_for_mode_{false};
};

}  // namespace packml_ros
