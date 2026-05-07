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

#include "packml_ros/transition_guard.hpp"

namespace packml_ros {

TransitionResult TransitionGuard::request_state(packml_sm::State target)
{
  TransitionResult result;

  // Already in the requested state — no-op success
  if (target == current_state_) {
    result.accepted = true;
    result.already_there = true;
    return result;
  }

  // Warn if another transition is in progress
  if (waiting_for_state_) {
    result.error = "State change requested while already waiting on a state transition";
  } else if (waiting_for_mode_) {
    result.error = "State change requested while a mode change is still active";
  }

  // Accept the transition (current behavior: accept despite warnings)
  waiting_for_state_ = true;
  switching_state_ = target;
  result.accepted = true;
  return result;
}

TransitionResult TransitionGuard::request_mode(packml_sm::ModeType target)
{
  TransitionResult result;

  // Already in the requested mode — no-op success
  if (target == current_mode_) {
    result.accepted = true;
    result.already_there = true;
    return result;
  }

  // Warn if another transition is in progress
  if (waiting_for_state_) {
    result.error = "Mode change requested while a state change is in progress";
  } else if (waiting_for_mode_) {
    result.error = "Mode change requested while already waiting on a mode transition";
  }

  // Accept the transition
  waiting_for_mode_ = true;
  switching_mode_ = target;
  result.accepted = true;
  return result;
}

bool TransitionGuard::on_status_update(packml_sm::State state, packml_sm::ModeType mode)
{
  bool changed = false;

  if (state != current_state_) {
    current_state_ = state;
    waiting_for_state_ = false;
    changed = true;
  }

  if (mode != current_mode_) {
    current_mode_ = mode;
    waiting_for_mode_ = false;
    changed = true;
  }

  return changed;
}

}  // namespace packml_ros
