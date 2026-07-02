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
  std::lock_guard<std::mutex> lk(state_mutex_);   // guard all transition state.
  TransitionResult result;

  // Already in the requested state — no-op success.
  if (target == current_state_) {
    result.accepted = true;
    result.already_there = true;
    return result;
  }

  // A transition is already in flight: do NOT override it with another.
  // Reading switching_state_ here distinguishes a benign repeat of the same
  // in-flight target from a conflicting new target, which is rejected. Either
  // way we do not re-arm the request.
  if (waiting_for_state_) {
    result.accepted = false;
    result.error = (target == switching_state_)
      ? "State transition already in progress"
      : "Rejected: a state transition is already in progress";
    return result;
  }
  if (waiting_for_mode_) {
    result.accepted = false;
    result.error = "Rejected: a mode change is in progress";
    return result;
  }

  // Accept and record the in-flight target.
  waiting_for_state_ = true;
  switching_state_ = target;
  result.accepted = true;
  return result;
}

TransitionResult TransitionGuard::request_mode(packml_sm::ModeType target)
{
  std::lock_guard<std::mutex> lk(state_mutex_);   // guard all transition state.
  TransitionResult result;

  // Already in the requested mode — no-op success.
  if (target == current_mode_) {
    result.accepted = true;
    result.already_there = true;
    return result;
  }

  // A transition is already in flight: do NOT override it. switching_mode_
  // distinguishes a benign repeat from a conflicting new target.
  if (waiting_for_mode_) {
    result.accepted = false;
    result.error = (target == switching_mode_)
      ? "Mode change already in progress"
      : "Rejected: a mode change is already in progress";
    return result;
  }
  if (waiting_for_state_) {
    result.accepted = false;
    result.error = "Rejected: a state change is in progress";
    return result;
  }

  // Accept and record the in-flight target.
  waiting_for_mode_ = true;
  switching_mode_ = target;
  result.accepted = true;
  return result;
}

bool TransitionGuard::on_status_update(packml_sm::State state, packml_sm::ModeType mode)
{
  std::lock_guard<std::mutex> lk(state_mutex_);   // guard all transition state.
  bool changed = false;

  // Clear the in-flight flag on ANY observed state change — even one that diverges
  // from the requested target (e.g. the SM faults to ABORTED). This avoids a stuck
  // waiting_ flag, which with the reject-while-pending behavior would otherwise deadlock
  // all future requests.
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
