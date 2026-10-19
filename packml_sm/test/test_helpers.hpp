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

#ifndef PACKML_SM__TEST__TEST_HELPERS_HPP_
#define PACKML_SM__TEST__TEST_HELPERS_HPP_

#include <chrono>
#include <memory>
#include <thread>

#include "packml_sm/common.hpp"
#include "packml_sm/logging.hpp"
#include "packml_sm/state_machine.hpp"

namespace packml_sm_test
{

// Default timeout for state-change waits.  PackML acting states use
// QtConcurrent::run which schedules on a separate thread, so polling at the
// granularity below leaves margin for the slowest "default delay" Acting
// states (Holding/Unholding default to 200 ms in the constructor).
constexpr auto kDefaultStateWaitTimeout = std::chrono::seconds(3);
constexpr auto kPollInterval            = std::chrono::milliseconds(5);

// Delay (ms) for acting states in test SMs.  Low to keep the suite fast.
// Must be >= 1 so QtConcurrent::run actually yields and state transitions
// complete in the expected asynchronous order.
constexpr int kTestDelayMs = 1;

// Block until the state machine reports the requested state (or timeout
// expires).  Returns true on success, false on timeout.  Does not depend on
// rclcpp -- pure std::chrono / std::this_thread.
inline bool wait_for_state(
  packml_sm::StateMachineInterface & sm,
  packml_sm::State target,
  std::chrono::milliseconds timeout = kDefaultStateWaitTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (sm.getCurrentState() == target) {
      PACKML_INFO_STREAM("packml_sm_test", "State reached: " << target);
      return true;
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  PACKML_WARN_STREAM(
    "packml_sm_test",
    "Timeout waiting for state " << target << "; current state: " << sm.getCurrentState());
  return false;
}

// Convenience: returns 0 (success) after a brief sleep so the SM stays in
// EXECUTE long enough for tests to observe transitions out of it.
inline int execute_success_long()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return 0;
}

// Convenience: returns 0 (success) immediately.
inline int execute_success_fast()
{
  return 0;
}

// Convenience: returns non-zero (error) so acting state posts ErrorEvent.
inline int execute_fail()
{
  return -1;
}

// Drive a freshly-activated SM (which boots in STOPPED per PackML) into
// IDLE so per-state tests can start from a known well-defined state.  Also
// installs a fully-open `AvailableStates` mask so every standard PackML
// transition is permitted.
inline bool enable_all_transitions(packml_sm::StateMachine & sm)
{
  packml_sm::AvailableStates open{
    {packml_sm::State::ABORTING, true}, {packml_sm::State::ABORTED, true},
    {packml_sm::State::CLEARING, true}, {packml_sm::State::STOPPING, true},
    {packml_sm::State::STOPPED, true},  {packml_sm::State::RESETTING, true},
    {packml_sm::State::IDLE, true},     {packml_sm::State::STARTING, true},
    {packml_sm::State::EXECUTE, true},  {packml_sm::State::HOLDING, true},
    {packml_sm::State::HELD, true},     {packml_sm::State::UNHOLDING, true},
    {packml_sm::State::SUSPENDING, true}, {packml_sm::State::SUSPENDED, true},
    {packml_sm::State::UNSUSPENDING, true}, {packml_sm::State::COMPLETING, true},
    {packml_sm::State::COMPLETE, true},
  };
  // Use a non-canonical ModeType (1) so this helper does not collide with
  // tests that exercise mode-specific behaviour.
  return sm.changeMode(1, open).has_value();
}

inline bool drive_to_idle(packml_sm::StateMachine & sm)
{
  if (!wait_for_state(sm, packml_sm::State::STOPPED)) {
    return false;
  }
  if (!enable_all_transitions(sm)) {
    return false;
  }
  if (!sm.reset()) {
    return false;
  }
  return wait_for_state(sm, packml_sm::State::IDLE);
}

// Installs a mode and its mask from STOPPED, which permits runtime changes to
// any target mode, and then resets to IDLE so tests can exercise the mask from
// a running state.
inline bool install_mode_then_idle(
  packml_sm::StateMachine & sm, packml_sm::ModeType mode,
  packml_sm::AvailableStates mask)
{
  if (!wait_for_state(sm, packml_sm::State::STOPPED)) {
    return false;
  }
  if (!sm.changeMode(mode, mask).has_value()) {
    return false;
  }
  if (!sm.reset()) {
    return false;
  }
  return wait_for_state(sm, packml_sm::State::IDLE);
}

}  // namespace packml_sm_test

#endif  // PACKML_SM__TEST__TEST_HELPERS_HPP_
