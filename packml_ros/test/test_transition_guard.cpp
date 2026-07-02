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
// Tests for TransitionGuard and PackmlNodeProtocol - the shared protocol logic
// (rclcpp + rclpy via pybind11) for state/mode transition coordination.

#include <gtest/gtest.h>
#include "packml_ros/transition_guard.hpp"
#include "packml_sm/default_modes.hpp"

using packml_ros::TransitionGuard;
using packml_ros::TransitionResult;
using packml_sm::State;
using packml_sm::ModeType;

class TransitionGuardTest : public ::testing::Test
{
protected:
  TransitionGuard guard;
};

// --- Initial state ---

// A freshly constructed guard starts in UNDEFINED state.
TEST_F(TransitionGuardTest, InitialStateIsUndefined)
{
  EXPECT_EQ(guard.current_state(), State::UNDEFINED);
}

// A freshly constructed guard starts in mode 0 (Invalid).
TEST_F(TransitionGuardTest, InitialModeIsZero)
{
  EXPECT_EQ(guard.current_mode(), packml_modes::Invalid);
}

// A fresh guard reports no state or mode transition in flight.
TEST_F(TransitionGuardTest, NotSwitchingInitially)
{
  EXPECT_FALSE(guard.is_switching_state());
  EXPECT_FALSE(guard.is_switching_mode());
}

// --- State transition requests ---

// A first state request (target != current) is accepted, is not a no-op, and carries no warning.
TEST_F(TransitionGuardTest, AcceptStateTransition)
{
  auto result = guard.request_state(State::STOPPED);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.already_there);
  EXPECT_TRUE(result.error.empty());
}

// Requesting the state the guard is already in is a no-op success: accepted + already_there, no warning.
TEST_F(TransitionGuardTest, AlreadyInState)
{
  // Force state to IDLE via status update
  guard.on_status_update(State::IDLE, 0);

  auto result = guard.request_state(State::IDLE);
  EXPECT_TRUE(result.already_there);
  EXPECT_TRUE(result.accepted);  // also "accepted" (no-op success)
  EXPECT_TRUE(result.error.empty());
}

// Accepting a state request marks the guard as switching state.
TEST_F(TransitionGuardTest, SwitchingStateFlag)
{
  guard.request_state(State::STOPPED);
  EXPECT_TRUE(guard.is_switching_state());
}

// A status update to the requested state clears the switching flag and adopts that state.
TEST_F(TransitionGuardTest, SwitchingStateClearedByStatusUpdate)
{
  guard.request_state(State::STOPPED);
  EXPECT_TRUE(guard.is_switching_state());

  guard.on_status_update(State::STOPPED, 0);
  EXPECT_FALSE(guard.is_switching_state());
  EXPECT_EQ(guard.current_state(), State::STOPPED);
}

// While a state transition is in flight, a request for a DIFFERENT state is rejected
// (accepted=false, with a reason) and the original in-flight target is preserved.
TEST_F(TransitionGuardTest, RejectDifferentStateWhileSwitching)
{
  guard.request_state(State::STOPPED);

  // A second, different target must be rejected, not silently override the in-flight STOPPED request.
  auto result = guard.request_state(State::IDLE);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());

  // The original target is preserved: a status update to STOPPED still completes it.
  EXPECT_TRUE(guard.is_switching_state());
  guard.on_status_update(State::STOPPED, packml_modes::Invalid);
  EXPECT_FALSE(guard.is_switching_state());
  EXPECT_EQ(guard.current_state(), State::STOPPED);
}

// --- Mode transition requests ---

// A first mode request (target != current) is accepted, is not a no-op, and carries no warning.
TEST_F(TransitionGuardTest, AcceptModeTransition)
{
  auto result = guard.request_mode(packml_modes::Production);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.already_there);
  EXPECT_TRUE(result.error.empty());
}

// Requesting the mode the guard is already in is a no-op success: accepted + already_there, no warning.
TEST_F(TransitionGuardTest, AlreadyInMode)
{
  guard.on_status_update(State::UNDEFINED, packml_modes::Maintenance);

  auto result = guard.request_mode(packml_modes::Maintenance);
  EXPECT_TRUE(result.already_there);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.error.empty());
}

// Accepting a mode request marks the guard as switching mode.
TEST_F(TransitionGuardTest, SwitchingModeFlag)
{
  guard.request_mode(packml_modes::Production);
  EXPECT_TRUE(guard.is_switching_mode());
}

// A status update to the requested mode clears the switching flag and adopts that mode.
TEST_F(TransitionGuardTest, SwitchingModeClearedByStatusUpdate)
{
  guard.request_mode(packml_modes::Manual);
  EXPECT_TRUE(guard.is_switching_mode());

  guard.on_status_update(State::UNDEFINED, packml_modes::Manual);
  EXPECT_FALSE(guard.is_switching_mode());
  EXPECT_EQ(guard.current_mode(), packml_modes::Manual);
}

// While a mode transition is in flight, a request for a DIFFERENT mode is rejected
// (accepted=false, with a reason) and the original in-flight mode target is preserved.
TEST_F(TransitionGuardTest, RejectDifferentModeWhileSwitching)
{
  guard.request_mode(packml_modes::Production);

  // A second, different mode target must be rejected, not override the in-flight one.
  auto result = guard.request_mode(packml_modes::Maintenance);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());

  // The original mode target is preserved.
  EXPECT_TRUE(guard.is_switching_mode());
  guard.on_status_update(State::UNDEFINED, packml_modes::Production);
  EXPECT_FALSE(guard.is_switching_mode());
  EXPECT_EQ(guard.current_mode(), packml_modes::Production);
}

// --- Cross-interaction: state & mode ---

// At most one transition in flight: a state request is rejected while a mode change
// is still pending, and the guard is NOT armed for the state switch.
TEST_F(TransitionGuardTest, RejectStateWhileModeSwitching)
{
  guard.request_mode(packml_modes::Production);

  // One transition at a time: a state request is rejected while a mode change is in flight.
  auto result = guard.request_state(State::IDLE);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());
  EXPECT_FALSE(guard.is_switching_state());  // not armed
}

// Symmetric to the above: a mode request is rejected while a state change is still pending.
TEST_F(TransitionGuardTest, RejectModeWhileStateSwitching)
{
  guard.request_state(State::STOPPED);

  auto result = guard.request_mode(1);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());
  EXPECT_FALSE(guard.is_switching_mode());  // not armed
}

// --- Status updates ---

// A status update adopts the reported state as the guard's current state.
TEST_F(TransitionGuardTest, StatusUpdateChangesState)
{
  guard.on_status_update(State::EXECUTE, 0);
  EXPECT_EQ(guard.current_state(), State::EXECUTE);
}

// A status update adopts the reported mode as the guard's current mode.
TEST_F(TransitionGuardTest, StatusUpdateChangesMode)
{
  guard.on_status_update(State::UNDEFINED, 5);
  EXPECT_EQ(guard.current_mode(), 5);
}

// A status update can adopt a new state and mode in the same call.
TEST_F(TransitionGuardTest, StatusUpdateChangesBoth)
{
  guard.on_status_update(State::IDLE, packml_modes::Maintenance);
  EXPECT_EQ(guard.current_state(), State::IDLE);
  EXPECT_EQ(guard.current_mode(), packml_modes::Maintenance);
}

// on_status_update returns true when the reported state/mode differs from the current one.
TEST_F(TransitionGuardTest, StatusUpdateReturnsTrueOnChange)
{
  EXPECT_TRUE(guard.on_status_update(State::IDLE, 0));
}

// on_status_update returns false when the reported state/mode is unchanged (idempotent no-op).
TEST_F(TransitionGuardTest, StatusUpdateReturnsFalseOnNoChange)
{
  guard.on_status_update(State::IDLE, 1);
  EXPECT_FALSE(guard.on_status_update(State::IDLE, 1));
}
