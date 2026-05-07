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
// Tests for TransitionGuard - the ROS-independent client protocol logic
// that validates state/mode transition requests from the manager.

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

TEST_F(TransitionGuardTest, InitialStateIsUndefined)
{
  EXPECT_EQ(guard.current_state(), State::UNDEFINED);
}

TEST_F(TransitionGuardTest, InitialModeIsZero)
{
  EXPECT_EQ(guard.current_mode(), packml_modes::Invalid);
}

TEST_F(TransitionGuardTest, NotSwitchingInitially)
{
  EXPECT_FALSE(guard.is_switching_state());
  EXPECT_FALSE(guard.is_switching_mode());
}

// --- State transition requests ---

TEST_F(TransitionGuardTest, AcceptStateTransition)
{
  auto result = guard.request_state(State::STOPPED);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.already_there);
  EXPECT_TRUE(result.error.empty());
}

TEST_F(TransitionGuardTest, AlreadyInState)
{
  // Force state to IDLE via status update
  guard.on_status_update(State::IDLE, 0);

  auto result = guard.request_state(State::IDLE);
  EXPECT_TRUE(result.already_there);
  EXPECT_TRUE(result.accepted);  // also "accepted" (no-op success)
  EXPECT_TRUE(result.error.empty());
}

TEST_F(TransitionGuardTest, SwitchingStateFlag)
{
  guard.request_state(State::STOPPED);
  EXPECT_TRUE(guard.is_switching_state());
}

TEST_F(TransitionGuardTest, SwitchingStateClearedByStatusUpdate)
{
  guard.request_state(State::STOPPED);
  EXPECT_TRUE(guard.is_switching_state());

  guard.on_status_update(State::STOPPED, 0);
  EXPECT_FALSE(guard.is_switching_state());
  EXPECT_EQ(guard.current_state(), State::STOPPED);
}

TEST_F(TransitionGuardTest, WarnWhenAlreadySwitchingState)
{
  guard.request_state(State::STOPPED);

  auto result = guard.request_state(State::IDLE);
  // Should still accept (matches original behavior with TODO)
  // but error string should contain a warning
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.error.empty());
}

// --- Mode transition requests ---

TEST_F(TransitionGuardTest, AcceptModeTransition)
{
  auto result = guard.request_mode(packml_modes::Production);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.already_there);
  EXPECT_TRUE(result.error.empty());
}

TEST_F(TransitionGuardTest, AlreadyInMode)
{
  guard.on_status_update(State::UNDEFINED, packml_modes::Maintenance);

  auto result = guard.request_mode(packml_modes::Maintenance);
  EXPECT_TRUE(result.already_there);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.error.empty());
}

TEST_F(TransitionGuardTest, SwitchingModeFlag)
{
  guard.request_mode(packml_modes::Production);
  EXPECT_TRUE(guard.is_switching_mode());
}

TEST_F(TransitionGuardTest, SwitchingModeClearedByStatusUpdate)
{
  guard.request_mode(packml_modes::Manual);
  EXPECT_TRUE(guard.is_switching_mode());

  guard.on_status_update(State::UNDEFINED, packml_modes::Manual);
  EXPECT_FALSE(guard.is_switching_mode());
  EXPECT_EQ(guard.current_mode(), packml_modes::Manual);
}

TEST_F(TransitionGuardTest, WarnWhenAlreadySwitchingMode)
{
  guard.request_mode(packml_modes::Production);

  auto result = guard.request_mode(packml_modes::Maintenance);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.error.empty());
}

// --- Cross-interaction: state & mode ---

TEST_F(TransitionGuardTest, WarnStateWhileModeSwitching)
{
  guard.request_mode(packml_modes::Production);

  auto result = guard.request_state(State::IDLE);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.error.empty());
}

TEST_F(TransitionGuardTest, WarnModeWhileStateSwitching)
{
  guard.request_state(State::STOPPED);

  auto result = guard.request_mode(1);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.error.empty());
}

// --- Status updates ---

TEST_F(TransitionGuardTest, StatusUpdateChangesState)
{
  guard.on_status_update(State::EXECUTE, 0);
  EXPECT_EQ(guard.current_state(), State::EXECUTE);
}

TEST_F(TransitionGuardTest, StatusUpdateChangesMode)
{
  guard.on_status_update(State::UNDEFINED, 5);
  EXPECT_EQ(guard.current_mode(), 5);
}

TEST_F(TransitionGuardTest, StatusUpdateChangesBoth)
{
  guard.on_status_update(State::IDLE, packml_modes::Maintenance);
  EXPECT_EQ(guard.current_state(), State::IDLE);
  EXPECT_EQ(guard.current_mode(), packml_modes::Maintenance);
}

TEST_F(TransitionGuardTest, StatusUpdateReturnsTrueOnChange)
{
  EXPECT_TRUE(guard.on_status_update(State::IDLE, 0));
}

TEST_F(TransitionGuardTest, StatusUpdateReturnsFalseOnNoChange)
{
  guard.on_status_update(State::IDLE, 1);
  EXPECT_FALSE(guard.on_status_update(State::IDLE, 1));
}
