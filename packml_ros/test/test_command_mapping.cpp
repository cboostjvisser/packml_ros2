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
// Tests for packml_ros::to_transition_cmd() mapping function.

#include <gtest/gtest.h>

#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"

using packml_ros::to_transition_cmd;
using Req = packml_msgs::srv::StateChange::Request;

TEST(CommandMapping, ResetMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::RESET), packml_sm::TransitionCmd::RESET);
}

TEST(CommandMapping, StartMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::START), packml_sm::TransitionCmd::START);
}

TEST(CommandMapping, StopMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::STOP), packml_sm::TransitionCmd::STOP);
}

TEST(CommandMapping, HoldMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::HOLD), packml_sm::TransitionCmd::HOLD);
}

TEST(CommandMapping, UnholdMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::UNHOLD), packml_sm::TransitionCmd::UNHOLD);
}

TEST(CommandMapping, SuspendMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::SUSPEND), packml_sm::TransitionCmd::SUSPEND);
}

TEST(CommandMapping, UnsuspendMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::UNSUSPEND), packml_sm::TransitionCmd::UNSUSPEND);
}

TEST(CommandMapping, AbortMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::ABORT), packml_sm::TransitionCmd::ABORT);
}

TEST(CommandMapping, ClearMapsCorrectly)
{
  EXPECT_EQ(to_transition_cmd(Req::CLEAR), packml_sm::TransitionCmd::CLEAR);
}

TEST(CommandMapping, NoCommandMapsToNoCommand)
{
  EXPECT_EQ(to_transition_cmd(Req::NO_COMMAND), packml_sm::TransitionCmd::NO_COMMAND);
}

TEST(CommandMapping, InvalidCommandMapsToNoCommand)
{
  // Any value outside 0-9 should map to NO_COMMAND
  EXPECT_EQ(to_transition_cmd(99), packml_sm::TransitionCmd::NO_COMMAND);
  EXPECT_EQ(to_transition_cmd(-1), packml_sm::TransitionCmd::NO_COMMAND);
  EXPECT_EQ(to_transition_cmd(10), packml_sm::TransitionCmd::NO_COMMAND);
}
