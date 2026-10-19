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
// Tests for PackmlManagerInterface state transition service (~/changeState).
// These verify the ROS service mechanism correctly dispatches to the SM and
// returns proper response codes. Exhaustive transition coverage is in packml_sm.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;
using Req = packml_msgs::srv::StateChange::Request;
using Resp = packml_msgs::srv::StateChange::Response;

class ManagerStateTransitionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_name_ = packml_ros_test::unique_node_name("mgr_state_test");
    node_ = rclcpp::Node::make_shared(node_name_);
    sm_node_ = std::make_unique<SMNode_new>(node_);

    client_ = node_->create_client<packml_msgs::srv::StateChange>(
      node_name_ + "/changeState");

    spinner_ = std::make_unique<packml_ros_test::SpinHelper>(node_);

    ASSERT_TRUE(client_->wait_for_service(5s)) << "changeState service not available";
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    spinner_.reset();
    client_.reset();
    sm_node_.reset();
    node_.reset();
  }

  Resp::SharedPtr send_command(int8_t cmd)
  {
    auto req = std::make_shared<Req>();
    req->command = cmd;
    auto future = client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SMNode_new> sm_node_;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client_;
  std::unique_ptr<packml_ros_test::SpinHelper> spinner_;
};

// Verify a valid transition returns SUCCESS response
TEST_F(ManagerStateTransitionTest, ValidTransitionReturnsSuccess)
{
  // SM starts in STOPPED; RESET is valid from STOPPED
  auto resp = send_command(Req::RESET);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(resp->error_code, Resp::SUCCESS);
}

// Verify an invalid transition returns INVALID_TRANSITION_REQUEST
TEST_F(ManagerStateTransitionTest, InvalidTransitionReturnsError)
{
  // SM starts in STOPPED; START is invalid from STOPPED (need IDLE first)
  auto resp = send_command(Req::START);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->error_code, Resp::INVALID_TRANSITION_REQUEST);
}

// Verify an unrecognized command returns UNRECOGNIZED_REQUEST
TEST_F(ManagerStateTransitionTest, UnrecognizedCommandReturnsError)
{
  auto resp = send_command(99);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->error_code, Resp::UNRECOGNIZED_REQUEST);
}

// Verify multi-step transitions work end-to-end via ROS
TEST_F(ManagerStateTransitionTest, MultiStepTransitionWorks)
{
  // SM starts in STOPPED → RESET → IDLE → START → EXECUTE
  auto resp = send_command(Req::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET failed: " << resp->message;
  std::this_thread::sleep_for(500ms);

  resp = send_command(Req::START);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "START failed: " << resp->message;
  std::this_thread::sleep_for(500ms);

  // Continuous cycle: SM stays in EXECUTE. STOP it.
  resp = send_command(Req::STOP);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "STOP failed: " << resp->message;
}
