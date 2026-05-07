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
// Tests for PackmlManagerInterface mode change service (~/changeMode).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/default_modes.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

class ManagerModeChangeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_name_ = packml_ros_test::unique_node_name("mgr_mode_test");
    node_ = rclcpp::Node::make_shared(node_name_);
    sm_node_ = std::make_unique<SMNode_new>(node_);

    mode_client_ = node_->create_client<packml_msgs::srv::ModeChange>(
      node_name_ + "/changeMode");
    state_client_ = node_->create_client<packml_msgs::srv::StateChange>(
      node_name_ + "/changeState");

    spinner_ = std::make_unique<packml_ros_test::SpinHelper>(node_);

    ASSERT_TRUE(mode_client_->wait_for_service(5s)) << "changeMode service not available";
    ASSERT_TRUE(state_client_->wait_for_service(5s)) << "changeState service not available";

    // Wait for SM to activate (starts in STOPPED)
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    spinner_.reset();
    mode_client_.reset();
    state_client_.reset();
    sm_node_.reset();
    node_.reset();
  }

  packml_msgs::srv::ModeChange::Response::SharedPtr send_mode(int8_t mode_val)
  {
    auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
    req->mode.val = mode_val;
    auto future = mode_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  packml_msgs::srv::StateChange::Response::SharedPtr send_state(int8_t cmd)
  {
    auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
    req->command = cmd;
    auto future = state_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  void drive_to_idle()
  {
    // SM starts in STOPPED → RESET → IDLE
    auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
    ASSERT_NE(resp, nullptr);
    ASSERT_TRUE(resp->success) << "Failed to RESET: " << resp->message;
    std::this_thread::sleep_for(300ms);
  }

  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SMNode_new> sm_node_;
  rclcpp::Client<packml_msgs::srv::ModeChange>::SharedPtr mode_client_;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;
  std::unique_ptr<packml_ros_test::SpinHelper> spinner_;
};

TEST_F(ManagerModeChangeTest, ModeChangeFromIdleSucceeds)
{
  drive_to_idle();

  auto resp = send_mode(packml_modes::Maintenance);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "Mode change failed: " << resp->message;
  EXPECT_EQ(resp->error_code, packml_msgs::srv::ModeChange::Response::SUCCESS);
}

TEST_F(ManagerModeChangeTest, ModeChangeFromInvalidStateReturnsError)
{
  // SM starts in STOPPED; mode change only allowed from IDLE.
  auto resp = send_mode(packml_modes::Maintenance);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_NE(resp->error_code, packml_msgs::srv::ModeChange::Response::SUCCESS);
}

TEST_F(ManagerModeChangeTest, ModeChangeToSameModeSucceeds)
{
  drive_to_idle();

  // Changing to a mode (even same one) from IDLE should succeed
  auto resp = send_mode(packml_modes::Production);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "Same-mode change failed: " << resp->message;
}
