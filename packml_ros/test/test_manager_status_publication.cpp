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
// Tests for PackmlManagerInterface status publication on /packml_status topic.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <string>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_msgs/msg/status.hpp"
#include "packml_msgs/msg/state.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;
using StateMsg = packml_msgs::msg::State;

class ManagerStatusPublicationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_name_ = packml_ros_test::unique_node_name("mgr_status_test");
    node_ = rclcpp::Node::make_shared(node_name_);
    sm_node_ = std::make_unique<SMNode_new>(node_);

    state_client_ = node_->create_client<packml_msgs::srv::StateChange>(
      node_name_ + "/changeState");

    last_status_state_.store(StateMsg::UNDEFINED);
    status_received_.store(false);

    // Subscribe to packml_status topic (uses SensorDataQoS)
    status_sub_ = node_->create_subscription<packml_msgs::msg::Status>(
      "packml_status", rclcpp::SensorDataQoS(),
      [this](const packml_msgs::msg::Status & msg) {
        last_status_state_.store(msg.state.val);
        last_status_mode_.store(msg.mode.val);
        status_received_.store(true);
      });

    spinner_ = std::make_unique<packml_ros_test::SpinHelper>(node_);

    ASSERT_TRUE(state_client_->wait_for_service(5s));
    // Wait for SM to activate (starts in STOPPED)
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    spinner_.reset();
    status_sub_.reset();
    state_client_.reset();
    sm_node_.reset();
    node_.reset();
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

  bool wait_for_status(int8_t expected_state, std::chrono::milliseconds timeout = 2000ms)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (last_status_state_.load() == expected_state) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SMNode_new> sm_node_;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;
  rclcpp::Subscription<packml_msgs::msg::Status>::SharedPtr status_sub_;
  std::unique_ptr<packml_ros_test::SpinHelper> spinner_;
  std::atomic<int8_t> last_status_state_;
  std::atomic<int8_t> last_status_mode_;
  std::atomic<bool> status_received_;
};

TEST_F(ManagerStatusPublicationTest, StatusPublishedAfterReset)
{
  // Drive SM from STOPPED -> RESET -> IDLE
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET failed: " << resp->message;

  // After RESET, SM goes through RESETTING -> IDLE; status should be published
  EXPECT_TRUE(wait_for_status(StateMsg::IDLE))
    << "Expected IDLE status publication, got state=" << (int)last_status_state_.load();
}

TEST_F(ManagerStatusPublicationTest, StatusPublishedAfterAbort)
{
  // SM starts in STOPPED; ABORT transitions to ABORTING -> ABORTED
  auto resp = send_state(packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "ABORT failed: " << resp->message;

  EXPECT_TRUE(wait_for_status(StateMsg::ABORTED))
    << "Expected ABORTED status publication, got state=" << (int)last_status_state_.load();
}

TEST_F(ManagerStatusPublicationTest, StatusContainsCorrectStateValue)
{
  // Drive to IDLE
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_status(StateMsg::IDLE));

  EXPECT_EQ(last_status_state_.load(), StateMsg::IDLE);
}

TEST_F(ManagerStatusPublicationTest, StatusTopicReceivesMessages)
{
  // Trigger a transition to get a publish
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  // Wait for any status message
  auto start = std::chrono::steady_clock::now();
  while (!status_received_.load() && std::chrono::steady_clock::now() - start < 3s) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(status_received_.load()) << "No status message received on packml_status topic";
}
