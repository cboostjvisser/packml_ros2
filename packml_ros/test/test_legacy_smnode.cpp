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
// Tests for the legacy SMNode (packml_ros.hpp) transition, mode, and status services.
//
// DESIGN NOTES:
// 1. On this branch, the SM starts in STOPPED (PackML power-on state).
// 2. continuousCycleSM is used — EXECUTE loops until STOP is issued.
// 3. SMNode does NOT call PackmlManagerInterface::init() to avoid duplicate
//    services. It creates its own ~/transition, ~/allStatus, ~/modeChange.
// 4. SMNode destructor properly deactivates the SM.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <string>

#include "packml_ros/packml_ros.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/srv/all_status.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

// =============================================================================
// Shared fixture: single SMNode instance reused across ALL tests.
// The SM is driven back to ABORTED between tests for a clean baseline.
// =============================================================================

class LegacySMNodeTest : public ::testing::Test
{
protected:
  // Shared across all tests - created once, destroyed in TearDownTestSuite
  static rclcpp::Node::SharedPtr node_;
  static std::unique_ptr<SMNode> sm_node_;
  static rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr trans_client_;
  static rclcpp::Client<packml_msgs::srv::AllStatus>::SharedPtr status_client_;
  static rclcpp::Client<packml_msgs::srv::ModeChange>::SharedPtr mode_client_;
  static std::unique_ptr<packml_ros_test::SpinHelper> spinner_;
  static std::string node_name_;
  static bool initialized_;

  static void SetUpTestSuite()
  {
    if (initialized_) return;
    initialized_ = true;

    node_name_ = packml_ros_test::unique_node_name("legacy_test");
    node_ = rclcpp::Node::make_shared(node_name_);
    sm_node_ = std::make_unique<SMNode>(node_);

    trans_client_ = node_->create_client<packml_msgs::srv::StateChange>(
      node_name_ + "/transition");
    status_client_ = node_->create_client<packml_msgs::srv::AllStatus>(
      node_name_ + "/allStatus");
    mode_client_ = node_->create_client<packml_msgs::srv::ModeChange>(
      node_name_ + "/modeChange");

    spinner_ = std::make_unique<packml_ros_test::SpinHelper>(node_);

    // Wait for services
    ASSERT_TRUE(trans_client_->wait_for_service(5s)) << "~/transition not available";
    ASSERT_TRUE(status_client_->wait_for_service(5s)) << "~/allStatus not available";
    ASSERT_TRUE(mode_client_->wait_for_service(5s)) << "~/modeChange not available";

    // SM starts in STOPPED. Wait for it to settle.
    std::this_thread::sleep_for(500ms);
  }

  static void TearDownTestSuite()
  {
    // Release ROS2 and SM resources before rclcpp::shutdown() is called
    // in main(). Failure to do this causes heap corruption in FastDDS
    // during process exit.
    spinner_.reset();
    trans_client_.reset();
    status_client_.reset();
    mode_client_.reset();
    sm_node_.reset();
    node_.reset();
  }

  void SetUp() override
  {
    // Drive SM back to STOPPED for a clean state
    drive_to_stopped();
  }

  void drive_to_stopped()
  {
    // Try STOP first (works from most states).
    // If in ABORTED, STOP fails — use CLEAR instead (ABORTED→CLEARING→STOPPED).
    auto resp = send_command(packml_msgs::srv::StateChange::Request::STOP);
    if (resp && !resp->success) {
      resp = send_command(packml_msgs::srv::StateChange::Request::CLEAR);
    }
    std::this_thread::sleep_for(500ms);
  }

  packml_msgs::srv::StateChange::Response::SharedPtr send_command(int8_t cmd)
  {
    auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
    req->command = cmd;
    auto future = trans_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  packml_msgs::srv::AllStatus::Response::SharedPtr query_status()
  {
    auto req = std::make_shared<packml_msgs::srv::AllStatus::Request>();
    req->command = true;
    auto future = status_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  void drive_to_idle()
  {
    // From STOPPED → RESET → IDLE
    auto resp = send_command(packml_msgs::srv::StateChange::Request::RESET);
    ASSERT_NE(resp, nullptr);
    ASSERT_TRUE(resp->success) << "RESET failed";
    std::this_thread::sleep_for(500ms);  // Allow RESETTING→IDLE to complete
  }
};

// Static member definitions
rclcpp::Node::SharedPtr LegacySMNodeTest::node_;
std::unique_ptr<SMNode> LegacySMNodeTest::sm_node_;
rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr LegacySMNodeTest::trans_client_;
rclcpp::Client<packml_msgs::srv::AllStatus>::SharedPtr LegacySMNodeTest::status_client_;
rclcpp::Client<packml_msgs::srv::ModeChange>::SharedPtr LegacySMNodeTest::mode_client_;
std::unique_ptr<packml_ros_test::SpinHelper> LegacySMNodeTest::spinner_;
std::string LegacySMNodeTest::node_name_;
bool LegacySMNodeTest::initialized_ = false;

// =============================================================================
// Transition service tests
// =============================================================================

TEST_F(LegacySMNodeTest, ResetFromStoppedSucceeds)
{
  // SM is in STOPPED (setup drives it there)
  auto resp = send_command(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(resp->error_code, packml_msgs::srv::StateChange::Response::SUCCESS);
}

TEST_F(LegacySMNodeTest, StartFromIdleSucceeds)
{
  drive_to_idle();
  auto resp = send_command(packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
}

TEST_F(LegacySMNodeTest, StopFromExecuteSucceeds)
{
  drive_to_idle();
  auto resp = send_command(packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  std::this_thread::sleep_for(300ms);  // Let SM reach EXECUTE

  // Continuous cycle: SM stays in EXECUTE. STOP it.
  resp = send_command(packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
}

TEST_F(LegacySMNodeTest, AbortFromStoppedSucceeds)
{
  auto resp = send_command(packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
}

TEST_F(LegacySMNodeTest, StartFromStoppedFails)
{
  // START is invalid from STOPPED (must RESET to IDLE first)
  auto resp = send_command(packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->error_code, packml_msgs::srv::StateChange::Response::INVALID_TRANSITION_REQUEST);
}

TEST_F(LegacySMNodeTest, InvalidCommandReturnsUnrecognized)
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = 99;
  auto future = trans_client_->async_send_request(req);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(resp->error_code, packml_msgs::srv::StateChange::Response::UNRECOGNIZED_REQUEST);
}

// =============================================================================
// Mode change service test
// =============================================================================

TEST_F(LegacySMNodeTest, ModeChangeFromIdleSucceeds)
{
  drive_to_idle();

  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = 2;
  auto future = mode_client_->async_send_request(req);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  auto resp = future.get();
  EXPECT_TRUE(resp->success) << "Mode change failed: " << resp->message;
}

// =============================================================================
// AllStatus service tests (integration with real SM state)
// =============================================================================

TEST_F(LegacySMNodeTest, AllStatusReportsStoppedInitially)
{
  auto resp = query_status();
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->stopped_state) << "Expected STOPPED state initially";
  EXPECT_FALSE(resp->idle_state);
  EXPECT_FALSE(resp->aborted_state);
}

TEST_F(LegacySMNodeTest, AllStatusReportsIdleAfterReset)
{
  drive_to_idle();
  auto resp = query_status();
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->idle_state) << "Expected IDLE after RESET";
  EXPECT_FALSE(resp->stopped_state);
}
