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
#include <fstream>
#include <cstdio>
#include <memory>
#include <thread>
#include <string>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/default_modes.hpp"
#include "packml_ros/ros_names.hpp"
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

// ===========================================================================
// A configured per-mode state mask must survive a RUNTIME mode change.
//
// modes_config_file says which states each mode allows a command into. Maintenance forbids
// HOLDING/HELD, so HOLD must be refused whenever Maintenance is the active mode -- no matter how
// the machine got into it.
//
// Two ways in exist, and they must agree. At boot the manager applies the mask it parsed from the
// file. At runtime ~/changeMode must apply that same mask, which means calling the two-argument
// changeMode(mode, mask). The single-argument changeMode(mode) is not a shorthand for it: that
// overload fabricates an all-states-available mask, so calling it here would overwrite the parsed
// restriction with a fully-open one, permanently -- nothing reloads the file afterwards.
//
// The symptom if that happens: HOLD is correctly refused at boot, and accepted after any
// ~/changeMode call, including one that re-selects Maintenance. Whether a restriction applies
// would depend on how the mode was set rather than on which mode is active.
TEST(ManagerModeMaskPersistence, ConfiguredMaskStillAppliesAfterRuntimeModeChange)
{
  // Maintenance (2) forbids HOLDING/HELD; Production (1) allows everything.
  const std::string yaml_path = "/tmp/packml_ros_test_mask_persistence.yaml";
  {
    std::ofstream f(yaml_path);
    f << "modes:\n"
      << "  Production: 1\n"
      << "  Maintenance: 2\n"
      << "state_masks:\n"
      << "  Maintenance:\n"
      << "    HOLDING: false\n"
      << "    HELD: false\n";
  }

  const auto node_name = packml_ros_test::unique_node_name("mgr_mask_persist");
  auto node = rclcpp::Node::make_shared(node_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter(packml_ros::kParamModesConfigFile, yaml_path),
      rclcpp::Parameter(packml_ros::kParamInitialMode, 1),   // boot in Production
    }));
  auto sm_node = std::make_unique<SMNode_new>(node);
  auto mode_client = node->create_client<packml_msgs::srv::ModeChange>(node_name + "/changeMode");
  auto state_client = node->create_client<packml_msgs::srv::StateChange>(node_name + "/changeState");
  packml_ros_test::SpinHelper spin(node);
  ASSERT_TRUE(mode_client->wait_for_service(5s));
  ASSERT_TRUE(state_client->wait_for_service(5s));
  std::this_thread::sleep_for(500ms);

  auto send_state = [&](int8_t cmd) {
      auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
      req->command = cmd;
      auto fut = state_client->async_send_request(req);
      return fut.wait_for(5s) == std::future_status::ready ? fut.get() : nullptr;
    };

  // Reach IDLE, the only state a mode change is permitted from.
  ASSERT_NE(send_state(packml_msgs::srv::StateChange::Request::STOP), nullptr);
  std::this_thread::sleep_for(300ms);
  auto reset_resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success) << "RESET rejected: " << reset_resp->message;
  std::this_thread::sleep_for(400ms);

  // Runtime switch into Maintenance, whose configured mask forbids HOLDING/HELD.
  auto mode_req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  mode_req->mode.val = 2;
  auto mode_fut = mode_client->async_send_request(mode_req);
  ASSERT_EQ(mode_fut.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(mode_fut.get()->success) << "runtime changeMode(Maintenance) rejected";
  std::this_thread::sleep_for(400ms);

  // Drive to EXECUTE, where HOLD is the meaningful command.
  auto start_resp = send_state(packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(start_resp, nullptr);
  ASSERT_TRUE(start_resp->success) << "START rejected: " << start_resp->message;
  std::this_thread::sleep_for(500ms);

  // THE ASSERTION: HOLD must be refused, because Maintenance's configured mask makes
  // HOLDING unavailable. If the mask was wiped by the mode change, HOLD is accepted.
  auto hold_resp = send_state(packml_msgs::srv::StateChange::Request::HOLD);
  ASSERT_NE(hold_resp, nullptr) << "manager did not answer HOLD";
  EXPECT_FALSE(hold_resp->success)
    << "HOLD was ACCEPTED in Maintenance, whose modes_config_file mask marks HOLDING "
       "unavailable -- the configured mask was replaced by an all-open one during the runtime "
       "mode change (on_change_mode must use mode_masks_, not changeMode()'s single-argument "
       "overload)";

  std::remove(yaml_path.c_str());
}
