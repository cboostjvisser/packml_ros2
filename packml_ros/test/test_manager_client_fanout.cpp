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
// Tests for PackmlManagerInterface client fanout behavior.
// Verifies that state and mode transitions are forwarded to registered PackML child nodes.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/srv/state_transition.hpp"
#include "packml_msgs/srv/mode_transition.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

class ManagerClientFanoutTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_name_ = packml_ros_test::unique_node_name("mgr_fanout_test");
    child_name_ = "test_child_module";

    // Create manager node WITH node_names parameter (registers a child client)
    node_ = rclcpp::Node::make_shared(node_name_,
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("node_names", std::vector<std::string>{child_name_})}));

    // Create child node that provides the services the manager will call
    child_node_ = rclcpp::Node::make_shared(child_name_);
    state_received_.store(0);
    mode_received_.store(0);

    // Child provides ~/packml_state_transition service
    child_state_srv_ = child_node_->create_service<packml_msgs::srv::StateTransition>(
      child_name_ + "/" + packml_ros::kStateTransitionService,
      [this](const std::shared_ptr<packml_msgs::srv::StateTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::StateTransition::Response> res) {
        state_received_.store(req->state.val);
        res->success = true;
      });

    // Child provides ~/packml_mode_transition service
    child_mode_srv_ = child_node_->create_service<packml_msgs::srv::ModeTransition>(
      child_name_ + "/" + packml_ros::kModeTransitionService,
      [this](const std::shared_ptr<packml_msgs::srv::ModeTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::ModeTransition::Response> res) {
        mode_received_.store(req->mode.val);
        res->success = true;
      });

    // Create the manager (will register client for child_name_)
    sm_node_ = std::make_unique<SMNode_new>(node_);

    // Manager's client for sending commands
    state_client_ = node_->create_client<packml_msgs::srv::StateChange>(
      node_name_ + "/" + packml_ros::kChangeStateService);

    // Spin both nodes
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    exec_->add_node(node_);
    exec_->add_node(child_node_);
    spin_thread_ = std::thread([this]() {
        while (running_.load()) {
          exec_->spin_some(5ms);
          std::this_thread::sleep_for(1ms);
        }
      });

    ASSERT_TRUE(state_client_->wait_for_service(5s));
    // SM starts in STOPPED
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    running_.store(false);
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    exec_.reset();
    state_client_.reset();
    child_state_srv_.reset();
    child_mode_srv_.reset();
    sm_node_.reset();
    child_node_.reset();
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

  std::string node_name_;
  std::string child_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr child_node_;
  std::unique_ptr<SMNode_new> sm_node_;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;
  rclcpp::Service<packml_msgs::srv::StateTransition>::SharedPtr child_state_srv_;
  rclcpp::Service<packml_msgs::srv::ModeTransition>::SharedPtr child_mode_srv_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{true};
  std::atomic<int8_t> state_received_;
  std::atomic<int8_t> mode_received_;
};

TEST_F(ManagerClientFanoutTest, DISABLED_StateTransitionFannedOutToChild)
{
  // NOTE: This test is disabled due to Qt thread + ROS executor interaction.
  // The on_state_changed callback runs on the Qt thread and calls
  // wait_all_futures(), which requires spinning the client's callback group.
  // This conflicts with the test's executor threading model.
  // This should be tested via launch_testing integration test instead.

  // SM starts in STOPPED; RESET should trigger on_state_changed, which calls child's state_transition
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "RESET failed: " << resp->message;

  // Wait for child to receive the state transition call
  auto start = std::chrono::steady_clock::now();
  while (state_received_.load() == 0 && std::chrono::steady_clock::now() - start < 3s) {
    std::this_thread::sleep_for(10ms);
  }
  // Child should have received a state value (RESETTING or IDLE)
  EXPECT_NE(state_received_.load(), 0) << "Child never received state transition";
}

// F19 regression: on_change_mode must not hang forever if a registered child's
// mode_transition service accepts the request but never responds within the wait
// window (simulates a hung Equipment Module). wait_all_futures() has a 5s bounded
// wait; expected: the manager returns success=false within that bound instead of
// blocking indefinitely (which would also starve the health-timeout timer).
TEST_F(ManagerClientFanoutTest, UnresponsiveChildModeChangeFailsWithinTimeout)
{
  auto slow_name = packml_ros_test::unique_node_name("mgr_slow_child_test");
  const std::string slow_child_name = "slow_child_module";

  auto slow_node = rclcpp::Node::make_shared(slow_name,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("node_names", std::vector<std::string>{slow_child_name})}));
  auto slow_child_node = rclcpp::Node::make_shared(slow_child_name);

  // Responds immediately to state transitions (needed to drive to IDLE below —
  // mode changes are only valid from IDLE — and to keep the RESET's own Qt-thread
  // fanout from adding unrelated delay to this test).
  auto slow_state_srv = slow_child_node->create_service<packml_msgs::srv::StateTransition>(
    slow_child_name + "/" + packml_ros::kStateTransitionService,
    [](const std::shared_ptr<packml_msgs::srv::StateTransition::Request>,
       std::shared_ptr<packml_msgs::srv::StateTransition::Response> res) {
      res->success = true;
    });

  // Accepts the mode-change request (so it IS in wait_all_futures' futures map) but
  // never returns within the manager's wait window — simulates a hung Equipment Module.
  auto slow_mode_srv = slow_child_node->create_service<packml_msgs::srv::ModeTransition>(
    slow_child_name + "/" + packml_ros::kModeTransitionService,
    [](const std::shared_ptr<packml_msgs::srv::ModeTransition::Request>,
       std::shared_ptr<packml_msgs::srv::ModeTransition::Response> res) {
      std::this_thread::sleep_for(6s);
      res->success = true;
    });

  auto slow_sm = std::make_unique<SMNode_new>(slow_node);
  auto slow_state_client = slow_node->create_client<packml_msgs::srv::StateChange>(
    slow_name + "/" + packml_ros::kChangeStateService);
  auto slow_mode_client = slow_node->create_client<packml_msgs::srv::ModeChange>(
    slow_name + "/" + packml_ros::kChangeModeService);

  // Each node gets its OWN dedicated spinner thread (like every other multi-node test
  // in this suite) — NOT the fixture's shared exec_. on_change_mode blocks its calling
  // thread for up to 5s inside wait_all_futures; sharing one single-threaded exec_
  // between the manager and the child would starve the child's own callback from ever
  // running concurrently (the same class of issue that disabled the state-fanout test
  // above), making this test measure executor contention instead of the real timeout.
  packml_ros_test::SpinHelper slow_spin(slow_node);
  packml_ros_test::SpinHelper slow_child_spin(slow_child_node);

  ASSERT_TRUE(slow_state_client->wait_for_service(5s));
  ASSERT_TRUE(slow_mode_client->wait_for_service(5s));

  // Mode changes are only valid from IDLE — drive there first (SM starts in STOPPED).
  auto state_req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  state_req->command = packml_msgs::srv::StateChange::Request::RESET;
  auto state_future = slow_state_client->async_send_request(state_req);
  ASSERT_EQ(state_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(state_future.get()->success) << "Setup RESET failed";
  std::this_thread::sleep_for(500ms);  // let RESETTING -> IDLE settle

  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = 1;
  auto future = slow_mode_client->async_send_request(req);

  // The manager's own bound is 5s; allow scheduling headroom above that.
  ASSERT_EQ(future.wait_for(8s), std::future_status::ready)
    << "Mode change request hung — the 5s bounded wait in wait_all_futures did not fire";
  auto resp = future.get();
  EXPECT_FALSE(resp->success) << "Expected failure: child never acknowledged in time";
}

TEST_F(ManagerClientFanoutTest, NoClientsRegisteredStillSucceeds)
{
  // Create a manager with no clients registered
  auto lonely_name = packml_ros_test::unique_node_name("mgr_lonely");
  auto lonely_node = rclcpp::Node::make_shared(lonely_name);
  exec_->add_node(lonely_node);

  SMNode_new lonely_sm(lonely_node);

  auto lonely_client = lonely_node->create_client<packml_msgs::srv::StateChange>(
    lonely_name + "/" + packml_ros::kChangeStateService);
  ASSERT_TRUE(lonely_client->wait_for_service(5s));

  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = packml_msgs::srv::StateChange::Request::RESET;
  auto future = lonely_client->async_send_request(req);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  auto resp = future.get();
  EXPECT_TRUE(resp->success) << "RESET without clients should succeed: " << resp->message;

  exec_->remove_node(lonely_node);
}
