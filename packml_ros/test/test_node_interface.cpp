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
// Tests for PackmlNodeInterface (equipment module node interface).
// Verifies that the child-node services respond correctly to state/mode
// transitions sent by the manager.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <string>

#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/srv/state_transition.hpp"
#include "packml_msgs/srv/mode_transition.hpp"
#include "packml_msgs/msg/status.hpp"
#include "packml_msgs/msg/state.hpp"
#include "packml_sm/default_modes.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

/// Concrete implementation of PackmlNodeInterface for testing
class TestPackmlNode : public PackmlNodeInterface
{
public:
  explicit TestPackmlNode(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  // Track what callbacks were invoked
  std::atomic<int> state_trans_req_count{0};
  std::atomic<int> mode_trans_req_count{0};
  std::atomic<int> status_changed_count{0};
  std::atomic<int8_t> last_requested_state{0};
  std::atomic<int8_t> last_requested_mode{0};

  // Control whether requests are approved
  std::atomic<bool> approve_state{true};
  std::atomic<bool> approve_mode{true};

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    last_requested_state.store(static_cast<int8_t>(state));
    state_trans_req_count.fetch_add(1);
    return approve_state.load();
  }

  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    last_requested_mode.store(static_cast<int8_t>(mode));
    mode_trans_req_count.fetch_add(1);
    return approve_mode.load();
  }

  void on_status_changed() override
  {
    status_changed_count.fetch_add(1);
  }
};

class NodeInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_name_ = packml_ros_test::unique_node_name("node_iface_test");
    node_ = rclcpp::Node::make_shared(node_name_);
    test_node_ = std::make_unique<TestPackmlNode>(node_);

    // Client for calling the node's services
    state_client_ = node_->create_client<packml_msgs::srv::StateTransition>(
      node_name_ + "/packml_state_transition");
    mode_client_ = node_->create_client<packml_msgs::srv::ModeTransition>(
      node_name_ + "/packml_mode_transition");

    spinner_ = std::make_unique<packml_ros_test::SpinHelper>(node_);

    ASSERT_TRUE(state_client_->wait_for_service(5s));
    ASSERT_TRUE(mode_client_->wait_for_service(5s));
  }

  void TearDown() override
  {
    spinner_.reset();
    state_client_.reset();
    mode_client_.reset();
    test_node_.reset();
    node_.reset();
  }

  packml_msgs::srv::StateTransition::Response::SharedPtr send_state(int8_t state_val)
  {
    auto req = std::make_shared<packml_msgs::srv::StateTransition::Request>();
    req->state.val = state_val;
    auto future = state_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  packml_msgs::srv::ModeTransition::Response::SharedPtr send_mode(int8_t mode_val)
  {
    auto req = std::make_shared<packml_msgs::srv::ModeTransition::Request>();
    req->mode.val = mode_val;
    auto future = mode_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<TestPackmlNode> test_node_;
  rclcpp::Client<packml_msgs::srv::StateTransition>::SharedPtr state_client_;
  rclcpp::Client<packml_msgs::srv::ModeTransition>::SharedPtr mode_client_;
  std::unique_ptr<packml_ros_test::SpinHelper> spinner_;
};

// When the subclass approves a state transition request, the service reports success
// and the callback receives the requested state exactly once.
TEST_F(NodeInterfaceTest, StateTransitionApprovedReturnsSuccess)
{
  test_node_->approve_state.store(true);
  auto resp = send_state(packml_msgs::msg::State::IDLE);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(test_node_->state_trans_req_count.load(), 1);
  EXPECT_EQ(test_node_->last_requested_state.load(), packml_msgs::msg::State::IDLE);
}

// When the subclass rejects a state transition request, the service reports failure,
// but the callback is still invoked exactly once (rejection is a business decision,
// not a dispatch failure).
TEST_F(NodeInterfaceTest, StateTransitionRejectedReturnsFailure)
{
  test_node_->approve_state.store(false);
  auto resp = send_state(packml_msgs::msg::State::EXECUTE);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(test_node_->state_trans_req_count.load(), 1);
}

// When the subclass approves a mode transition request, the service reports success
// and the callback receives the requested mode exactly once.
TEST_F(NodeInterfaceTest, ModeTransitionApprovedReturnsSuccess)
{
  test_node_->approve_mode.store(true);
  auto resp = send_mode(packml_modes::Maintenance);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(test_node_->mode_trans_req_count.load(), 1);
  EXPECT_EQ(test_node_->last_requested_mode.load(), packml_modes::Maintenance);
}

// When the subclass rejects a mode transition request, the service reports failure,
// and the callback is still invoked exactly once.
TEST_F(NodeInterfaceTest, ModeTransitionRejectedReturnsFailure)
{
  test_node_->approve_mode.store(false);
  auto resp = send_mode(packml_modes::Manual);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_EQ(test_node_->mode_trans_req_count.load(), 1);
}

// Requesting a state the node already reports (via the packml_status subscription,
// not a prior transition request) hits the "already there" shortcut: the service
// returns success immediately WITHOUT invoking on_state_trans_req() again.
TEST_F(NodeInterfaceTest, SameStateTransitionReturnsSuccessImmediately)
{
  // Establish current_state = IDLE via a real status publication (matches the
  // manager's latched QoS), exactly as it happens in production.
  auto status_pub = node_->create_publisher<packml_msgs::msg::Status>(
    "packml_status", rclcpp::QoS(1).transient_local().reliable());
  std::this_thread::sleep_for(100ms);
  packml_msgs::msg::Status status_msg;
  status_msg.state.val = packml_msgs::msg::State::IDLE;
  status_pub->publish(status_msg);
  std::this_thread::sleep_for(200ms);

  test_node_->approve_state.store(true);
  auto resp = send_state(packml_msgs::msg::State::IDLE);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
  EXPECT_EQ(test_node_->state_trans_req_count.load(), 0)
    << "already-in-state shortcut should bypass on_state_trans_req()";
}

TEST_F(NodeInterfaceTest, StatusSubscriptionTriggersCallback)
{
  // Publish a Status message to packml_status that the node is subscribed to.
  // Must match the node's status subscription QoS (the manager publishes the status
  // topic latched: TRANSIENT_LOCAL + RELIABLE); a best-effort publisher would be
  // QoS-incompatible with the reliable subscription and never deliver.
  auto publisher = node_->create_publisher<packml_msgs::msg::Status>(
    "packml_status", rclcpp::QoS(1).transient_local().reliable());

  packml_msgs::msg::Status status_msg;
  status_msg.state.val = packml_msgs::msg::State::EXECUTE;
  status_msg.mode.val = 1;

  // Give subscription time to connect
  std::this_thread::sleep_for(100ms);

  publisher->publish(status_msg);
  std::this_thread::sleep_for(200ms);

  EXPECT_GE(test_node_->status_changed_count.load(), 1);
}
