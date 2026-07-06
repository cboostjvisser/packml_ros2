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
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/mode_transition.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_msgs/msg/state.hpp"
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

// A state transition on the manager is fanned out to every registered child's
// packml_state_transition service. Expected: after RESET the child receives a state
// value (RESETTING or IDLE). Re-enabled: it was disabled while the fan-out blocked
// the Qt thread and manually spun per-client executors (which conflicted with the
// test's executor); the fan-out is asynchronous now, so the requests are plain
// service calls served by the test's executor.
TEST_F(ManagerClientFanoutTest, StateTransitionFannedOutToChild)
{
  // The activation-time STOPPED fan-out during SetUp may already have delivered a
  // state to the child — discard it, so this test can only pass on the fan-out the
  // RESET below actually triggers.
  state_received_.store(0);

  // SM starts in STOPPED; RESET should trigger on_state_changed, which calls child's state_transition
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "RESET failed: " << resp->message;

  // Wait for child to receive the state transition call
  auto start = std::chrono::steady_clock::now();
  while (state_received_.load() == 0 && std::chrono::steady_clock::now() - start < 3s) {
    std::this_thread::sleep_for(10ms);
  }
  // The child must have received specifically the RESET-driven state, not just anything.
  const auto received = state_received_.load();
  EXPECT_TRUE(received == packml_msgs::msg::State::RESETTING ||
              received == packml_msgs::msg::State::IDLE)
    << "Child never received the RESET-driven state transition (got "
    << static_cast<int>(received) << ")";
}

// A mode change on the manager is fanned out to every registered child's
// packml_mode_transition service. Expected: after ~/changeMode from IDLE, the child's
// mode service receives the requested mode value. (Without this, mode fan-out
// delivery would have no direct coverage at all — the alarm tests only cover its
// failure paths.)
TEST_F(ManagerClientFanoutTest, ModeTransitionFannedOutToChild)
{
  // Drive to IDLE first — mode changes are only valid from IDLE.
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "Setup RESET failed: " << resp->message;
  std::this_thread::sleep_for(500ms);  // let RESETTING -> IDLE settle

  auto mode_client = node_->create_client<packml_msgs::srv::ModeChange>(
    node_name_ + "/" + packml_ros::kChangeModeService);
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  mode_received_.store(0);
  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = 3;
  auto future = mode_client->async_send_request(req);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(future.get()->success);

  auto start = std::chrono::steady_clock::now();
  while (mode_received_.load() != 3 && std::chrono::steady_clock::now() - start < 3s) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(mode_received_.load(), 3) << "Child never received the mode transition";
}

// A registered child whose mode_transition service accepts the request but never
// responds within the fan-out deadline (simulates a hung Equipment Module) must not
// block the manager: the ~/changeMode response arrives promptly with success=true
// (the state machine accepted; acks are asynchronous), and the missing
// acknowledgement is surfaced out-of-band as a WARN Alarm on packml_alarms naming
// the silent child.
TEST_F(ManagerClientFanoutTest, UnresponsiveChildModeChangeDoesNotBlockService)
{
  auto slow_name = packml_ros_test::unique_node_name("mgr_slow_child_test");
  const std::string slow_child_name = "slow_child_module";

  auto slow_node = rclcpp::Node::make_shared(slow_name,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("node_names", std::vector<std::string>{slow_child_name})}));
  auto slow_child_node = rclcpp::Node::make_shared(slow_child_name);

  // Responds immediately to state transitions (needed to drive to IDLE below —
  // mode changes are only valid from IDLE).
  auto slow_state_srv = slow_child_node->create_service<packml_msgs::srv::StateTransition>(
    slow_child_name + "/" + packml_ros::kStateTransitionService,
    [](const std::shared_ptr<packml_msgs::srv::StateTransition::Request>,
       std::shared_ptr<packml_msgs::srv::StateTransition::Response> res) {
      res->success = true;
    });

  // Accepts the mode-change request but does not respond until well past the
  // manager's fan-out deadline — simulates a hung Equipment Module.
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

  // Each node gets its OWN dedicated spinner thread: the child's 6s-sleeping service
  // callback must not stall the manager's executor, which has to keep delivering the
  // deadline timer and any response callbacks for this test to measure the real
  // fan-out behavior rather than executor contention.
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

  // Watch the alarm topic for the fan-out failure report (subscribe before acting;
  // volatile QoS deliberately ignores any earlier retained alarms). Wait for the
  // subscription to match the manager's alarm publisher — matching is asynchronous,
  // and an alarm published before it completes would be lost to a volatile reader.
  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = slow_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(50),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    const auto match_deadline = std::chrono::steady_clock::now() + 2s;
    while (alarm_sub->get_publisher_count() == 0 &&
           std::chrono::steady_clock::now() < match_deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u) << "Alarm subscription never matched";
  }

  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = 1;
  auto future = slow_mode_client->async_send_request(req);

  // Non-blocking: the response must arrive promptly (well under the 5s fan-out
  // deadline) and report the accepted mode change.
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready)
    << "Mode change blocked on the unresponsive child — fan-out is not asynchronous";
  auto resp = future.get();
  EXPECT_TRUE(resp->success)
    << "Mode change accepted by the SM must report success; acks are asynchronous";

  // The missing acknowledgement is reported out-of-band once the 5s fan-out deadline
  // expires. Match the DEADLINE reason specifically ("no response within") so this
  // cannot false-pass on a service-unavailable alarm from a different failure path.
  const auto deadline = std::chrono::steady_clock::now() + 8s;
  bool found_ack_alarm = false;
  while (!found_ack_alarm && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      for (const auto & a : alarms) {
        if (a.node_name == slow_child_name && a.trigger &&
            a.severity == packml_msgs::msg::Alarm::WARN &&
            a.message.find("mode") != std::string::npos &&
            a.message.find("no response within") != std::string::npos)
        {
          found_ack_alarm = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found_ack_alarm)
    << "No WARN alarm reported the child's missing mode-transition acknowledgement";
}

// A registered child that is OFFLINE (no services at all) must not block or fail the
// mode change either: the response reports the accepted change immediately, and the
// unreachable child is flagged with a WARN "service unavailable" Alarm once the
// fan-out deadline expires (the fan-out keeps retrying discovery until then, so a
// merely slow-to-discover child is NOT falsely flagged).
TEST_F(ManagerClientFanoutTest, OfflineChildFlaggedByFanoutAlarm)
{
  auto mgr_name = packml_ros_test::unique_node_name("mgr_offline_child_test");
  const std::string offline_child_name = "offline_child_module";

  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("node_names", std::vector<std::string>{offline_child_name})}));

  auto mgr_sm = std::make_unique<SMNode_new>(mgr_node);
  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/" + packml_ros::kChangeStateService);
  auto mode_client = mgr_node->create_client<packml_msgs::srv::ModeChange>(
    mgr_name + "/" + packml_ros::kChangeModeService);

  packml_ros_test::SpinHelper mgr_spin(mgr_node);
  ASSERT_TRUE(state_client->wait_for_service(5s));
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  // Drive to IDLE (mode changes are only valid from IDLE).
  auto state_req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  state_req->command = packml_msgs::srv::StateChange::Request::RESET;
  auto state_future = state_client->async_send_request(state_req);
  ASSERT_EQ(state_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(state_future.get()->success) << "Setup RESET failed";
  std::this_thread::sleep_for(500ms);

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(50),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    const auto match_deadline = std::chrono::steady_clock::now() + 2s;
    while (alarm_sub->get_publisher_count() == 0 &&
           std::chrono::steady_clock::now() < match_deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u) << "Alarm subscription never matched";
  }

  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = 1;
  auto future = mode_client->async_send_request(req);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready)
    << "Mode change blocked on the offline child";
  EXPECT_TRUE(future.get()->success);

  // The child never appears, so discovery retries exhaust the 5s fan-out deadline
  // and the unavailable-alarm fires then.
  const auto deadline = std::chrono::steady_clock::now() + 8s;
  bool found_unavailable_alarm = false;
  while (!found_unavailable_alarm && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      for (const auto & a : alarms) {
        if (a.node_name == offline_child_name && a.trigger &&
            a.severity == packml_msgs::msg::Alarm::WARN &&
            a.message.find("mode") != std::string::npos &&
            a.message.find("unavailable") != std::string::npos)
        {
          found_unavailable_alarm = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found_unavailable_alarm)
    << "No WARN alarm flagged the offline child's unavailable mode-transition service";
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
