// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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
// The state side is a ~/packml_state_transition ACTION, so the "child" here is a real
// PackmlNodeInterface subclass exercising the actual action server rather than a hand-rolled
// service, exactly the way any real Equipment Module receives it.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/action/state_transition.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/mode_transition.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_msgs/msg/state.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

/// Minimal Equipment Module used as the manager's fan-out target in these tests. Approves
/// every state transition instantly (defers_completion() stays false — the default), records
/// the last state it was asked to transition to, and can be configured to delay its
/// mode-transition response (still a plain service, unaffected by this design) to simulate an
/// unresponsive child.
class ChildEquipmentModule : public PackmlNodeInterface
{
public:
  explicit ChildEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  std::atomic<int8_t> last_state_received{0};
  std::atomic<int32_t> last_mode_received{-1};
  std::chrono::milliseconds mode_response_delay{0};

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    last_state_received.store(static_cast<int8_t>(state));
    return true;
  }

  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    if (mode_response_delay.count() > 0) {
      std::this_thread::sleep_for(mode_response_delay);
    }
    last_mode_received.store(mode);
    return true;
  }

  void on_status_changed() override {}
};

/// A child that serves ONLY the state-transition action (auto-accepting and
/// immediately succeeding every goal, like a non-deferring Equipment Module) and has
/// NO mode-transition service at all. Used to test the mode fan-out's "service
/// unavailable" alarm path without ALSO blocking state-transition completion
/// tracking. Every registered node participates in coordinated-state completion,
/// so this class accepts and completes state goals while omitting the mode service.
class StateOnlyChild
{
public:
  using StateTransitionAction = packml_msgs::action::StateTransition;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StateTransitionAction>;

  explicit StateOnlyChild(rclcpp::Node::SharedPtr node)
  {
    auto handle_goal = [](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const StateTransitionAction::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };
    auto handle_cancel = [](const std::shared_ptr<GoalHandle> &) {
        return rclcpp_action::CancelResponse::ACCEPT;
      };
    auto handle_accepted = [](const std::shared_ptr<GoalHandle> & goal_handle) {
        auto result = std::make_shared<StateTransitionAction::Result>();
        result->success = true;
        result->error_code = StateTransitionAction::Result::SUCCESS;
        goal_handle->succeed(result);
      };
    server_ = rclcpp_action::create_server<StateTransitionAction>(
      node, "~/" + std::string(packml_ros::kStateTransitionAction),
      handle_goal, handle_cancel, handle_accepted);
  }

private:
  rclcpp_action::Server<StateTransitionAction>::SharedPtr server_;
};

}  // namespace

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

    // Create child node/EM that serves both the state-transition action and the
    // mode-transition service the manager will call.
    child_node_ = rclcpp::Node::make_shared(child_name_);
    child_em_ = std::make_shared<ChildEquipmentModule>(child_node_);

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
    child_em_.reset();
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
  std::shared_ptr<ChildEquipmentModule> child_em_;
  std::unique_ptr<SMNode_new> sm_node_;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::atomic<bool> running_{true};
};

// A state transition on the manager is fanned out to every registered child's
// ~/packml_state_transition action. Expected: after RESET the child receives a state
// value (RESETTING or IDLE).
TEST_F(ManagerClientFanoutTest, StateTransitionFannedOutToChild)
{
  // The activation-time STOPPED fan-out during SetUp may already have delivered a
  // state to the child — discard it, so this test can only pass on the fan-out the
  // RESET below actually triggers.
  child_em_->last_state_received.store(0);

  // SM starts in STOPPED; RESET should trigger on_state_changed, which sends the child a
  // state-transition goal.
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "RESET failed: " << resp->message;

  // Wait for child to receive the state transition goal
  auto start = std::chrono::steady_clock::now();
  while (child_em_->last_state_received.load() == 0 &&
    std::chrono::steady_clock::now() - start < 3s)
  {
    std::this_thread::sleep_for(10ms);
  }
  // The child must have received specifically the RESET-driven state, not just anything.
  const auto received = child_em_->last_state_received.load();
  EXPECT_TRUE(received == packml_msgs::msg::State::RESETTING ||
              received == packml_msgs::msg::State::IDLE)
    << "Child never received the RESET-driven state transition (got "
    << static_cast<int>(received) << ")";
}

// A mode change on the manager is fanned out to every registered child's
// packml_mode_transition service. The child receives the requested mode value.
TEST_F(ManagerClientFanoutTest, ModeTransitionFannedOutToChild)
{
  // The state machine starts in STOPPED, which permits runtime mode changes.
  auto mode_client = node_->create_client<packml_msgs::srv::ModeChange>(
    node_name_ + "/" + packml_ros::kChangeModeService);
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = 3;
  auto future = mode_client->async_send_request(req);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(future.get()->success);

  auto start = std::chrono::steady_clock::now();
  while (child_em_->last_mode_received.load() != 3 &&
    std::chrono::steady_clock::now() - start < 3s)
  {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(child_em_->last_mode_received.load(), 3) << "Child never received the mode transition";
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

  // State-transition goals complete immediately. The mode response is delayed
  // past the manager's fan-out deadline to simulate an unresponsive Equipment Module.
  auto slow_child_em = std::make_shared<ChildEquipmentModule>(slow_child_node);
  slow_child_em->mode_response_delay = 6s;

  auto slow_sm = std::make_unique<SMNode_new>(slow_node);
  auto slow_state_client = slow_node->create_client<packml_msgs::srv::StateChange>(
    slow_name + "/" + packml_ros::kChangeStateService);
  auto slow_mode_client = slow_node->create_client<packml_msgs::srv::ModeChange>(
    slow_name + "/" + packml_ros::kChangeModeService);

  // Each node gets its OWN dedicated spinner thread: the child's 6s-sleeping hook
  // callback must not stall the manager's executor, which has to keep delivering the
  // deadline timer and any response callbacks for this test to measure the real
  // fan-out behavior rather than executor contention.
  packml_ros_test::SpinHelper slow_spin(slow_node);
  packml_ros_test::SpinHelper slow_child_spin(slow_child_node);

  ASSERT_TRUE(slow_state_client->wait_for_service(5s));
  ASSERT_TRUE(slow_mode_client->wait_for_service(5s));

  // The state machine starts in STOPPED, which permits runtime mode changes.

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
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
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

// A registered child whose mode-transition service is OFFLINE (no such service at
// all, though it does serve state transitions — see StateOnlyChild) must not block
// or fail the mode change either: the response reports the accepted change
// immediately, and the unreachable child is flagged with a WARN "service
// unavailable" Alarm once the fan-out deadline expires (the fan-out keeps retrying
// discovery until then, so a merely slow-to-discover child is NOT falsely flagged).
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

  // Every registered node participates in coordinated-state completion. StateOnlyChild
  // completes state goals and omits the mode-transition service, isolating the
  // unavailable mode-endpoint behavior.
  auto offline_child_node = rclcpp::Node::make_shared(offline_child_name);
  StateOnlyChild offline_child(offline_child_node);
  packml_ros_test::SpinHelper offline_child_spin(offline_child_node);

  packml_ros_test::SpinHelper mgr_spin(mgr_node);
  ASSERT_TRUE(state_client->wait_for_service(5s));
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  // The state machine starts in STOPPED, which permits runtime mode changes.

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(100).reliable().transient_local(),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
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

// A registered child whose state-transition ACTION SERVER is not yet discoverable when the
// manager first fans out RESETTING must not be abandoned forever: fanout_state_transition()
// now retries discovery on every check_fanout_deadlines() tick (mirroring the mode fan-out's
// existing retry), so a child that comes up shortly after (well within the 5s fan-out
// deadline) still receives the goal, and the coordinated wait still resolves normally.
TEST_F(ManagerClientFanoutTest, LateDiscoveredStateChildEventuallyReceivesTransition)
{
  auto mgr_name = packml_ros_test::unique_node_name("mgr_late_child_test");
  const std::string late_child_name = "late_child_module";

  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("node_names", std::vector<std::string>{late_child_name})}));
  auto mgr_sm = std::make_unique<SMNode_new>(mgr_node);
  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/" + packml_ros::kChangeStateService);
  packml_ros_test::SpinHelper mgr_spin(mgr_node);
  ASSERT_TRUE(state_client->wait_for_service(5s));

  // RESET fans out RESETTING while late_child_module's action server does not exist yet —
  // this send attempt must be queued for retry, not dropped.
  auto state_req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  state_req->command = packml_msgs::srv::StateChange::Request::RESET;
  auto state_future = state_client->async_send_request(state_req);
  ASSERT_EQ(state_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(state_future.get()->success) << "RESET should be accepted regardless of fan-out";

  // Let at least one retry tick (200ms) pass with the child still absent, then bring it up —
  // well within the 5s fan-out deadline.
  std::this_thread::sleep_for(1500ms);
  auto late_child_node = rclcpp::Node::make_shared(late_child_name);
  auto late_child_em = std::make_shared<ChildEquipmentModule>(late_child_node);
  packml_ros_test::SpinHelper late_child_spin(late_child_node);

  // One deadline shared with the IDLE wait below: both are steps of the same retry, and giving
  // each its own budget would double how long a genuinely stuck run takes to fail.
  const auto deadline = std::chrono::steady_clock::now() + 8s;
  while (late_child_em->last_state_received.load() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(20ms);
  }
  const auto received = late_child_em->last_state_received.load();
  EXPECT_TRUE(received == packml_msgs::msg::State::RESETTING ||
              received == packml_msgs::msg::State::IDLE)
    << "Late-discovered child never received the retried state transition (got "
    << static_cast<int>(received) << ")";

  // The coordinated wait must still resolve normally (IDLE), not fail out to ABORTING —
  // proving the retried goal's real result reached completion_tracker_.
  while (mgr_sm->getCurrentState() != packml_sm::State::IDLE &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(mgr_sm->getCurrentState(), packml_sm::State::IDLE)
    << "RESET did not complete normally once the late child was discovered";
}

// A registered child whose state-transition action server never appears at all must not
// leave the coordinated wait riding out the full (and, in this test, much longer)
// state_complete_timeout_ms: fanout_state_transition()'s ClientFanout now reports the
// never-discovered client to completion_tracker_ (via on_client_failed) as soon as its own
// 5s fan-out deadline expires, so the machine fails out to ABORTING promptly instead.
TEST_F(ManagerClientFanoutTest, NeverDiscoveredStateChildFailsFastNotAfterFullTimeout)
{
  auto mgr_name = packml_ros_test::unique_node_name("mgr_never_child_test");
  const std::string offline_child_name = "never_discovered_child_module";

  // Configured well above the ~5s fan-out deadline: reaching ABORTING promptly (not after
  // riding out this whole window) is what proves the fast on_client_failed path fired.
  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("node_names", std::vector<std::string>{offline_child_name}),
       rclcpp::Parameter("state_complete_timeout_ms", 15000)}));
  auto mgr_sm = std::make_unique<SMNode_new>(mgr_node);
  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/" + packml_ros::kChangeStateService);
  packml_ros_test::SpinHelper mgr_spin(mgr_node);
  ASSERT_TRUE(state_client->wait_for_service(5s));

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(100).reliable().transient_local(),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u) << "Alarm subscription never matched";
  }

  const auto start = std::chrono::steady_clock::now();
  auto state_req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  state_req->command = packml_msgs::srv::StateChange::Request::RESET;
  auto state_future = state_client->async_send_request(state_req);
  ASSERT_EQ(state_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(state_future.get()->success) << "RESET should be accepted regardless of fan-out";

  packml_ros_test::wait_until(
    [&] {
      const auto state = mgr_sm->getCurrentState();
      return state == packml_sm::State::ABORTING || state == packml_sm::State::ABORTED;
    }, 10s, 20ms);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_TRUE(
    mgr_sm->getCurrentState() == packml_sm::State::ABORTING ||
    mgr_sm->getCurrentState() == packml_sm::State::ABORTED)
    << "Machine never failed out after the never-discovered child's fan-out deadline expired";
  EXPECT_LT(elapsed, 10s)
    << "Machine took as long as the full state_complete_timeout_ms — on_client_failed did "
    << "not short-circuit the wait as expected";

  bool found_unavailable_alarm = false;
  {
    std::lock_guard<std::mutex> lk(alarms_mutex);
    for (const auto & a : alarms) {
      if (a.node_name == offline_child_name && a.trigger &&
          a.severity == packml_msgs::msg::Alarm::WARN &&
          a.message.find("state") != std::string::npos &&
          a.message.find("unavailable") != std::string::npos)
      {
        found_unavailable_alarm = true;
        break;
      }
    }
  }
  EXPECT_TRUE(found_unavailable_alarm)
    << "No WARN alarm flagged the never-discovered child's unavailable state-transition action";
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
