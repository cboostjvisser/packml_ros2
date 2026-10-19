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
// Integration tests: PackML manager + HealthMonitor.
// These tests run a real SMNode_new (with Qt state machine) alongside real
// Equipment Module nodes (ConfigurableEquipmentModule) and verify that:
//   - The health gate blocks RESET until all required EMs are healthy.
//   - Health events (HOLD/SUSPEND/ABORT) from EMs drive state machine transitions.
//   - The gate opens after all required EMs report HEALTHY.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/msg/status.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/msg/node_heartbeat.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

using packml_ros_test::send_state_change;
using NodeHealth = packml_msgs::msg::NodeHealth;
using NodeHeartbeat = packml_msgs::msg::NodeHeartbeat;

// ============================================================================
// Helpers
// ============================================================================

/// Minimal Equipment Module: publishes heartbeats on demand.
class SimEquipmentModule : public PackmlNodeInterface
{
public:
  explicit SimEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  // Re-expose the protected simulation helper so the test fixture can silence/resume
  // this module's heartbeat directly (it is protected on the production base class).
  using PackmlNodeInterface::set_heartbeat_active;

  std::atomic<int32_t> health_status{NodeHealth::HEALTHY};
  std::atomic<int32_t> health_action{NodeHealth::NONE};

  NodeHealth get_health_status() override
  {
    NodeHealth h;
    h.status = health_status.load();
    h.action = health_action.load();
    return h;
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
};

/// Call ~/changeState and return response (or nullptr on timeout).
///
/// The node is taken but unused: a background SpinHelper pumps it for the whole fixture, so this
/// waits on the future rather than spinning here. Kept in the signature so every call site reads
/// the same as the other send_* helpers in this suite.
// ============================================================================
// Test fixture
// ============================================================================

class HealthIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Manager node
    mgr_node_name_ = packml_ros_test::unique_node_name("health_mgr");
    mgr_node_ = rclcpp::Node::make_shared(mgr_node_name_);

    // Pre-set required_nodes and timeout_factor BEFORE SMNode_new so the
    // manager init() picks them up.  Use set_parameter after declaring them.
    mgr_node_->declare_parameter(
      "required_nodes",
      std::vector<std::string>{"sim_em_a", "sim_em_b"});
    mgr_node_->declare_parameter("heartbeat_timeout_factor", 3.0);

    sm_node_ = std::make_unique<SMNode_new>(mgr_node_);

    // Equipment Module nodes — their heartbeat topics are /<name>/heartbeat
    em_a_node_ = rclcpp::Node::make_shared("sim_em_a");
    em_b_node_ = rclcpp::Node::make_shared("sim_em_b");
    // Fast heartbeat (production default is 1000ms) so the manager receives and
    // reacts quickly and the heartbeat-paced waits below can be short.
    // Per-node timeout becomes 100ms x factor(3) = 300ms, still > the manager's
    // 200ms timeout-check interval, so continuous heartbeats never falsely time out.
    em_a_node_->declare_parameter("heartbeat_interval_ms", 100);
    em_b_node_->declare_parameter("heartbeat_interval_ms", 100);
    em_a_ = std::make_shared<SimEquipmentModule>(em_a_node_);
    em_b_ = std::make_shared<SimEquipmentModule>(em_b_node_);

    // Start with heartbeats paused so the manager has NOT yet "seen" any required
    // EM. With the fast 100ms test interval an active EM would be observed during
    // the settle below, opening the gate before a test that means to find it shut.
    // make_all_healthy() resumes publishing for the tests that need healthy EMs.
    em_a_->set_heartbeat_active(false);
    em_b_->set_heartbeat_active(false);

    // Service client talking to the manager
    state_client_ = mgr_node_->create_client<packml_msgs::srv::StateChange>(
      mgr_node_name_ + "/changeState");

    // Each node needs its own SpinHelper (each creates a single-threaded executor).
    spin_ = std::make_shared<packml_ros_test::SpinHelper>(mgr_node_);
    em_spin_a_ = std::make_shared<packml_ros_test::SpinHelper>(em_a_node_);
    em_spin_b_ = std::make_shared<packml_ros_test::SpinHelper>(em_b_node_);

    ASSERT_TRUE(state_client_->wait_for_service(5s));
    std::this_thread::sleep_for(300ms);  // let everything settle
  }

  void TearDown() override
  {
    em_spin_b_.reset();
    em_spin_a_.reset();
    spin_.reset();
    state_client_.reset();
    em_b_.reset();
    em_a_.reset();
    em_b_node_.reset();
    em_a_node_.reset();
    sm_node_.reset();
    mgr_node_.reset();
  }

  /// Bring both EMs to HEALTHY and wait for the gate to open. The default wait
  /// allows ~10 heartbeats (interval=100ms) from each EM plus pub/sub discovery to
  /// reach the manager — generous margin so the gate-open tests don't flake on CI.
  void make_all_healthy(std::chrono::milliseconds wait = 1000ms)
  {
    em_a_->health_status = NodeHealth::HEALTHY;
    em_a_->health_action = NodeHealth::NONE;
    em_b_->health_status = NodeHealth::HEALTHY;
    em_b_->health_action = NodeHealth::NONE;
    // Resume heartbeats (paused in SetUp) now that the EMs report healthy.
    em_a_->set_heartbeat_active(true);
    em_b_->set_heartbeat_active(true);
    std::this_thread::sleep_for(wait);
  }

  /// Poll the manager state machine until it reaches `target` (or times out).
  bool wait_for_sm_state(packml_sm::State target, std::chrono::milliseconds timeout = 3s)
  {
    return packml_ros_test::wait_for_state(sm_node_, target, timeout);
  }

  std::string mgr_node_name_;
  rclcpp::Node::SharedPtr mgr_node_;
  std::unique_ptr<SMNode_new> sm_node_;

  rclcpp::Node::SharedPtr em_a_node_;
  rclcpp::Node::SharedPtr em_b_node_;
  std::shared_ptr<SimEquipmentModule> em_a_;
  std::shared_ptr<SimEquipmentModule> em_b_;

  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;

  std::shared_ptr<packml_ros_test::SpinHelper> spin_;
  std::shared_ptr<packml_ros_test::SpinHelper> em_spin_a_;
  std::shared_ptr<packml_ros_test::SpinHelper> em_spin_b_;
};

// ============================================================================
// Manager + HealthMonitor end-to-end integration
// ============================================================================

// RESET from STOPPED is blocked by the health gate while required EMs have not yet
// sent a healthy heartbeat. Expected: changeState(RESET) returns success=false.
TEST_F(HealthIntegrationTest, GateBlocksResetUntilNodesHealthy)
{
  // Machine starts in STOPPED (after being initialised with undefined state).
  // Drive to STOPPED via STOP command first.
  // Actually: at startup state is UNDEFINED → ABORT → ABORTED → CLEAR → STOPPED
  // The manager starts in UNDEFINED. STOP drives to STOPPED.
  // We need to get to STOPPED first.
  auto stop_resp = send_state_change(
    state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  // May fail if state machine isn't in a stoppable state — that's OK for this test.
  (void)stop_resp;

  std::this_thread::sleep_for(300ms);

  // EMs have NOT sent healthy heartbeats yet → gate should block RESET
  auto resp = send_state_change(
    state_client_,
    packml_msgs::srv::StateChange::Request::RESET);

  ASSERT_NE(resp, nullptr);
  // Should be rejected because no heartbeats received yet
  EXPECT_FALSE(resp->success)
    << "RESET should be blocked when required nodes have not sent healthy heartbeats";
}

// Once every required EM has sent a healthy heartbeat, the gate opens.
// Expected: changeState(RESET) returns success=true.
TEST_F(HealthIntegrationTest, GateOpensAfterAllNodesHealthy)
{
  // Drive to STOPPED
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);

  // Send healthy heartbeats from both EMs and wait for manager to receive them
  make_all_healthy();

  // RESET should now succeed
  auto resp = send_state_change(
    state_client_,
    packml_msgs::srv::StateChange::Request::RESET);

  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "RESET should succeed once all required EMs are healthy";
}

// An EM reporting HOLD while the machine is EXECUTE drives a manager transition.
// Expected: the SM ends up in HOLDING or HELD.
TEST_F(HealthIntegrationTest, EquipmentModuleHoldTriggersManagerHold)
{
  // Get to EXECUTE: STOP → (healthy) → RESET → IDLE → START → EXECUTE
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::IDLE))
    << "machine did not reach IDLE after RESET";

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::START);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::EXECUTE))
    << "machine did not reach EXECUTE before fault injection";

  // Now inject HOLD from EM-A
  em_a_->health_status = NodeHealth::ERROR;
  em_a_->health_action = NodeHealth::HOLD;

  // Wait for at most 2 heartbeat cycles (2 × 1000 ms) for the manager to react
  std::this_thread::sleep_for(1000ms);

  // The state machine should now be in HOLDING or HELD
  auto sm_state = sm_node_->getCurrentState();
  EXPECT_TRUE(
    sm_state == packml_sm::State::HOLDING ||
    sm_state == packml_sm::State::HELD)
    << "Expected HOLDING or HELD after EM reports HOLD, got: "
    << static_cast<int>(sm_state);
}

// An EM reporting ABORT while the machine is EXECUTE drives the machine to abort.
// Expected: the SM ends up in ABORTING or ABORTED.
TEST_F(HealthIntegrationTest, EquipmentModuleAbortTriggersManagerAbort)
{
  // Get to EXECUTE
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::IDLE))
    << "machine did not reach IDLE after RESET";

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::START);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::EXECUTE))
    << "machine did not reach EXECUTE before fault injection";

  // Inject ABORT from EM-B
  em_b_->health_status = NodeHealth::ERROR;
  em_b_->health_action = NodeHealth::ABORT;

  std::this_thread::sleep_for(1000ms);

  auto sm_state = sm_node_->getCurrentState();
  EXPECT_TRUE(
    sm_state == packml_sm::State::ABORTING ||
    sm_state == packml_sm::State::ABORTED)
    << "Expected ABORTING or ABORTED after EM reports ABORT, got: "
    << static_cast<int>(sm_state);
}

// A required EM whose heartbeat sequence resets after a period of silence -- simulating
// the EM process restarting -- drives the machine to ABORT. See
// HealthMonitor::HeartbeatResult::RESTART and the fire_packml_action(ABORT) call in the
// heartbeat-subscription callback (PackmlManagerInterface::init()): a restarted node's own
// state is unknown, and PackML recovery goes through CLEAR->RESET regardless of what
// triggered it, so RESTART is routed through the same health-escalation path as HOLD/
// SUSPEND/ABORT rather than silently re-baselining. Expected: the SM ends up in ABORTING
// or ABORTED. Note: at this fixture's 200ms check_timeouts() cadence, the heartbeat-TIMEOUT
// path (checked independently, on the same silence) can also reach that outcome on its own, so
// this test verifies the required end state rather than isolating the RESTART path.
TEST_F(HealthIntegrationTest, RequiredNodeRestartTriggersManagerAbort)
{
  // Get to EXECUTE
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::IDLE))
    << "machine did not reach IDLE after RESET";

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::START);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::EXECUTE))
    << "machine did not reach EXECUTE before fault injection";

  // Silence EM-B long enough to be observed absent (its per-node timeout is
  // heartbeat_interval_ms(100) x heartbeat_timeout_factor(3) = 300ms), then replace it
  // with a fresh instance on the same node/topic. A fresh HeartbeatState starts its
  // sequence counter at 0, so its first published heartbeat (seq=1) is a backward jump
  // from whatever EM-B last reported -- exactly the sequence-number collapse a real
  // process restart produces.
  em_b_->set_heartbeat_active(false);
  std::this_thread::sleep_for(600ms);
  em_b_.reset();
  em_b_ = std::make_shared<SimEquipmentModule>(em_b_node_);

  std::this_thread::sleep_for(1000ms);

  auto sm_state = sm_node_->getCurrentState();
  EXPECT_TRUE(
    sm_state == packml_sm::State::ABORTING ||
    sm_state == packml_sm::State::ABORTED)
    << "Expected ABORTING or ABORTED after EM-B's heartbeat sequence reset "
    << "(simulated restart), got: " << static_cast<int>(sm_state);
}

// An EM reporting SUSPEND while the machine is EXECUTE drives the machine to suspend.
// Expected: the SM ends up in SUSPENDING or SUSPENDED.
TEST_F(HealthIntegrationTest, EquipmentModuleSuspendTriggersManagerSuspend)
{
  // Get to EXECUTE
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::IDLE))
    << "machine did not reach IDLE after RESET";

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::START);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::EXECUTE))
    << "machine did not reach EXECUTE before fault injection";

  // Inject SUSPEND from EM-A
  em_a_->health_status = NodeHealth::ERROR;
  em_a_->health_action = NodeHealth::SUSPEND;

  std::this_thread::sleep_for(1000ms);

  auto sm_state = sm_node_->getCurrentState();
  EXPECT_TRUE(
    sm_state == packml_sm::State::SUSPENDING ||
    sm_state == packml_sm::State::SUSPENDED)
    << "Expected SUSPENDING or SUSPENDED after EM reports SUSPEND, got: "
    << static_cast<int>(sm_state);
}

// WARN is observe-only: an EM reporting WARN must not drive any transition.
// Expected: the SM stays in EXECUTE.
TEST_F(HealthIntegrationTest, EquipmentModuleWarnDoesNotTriggerTransition)
{
  // Get to EXECUTE
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::IDLE))
    << "machine did not reach IDLE after RESET";

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::START);
  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::EXECUTE))
    << "machine did not reach EXECUTE before fault injection";

  // Inject WARN from EM-A
  em_a_->health_status = NodeHealth::DEGRADED;
  em_a_->health_action = NodeHealth::WARN;

  std::this_thread::sleep_for(1000ms);

  // SM should still be in EXECUTE (WARN is log-only)
  auto sm_state = sm_node_->getCurrentState();
  EXPECT_EQ(sm_state, packml_sm::State::EXECUTE)
    << "WARN should not trigger a state transition, expected EXECUTE, got: "
    << static_cast<int>(sm_state);
}

// A HOLD from an EM while the machine is STOPPED must NOT change state, and the gate
// must stay closed (the SM rejects HOLD-from-STOPPED, and HealthMonitor fires it only
// once per new event so there is no repeat/log spam).
// Expected: the SM stays STOPPED and a subsequent RESET is rejected.
TEST_F(HealthIntegrationTest, HoldWhileStoppedDoesNotChangeState)
{
  // Drive to STOPPED and wait for gate to open.
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  // Now inject HOLD from EM-A while machine is still STOPPED.
  em_a_->health_status = NodeHealth::ERROR;
  em_a_->health_action = NodeHealth::HOLD;

  // Wait for two heartbeat cycles.
  std::this_thread::sleep_for(1000ms);

  // Machine must still be STOPPED — HOLD is suppressed in inactive cycle.
  auto sm_state = sm_node_->getCurrentState();
  EXPECT_EQ(sm_state, packml_sm::State::STOPPED)
    << "Machine should stay STOPPED when HOLD is injected while stopped, got: "
    << static_cast<int>(sm_state);

  // Gate must be closed (EM in error blocks RESET).
  auto resp = send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success) << "RESET should be blocked while EM has active HOLD";
}

// Toggling a fault on an EM publishes a raise then a clear on the alarm topic.
// Expected: at least one trigger=true (HOLD) and one trigger=false event for that EM.
// (Regression: a fault clear must be visible on the alarm topic, not just the raise.)
TEST_F(HealthIntegrationTest, AlarmClearPublishedOnFaultClear)
{
  // Subscribe to the alarm topic on the manager node.
  std::vector<packml_msgs::msg::Alarm> alarms;
  std::mutex alarms_mutex;
  auto alarm_sub = mgr_node_->create_subscription<packml_msgs::msg::Alarm>(
    "packml_alarms", rclcpp::QoS(50),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });

  // Drive to STOPPED and make EMs healthy.
  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  const size_t alarms_before = [&] {
    std::lock_guard<std::mutex> lk(alarms_mutex);
    return alarms.size();
  }();

  // Inject HOLD fault from EM-A.
  em_a_->health_status = NodeHealth::ERROR;
  em_a_->health_action = NodeHealth::HOLD;
  std::this_thread::sleep_for(800ms);  // wait for heartbeat to propagate

  // Clear fault.
  em_a_->health_status = NodeHealth::HEALTHY;
  em_a_->health_action = NodeHealth::NONE;
  std::this_thread::sleep_for(800ms);  // wait for clear heartbeat to propagate

  std::lock_guard<std::mutex> lk(alarms_mutex);
  ASSERT_GE(alarms.size(), alarms_before + 2)
    << "Expected at least one alarm-raised and one alarm-cleared event";

  // Find the raise and clear events from sim_em_a.
  bool found_raise = false;
  bool found_clear = false;
  for (const auto & a : alarms) {
    if (a.node_name == "sim_em_a") {
      EXPECT_FALSE(a.is_timeout) << "a node-reported HOLD must not be flagged as a timeout alarm";
      if (a.trigger) {
        found_raise = true;
        EXPECT_EQ(a.severity, NodeHealth::HOLD);
      } else {
        found_clear = true;
        EXPECT_EQ(a.severity, NodeHealth::HOLD) << "clear should preserve the severity that cleared";
      }
    }
  }
  EXPECT_TRUE(found_raise) << "No alarm-raised event seen on packml_alarms for sim_em_a";
  EXPECT_TRUE(found_clear) << "No alarm-cleared event seen on packml_alarms for sim_em_a";
}

// Alarm::stop_event_id correlates every alarm in one gate-closed episode: the raise
// and the clear that ends it share the same nonzero id; once the gate has reopened
// (the periodic timer observes it) a later, separate episode gets a NEW id.
TEST_F(HealthIntegrationTest, StopEventIdCorrelatesEpisodeThenAdvances)
{
  std::vector<packml_msgs::msg::Alarm> alarms;
  std::mutex alarms_mutex;
  auto alarm_sub = mgr_node_->create_subscription<packml_msgs::msg::Alarm>(
    "packml_alarms", rclcpp::QoS(50),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  auto snapshot = [&] {
    std::lock_guard<std::mutex> lk(alarms_mutex);
    return alarms;
  };

  send_state_change(state_client_,
    packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);
  make_all_healthy();

  // --- Episode 1: raise then clear ---
  em_a_->health_status = NodeHealth::ERROR;
  em_a_->health_action = NodeHealth::HOLD;
  std::this_thread::sleep_for(800ms);
  em_a_->health_status = NodeHealth::HEALTHY;
  em_a_->health_action = NodeHealth::NONE;
  std::this_thread::sleep_for(800ms);

  uint64_t episode1_id = 0;
  bool found_raise = false;
  for (const auto & a : snapshot()) {
    if (a.node_name == "sim_em_a") {
      EXPECT_NE(a.stop_event_id, 0u) << "a HOLD-severity alarm must belong to a stop episode";
      if (episode1_id == 0) {
        episode1_id = a.stop_event_id;
      } else {
        EXPECT_EQ(a.stop_event_id, episode1_id)
          << "the raise and its clear must share the same episode id";
      }
      if (a.trigger) {
        found_raise = true;
        EXPECT_NE(a.state.val, packml_msgs::msg::State::UNDEFINED);
      }
    }
  }
  ASSERT_TRUE(found_raise);
  ASSERT_NE(episode1_id, 0u);

  // Give the periodic timer time to observe the reopened gate and reset the episode.
  std::this_thread::sleep_for(500ms);
  {
    std::lock_guard<std::mutex> lk(alarms_mutex);
    alarms.clear();
  }

  // --- Episode 2: a fresh, unrelated raise must get a NEW id ---
  em_a_->health_status = NodeHealth::ERROR;
  em_a_->health_action = NodeHealth::HOLD;
  std::this_thread::sleep_for(800ms);

  bool found_second_raise = false;
  for (const auto & a : snapshot()) {
    if (a.node_name == "sim_em_a" && a.trigger) {
      found_second_raise = true;
      EXPECT_NE(a.stop_event_id, 0u);
      EXPECT_NE(a.stop_event_id, episode1_id)
        << "a new episode after the gate reopened must get a new id";
    }
  }
  EXPECT_TRUE(found_second_raise) << "No second raise event observed for sim_em_a";
}
