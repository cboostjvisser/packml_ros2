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
// Tests for HealthMonitor and PackmlNodeInterface heartbeat.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/interface/packml_interface.hpp"
#include "packml_ros/health_monitor.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/msg/node_heartbeat.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;
using NodeHealth = packml_msgs::msg::NodeHealth;
using NodeHeartbeat = packml_msgs::msg::NodeHeartbeat;

// ============================================================================
// Helpers
// ============================================================================

/// Equipment Module that returns a configurable health state.
class ConfigurableEquipmentModule : public PackmlNodeInterface
{
public:
  explicit ConfigurableEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  // Set this to control what get_health_status() returns
  std::atomic<int32_t> health_status{NodeHealth::HEALTHY};
  std::atomic<int32_t> health_action{NodeHealth::NONE};
  std::string health_message;
  std::atomic<int32_t> health_error_code{0};

  NodeHealth get_health_status() override
  {
    NodeHealth h;
    h.status = health_status.load();
    h.action = health_action.load();
    h.message = health_message;
    h.error_code = health_error_code.load();
    return h;
  }

protected:
  bool on_state_trans_req(packml_sm::State) override { return true; }
  bool on_mode_trans_req(packml_sm::ModeType) override { return true; }
  void on_status_changed() override {}
};

/// Captures all received heartbeats from a topic.
class HeartbeatCapture
{
public:
  explicit HeartbeatCapture(rclcpp::Node::SharedPtr node, const std::string & topic)
  {
    sub_ = node->create_subscription<NodeHeartbeat>(
      topic, rclcpp::SensorDataQoS(),
      [this](NodeHeartbeat::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        messages_.push_back(*msg);
        count_.fetch_add(1);
      });
  }

  int count() const { return count_.load(); }

  NodeHeartbeat last() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return messages_.empty() ? NodeHeartbeat{} : messages_.back();
  }

  std::vector<NodeHeartbeat> all() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return messages_;
  }

  void wait_for(int n, std::chrono::milliseconds timeout = 3s)
  {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (count_.load() < n && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
  }

private:
  rclcpp::Subscription<NodeHeartbeat>::SharedPtr sub_;
  mutable std::mutex mutex_;
  std::vector<NodeHeartbeat> messages_;
  std::atomic<int> count_{0};
};

// ============================================================================
// Test fixture for heartbeat publisher tests
// ============================================================================

class HeartbeatPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("hb_test_node");
    // Fast heartbeat so timing-based tests finish quickly (production default 1000ms).
    node_->declare_parameter("heartbeat_interval_ms", 100);
    equipment_ = std::make_shared<ConfigurableEquipmentModule>(node_);
    capture_ = std::make_shared<HeartbeatCapture>(node_, "~/heartbeat");
    spin_ = std::make_shared<packml_ros_test::SpinHelper>(node_);
  }

  void TearDown() override
  {
    spin_.reset();
    capture_.reset();
    equipment_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ConfigurableEquipmentModule> equipment_;
  std::shared_ptr<HeartbeatCapture> capture_;
  std::shared_ptr<packml_ros_test::SpinHelper> spin_;
};

// ============================================================================
// NodeHealth Message — PackmlNodeInterface Unit Tests
// ============================================================================

// A freshly initialized module reports HEALTHY status with NONE action by default.
TEST_F(HeartbeatPublisherTest, DefaultHealthIsHealthy)
{
  // TODO: implement once PackmlNodeInterface::get_health_status() exists
  auto h = equipment_->get_health_status();
  EXPECT_EQ(h.status, NodeHealth::HEALTHY);
  EXPECT_EQ(h.action, NodeHealth::NONE);
}

// At least one heartbeat is published within two heartbeat intervals of startup.
TEST_F(HeartbeatPublisherTest, HeartbeatPublishedWithinTwoIntervals)
{
  // Default interval is 1000ms; wait up to 2500ms
  capture_->wait_for(1, 2500ms);
  EXPECT_GE(capture_->count(), 1) << "No heartbeat received within 2 × interval";
}

// A published heartbeat's node_name field matches the ROS node's name.
TEST_F(HeartbeatPublisherTest, HeartbeatNodeNameMatchesRosName)
{
  capture_->wait_for(1);
  ASSERT_GE(capture_->count(), 1);
  EXPECT_EQ(capture_->last().node_name, node_->get_name());
}

// The heartbeat_interval_ms field in the message equals the heartbeat_interval_ms parameter.
TEST_F(HeartbeatPublisherTest, HeartbeatIntervalMatchesParameter)
{
  capture_->wait_for(1, 2500ms);
  ASSERT_GE(capture_->count(), 1);
  const auto expected = static_cast<uint32_t>(
    node_->get_parameter("heartbeat_interval_ms").as_int());
  EXPECT_EQ(capture_->last().heartbeat_interval_ms, expected);
}

// Across consecutive heartbeats the sequence_number strictly increases.
TEST_F(HeartbeatPublisherTest, SequenceNumberMonotonicallyIncreases)
{
  capture_->wait_for(3, 5s);
  ASSERT_GE(capture_->count(), 3);
  auto msgs = capture_->all();
  for (size_t i = 1; i < msgs.size(); ++i) {
    EXPECT_GT(msgs[i].sequence_number, msgs[i - 1].sequence_number)
      << "sequence_number did not increase at index " << i;
  }
}

// When get_health_status() reports DEGRADED/WARN with a message, the published
// heartbeat carries that status, action, and message verbatim.
TEST_F(HeartbeatPublisherTest, OverrideDegradedWarn)
{
  equipment_->health_status = NodeHealth::DEGRADED;
  equipment_->health_action = NodeHealth::WARN;
  equipment_->health_message = "test warning";

  capture_->wait_for(1, 2500ms);
  ASSERT_GE(capture_->count(), 1);
  auto msg = capture_->last();
  EXPECT_EQ(msg.health.status, NodeHealth::DEGRADED);
  EXPECT_EQ(msg.health.action, NodeHealth::WARN);
  EXPECT_EQ(msg.health.message, "test warning");
}

// When get_health_status() reports ERROR/HOLD, the published heartbeat carries the HOLD action.
TEST_F(HeartbeatPublisherTest, OverrideErrorHold)
{
  equipment_->health_status = NodeHealth::ERROR;
  equipment_->health_action = NodeHealth::HOLD;

  capture_->wait_for(1, 2500ms);
  ASSERT_GE(capture_->count(), 1);
  EXPECT_EQ(capture_->last().health.action, NodeHealth::HOLD);
}

// When get_health_status() reports ERROR/ABORT, the published heartbeat carries the ABORT action.
TEST_F(HeartbeatPublisherTest, OverrideErrorAbort)
{
  equipment_->health_status = NodeHealth::ERROR;
  equipment_->health_action = NodeHealth::ABORT;

  capture_->wait_for(1, 2500ms);
  EXPECT_EQ(capture_->last().health.action, NodeHealth::ABORT);
}

// Changing the reported health between heartbeat ticks is reflected in the next published heartbeat.
TEST_F(HeartbeatPublisherTest, OverrideChangedBeforeNextTick)
{
  capture_->wait_for(1, 2500ms);
  int count_before = capture_->count();

  equipment_->health_status = NodeHealth::ERROR;
  equipment_->health_action = NodeHealth::ABORT;

  capture_->wait_for(count_before + 1, 2500ms);
  EXPECT_EQ(capture_->last().health.action, NodeHealth::ABORT);
}

// The error_code reported by get_health_status() is carried unchanged in the published heartbeat.
TEST_F(HeartbeatPublisherTest, ErrorCodePreserved)
{
  equipment_->health_error_code = 999;
  capture_->wait_for(1, 2500ms);
  EXPECT_EQ(capture_->last().health.error_code, 999);
}

// ============================================================================
// post_event() — Immediate publish
// ============================================================================

// post_event() with an ABORT health publishes an extra heartbeat immediately,
// before the next periodic timer tick, carrying the ABORT action.
TEST_F(HeartbeatPublisherTest, PostEventPublishesImmediately)
{
  // Wait for the first periodic beat so the best-effort subscription has matched
  // the publisher — otherwise the immediate post_event below could be dropped
  // during pub/sub discovery and the count would never increment.
  capture_->wait_for(1, 2500ms);
  ASSERT_GE(capture_->count(), 1);

  int count_before = capture_->count();

  NodeHealth h;
  h.status = NodeHealth::ERROR;
  h.action = NodeHealth::ABORT;
  equipment_->post_event(h);

  // The immediate publish should arrive well within one timer interval (100ms here).
  capture_->wait_for(count_before + 1, 200ms);
  EXPECT_GT(capture_->count(), count_before);
  EXPECT_EQ(capture_->last().health.action, NodeHealth::ABORT);
}

// get_health_status() must be a pure getter and not call post_event(). Verifies
// heartbeat sequence numbers stay monotonic and every beat carries the reported
// state, guarding against double heartbeats and spurious alarm raise/clear cycles.
TEST_F(HeartbeatPublisherTest, GetHealthStatusHasNoSideEffects)
{
  // Override health to return ERROR/ABORT (simulating a fault, NO post_event inside).
  equipment_->health_status = NodeHealth::ERROR;
  equipment_->health_action = NodeHealth::ABORT;

  // Wait for at least 2 heartbeat ticks.
  capture_->wait_for(2, 3000ms);
  ASSERT_GE(capture_->count(), 2);

  auto msgs = capture_->all();
  // Sequence numbers must be strictly monotonically increasing.
  // If post_event were called inside get_health_status, the post_event heartbeat
  // would have seq=N+1 but the timer heartbeat would have seq=N (lower), causing
  // inversion.
  for (size_t i = 1; i < msgs.size(); ++i) {
    EXPECT_GT(msgs[i].sequence_number, msgs[i - 1].sequence_number)
      << "Sequence number inversion at index " << i
      << ": got " << msgs[i].sequence_number
      << " after " << msgs[i - 1].sequence_number
      << " — likely caused by post_event() called inside get_health_status()";
  }

  // Each heartbeat must carry the ABORT state (no accidental HEALTHY in between).
  for (size_t i = 0; i < msgs.size(); ++i) {
    EXPECT_EQ(msgs[i].health.action, NodeHealth::ABORT)
      << "Heartbeat " << i << " has unexpected action (expected ABORT). "
      << "Spurious HEALTHY heartbeats indicate get_health_status() has side effects.";
  }
}

// After post_event(fault), the latch makes periodic heartbeats keep repeating the
// fault even though get_health_status() still returns HEALTHY — no raise/clear flap.
// Clearing via post_event(healthy) resumes get_health_status()-driven publishing.
TEST_F(HeartbeatPublisherTest, PeriodicHeartbeatAfterPostEventRetainsSameState)
{
  // get_health_status() stays HEALTHY/NONE (defaults); only post_event posts a fault.
  const int count_before = capture_->count();
  NodeHealth h;
  h.status = NodeHealth::ERROR;
  h.action = NodeHealth::ABORT;
  h.error_code = 42;
  equipment_->post_event(h);

  // Immediate publish + at least two periodic ticks (interval 100ms in this fixture).
  capture_->wait_for(count_before + 3, 2000ms);
  ASSERT_GE(capture_->count(), count_before + 3);

  auto msgs = capture_->all();
  for (size_t i = (msgs.size() >= 3 ? msgs.size() - 3 : 0); i < msgs.size(); ++i) {
    EXPECT_EQ(msgs[i].health.action, NodeHealth::ABORT)
      << "Latched ABORT must repeat on periodic ticks (index " << i << ")";
  }

  // Clear the latch: subsequent periodic heartbeats follow get_health_status() again.
  NodeHealth healthy;
  healthy.status = NodeHealth::HEALTHY;
  healthy.action = NodeHealth::NONE;
  const int count_at_clear = capture_->count();
  equipment_->post_event(healthy);
  capture_->wait_for(count_at_clear + 2, 2000ms);
  ASSERT_GE(capture_->count(), count_at_clear + 2);
  EXPECT_EQ(capture_->last().health.action, NodeHealth::NONE)
    << "After clearing the latch, periodic heartbeats return to get_health_status()";
}

// ============================================================================
// HealthMonitor — State Transition Logic (pure unit tests)
// ============================================================================

// These tests drive HealthMonitor directly without a live ROS node.
// Pure C++ unit tests — no ROS executor or spin needed.

class HealthMonitorLogicTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    action_calls_.clear();
    monitor_ = std::make_unique<HealthMonitor>(
      [this](int32_t action) {action_calls_.push_back(action);});
  }

  std::vector<int32_t> action_calls_;
  std::unique_ptr<HealthMonitor> monitor_;

  /// Inject a heartbeat from a named node into the monitor.
  void inject(
    const std::string & node, int32_t status, int32_t action)
  {
    NodeHeartbeat hb;
    hb.node_name = node;
    hb.health.status = status;
    hb.health.action = action;
    hb.heartbeat_interval_ms = 1000;
    monitor_->on_heartbeat(hb);
  }
};

// A first heartbeat reporting HEALTHY/NONE triggers no SM action.
TEST_F(HealthMonitorLogicTest, FirstHeartbeatNoneNoAction)
{
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  EXPECT_TRUE(action_calls_.empty());
}

// A first heartbeat reporting ERROR/HOLD fires exactly one HOLD action.
TEST_F(HealthMonitorLogicTest, FirstHeartbeatHoldFiresHold)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::HOLD);
}

// A first heartbeat reporting ERROR/ABORT fires exactly one ABORT action.
TEST_F(HealthMonitorLogicTest, FirstHeartbeatAbortFiresAbort)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT);
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::ABORT);
}

// A heartbeat reporting DEGRADED/WARN fires no SM action (WARN is observer-only).
TEST_F(HealthMonitorLogicTest, WarnDoesNotFireAction)
{
  inject("motor", NodeHealth::DEGRADED, NodeHealth::WARN);
  EXPECT_TRUE(action_calls_.empty());
}

// Repeated identical HOLD heartbeats fire the HOLD action only once.
TEST_F(HealthMonitorLogicTest, RepeatedHoldFiresOnce)
{
  for (int i = 0; i < 10; ++i) {
    inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  }
  EXPECT_EQ(action_calls_.size(), 1u);
}

// A WARN heartbeat fires nothing; a following HOLD heartbeat then fires one HOLD action.
TEST_F(HealthMonitorLogicTest, WarnThenHoldFiresHold)
{
  inject("motor", NodeHealth::DEGRADED, NodeHealth::WARN);
  EXPECT_TRUE(action_calls_.empty());
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::HOLD);
}

// Escalating HOLD then ABORT fires two actions, the second being ABORT.
TEST_F(HealthMonitorLogicTest, HoldThenAbortFiresAbort)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT);
  ASSERT_EQ(action_calls_.size(), 2u);
  EXPECT_EQ(action_calls_[1], NodeHealth::ABORT);
}

// De-escalating from ABORT to HOLD fires no new action (the count stays unchanged).
TEST_F(HealthMonitorLogicTest, AbortThenHoldDeEscalationSuppressed)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT);
  size_t count_after_abort = action_calls_.size();
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  EXPECT_EQ(action_calls_.size(), count_after_abort) << "De-escalation should be suppressed";
}

// ABORT, then HOLD (suppressed), then ABORT again re-escalates and fires ABORT a
// second time — total two ABORT actions.
TEST_F(HealthMonitorLogicTest, AbortHoldAbortReEscalationFires)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT);  // fires ABORT
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);   // de-escalate, suppress
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT);  // re-escalate, fires ABORT
  ASSERT_EQ(action_calls_.size(), 2u);
  EXPECT_EQ(action_calls_[0], NodeHealth::ABORT);
  EXPECT_EQ(action_calls_[1], NodeHealth::ABORT);
}

// Returning to HEALTHY/NONE after a HOLD clears the fault without firing any action.
TEST_F(HealthMonitorLogicTest, HoldClearedByHealthy)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  size_t count_after_hold = action_calls_.size();
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  EXPECT_EQ(action_calls_.size(), count_after_hold) << "Clear should not fire action";
}

// After a HOLD is cleared by HEALTHY, a new HOLD heartbeat fires the action again (two total).
TEST_F(HealthMonitorLogicTest, AfterClearNewHoldFiresAgain)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);  // clear
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD);    // new event
  EXPECT_EQ(action_calls_.size(), 2u) << "Second HOLD should fire again after clear";
}

// ============================================================================
// Heartbeat Timeout Logic
// ============================================================================

class HeartbeatTimeoutTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    {
      std::lock_guard<std::mutex> lk(calls_mutex_);
      action_calls_.clear();
    }
    monitor_ = std::make_unique<HealthMonitor>(
      [this](int32_t a) {
        std::lock_guard<std::mutex> lk(calls_mutex_);
        action_calls_.push_back(a);
      },
      /*check_interval_ms=*/5);
    // Default startup_interval_ms=100, factor=3 → timeout after 300ms.
    // A short 5ms poll interval keeps timeout-detection latency low, which widens
    // the safety margin of the sleep-based timeout tests below (less CI flakiness).
    monitor_->register_required_node("motor", /*timeout_factor=*/3);
  }

  void TearDown() override
  {
    monitor_.reset();  // joins background thread before test ends
  }

  int call_count()
  {
    std::lock_guard<std::mutex> lk(calls_mutex_);
    return static_cast<int>(action_calls_.size());
  }

  int32_t last_call()
  {
    std::lock_guard<std::mutex> lk(calls_mutex_);
    return action_calls_.empty() ? -1 : action_calls_.back();
  }

  std::vector<int32_t> action_calls_;
  std::mutex calls_mutex_;
  std::unique_ptr<HealthMonitor> monitor_;
};

// A required node that never sends a heartbeat times out and the monitor fires ABORT.
TEST_F(HeartbeatTimeoutTest, NoHeartbeatCausesAbort)
{
  // startup_interval_ms=100 (default), factor=3 → fires after 300ms
  std::this_thread::sleep_for(400ms);
  ASSERT_GT(call_count(), 0);
  EXPECT_EQ(last_call(), NodeHealth::ABORT);
}

// A single timeout fires the action exactly once even after multiple check intervals elapse.
// 450ms is well past the 300ms timeout, so a spurious second fire would be caught here.
TEST_F(HeartbeatTimeoutTest, TimeoutFiresOnce)
{
  std::this_thread::sleep_for(450ms);
  EXPECT_EQ(call_count(), 1) << "Timeout should fire exactly once";
}

// The timeout window is learned from the heartbeat's advertised interval (interval ×
// factor): a node that beats once then goes silent does not time out before the
// window elapses, and fires ABORT once it does.
TEST_F(HeartbeatTimeoutTest, TimeoutIntervalRespected)
{
  // Inject one heartbeat with interval=100ms, then stop.
  // timeout = 100ms × 3 = 300ms from that heartbeat.
  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  hb.heartbeat_interval_ms = 100;
  monitor_->on_heartbeat(hb);

  // Should NOT have fired yet.
  EXPECT_EQ(call_count(), 0);

  // After 400ms (> 300ms timeout) ABORT should have fired.
  std::this_thread::sleep_for(400ms);
  EXPECT_GT(call_count(), 0);
  EXPECT_EQ(last_call(), NodeHealth::ABORT);
}

// After a timeout has fired its ABORT, the node resumes sending heartbeats: the
// timed-out flag and liveness stamp reset, and NO second ABORT fires while the node
// stays within its (new) timeout window. Expected: exactly one ABORT total, and the
// gate reopens.
TEST_F(HeartbeatTimeoutTest, TimeoutRecoveryNoSecondAbort)
{
  std::vector<int32_t> calls;
  HealthMonitor mon([&calls](int32_t a) {calls.push_back(a);});
  mon.register_required_node("motor", 3.0, /*startup_ms=*/1);   // 3ms timeout

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  mon.check_timeouts();
  ASSERT_EQ(calls.size(), 1u);
  EXPECT_EQ(calls[0], NodeHealth::ABORT);

  // Node resumes with a healthy heartbeat advertising a 1s interval (3s timeout).
  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  hb.heartbeat_interval_ms = 1000;
  mon.on_heartbeat(hb);

  // Checks well inside the new window fire nothing further.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  mon.check_timeouts();
  mon.check_timeouts();
  EXPECT_EQ(calls.size(), 1u);
  EXPECT_TRUE(mon.can_transition_from_stopped());
}

// Each node's timeout window derives from ITS OWN advertised interval: with the same
// factor and the same silence, the fast-interval node times out while the
// slow-interval node does not.
TEST_F(HeartbeatTimeoutTest, PerNodeIntervalUsedForTimeout)
{
  std::vector<int32_t> calls;
  HealthMonitor mon([&calls](int32_t a) {calls.push_back(a);});
  mon.register_required_node("fast_node", 3.0, /*startup_ms=*/10);
  mon.register_required_node("slow_node", 3.0, /*startup_ms=*/10);

  // Both beat once; fast advertises 10ms (30ms timeout), slow 10s (30s timeout).
  NodeHeartbeat hb;
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  hb.node_name = "fast_node";
  hb.heartbeat_interval_ms = 10;
  mon.on_heartbeat(hb);
  hb.node_name = "slow_node";
  hb.heartbeat_interval_ms = 10000;
  mon.on_heartbeat(hb);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  mon.check_timeouts();

  ASSERT_EQ(calls.size(), 1u);   // only the fast node's window elapsed
  EXPECT_EQ(calls[0], NodeHealth::ABORT);
  const auto summary = mon.gate_block_summary();
  EXPECT_NE(summary.find("fast_node (timed out"), std::string::npos);
  EXPECT_EQ(summary.find("slow_node"), std::string::npos);
}

// A per-node timeout_factor override is honored: with identical startup intervals,
// the node registered with a small factor times out while the node registered with a
// large per-node factor does not.
TEST_F(HeartbeatTimeoutTest, PerNodeTimeoutFactorOverride)
{
  std::vector<int32_t> calls;
  HealthMonitor mon([&calls](int32_t a) {calls.push_back(a);});
  mon.register_required_node("short_factor", 2.0, /*startup_ms=*/10);     // 20ms
  mon.register_required_node("long_factor", 1000.0, /*startup_ms=*/10);   // 10s

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  mon.check_timeouts();

  ASSERT_EQ(calls.size(), 1u);
  EXPECT_EQ(calls[0], NodeHealth::ABORT);
  const auto summary = mon.gate_block_summary();
  EXPECT_NE(summary.find("short_factor (timed out"), std::string::npos);
  // long_factor's window has not elapsed; it blocks only as not-yet-seen.
  EXPECT_NE(summary.find("long_factor (never seen"), std::string::npos);
}

// ============================================================================
// Multi-Node Scenarios
// ============================================================================

class MultiNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    action_calls_.clear();
    monitor_ = std::make_unique<HealthMonitor>(
      [this](int32_t a) {action_calls_.push_back(a);});
    monitor_->register_required_node("node_a");
    monitor_->register_required_node("node_b");
  }

  std::vector<int32_t> action_calls_;
  std::unique_ptr<HealthMonitor> monitor_;

  void inject(const std::string & node, int32_t status, int32_t action)
  {
    NodeHeartbeat hb;
    hb.node_name = node;
    hb.health.status = status;
    hb.health.action = action;
    hb.heartbeat_interval_ms = 1000;
    monitor_->on_heartbeat(hb);
  }
};

// With node_a healthy and node_b reporting ABORT, the monitor fires ABORT exactly
// once (driven by node_b only).
TEST_F(MultiNodeTest, NodeBAbortDoesNotAffectNodeA)
{
  inject("node_a", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("node_b", NodeHealth::ERROR, NodeHealth::ABORT);
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::ABORT);
}

// When several required nodes time out in the same check, ABORT fires exactly ONCE —
// it is a single machine-level reaction, deduplicated per tick — while each timed-out
// node still gets its own timeout alarm carrying the per-node detail.
TEST_F(MultiNodeTest, BothNodesTimeoutAbortFiresOnce)
{
  std::vector<int32_t> calls;
  std::vector<AlarmEvent> alarms;
  HealthMonitor mon(
    [&calls](int32_t a) {calls.push_back(a);},
    0,
    [&alarms](const AlarmEvent & ev) {alarms.push_back(ev);});
  mon.register_required_node("node_a", 3.0, /*startup_ms=*/1);
  mon.register_required_node("node_b", 3.0, /*startup_ms=*/1);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  mon.check_timeouts();

  ASSERT_EQ(calls.size(), 1u);   // one machine-level ABORT, not one per node
  EXPECT_EQ(calls[0], NodeHealth::ABORT);
  ASSERT_EQ(alarms.size(), 2u);  // but one timeout alarm per node
  EXPECT_TRUE(alarms[0].is_timeout);
  EXPECT_TRUE(alarms[1].is_timeout);
}

// When every required node is HEALTHY, the STOPPED-transition gate is open.
TEST_F(MultiNodeTest, AllHealthyGateOpen)
{
  inject("node_a", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("node_b", NodeHealth::HEALTHY, NodeHealth::NONE);
  EXPECT_TRUE(monitor_->can_transition_from_stopped());
}

// When one required node is in ERROR/HOLD, the STOPPED-transition gate is closed.
TEST_F(MultiNodeTest, OneNodeErrorGateClosed)
{
  inject("node_a", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("node_b", NodeHealth::ERROR, NodeHealth::HOLD);
  EXPECT_FALSE(monitor_->can_transition_from_stopped());
}

// A timed-out required node keeps the gate closed in both normal and MANUAL mode —
// a timeout is non-bypassable.
TEST_F(MultiNodeTest, TimeoutBlocksGateEvenInManual)
{
  // Fresh monitor with very short startup intervals to force fast timeout.
  std::vector<int32_t> calls;
  auto mon = std::make_unique<HealthMonitor>(
    [&calls](int32_t a) {calls.push_back(a);});
  // startup_interval_ms=1, factor=3 → timeout fires after 3ms
  mon->register_required_node("node_a", 3.0, /*startup_ms=*/1);
  mon->register_required_node("node_b", 3.0, /*startup_ms=*/1);

  // node_b sends a healthy heartbeat; node_a stays silent.
  NodeHeartbeat hb;
  hb.node_name = "node_b";
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  hb.heartbeat_interval_ms = 1000;
  mon->on_heartbeat(hb);

  // Wait > 3ms, then trigger timeout check manually.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  mon->check_timeouts();

  // Gate must be closed in both normal and MANUAL mode.
  EXPECT_FALSE(mon->can_transition_from_stopped(false));
  EXPECT_FALSE(mon->can_transition_from_stopped(true));
}

// The gate must detect a stale heartbeat on its own, WITHOUT relying on check_timeouts()
// having run first — otherwise a dead node could let the machine leave STOPPED during
// the window before the next periodic check. Expected: gate closed in both modes purely
// from elapsed time, even though check_timeouts() is never called and timed_out stays false.
TEST_F(MultiNodeTest, GateDetectsStaleHeartbeatWithoutCheckTimeouts)
{
  std::vector<int32_t> calls;
  auto mon = std::make_unique<HealthMonitor>(
    [&calls](int32_t a) {calls.push_back(a);});
  // startup_interval_ms=1, factor=3 → timeout after 3ms.
  mon->register_required_node("node_a", 3.0, /*startup_ms=*/1);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));  // node_a now stale

  // check_timeouts() is deliberately NOT called here.
  EXPECT_FALSE(mon->can_transition_from_stopped(false));
  EXPECT_FALSE(mon->can_transition_from_stopped(true));
  EXPECT_NE(mon->gate_block_summary(false).find("node_a"), std::string::npos);
}

// An ERROR from an optional (non-registered) node does not close the gate when all
// required nodes are healthy.
TEST_F(MultiNodeTest, OptionalNodeErrorDoesNotBlockGate)
{
  // Both required nodes healthy.
  inject("node_a", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("node_b", NodeHealth::HEALTHY, NodeHealth::NONE);

  // An optional (non-registered) node reports error.
  NodeHeartbeat hb;
  hb.node_name = "optional_sensor";
  hb.health.status = NodeHealth::ERROR;
  hb.health.action = NodeHealth::HOLD;
  hb.heartbeat_interval_ms = 500;
  monitor_->on_heartbeat(hb);

  EXPECT_TRUE(monitor_->can_transition_from_stopped());
}

// A required node in ERROR closes the gate normally but MANUAL mode bypasses it,
// opening the gate.
TEST_F(MultiNodeTest, ManualModeAllowsErrorBypass)
{
  inject("node_a", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("node_b", NodeHealth::ERROR, NodeHealth::HOLD);
  EXPECT_FALSE(monitor_->can_transition_from_stopped(false));   // normal: blocked
  EXPECT_TRUE(monitor_->can_transition_from_stopped(true));     // manual: open
}

// MANUAL mode does not bypass a timeout: a timed-out required node keeps the gate
// closed even in MANUAL mode.
TEST_F(MultiNodeTest, ManualModeDoesNotBypassTimeout)
{
  std::vector<int32_t> calls;
  auto mon = std::make_unique<HealthMonitor>(
    [&calls](int32_t a) {calls.push_back(a);});
  mon->register_required_node("motor", 3.0, /*startup_ms=*/1);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  mon->check_timeouts();

  // MANUAL mode must NOT bypass a timeout.
  EXPECT_FALSE(mon->can_transition_from_stopped(true));
}

// ============================================================================
// Protocol Edge Cases
// ============================================================================

class EdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  std::vector<int32_t> action_calls_;
};

// An ABORT carried by a node's very first heartbeat fires the ABORT action immediately.
TEST_F(EdgeCaseTest, AbortInFirstHeartbeatFiresImmediately)
{
  HealthMonitor mon([this](int32_t a) {action_calls_.push_back(a);});

  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.health.status = NodeHealth::ERROR;
  hb.health.action = NodeHealth::ABORT;
  hb.heartbeat_interval_ms = 1000;
  mon.on_heartbeat(hb);

  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::ABORT);
}

// A heartbeat with sequence_number at uint64 max (wrap boundary) is processed without throwing.
TEST_F(EdgeCaseTest, SequenceNumberWrapNoCrash)
{
  HealthMonitor mon([this](int32_t a) {action_calls_.push_back(a);});

  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.sequence_number = std::numeric_limits<uint64_t>::max();
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  hb.heartbeat_interval_ms = 1000;
  EXPECT_NO_THROW(mon.on_heartbeat(hb));
}

// A heartbeat with heartbeat_interval_ms=0 is ignored for interval tracking, keeping
// the previously learned interval, and fires no spurious action.
TEST_F(EdgeCaseTest, ZeroIntervalUsesDefault)
{
  HealthMonitor mon([this](int32_t a) {action_calls_.push_back(a);});

  // First heartbeat establishes interval=500ms.
  {
    NodeHeartbeat hb;
    hb.node_name = "motor";
    hb.health.status = NodeHealth::HEALTHY;
    hb.health.action = NodeHealth::NONE;
    hb.heartbeat_interval_ms = 500;
    mon.on_heartbeat(hb);
  }
  // Second heartbeat with interval=0 should not overwrite 500ms.
  {
    NodeHeartbeat hb;
    hb.node_name = "motor";
    hb.health.status = NodeHealth::HEALTHY;
    hb.health.action = NodeHealth::NONE;
    hb.heartbeat_interval_ms = 0;
    mon.on_heartbeat(hb);
  }
  // No crash, no spurious action fired.
  EXPECT_TRUE(action_calls_.empty());
}

// Before any required node has reported, the gate is blocked; it opens once the first
// healthy heartbeat arrives for the required node.
TEST_F(EdgeCaseTest, GateBlockedUntilAllRequiredNodesReady)
{
  HealthMonitor mon([this](int32_t a) {action_calls_.push_back(a);});
  mon.register_required_node("motor");

  // Before any heartbeat: gate blocked.
  EXPECT_FALSE(mon.can_transition_from_stopped());

  // After first healthy heartbeat: gate opens.
  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  hb.heartbeat_interval_ms = 1000;
  mon.on_heartbeat(hb);

  EXPECT_TRUE(mon.can_transition_from_stopped());
}
// ============================================================================
// Alarm callback
// ============================================================================

class AlarmCallbackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    alarm_calls_.clear();
    action_calls_.clear();
    monitor_ = std::make_unique<HealthMonitor>(
      [this](int32_t a) {action_calls_.push_back(a);},
      0,
      [this](const AlarmEvent & ev) {alarm_calls_.push_back(ev);});
  }

  std::vector<AlarmEvent> alarm_calls_;
  std::vector<int32_t> action_calls_;
  std::unique_ptr<HealthMonitor> monitor_;

  void inject(const std::string & node, int32_t status, int32_t action,
    int32_t error_code = 0,
    const std::string & message = "",
    const std::string & instance_id = "")
  {
    NodeHeartbeat hb;
    hb.node_name = node;
    hb.health.status = status;
    hb.health.action = action;
    hb.health.error_code = error_code;
    hb.health.message = message;
    hb.health.instance_id = instance_id;
    hb.heartbeat_interval_ms = 1000;
    monitor_->on_heartbeat(hb);
  }
};

// The first ERROR heartbeat fires one alarm "raise" callback carrying the node name,
// error code, action, and message from the heartbeat.
TEST_F(AlarmCallbackTest, AlarmRaisedOnFirstError)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD, 101, "over-temp");
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_TRUE(alarm_calls_[0].trigger);
  EXPECT_EQ(alarm_calls_[0].node_name, "motor");
  EXPECT_EQ(alarm_calls_[0].error_code, 101);
  EXPECT_EQ(alarm_calls_[0].severity, NodeHealth::HOLD);
  EXPECT_EQ(alarm_calls_[0].message, "over-temp");
  EXPECT_FALSE(alarm_calls_[0].is_timeout);
}

// A WARN heartbeat fires the alarm "raise" callback (observer-only) but fires no SM action.
TEST_F(AlarmCallbackTest, AlarmRaisedForWarn)
{
  inject("motor", NodeHealth::DEGRADED, NodeHealth::WARN, 100);
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_TRUE(alarm_calls_[0].trigger);
  EXPECT_EQ(alarm_calls_[0].severity, NodeHealth::WARN);
  // WARN does not fire fire_action_ callback
  EXPECT_TRUE(action_calls_.empty());
}

// Returning to HEALTHY after an error fires an alarm "clear" callback (trigger=false)
// that preserves the original severity and error code (not reset to a neutral value).
TEST_F(AlarmCallbackTest, AlarmClearedOnHealthy)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD, 101);
  ASSERT_EQ(alarm_calls_.size(), 1u);

  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_FALSE(alarm_calls_[1].trigger);
  EXPECT_EQ(alarm_calls_[1].node_name, "motor");
  EXPECT_EQ(alarm_calls_[1].severity, NodeHealth::HOLD);    // preserved, not zeroed
  EXPECT_EQ(alarm_calls_[1].error_code, 101);               // preserved, not zeroed
  EXPECT_FALSE(alarm_calls_[1].is_timeout);                 // a node-reported alarm, not a timeout
}

// After one raise and one clear, further HEALTHY heartbeats fire no additional alarm callbacks.
TEST_F(AlarmCallbackTest, ClearNotRepeatedForHealthy)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD, 101);
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  ASSERT_EQ(alarm_calls_.size(), 2u);  // raised + cleared

  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  EXPECT_EQ(alarm_calls_.size(), 2u);  // no further callbacks
}

// An error that recurs after being cleared fires a fresh alarm "raise" callback (third event).
TEST_F(AlarmCallbackTest, SecondRaiseAfterClearFiresNewAlarm)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD, 101);
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD, 101);
  ASSERT_EQ(alarm_calls_.size(), 3u);
  EXPECT_TRUE(alarm_calls_[2].trigger);
}

// Repeating the same instanced fault (same action/code/instance) does not refire —
// mirrors RepeatedHoldFiresOnce, but with instance_id held constant.
TEST_F(AlarmCallbackTest, SameInstanceRepeatedDoesNotRefire)
{
  inject("gripper", NodeHealth::ERROR, NodeHealth::HOLD, 101, "vacuum lost", "cell_north");
  inject("gripper", NodeHealth::ERROR, NodeHealth::HOLD, 101, "vacuum lost", "cell_north");
  ASSERT_EQ(alarm_calls_.size(), 1u);
}

// The same action/error_code at a DIFFERENT instance (e.g. a second, simultaneous
// E-stop location) is a distinct fault and must refire, not be suppressed as a repeat.
TEST_F(AlarmCallbackTest, DifferentInstanceSameCodeRefires)
{
  inject("gripper", NodeHealth::ERROR, NodeHealth::HOLD, 101, "vacuum lost", "cell_north");
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_EQ(alarm_calls_[0].instance_id, "cell_north");

  inject("gripper", NodeHealth::ERROR, NodeHealth::HOLD, 101, "vacuum lost", "cell_south");
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_TRUE(alarm_calls_[1].trigger);
  EXPECT_EQ(alarm_calls_[1].instance_id, "cell_south");
}

// Clearing an instanced fault preserves its instance_id on the clear event, same as
// severity/error_code (AlarmClearedOnHealthy) — a consumer needs to know WHICH
// instance cleared.
TEST_F(AlarmCallbackTest, ClearPreservesInstanceId)
{
  inject("gripper", NodeHealth::ERROR, NodeHealth::HOLD, 101, "vacuum lost", "cell_north");
  inject("gripper", NodeHealth::HEALTHY, NodeHealth::NONE);
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_FALSE(alarm_calls_[1].trigger);
  EXPECT_EQ(alarm_calls_[1].instance_id, "cell_north");
}

// A heartbeat timeout is node-level, never instanced — is_timeout alarms always carry
// an empty instance_id regardless of what the node last reported.
TEST_F(HeartbeatTimeoutTest, TimeoutAlarmHasNoInstanceId)
{
  std::vector<AlarmEvent> alarms;
  auto mon = std::make_unique<HealthMonitor>(
    [](int32_t) {}, 0,
    [&alarms](const AlarmEvent & ev) {alarms.push_back(ev);});
  mon->register_required_node("motor", 3.0, /*startup_ms=*/1);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  mon->check_timeouts();
  ASSERT_EQ(alarms.size(), 1u);
  EXPECT_TRUE(alarms[0].is_timeout);
  EXPECT_TRUE(alarms[0].instance_id.empty());
}

// A single interleaved HEALTHY heartbeat (e.g. from a zombie node with the same name)
// causes the alarm to clear and then re-raise on the next ABORT heartbeat, producing a
// raise/clear/raise cycle. This documents the expected HealthMonitor behavior: it is
// the caller's responsibility to keep get_health_status() a pure getter so only one
// publisher sends heartbeats per node_name.
TEST_F(AlarmCallbackTest, InterleavedHealthyCausesRaiseClearRaiseCycle)
{
  // Simulates the zombie-node scenario:
  // publisher-A sends ABORT, publisher-B (zombie) sends HEALTHY, repeat.
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT, 200);
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_TRUE(alarm_calls_[0].trigger);   // raised

  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_FALSE(alarm_calls_[1].trigger);  // cleared by zombie

  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT, 200);
  ASSERT_EQ(alarm_calls_.size(), 3u);
  EXPECT_TRUE(alarm_calls_[2].trigger);   // re-raised

  // The SM action callback should have fired only ONCE (the first ABORT)
  // because fire_packml_action guards against ABORT while already ABORTED.
  // For this pure unit test the action is still called each time since there's
  // no state machine to check — just verify the alarm callback cycle.
  EXPECT_EQ(action_calls_.size(), 2u);  // fired on first ABORT and on re-escalation
}

// Escalating from HOLD to ABORT fires a second alarm "raise" callback reflecting the
// new ABORT action and updated error code.
TEST_F(AlarmCallbackTest, EscalationFiresUpdatedAlarm)
{
  inject("motor", NodeHealth::ERROR, NodeHealth::HOLD, 101);
  inject("motor", NodeHealth::ERROR, NodeHealth::ABORT, 200);
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_TRUE(alarm_calls_[1].trigger);
  EXPECT_EQ(alarm_calls_[1].severity, NodeHealth::ABORT);
  EXPECT_EQ(alarm_calls_[1].error_code, 200);
}

// After a timeout, an operator clear + re-arm re-fires ABORT (so the machine cannot
// leave STOPPED while a required node is dead) but does NOT re-raise the timeout alarm.
TEST_F(AlarmCallbackTest, RearmRefiresAbortWithoutDuplicateRaiseAlarm)
{
  // Required node with a very short timeout (1ms x 3 = 3ms).
  monitor_->register_required_node("motor", 3.0, /*startup_ms=*/1);

  // First timeout: ABORT fires + exactly one "raise" alarm.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  monitor_->check_timeouts();
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::ABORT);
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_TRUE(alarm_calls_[0].trigger);
  EXPECT_TRUE(alarm_calls_[0].is_timeout);

  // A repeat check while still timed out does nothing (already flagged).
  monitor_->check_timeouts();
  EXPECT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(alarm_calls_.size(), 1u);

  // Operator CLEAR -> re-arm; the node is still silent.
  monitor_->rearm_timed_out_nodes();
  monitor_->check_timeouts();

  // ABORT MUST re-fire (so the machine can't stay out of STOPPED while a
  // required node is dead)...
  ASSERT_EQ(action_calls_.size(), 2u);
  EXPECT_EQ(action_calls_[1], NodeHealth::ABORT);
  // ...but the alarm must NOT be re-raised with no clear between.
  EXPECT_EQ(alarm_calls_.size(), 1u);
}

// When a timed-out node resumes with a healthy heartbeat, the clear event for that
// alarm carries is_timeout=true — it clears what was a manager-synthesized timeout
// alarm, not a node-reported one, and a consumer shouldn't have to correlate back to
// the original raise event to know that.
TEST_F(AlarmCallbackTest, ClearAfterTimeoutCarriesIsTimeout)
{
  monitor_->register_required_node("motor", 3.0, /*startup_ms=*/1);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  monitor_->check_timeouts();
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_TRUE(alarm_calls_[0].trigger);
  EXPECT_TRUE(alarm_calls_[0].is_timeout);

  // Node resumes with a healthy heartbeat — clears the timeout alarm.
  inject("motor", NodeHealth::HEALTHY, NodeHealth::NONE);
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_FALSE(alarm_calls_[1].trigger);
  EXPECT_TRUE(alarm_calls_[1].is_timeout);
  EXPECT_EQ(alarm_calls_[1].severity, NodeHealth::ABORT);  // preserved from the timeout raise
}

// A HealthMonitor constructed without an on_alarm callback processes error and clear
// heartbeats without throwing.
TEST_F(EdgeCaseTest, NoAlarmCallbackDoesNotCrash)
{
  // HealthMonitor without on_alarm — should not crash
  HealthMonitor mon([this](int32_t a) {action_calls_.push_back(a);});  // no on_alarm

  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.health.status = NodeHealth::ERROR;
  hb.health.action = NodeHealth::HOLD;
  hb.heartbeat_interval_ms = 1000;
  EXPECT_NO_THROW(mon.on_heartbeat(hb));

  // Clear the fault — also should not crash without on_alarm
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  EXPECT_NO_THROW(mon.on_heartbeat(hb));
}

// ============================================================================
// Sequence-number checks
// ============================================================================

class SequenceCheckTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    action_calls_.clear();
    alarm_calls_.clear();
    monitor_ = std::make_unique<HealthMonitor>(
      [this](int32_t a) {action_calls_.push_back(a);},
      0,
      [this](const AlarmEvent & ev) {alarm_calls_.push_back(ev);});
  }

  std::vector<int32_t> action_calls_;
  std::vector<AlarmEvent> alarm_calls_;
  std::unique_ptr<HealthMonitor> monitor_;

  HealthMonitor::HeartbeatResult inject(
    const std::string & node, uint64_t seq, int32_t status, int32_t action,
    int32_t code = 0, uint32_t interval_ms = 1000)
  {
    NodeHeartbeat hb;
    hb.node_name = node;
    hb.sequence_number = seq;
    hb.health.status = status;
    hb.health.action = action;
    hb.health.error_code = code;
    hb.heartbeat_interval_ms = interval_ms;
    return monitor_->on_heartbeat(hb);
  }
};

// Heartbeats with strictly increasing sequence numbers are all ACCEPTED.
TEST_F(SequenceCheckTest, MonotonicAccepted)
{
  EXPECT_EQ(inject("motor", 1, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  EXPECT_EQ(inject("motor", 2, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  EXPECT_EQ(inject("motor", 3, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::ACCEPTED);
}

// A stale/duplicate heartbeat (seq <= last seen) is DROPPED_STALE and does not flap the
// alarm or fire an action, preserving the existing fault state.
TEST_F(SequenceCheckTest, StaleDroppedNoFlap)
{
  EXPECT_EQ(inject("motor", 5, NodeHealth::ERROR, NodeHealth::ABORT, 200),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  ASSERT_EQ(alarm_calls_.size(), 1u);
  ASSERT_EQ(action_calls_.size(), 1u);

  // Zombie publisher sends a stale HEALTHY heartbeat (seq 3 <= 5): dropped.
  EXPECT_EQ(inject("motor", 3, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::DROPPED_STALE);

  // Fault state preserved — no spurious clear, no extra action.
  EXPECT_EQ(alarm_calls_.size(), 1u);
  EXPECT_EQ(action_calls_.size(), 1u);
}

// A backward sequence number arriving after an observed absence gap (node silent past
// its timeout) is treated as a RESTART: accepted and re-baselined, after which normal
// processing resumes (a following HOLD fires the action).
TEST_F(SequenceCheckTest, RestartAfterGapDetected)
{
  EXPECT_EQ(inject("motor", 5000, NodeHealth::HEALTHY, NodeHealth::NONE, 0, /*interval=*/30),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  // Gap: stay silent past the timeout so the node counts as "absent".
  std::this_thread::sleep_for(150ms);
  // Returns with a reset counter (seq=2; seq=1 was lost on the wire) — restart.
  EXPECT_EQ(inject("motor", 2, NodeHealth::HEALTHY, NodeHealth::NONE, 0, 30),
    HealthMonitor::HeartbeatResult::RESTART);
  EXPECT_EQ(inject("motor", 3, NodeHealth::ERROR, NodeHealth::HOLD, 101, 30),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::HOLD);
}

// The deliberate fast-restart trade-off: a node that crashes and returns FASTER than
// its own timeout has no observed absence, so its reset sequence numbers are dropped
// as stale (indistinguishable from a zombie publisher). The drops do not refresh the
// liveness stamp, so the monitor then times the node out — ONE safe-direction ABORT —
// after which the observed absence makes the next backward sequence a RESTART and
// normal processing resumes.
TEST_F(SequenceCheckTest, FastRestartDropsUntilTimeoutThenRecovers)
{
  monitor_->register_required_node("motor", 3.0, /*startup_ms=*/10);

  // Live baseline: seq 50, advertised interval 10ms -> 30ms timeout.
  EXPECT_EQ(inject("motor", 50, NodeHealth::HEALTHY, NodeHealth::NONE, 0, /*interval=*/10),
    HealthMonitor::HeartbeatResult::ACCEPTED);

  // "Fast restart": the node is back within its own timeout window with a reset
  // counter. No absence was observed -> dropped, no action fired.
  EXPECT_EQ(inject("motor", 1, NodeHealth::HEALTHY, NodeHealth::NONE, 0, 10),
    HealthMonitor::HeartbeatResult::DROPPED_STALE);
  EXPECT_EQ(inject("motor", 2, NodeHealth::HEALTHY, NodeHealth::NONE, 0, 10),
    HealthMonitor::HeartbeatResult::DROPPED_STALE);
  EXPECT_TRUE(action_calls_.empty());

  // The drops did NOT refresh the liveness stamp, so the node times out: the one
  // safe-direction ABORT plus its timeout alarm.
  std::this_thread::sleep_for(50ms);
  monitor_->check_timeouts();
  ASSERT_EQ(action_calls_.size(), 1u);
  EXPECT_EQ(action_calls_[0], NodeHealth::ABORT);
  ASSERT_EQ(alarm_calls_.size(), 1u);
  EXPECT_TRUE(alarm_calls_[0].is_timeout);

  // With the absence observed, the (still backward) next sequence is recognized as a
  // RESTART, re-baselined, and clears the timeout alarm...
  EXPECT_EQ(inject("motor", 3, NodeHealth::HEALTHY, NodeHealth::NONE, 0, 10),
    HealthMonitor::HeartbeatResult::RESTART);
  ASSERT_EQ(alarm_calls_.size(), 2u);
  EXPECT_FALSE(alarm_calls_[1].trigger);
  EXPECT_TRUE(alarm_calls_[1].is_timeout);

  // ...and normal processing resumes: monotonic follow-ups accepted, gate open.
  EXPECT_EQ(inject("motor", 4, NodeHealth::HEALTHY, NodeHealth::NONE, 0, 10),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  EXPECT_TRUE(monitor_->can_transition_from_stopped());
}

// A large backward sequence jump with no observed gap (node still actively publishing)
// is NOT treated as a restart but DROPPED_STALE — jump magnitude alone must not be read
// as a restart.
TEST_F(SequenceCheckTest, BackwardSeqWithoutGapIsDropped)
{
  EXPECT_EQ(inject("motor", 5000, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  // Immediately (no gap) a far-lower seq arrives — looks like a "restart" by size,
  // but there was no observed absence, so it must be dropped.
  EXPECT_EQ(inject("motor", 2, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::DROPPED_STALE);
}

// Unsequenced heartbeats (seq == 0, legacy publishers) are always ACCEPTED and
// processed normally, so a HOLD still fires its action.
TEST_F(SequenceCheckTest, UnsequencedAlwaysAccepted)
{
  EXPECT_EQ(inject("motor", 0, NodeHealth::HEALTHY, NodeHealth::NONE),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  EXPECT_EQ(inject("motor", 0, NodeHealth::ERROR, NodeHealth::HOLD, 1),
    HealthMonitor::HeartbeatResult::ACCEPTED);
  EXPECT_EQ(action_calls_.size(), 1u);  // HOLD fired, not dropped as "0 <= 0"
}

// An advertised heartbeat interval is clamped to the configured maximum, so a node
// cannot dodge timeout detection by advertising a huge interval; the clamped value
// still produces a timeout that fires ABORT.
TEST_F(SequenceCheckTest, IntervalClampedToMaxEnablesTimeout)
{
  monitor_->set_max_expected_interval_ms(100);          // cap = 100ms
  // LARGE startup interval (5s) so a not-yet-clamped value can NOT produce a fast
  // timeout — only the clamped heartbeat interval can. This isolates the clamp:
  //   - clamp works:   advertised 60000 -> 100ms -> timeout 300ms -> fires in 500ms
  //   - clamp missing: interval stays 60000 -> timeout 180s -> does NOT fire
  //   - interval not learned: stays 5000 -> timeout 15s -> does NOT fire
  monitor_->register_required_node("motor", 3.0, 5000);

  NodeHeartbeat hb;
  hb.node_name = "motor";
  hb.sequence_number = 1;
  hb.heartbeat_interval_ms = 60000;                    // absurd advertised interval
  hb.health.status = NodeHealth::HEALTHY;
  hb.health.action = NodeHealth::NONE;
  monitor_->on_heartbeat(hb);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // > clamped timeout (300ms)
  monitor_->check_timeouts();
  ASSERT_FALSE(action_calls_.empty());
  EXPECT_EQ(action_calls_.back(), NodeHealth::ABORT);
}