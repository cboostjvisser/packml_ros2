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
// "Monkey" scenarios, health/heartbeat group: extends test_monkey_scenarios.cpp's chaos-style
// stress testing of the whole PackML manager/EM pair to the Equipment Module's HEALTH
// REPORTING itself being the unreliable party, rather than the operator or the manager's own
// command handling (which that file covers). Same house style: scenario-specific helper classes
// in this file's own anonymous namespace (generic scaffolding lives in test_helpers.hpp), a Rig
// struct + begin_setup()/finish_setup() pattern building a standalone manager (+ EM/publisher,
// where one is needed) per test with unique node names.
//
// Covers the "health/heartbeat chaos" group -- the EM's heartbeat/health reporting is the
// unreliable party here, not the operator or the manager's own command handling:
//   1. Rapid HEALTHY/ERROR flapping during EXECUTE -- alarms must stay bounded to the
//      heartbeat rate, not spam independently, and the machine must settle once it stops.
//   2. A required EM's heartbeat-sequence "restart" (see RequiredNodeRestartTriggersManagerAbort
//      in test_health_monitor_integration.cpp for exactly how that's faked) combined with an
//      operator mashing CLEAR/RESET with no delay -- new combined territory; each half is
//      already covered separately elsewhere, but not together.
//   3. Scrambled/reordered/duplicated heartbeat sequence numbers, via a plain rclcpp::Publisher
//      rather than the clean "goes silent, then comes back" RESTART pattern -- stresses
//      HealthMonitor::on_heartbeat()'s DROPPED_STALE/RESTART classification against
//      adversarial rather than clean input.
//   4. A wildly flapping advertised heartbeat_interval_ms (instead of one fixed value) --
//      proves HealthMonitor's anti-evasion clamp (set_max_expected_interval_ms()) actually
//      bounds the liveness timeout regardless of what a node claims on any given beat.
//
// All four are written as confirmed-safe assertions of the CURRENTLY-INTENDED behavior, based
// on reading health_monitor.hpp's implementation -- none targets an already-confirmed gap the
// way test_monkey_scenarios.cpp's RESET->ABORT->RESET test does. They have not been observed
// running yet (this suite is built, not wired/run, per this pass's own constraints); adjust per
// real behavior once built, per this suite's own convention of not silently weakening a red
// result if one of these turns out to expose a real gap instead.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_ros/ros_names.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/msg/node_heartbeat.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_settled_state;
using packml_ros_test::wait_for_state;

/// Fire-and-don't-wait: send a command and return immediately with the pending future,
/// without blocking for the response. Used by the restart+mash test below, where the whole
/// point is to not wait for one command to settle before sending the next.
std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr> send_state_change_async(
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client,
  int8_t command)
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = command;
  return client->async_send_request(req).future.share();
}

/// Equipment Module whose reported health flaps between HEALTHY and a configurable fault
/// action on alternating heartbeats while flapping is enabled. Alternates inside
/// get_health_status() itself (rather than via a second, independent timer racing the
/// heartbeat one) so the flap rate exactly tracks the node's own heartbeat_interval_ms
/// (set fast via a parameter, e.g. 60ms) with no extra synchronization needed.
class FlappingHealthEquipmentModule : public PackmlNodeInterface
{
public:
  FlappingHealthEquipmentModule(rclcpp::Node::SharedPtr node, int32_t fault_action)
  : fault_action_(fault_action)
  {
    init(node);
  }

  /// Turn flapping on/off. While off, always reports HEALTHY/NONE.
  void set_flapping(bool on) {flapping_.store(on);}

  packml_msgs::msg::NodeHealth get_health_status() override
  {
    packml_msgs::msg::NodeHealth h;
    if (!flapping_.load()) {
      h.status = packml_msgs::msg::NodeHealth::HEALTHY;
      h.action = packml_msgs::msg::NodeHealth::NONE;
      return h;
    }
    const auto tick = tick_.fetch_add(1);
    if (tick % 2 == 0) {
      h.status = packml_msgs::msg::NodeHealth::ERROR;
      h.action = fault_action_;
    } else {
      h.status = packml_msgs::msg::NodeHealth::HEALTHY;
      h.action = packml_msgs::msg::NodeHealth::NONE;
    }
    return h;
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}

private:
  int32_t fault_action_;
  std::atomic<bool> flapping_{false};
  std::atomic<uint64_t> tick_{0};
};

/// Minimal Equipment Module that re-exposes set_heartbeat_active() (protected on the base
/// class) so a test can silence it to simulate a crashed/restarting node -- mirrors
/// SimEquipmentModule in test_health_monitor_integration.cpp exactly.
class RestartableEquipmentModule : public PackmlNodeInterface
{
public:
  explicit RestartableEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  using PackmlNodeInterface::set_heartbeat_active;

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
};

/// Build a NodeHeartbeat message for direct publication by a plain rclcpp::Publisher, bypassing
/// PackmlNodeInterface's own heartbeat plumbing entirely -- used by the sequence-chaos and
/// interval-flapping scenarios below, which need to control sequence_number and
/// heartbeat_interval_ms directly rather than through a real node's HeartbeatState.
packml_msgs::msg::NodeHeartbeat make_heartbeat(
  const std::string & node_name, uint64_t seq, uint32_t interval_ms,
  int32_t status = packml_msgs::msg::NodeHealth::HEALTHY,
  int32_t action = packml_msgs::msg::NodeHealth::NONE)
{
  packml_msgs::msg::NodeHeartbeat hb;
  hb.node_name = node_name;
  hb.sequence_number = seq;
  hb.heartbeat_interval_ms = interval_ms;
  hb.health.status = status;
  hb.health.action = action;
  return hb;
}

}  // namespace

class MonkeyHealthTest : public ::testing::Test
{
protected:
  /// Every test builds its own standalone manager (+ EM/publisher), rather than sharing
  /// fixture state -- see test_monkey_scenarios.cpp's own Rig for the same rationale.
  struct Rig
  {
    std::string mgr_name;
    rclcpp::Node::SharedPtr mgr_node;
    std::unique_ptr<SMNode_new> sm_node;
    rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client;
    std::shared_ptr<packml_ros_test::SpinHelper> mgr_spin;
  };

  /// Builds the manager alone, with exactly one required node (`em_name`) registered for the
  /// health monitor. These scenarios are about heartbeat/health handling, not coordinated-
  /// completion fan-out, so node_names is deliberately left unset (empty) -- no action-based EM
  /// is registered for state-transition fan-out, which lets RESET/START/etc. complete
  /// immediately once the health gate allows it. A test that also needs a real
  /// PackmlNodeInterface-derived EM constructs it separately, on its own node, after this
  /// returns. `heartbeat_startup_grace_ms_override`, if >= 0, overrides the default 30000ms
  /// grace (see health_monitor.hpp's set_max_expected_interval_ms()) -- used by the
  /// interval-flapping test to get a small, quickly-observable liveness cap.
  Rig begin_setup(
    const std::string & mgr_prefix, const std::string & em_name,
    int heartbeat_startup_grace_ms_override = -1)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name);
    rig.mgr_node->declare_parameter(
      packml_ros::kParamRequiredNodes, std::vector<std::string>{em_name});
    if (heartbeat_startup_grace_ms_override >= 0) {
      rig.mgr_node->declare_parameter(
        packml_ros::kParamHeartbeatStartupGraceMs, heartbeat_startup_grace_ms_override);
    }
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/" + packml_ros::kChangeStateService);
    rig.mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.mgr_node);
    return rig;
  }

  void finish_setup(Rig & rig)
  {
    ASSERT_TRUE(rig.state_client->wait_for_service(5s));
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
    std::this_thread::sleep_for(300ms);
  }
};

// ============================================================================
// Confirmed-safe: HealthMonitor's alarm computation (compute_alarm() in health_monitor.hpp)
// fires at most once per accepted heartbeat -- rapid HEALTHY/ERROR flapping should therefore
// produce a BOUNDED number of alarms tied to the heartbeat rate, never an independent runaway
// storm, and repeatedly toggling the underlying PackML action (HOLD here) should settle the
// machine into a single recognized state rather than leaving it oscillating once the flapping
// stops and health returns to HEALTHY for good.
TEST_F(MonkeyHealthTest, MonkeyHealthFlapping_BoundedAlarmsAndMachineSettles)
{
  auto em_name = packml_ros_test::unique_node_name("flap_em");
  auto rig = begin_setup("monkey_health_flap", em_name);
  finish_setup(rig);

  auto em_node = rclcpp::Node::make_shared(em_name);
  // Fast heartbeat so alternating HEALTHY/ERROR every beat flaps roughly every 60ms, in the
  // "50-100ms" range this scenario is meant to reproduce.
  em_node->declare_parameter(packml_ros::kParamHeartbeatIntervalMs, 60);
  auto em = std::make_shared<FlappingHealthEquipmentModule>(
    em_node, packml_msgs::msg::NodeHealth::HOLD);
  packml_ros_test::SpinHelper em_spin(em_node);

  // Drive to EXECUTE first, with flapping OFF (em starts non-flapping/HEALTHY) so the health
  // gate opens cleanly without racing the toggle -- the same STOPPED->healthy->RESET->IDLE->
  // START->EXECUTE chain test_health_monitor_integration.cpp's fixture uses.
  std::this_thread::sleep_for(300ms);  // let a few healthy heartbeats land
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success) << "RESET blocked: " << reset_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 1s))
    << "machine did not reach IDLE before fault injection";

  auto start_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(start_resp, nullptr);
  ASSERT_TRUE(start_resp->success) << "START blocked: " << start_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::EXECUTE, 1s))
    << "machine did not reach EXECUTE before fault injection";

  // Subscribe to packml_alarms and wait for the subscription to actually match before flapping
  // starts, so no early alarm is lost to async discovery.
  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = rig.mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(200).reliable().transient_local(),
    [&alarms, &alarms_mutex, &em_name](packml_msgs::msg::Alarm::SharedPtr msg) {
      if (msg->node_name != em_name) {
        return;
      }
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u) << "Alarm subscription never matched";
  }

  // Flap rapidly (alternating every ~60ms heartbeat) for 2 seconds, then stop for good.
  static constexpr auto kFlapDuration = 2000ms;
  static constexpr int kHeartbeatIntervalMs = 60;
  em->set_flapping(true);
  std::this_thread::sleep_for(kFlapDuration);
  em->set_flapping(false);

  const size_t alarm_count = [&] {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      return alarms.size();
    }();

  // Upper bound: compute_alarm() fires at most once per accepted heartbeat -- with a heartbeat
  // roughly every 60ms over 2s, that's on the order of ~33 heartbeats; a generous 3x margin
  // catches any true runaway (e.g. firing per executor tick instead of per heartbeat) without
  // being sensitive to normal scheduling jitter.
  const size_t max_expected =
    3 * static_cast<size_t>(kFlapDuration.count() / kHeartbeatIntervalMs) + 10;
  EXPECT_GT(alarm_count, 0u) << "flapping produced no alarms at all -- detection may be broken";
  EXPECT_LE(alarm_count, max_expected)
    << "flapping produced " << alarm_count << " alarms, more than the heartbeat-rate-bounded "
    << max_expected << " expected -- possible unbounded alarm spam";

  // Once flapping stops (health reports HEALTHY and stays there), the machine must settle into
  // a single, stable, recognized state rather than continuing to oscillate.
  EXPECT_TRUE(wait_for_settled_state(rig.sm_node, 3s).has_value())
    << "machine did not settle into a stable state after flapping stopped, last seen: "
    << static_cast<int>(rig.sm_node->getCurrentState());
}

// ============================================================================
// Confirmed-safe (crash/hang-safety + eventual recoverability): combines two things already
// covered separately -- a required EM's heartbeat-sequence "restart" (see
// RequiredNodeRestartTriggersManagerAbort in test_health_monitor_integration.cpp) and an
// operator mashing recovery commands with no delay between sends (see MonkeyCommandFlood_...
// in test_monkey_scenarios.cpp) -- but not their combination, which is what this covers.
// Like that flood test, this deliberately does NOT assert a specific final state
// (which one it lands on is genuinely order/timing-dependent), only that nothing hangs and the
// manager stays responsive/recoverable afterward.
TEST_F(MonkeyHealthTest, MonkeyRestartFlappingWithOperatorMashing_SettlesRecoverable)
{
  auto em_name = packml_ros_test::unique_node_name("restart_mash_em");
  auto rig = begin_setup("monkey_restart_mash", em_name);
  finish_setup(rig);

  auto em_node = rclcpp::Node::make_shared(em_name);
  // Per-node timeout becomes 100ms x factor(3) = 300ms, above the manager's 200ms
  // check_timeouts() cadence, so a continuously-heartbeating EM never falsely times out.
  em_node->declare_parameter(packml_ros::kParamHeartbeatIntervalMs, 100);
  auto em = std::make_shared<RestartableEquipmentModule>(em_node);
  auto em_spin = std::make_shared<packml_ros_test::SpinHelper>(em_node);

  // Drive to EXECUTE first, cleanly.
  std::this_thread::sleep_for(300ms);  // let a few healthy heartbeats land
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success) << "RESET blocked: " << reset_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 1s))
    << "machine did not reach IDLE before fault injection";

  auto start_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(start_resp, nullptr);
  ASSERT_TRUE(start_resp->success) << "START blocked: " << start_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::EXECUTE, 1s))
    << "machine did not reach EXECUTE before fault injection";

  // --- The operator-mashing half: fire CLEAR/RESET back-to-back with NO delay at all, on a
  // separate thread so it genuinely overlaps the silence/restart sequence below rather than
  // running strictly before or after it (a tight, no-sleep loop of async sends typically
  // finishes in well under a millisecond of wall-clock time locally, so exact overlap with the
  // sequence-reset instant itself isn't guaranteed -- but it does overlap the silence window
  // this test observes the "restart" through, which is the chaotic combination this scenario is
  // after). mash_futures is only written by this thread and only read after masher.join(), so
  // no lock is needed for it.
  // A truly zero-delay loop can outrun the service's own QoS queue depth before the
  // single-threaded executor gets a chance to drain any of it -- see
  // MonkeyCommandFlood_ManagerSurvivesAndStaysResponsive's own comment in
  // test_monkey_scenarios.cpp for this exact, already-documented lesson. A tiny gap keeps this
  // "mashing," not a queue-overrun artifact unrelated to the state machine itself.
  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> mash_futures;
  std::thread masher([&]() {
      static constexpr int kMashCount = 20;
      for (int i = 0; i < kMashCount; ++i) {
        mash_futures.push_back(
          send_state_change_async(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR));
        std::this_thread::sleep_for(5ms);
        mash_futures.push_back(
          send_state_change_async(rig.state_client, packml_msgs::srv::StateChange::Request::RESET));
        std::this_thread::sleep_for(5ms);
      }
    });

  // --- The restart-flapping half: silence past the node's own timeout, then replace the EM
  // instance on the same underlying node so its HeartbeatState sequence counter restarts at 0
  // -- exactly RequiredNodeRestartTriggersManagerAbort's own recipe.
  em->set_heartbeat_active(false);
  std::this_thread::sleep_for(600ms);  // long enough to be observed absent (timeout 300ms)
  em.reset();
  em = std::make_shared<RestartableEquipmentModule>(em_node);
  std::this_thread::sleep_for(300ms);  // let the restart detection (and any tail mashing) land

  masher.join();

  // No mashed command may hang -- every one of them must get SOME response.
  for (size_t i = 0; i < mash_futures.size(); ++i) {
    ASSERT_EQ(mash_futures[i].wait_for(3s), std::future_status::ready)
      << "mashed command #" << i << " never got a response -- possible hang";
  }

  // The machine must settle into a stable, recognized state, not keep oscillating.
  EXPECT_TRUE(wait_for_settled_state(rig.sm_node, 3s).has_value())
    << "machine never settled after the restart+mash chaos, last seen: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // The manager must still be responsive to a clean command afterward -- demonstrating it
  // stayed recoverable rather than wedged (mirrors MonkeyCommandFlood's own final check).
  auto stop_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_resp, nullptr) << "manager stopped responding after the restart+mash chaos";
}

// ============================================================================
// Confirmed-safe: HealthMonitor::on_heartbeat()'s DROPPED_STALE/RESTART classification (see its
// own doc comment in health_monitor.hpp) must hold up against adversarial, scrambled input --
// not just the clean "goes silent, then comes back" pattern the existing RESTART test uses.
// Publishes raw NodeHeartbeat messages directly via a plain rclcpp::Publisher (bypassing
// PackmlNodeInterface entirely) so the test controls sequence_number precisely.
TEST_F(MonkeyHealthTest, MonkeyHeartbeatSequenceChaos_ConvergesAfterCleanRestart)
{
  auto em_name = packml_ros_test::unique_node_name("seq_chaos_em");
  auto rig = begin_setup("monkey_seq_chaos", em_name);
  finish_setup(rig);

  auto pub_node = rclcpp::Node::make_shared(packml_ros_test::unique_node_name("seq_chaos_pub"));
  auto publisher = pub_node->create_publisher<packml_msgs::msg::NodeHeartbeat>(
    "/" + em_name + "/" + std::string(packml_ros::kHeartbeatTopic), rclcpp::SensorDataQoS());
  {
    packml_ros_test::wait_until(
      [&] {return publisher->get_subscription_count() > 0;}, 2s, 10ms);
    ASSERT_GT(publisher->get_subscription_count(), 0u)
      << "heartbeat publisher never matched the manager's subscription";
  }

  // Scrambled/reordered/duplicated sequence numbers, all reporting HEALTHY -- deliberately NOT
  // the clean silent-then-comes-back RESTART pattern. Sent with no real time gap between them,
  // so none of these can be observed "absent" (see on_heartbeat()'s was_absent check) -- every
  // backward/duplicate one should be classified DROPPED_STALE, not RESTART, and none of it
  // should crash or wedge the manager.
  static constexpr uint64_t kChaosSeq[] = {10, 3, 10, 7, 2, 15, 14, 1, 20, 19, 18, 20};
  for (const auto seq : kChaosSeq) {
    publisher->publish(make_heartbeat(em_name, seq, 100));
    std::this_thread::sleep_for(5ms);
  }

  // The manager must still be alive and responsive after the chaotic burst.
  auto probe_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(probe_resp, nullptr) << "manager stopped responding after the sequence-chaos burst";

  // Clean, monotonically increasing, in-order heartbeats resume, above every chaotic value sent.
  for (uint64_t seq = 21; seq < 26; ++seq) {
    publisher->publish(make_heartbeat(em_name, seq, 100));
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(100ms);

  // Normalize back to STOPPED regardless of whether any chaotic entry was misclassified as a
  // RESTART along the way (which would have fired ABORT) -- CLEAR is a harmless no-op from
  // STOPPED and recovers to STOPPED from ABORTED either way, so the RESET below can only be
  // testing health convergence, not incidentally failing on an unrelated state mismatch.
  send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  std::this_thread::sleep_for(200ms);

  // The gate's own staleness check (gate_block_reason()) is a fresh, real-time comparison
  // against the LAST accepted heartbeat, independent of check_timeouts()'s periodic timer --
  // the 20ms-spaced burst above plus the settle sleeps since then already exceeds this node's
  // 300ms effective timeout (100ms interval x 3.0 factor) by the time RESET is evaluated below.
  // One more fresh heartbeat, immediately before the check, is what "clean heartbeats have
  // resumed" actually needs to mean here -- this is the same requirement
  // GateOpensAfterAllNodesHealthy satisfies by resuming continuous heartbeats, not a one-shot
  // burst followed by silence.
  publisher->publish(make_heartbeat(em_name, 26, 100));
  std::this_thread::sleep_for(20ms);

  // Health should converge to "sane" -- RESET succeeds once clean, healthy heartbeats have
  // resumed, exactly like GateOpensAfterAllNodesHealthy's own assertion.
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  EXPECT_TRUE(reset_resp->success)
    << "health did not converge to healthy after clean heartbeats resumed following sequence "
       "chaos: " << reset_resp->message;
}

// ============================================================================
// Confirmed-safe: HealthMonitor::set_max_expected_interval_ms() (see health_monitor.hpp) caps
// whatever heartbeat_interval_ms a node advertises, specifically so a node cannot evade its own
// liveness detection by claiming a huge interval. This test proves the cap actually holds under
// an EM whose advertised interval flaps wildly from beat to beat instead of staying fixed --
// culminating in a deliberately huge, evasive claim on its LAST beat before going silent.
TEST_F(MonkeyHealthTest, MonkeyHeartbeatIntervalFlapping_TimeoutStaysBounded)
{
  auto em_name = packml_ros_test::unique_node_name("interval_flap_em");
  // heartbeat_startup_grace_ms=1500 -> cap = 1500 / heartbeat_timeout_factor(3.0) = 500ms, so
  // the effective per-node timeout after the clamp is 500 * 3 = 1500ms -- small enough to
  // observe within a few seconds, regardless of what interval the node advertises.
  auto rig = begin_setup("monkey_interval_flap", em_name, /*heartbeat_startup_grace_ms_override=*/1500);
  finish_setup(rig);

  auto pub_node = rclcpp::Node::make_shared(packml_ros_test::unique_node_name("interval_flap_pub"));
  auto publisher = pub_node->create_publisher<packml_msgs::msg::NodeHeartbeat>(
    "/" + em_name + "/" + std::string(packml_ros::kHeartbeatTopic), rclcpp::SensorDataQoS());
  {
    packml_ros_test::wait_until(
      [&] {return publisher->get_subscription_count() > 0;}, 2s, 10ms);
    ASSERT_GT(publisher->get_subscription_count(), 0u)
      << "heartbeat publisher never matched the manager's subscription";
  }

  // Wildly different heartbeat_interval_ms on every beat -- some far below the cap, some far
  // above/near it -- with the LAST one a deliberately huge, evasive claim right before silence.
  static constexpr uint32_t kFlappingIntervals[] = {50, 2000000, 10, 9999999, 5, 5000000};
  uint64_t seq = 1;
  for (const auto interval : kFlappingIntervals) {
    publisher->publish(make_heartbeat(em_name, seq++, interval));
    std::this_thread::sleep_for(80ms);
  }

  // Go silent. If the clamp did NOT hold (i.e. the last, huge claimed interval were trusted
  // literally), the effective timeout would be enormous and RESET would incorrectly still
  // succeed here; the clamp caps it at 1500ms, so waiting comfortably past that must leave the
  // node timed out.
  std::this_thread::sleep_for(2000ms);

  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  EXPECT_FALSE(reset_resp->success)
    << "RESET succeeded despite 2s of silence after a node advertising a huge "
       "heartbeat_interval_ms -- the anti-evasion clamp did not bound its liveness timeout";

  // The manager must still be alive and responsive after the whole flapping+silence sequence.
  auto stop_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_resp, nullptr) << "manager stopped responding after interval-flapping chaos";
}
