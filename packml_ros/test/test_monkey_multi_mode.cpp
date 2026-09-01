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
// "Monkey" scenarios, multi-EM / mode / operator-side group: extends test_monkey_scenarios.cpp
// into mechanisms that file doesn't touch -- several Equipment Modules registered together
// (mixed speeds, cycled repeatedly), the MODE-transition fan-out path (a separate path from
// state, with no monkey coverage at all before this file), a genuinely concurrent (not just
// fast sequential) multi-command burst from several threads, an operator client that vanishes
// mid-request, and a mode-change flood mirroring the existing state-change flood test. Same
// house style as test_monkey_scenarios.cpp: scenario-specific helper classes in this file's own
// anonymous namespace (generic scaffolding lives in test_helpers.hpp), a Rig struct +
// begin_setup()/finish_setup()
// pattern (generalized here to a variable-size EM roster, since several of these scenarios need
// more than one registered child), a wait_for_state() polling helper, and every test builds its
// own standalone manager + EM set with unique node names rather than sharing fixture state.
//
// All tests here are "confirmed safe" expectations, not known-gap regression markers like
// test_monkey_scenarios.cpp's MonkeyResetAbortReset_StaleReportResolvesLaterResetTooEarly: none
// of the mechanisms exercised below are known, ahead of time, to be broken -- they push on
// mechanisms (CompletionTracker's per-round bookkeeping, the mode fan-out's own
// report_fanout_failure() path, the manager's single-threaded executor, service-client
// lifecycle) that already have direct, passing single-EM/sequential precedent elsewhere in this
// test suite, just exercised more chaotically (more EMs, genuine thread concurrency, a
// vanishing caller). Every assertion below states the intended CORRECT behavior, same as every
// other test in this suite -- if the real build surfaces a genuine gap here, that warrants a
// comment update (and likely a KNOWN GAP rewrite), not silently loosening the assertion.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "packml_sm/default_modes.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// Plain, non-deferring EM -- completes every coordinated state instantly. Duplicated from
/// test_monkey_scenarios.cpp per this test/ directory's own convention (no shared helper
/// header): used here wherever a scenario needs a "well-behaved" EM alongside the actual
/// monkey, so the monkey's own misbehavior is the only variable under test.
class PlainEquipmentModule : public PackmlNodeInterface
{
public:
  explicit PlainEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
};

/// Fast, non-deferring EM (behaves like PlainEquipmentModule) that additionally records every
/// state it is asked to transition to, in order -- used by the mixed-speed cycling scenario
/// below to check each fast EM's own observed sequence stays sane (no state fanned out to it
/// an unexpected number of times) across many repeated RESET/ABORT/CLEAR cycles.
class TrackingEquipmentModule : public PackmlNodeInterface
{
public:
  explicit TrackingEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  std::vector<packml_sm::State> received_states() const
  {
    std::lock_guard<std::mutex> lk(states_mutex_);
    return states_;
  }

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    std::lock_guard<std::mutex> lk(states_mutex_);
    states_.push_back(state);
    return true;
  }
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}

private:
  mutable std::mutex states_mutex_;
  std::vector<packml_sm::State> states_;
};

/// EM whose work for a given coordinated `state` is real background work decoupled from the
/// ROS goal's own cancellation, reporting completion whenever the background thread finishes.
/// Duplicated verbatim (same shape/semantics) from test_monkey_scenarios.cpp's own
/// MonkeyStaleWorkEquipmentModule -- used here as the "slow monkey" among otherwise-fast EMs.
class MonkeyStaleWorkEquipmentModule : public PackmlNodeInterface
{
public:
  MonkeyStaleWorkEquipmentModule(
    rclcpp::Node::SharedPtr node, packml_sm::State monkey_state,
    std::chrono::milliseconds work_duration = 1500ms)
  : monkey_state_(monkey_state), work_duration_(work_duration)
  {
    init(node);
  }

  std::atomic<int> work_started{0};
  std::atomic<int> work_finished{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == monkey_state_;}
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    work_started.fetch_add(1);
    std::thread([this, completion]() {
        std::this_thread::sleep_for(work_duration_);
        work_finished.fetch_add(1);
        completion.report(true);
      }).detach();
  }

private:
  packml_sm::State monkey_state_;
  std::chrono::milliseconds work_duration_;
};

/// EM whose ~/packml_mode_transition service handler is unreliable: optionally sleeps before
/// responding (simulating a node barely keeping up) and/or rejects outright. Its
/// state-transition handler accepts and completes goals immediately, isolating the
/// mode-transition failure behavior.
class ModeMonkeyEquipmentModule : public PackmlNodeInterface
{
public:
  explicit ModeMonkeyEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  std::chrono::milliseconds mode_delay{0};
  std::atomic<bool> reject_mode{false};
  std::atomic<int> mode_requests_seen{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override
  {
    mode_requests_seen.fetch_add(1);
    if (mode_delay.count() > 0) {
      std::this_thread::sleep_for(mode_delay);
    }
    return !reject_mode.load();
  }
  void on_status_changed() override {}
};

}  // namespace

class MonkeyMultiModeTest : public ::testing::Test
{
protected:
  /// Generalizes test_monkey_scenarios.cpp's own single-EM Rig to a variable-size roster of
  /// Equipment Modules -- several of this file's own scenarios (mixed-speed multi-EM cycling,
  /// mode-transition fan-out with a normal EM alongside a monkey one) need more than one child
  /// registered against the same manager, which the original single-EM Rig has no room for.
  /// Every test still builds its own standalone manager + EM set (unique node names), never a
  /// shared fixture instance.
  struct Rig
  {
    std::string mgr_name;
    rclcpp::Node::SharedPtr mgr_node;
    std::unique_ptr<SMNode_new> sm_node;
    std::vector<rclcpp::Node::SharedPtr> em_nodes;
    rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client;
    std::shared_ptr<packml_ros_test::SpinHelper> mgr_spin;
    std::vector<std::shared_ptr<packml_ros_test::SpinHelper>> em_spins;
  };

  /// Constructs the manager and every EM's own bare node, and gets the manager spinning, but
  /// does NOT construct the EMs themselves or start their spinners -- callers need different EM
  /// subclasses per node, so they construct each one on rig.em_nodes[i] themselves (in the same
  /// order as `em_names`), then call finish_setup().
  Rig begin_setup(
    const std::string & mgr_prefix, const std::vector<std::string> & em_names,
    int state_complete_timeout_ms = 5000)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("node_names", em_names),
        rclcpp::Parameter("state_complete_timeout_ms", state_complete_timeout_ms),
      }));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    for (const auto & name : em_names) {
      rig.em_nodes.push_back(rclcpp::Node::make_shared(name));
    }
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/changeState");
    rig.mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.mgr_node);
    return rig;
  }

  /// Starts every EM node's own spinner (each EM must already be constructed on its
  /// rig.em_nodes[i] by the caller) and drives the machine to STOPPED.
  void finish_setup(Rig & rig)
  {
    for (const auto & em_node : rig.em_nodes) {
      rig.em_spins.push_back(std::make_shared<packml_ros_test::SpinHelper>(em_node));
    }
    ASSERT_TRUE(rig.state_client->wait_for_service(5s));
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
    std::this_thread::sleep_for(300ms);
  }
};

// ============================================================================
// Confirmed-safe: two fast, non-deferring EMs plus one "monkey" that defers RESETTING with a
// real background-thread delay (same shape as MonkeyStaleWorkEquipmentModule in
// test_completion_integration.cpp / test_monkey_scenarios.cpp), all registered together and
// driven through several full RESET -> ABORT -> CLEAR cycles in a row -- always letting each
// cycle's coordinated wait resolve normally before starting the next (no interruption; that
// specific race is the separate, already KNOWN-GAP scenario in test_monkey_scenarios.cpp).
// CompletionTracker::begin_round() resets its whole per-round map on every fan-out, keyed by
// the just-accepted goal id -- with no interruption racing a fresh round's registration, this
// is expected to hold up cleanly across many rounds: each fast EM's own count of RESETTING/
// ABORTING fan-outs it has seen must track the cycle count exactly (never duplicated, never
// skipped, never leaking an extra one from a neighboring round), and the monkey EM's own
// work-started/work-finished counters must advance exactly once per cycle.
TEST_F(MonkeyMultiModeTest, MixedSpeedEMsRepeatedCycling_NoCrossRoundContamination)
{
  static const std::string kFastA = "mixed_speed_fast_a";
  static const std::string kFastB = "mixed_speed_fast_b";
  static const std::string kMonkey = "mixed_speed_monkey";
  auto rig = begin_setup("monkey_mixed_speed", {kFastA, kFastB, kMonkey});

  auto fast_a = std::make_shared<TrackingEquipmentModule>(rig.em_nodes[0]);
  auto fast_b = std::make_shared<TrackingEquipmentModule>(rig.em_nodes[1]);
  auto monkey_em = std::make_shared<MonkeyStaleWorkEquipmentModule>(
    rig.em_nodes[2], packml_sm::State::RESETTING, 200ms);
  finish_setup(rig);

  static constexpr int kCycles = 3;
  for (int cycle = 1; cycle <= kCycles; ++cycle) {
    auto reset_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
    ASSERT_NE(reset_resp, nullptr) << "cycle " << cycle << ": RESET got no response";
    ASSERT_TRUE(reset_resp->success) << "cycle " << cycle << ": RESET rejected";

    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
      << "cycle " << cycle << ": machine did not enter RESETTING";

    // Wait for THIS cycle's own background work to actually start before trusting the
    // counters below (the fan-out reaching the EM is itself asynchronous).
    {
      packml_ros_test::wait_until(
        [&] {return monkey_em->work_started.load() >= cycle;}, 500ms, 10ms);
    }
    ASSERT_EQ(monkey_em->work_started.load(), cycle)
      << "cycle " << cycle << ": monkey EM's background work did not start exactly once "
         "for this round";

    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 1s))
      << "cycle " << cycle << ": machine did not reach IDLE after the monkey's own report";
    EXPECT_EQ(monkey_em->work_finished.load(), cycle)
      << "cycle " << cycle << ": IDLE was reached before this cycle's own background work "
         "finished -- a stale earlier-round report must have resolved it instead";

    auto abort_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
    ASSERT_NE(abort_resp, nullptr) << "cycle " << cycle << ": ABORT got no response";
    ASSERT_TRUE(abort_resp->success) << "cycle " << cycle << ": ABORT rejected";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1s))
      << "cycle " << cycle << ": machine did not reach ABORTED";

    auto clear_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
    ASSERT_NE(clear_resp, nullptr) << "cycle " << cycle << ": CLEAR got no response";
    ASSERT_TRUE(clear_resp->success) << "cycle " << cycle << ": CLEAR rejected";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 1s))
      << "cycle " << cycle << ": machine did not reach STOPPED";

    // Neither fast EM's own view of "how many times has RESETTING/ABORTING happened so far"
    // may run ahead of or behind the cycle count -- a mismatch would mean a round's fan-out
    // was sent twice, was skipped, or a stale result let the machine (and thus the next
    // fan-out) race ahead of where this test's own bookkeeping expects it to be.
    for (const auto & fast : {fast_a, fast_b}) {
      const auto states = fast->received_states();
      const auto resetting_count =
        std::count(states.begin(), states.end(), packml_sm::State::RESETTING);
      const auto aborting_count =
        std::count(states.begin(), states.end(), packml_sm::State::ABORTING);
      EXPECT_EQ(resetting_count, cycle)
        << "cycle " << cycle << ": a fast EM saw " << resetting_count
        << " RESETTING fan-out(s) so far, expected exactly " << cycle;
      EXPECT_EQ(aborting_count, cycle)
        << "cycle " << cycle << ": a fast EM saw " << aborting_count
        << " ABORTING fan-out(s) so far, expected exactly " << cycle;
    }
  }

  // Local variables destruct in reverse declaration order: fast_a/fast_b/monkey_em (declared
  // after rig) would otherwise be torn down BEFORE rig's own em_spins stop spinning, letting a
  // still-in-flight fan-out callback call into an object whose vtable is already gone ("pure
  // virtual method called") -- confirmed for real via gdb in ClientDisconnectMidRequest below.
  // Stopping the spinners first, explicitly, removes the race instead of just outrunning it.
  rig.em_spins.clear();
}

// ============================================================================
// Confirmed-safe: a registered EM whose mode-transition service handler is slow enough to blow
// through the mode fan-out's own hardcoded 5s deadline (mirrors
// ManagerClientFanoutTest::UnresponsiveChildModeChangeDoesNotBlockService's own template in
// test_manager_client_fanout.cpp, but registered alongside a second, normally-behaving EM) must
// not make ~/changeMode itself hang: the response comes back promptly (the state machine
// decides acceptance independent of any child's own ack), and the silent child is reported
// out-of-band as a WARN Alarm once the deadline actually expires -- the same
// report_fanout_failure() path already covered for a single EM, exercised here for the first
// time alongside a second, well-behaved registered node.
TEST_F(MonkeyMultiModeTest, ModeMonkeySlowEM_DoesNotBlockChangeModeAndReportsFanoutFailure)
{
  static const std::string kNormal = "mode_monkey_normal_em";
  static const std::string kSlow = "mode_monkey_slow_em";
  auto rig = begin_setup("monkey_mode_slow", {kNormal, kSlow});

  auto normal_em = std::make_shared<PlainEquipmentModule>(rig.em_nodes[0]);
  auto slow_em = std::make_shared<ModeMonkeyEquipmentModule>(rig.em_nodes[1]);
  // Deliberately past the fan-out's own hardcoded 5s deadline (see
  // PackmlManagerInterface::fanout_transition_to_clients) -- anything shorter would let the
  // child eventually acknowledge instead of ever being flagged as unresponsive, same
  // reasoning as the existing single-EM test this mirrors.
  slow_em->mode_delay = 6s;
  finish_setup(rig);

  // The state machine starts in STOPPED, which permits runtime mode changes.
  auto mode_client = rig.mgr_node->create_client<packml_msgs::srv::ModeChange>(
    rig.mgr_name + "/changeMode");
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  // Subscribe (and wait for it to actually match the manager's publisher) BEFORE acting, so
  // the fan-out-failure alarm this test waits for below cannot be lost to a race.
  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = rig.mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(50).reliable().transient_local(),
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
  req->mode.val = static_cast<int8_t>(packml_modes::Maintenance);

  const auto start = std::chrono::steady_clock::now();
  auto future = mode_client->async_send_request(req);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready)
    << "changeMode blocked on the slow mode-monkey EM -- fan-out is supposed to be asynchronous";
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_TRUE(future.get()->success);
  EXPECT_LT(elapsed, 1s)
    << "changeMode took suspiciously long given the fan-out is meant to be non-blocking";

  const auto deadline = std::chrono::steady_clock::now() + 8s;
  bool found_alarm = false;
  while (!found_alarm && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      for (const auto & a : alarms) {
        if (a.node_name == kSlow && a.trigger &&
          a.severity == packml_msgs::msg::Alarm::WARN &&
          a.message.find("mode") != std::string::npos &&
          a.message.find("no response within") != std::string::npos)
        {
          found_alarm = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found_alarm)
    << "no WARN alarm reported the slow mode-monkey EM's missing acknowledgement";

  // Manager must still be normally responsive to a follow-up mode change afterward.
  auto follow_up = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  follow_up->mode.val = static_cast<int8_t>(packml_modes::Production);
  auto follow_up_future = mode_client->async_send_request(follow_up);
  ASSERT_EQ(follow_up_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(follow_up_future.get()->success);

  // See ModeChangeFlood_ManagerSurvivesAndStaysResponsive's own comment (this file):
  // on_change_mode() spawns a detached, unjoined thread per
  // request to do the fan-out/wait/publish_status() work, with no shutdown synchronization
  // against rig's own destruction. This test's own long waits above make it unlikely (not
  // impossible) that thread has finished by the time this function returns -- a generous,
  // deliberate margin here, not a fix for the underlying gap.
  std::this_thread::sleep_for(500ms);
  // See MixedSpeedEMsRepeatedCycling's own comment above: stop em_spins before normal_em/
  // slow_em (declared after rig) destruct.
  rig.em_spins.clear();
}

// ============================================================================
// Confirmed-safe: same shape as ModeMonkeySlowEM above, but the mode-monkey EM answers
// IMMEDIATELY with an outright rejection instead of staying silent -- exercises
// try_send_to_client<T>()'s OTHER failure path (an explicit response->success == false,
// reported via report_fanout_failure() the moment the response callback runs, not only after
// the fan-out's own 5s deadline expires).
TEST_F(MonkeyMultiModeTest, ModeMonkeyRejectingEM_DoesNotBlockChangeModeAndReportsFanoutFailure)
{
  static const std::string kNormal = "mode_monkey_reject_normal_em";
  static const std::string kReject = "mode_monkey_reject_em";
  auto rig = begin_setup("monkey_mode_reject", {kNormal, kReject});

  auto normal_em = std::make_shared<PlainEquipmentModule>(rig.em_nodes[0]);
  auto reject_em = std::make_shared<ModeMonkeyEquipmentModule>(rig.em_nodes[1]);
  reject_em->reject_mode.store(true);
  finish_setup(rig);

  // The state machine starts in STOPPED, which permits runtime mode changes.
  auto mode_client = rig.mgr_node->create_client<packml_msgs::srv::ModeChange>(
    rig.mgr_name + "/changeMode");
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = rig.mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(50).reliable().transient_local(),
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
  req->mode.val = static_cast<int8_t>(packml_modes::Manual);
  auto future = mode_client->async_send_request(req);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success)
    << "the manager's own changeMode response reports SM acceptance, independent of any "
       "child's rejection";

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  bool found_alarm = false;
  while (!found_alarm && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      for (const auto & a : alarms) {
        if (a.node_name == kReject && a.trigger &&
          a.severity == packml_msgs::msg::Alarm::WARN &&
          a.message.find("mode") != std::string::npos &&
          a.message.find("rejected") != std::string::npos)
        {
          found_alarm = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(found_alarm)
    << "no WARN alarm reported the mode-monkey EM's outright rejection";
  EXPECT_EQ(reject_em->mode_requests_seen.load(), 1);

  // Manager must still be normally responsive to a follow-up mode change afterward.
  auto follow_up = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  follow_up->mode.val = static_cast<int8_t>(packml_modes::Production);
  auto follow_up_future = mode_client->async_send_request(follow_up);
  ASSERT_EQ(follow_up_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(follow_up_future.get()->success);

  // Unlike ModeMonkeySlowEM above, nothing else in this test rides out a long wait -- both mode
  // changes above resolve almost immediately, so their own on_change_mode() detached threads
  // (see that test's comment) are very plausibly still running
  // when this function would otherwise return. Confirmed the hard way: this exact test crashed
  // the whole shared binary ("pure virtual method called") under full-suite load before this
  // margin was added. A generous, deliberate wait, not a fix for the underlying gap.
  std::this_thread::sleep_for(2500ms);
  // See MixedSpeedEMsRepeatedCycling's own comment above: stop em_spins before normal_em/
  // slow_em (declared after rig) destruct.
  rig.em_spins.clear();
}

// ============================================================================
// Confirmed-safe: unlike MonkeyCommandFlood_ManagerSurvivesAndStaysResponsive in
// test_monkey_scenarios.cpp (sequential sends with a deliberate 5ms gap between them), this
// fires three different interrupt-class commands from three SEPARATE std::thread objects, each
// using its own pre-created service client, with NO coordinated delay between the threads
// starting at all -- genuine concurrent sends racing the manager's single-threaded executor,
// not just fast sequential ones. Does not assert which command "wins" (that's a real race, not
// the point); asserts only that nothing crashes, every request eventually gets a response, and
// the machine settles into one recognized final state.
TEST_F(MonkeyMultiModeTest, TrueConcurrentCommandBurst_NoCrashAndSettlesToRecognizedState)
{
  auto rig = begin_setup("monkey_concurrent_burst", {"concurrent_burst_em"});
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_nodes[0]);
  finish_setup(rig);

  // Drive to EXECUTE first so HOLD/ABORT/SUSPEND are all meaningful interrupt-class commands
  // rather than mostly-rejected no-ops from STOPPED.
  auto reset_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 1s));
  auto start_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(start_resp, nullptr);
  ASSERT_TRUE(start_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::EXECUTE, 1s));

  static constexpr int8_t kCommands[] = {
    packml_msgs::srv::StateChange::Request::HOLD,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::SUSPEND,
  };
  static constexpr size_t kNumCommands = sizeof(kCommands) / sizeof(kCommands[0]);

  // Pre-create every client (each thread gets its own, per the assignment's own guidance) on
  // the main thread, BEFORE spawning the threads that use them, so client construction itself
  // is never part of the concurrent race under test.
  std::vector<rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr> clients;
  for (size_t i = 0; i < kNumCommands; ++i) {
    clients.push_back(rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/changeState"));
  }

  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> futures(
    kNumCommands);
  std::vector<std::thread> threads;
  for (size_t i = 0; i < kNumCommands; ++i) {
    threads.emplace_back(
      [&clients, &futures, i]() {
        auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
        req->command = kCommands[i];
        // Each thread writes only its own element -- no shared mutable state to guard.
        futures[i] = clients[i]->async_send_request(req).future.share();
      });
  }
  for (auto & t : threads) {
    t.join();
  }

  for (size_t i = 0; i < kNumCommands; ++i) {
    ASSERT_EQ(futures[i].wait_for(5s), std::future_status::ready)
      << "concurrent command #" << i << " (cmd=" << static_cast<int>(kCommands[i])
      << ") never got a response -- possible hang";
  }

  // The machine must settle into SOME stable state shortly after the burst (mirrors
  // MonkeyCommandFlood_ManagerSurvivesAndStaysResponsive's own settle-detection loop).
  const auto settled = packml_ros_test::wait_for_settled_state(rig.sm_node, 3s, 20);
  ASSERT_TRUE(settled.has_value())
    << "machine never settled into a stable state within 3s of the concurrent burst, currently: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  static constexpr packml_sm::State kRecognizedFinalStates[] = {
    packml_sm::State::HOLDING, packml_sm::State::HELD,
    packml_sm::State::SUSPENDING, packml_sm::State::SUSPENDED,
    packml_sm::State::ABORTING, packml_sm::State::ABORTED,
  };
  bool recognized = false;
  for (const auto s : kRecognizedFinalStates) {
    if (s == *settled) {
      recognized = true;
      break;
    }
  }
  EXPECT_TRUE(recognized)
    << "machine settled into an unrecognized state after the concurrent burst: "
    << static_cast<int>(*settled);

  // Manager must still be normally responsive to a clean command after the burst (does not
  // assert success -- CLEAR may legitimately be rejected depending on which state the race
  // above settled into; only that a real response comes back, not a hang).
  auto clear_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr) << "manager stopped responding after the concurrent burst";

  // See MixedSpeedEMsRepeatedCycling's own comment above: stop em_spins before em (declared
  // after rig) destructs.
  rig.em_spins.clear();
}

// ============================================================================
// Confirmed-safe: a ~/changeState caller that vanishes (its client and node destroyed) before
// ever pulling on the response future -- simulating an operator UI process crashing or being
// killed mid-request -- must not crash or wedge the manager. The request is fired over the
// wire and its caller's own executor is stopped (then the client and node destroyed) before any
// response could plausibly be pulled; a FRESH, unrelated client must still get a normal
// response afterward.
TEST_F(MonkeyMultiModeTest, ClientDisconnectMidRequest_ManagerStaysResponsiveAfterward)
{
  auto rig = begin_setup("monkey_client_disconnect", {"client_disconnect_em"});
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_nodes[0]);
  finish_setup(rig);

  {
    auto client_node = rclcpp::Node::make_shared(
      packml_ros_test::unique_node_name("monkey_disconnect_client"));
    auto disposable_client = client_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/changeState");
    {
      packml_ros_test::SpinHelper client_spin(client_node);
      ASSERT_TRUE(disposable_client->wait_for_service(2s));

      auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
      req->command = packml_msgs::srv::StateChange::Request::RESET;
      auto pending = disposable_client->async_send_request(req);
      (void)pending;
      // client_spin goes out of scope HERE, stopping this caller's own executor before its
      // response could plausibly be pulled -- exactly the scenario under test: the manager may
      // still be mid-flight on this request when its caller effectively vanishes.
    }
    disposable_client.reset();
    client_node.reset();
  }

  // Give the manager time to actually process (and try, and fail, to deliver a response to)
  // the now-orphaned request.
  std::this_thread::sleep_for(300ms);

  // A FRESH client must still get a normal response -- the manager must not have crashed or
  // wedged itself trying to answer a caller that no longer exists.
  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(resp, nullptr)
    << "manager stopped responding to a fresh client after an earlier caller disconnected "
       "mid-request";

  // See MixedSpeedEMsRepeatedCycling's own comment above: stop em_spins before em (declared
  // after rig) destructs. This exact test is where the race was first confirmed via gdb: the
  // fresh ABORT above fans out to em just as this function is about to return, and em (destructs
  // before rig on the way out) was torn down mid-callback -- "pure virtual method called" inside
  // PackmlNodeInterface::begin_transition(), caught with a live backtrace under full-suite load.
  rig.em_spins.clear();
}

// ============================================================================
// Send mode-change requests with different values back-to-back from STOPPED without
// waiting between sends. Every request resolves, and the manager accepts a follow-up
// mode change after the flood.
TEST_F(MonkeyMultiModeTest, ModeChangeFlood_ManagerSurvivesAndStaysResponsive)
{
  auto rig = begin_setup("monkey_mode_flood", {"mode_flood_em"});
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_nodes[0]);
  finish_setup(rig);

  // The state machine starts in STOPPED, which permits the mode-change flood.
  auto mode_client = rig.mgr_node->create_client<packml_msgs::srv::ModeChange>(
    rig.mgr_name + "/changeMode");
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  static constexpr int8_t kModeFlood[] = {
    packml_modes::Production, packml_modes::Maintenance, packml_modes::Manual,
    packml_modes::Production, packml_modes::Maintenance, packml_modes::Manual,
    packml_modes::Production, packml_modes::Maintenance,
  };

  std::vector<std::shared_future<packml_msgs::srv::ModeChange::Response::SharedPtr>> pending;
  for (const auto mode_val : kModeFlood) {
    auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
    req->mode.val = mode_val;
    pending.push_back(mode_client->async_send_request(req).future.share());
    // A small, non-zero gap -- see MonkeyCommandFlood_ManagerSurvivesAndStaysResponsive's own
    // comment for why a true zero-delay send loop exercises the service queue more than the
    // state machine.
    std::this_thread::sleep_for(5ms);
  }

  for (size_t i = 0; i < pending.size(); ++i) {
    ASSERT_EQ(pending[i].wait_for(5s), std::future_status::ready)
      << "mode flood command #" << i << " (mode=" << static_cast<int>(kModeFlood[i])
      << ") never got a response -- possible hang";
  }

  std::this_thread::sleep_for(300ms);

  // Manager must still be normally responsive to a clean mode-change request after the flood.
  auto follow_up = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  follow_up->mode.val = static_cast<int8_t>(packml_modes::Production);
  auto follow_up_future = mode_client->async_send_request(follow_up);
  ASSERT_EQ(follow_up_future.wait_for(5s), std::future_status::ready)
    << "manager stopped responding to mode changes after the flood";
  EXPECT_TRUE(follow_up_future.get()->success)
    << "manager did not answer a clean follow-up mode-change request normally after the flood";

  // No settle sleep before teardown, deliberately, and that absence is the assertion.
  //
  // on_change_mode() detaches an unjoined thread per mode-change request and a flood keeps several
  // alive at once, so destroying the rig under one of them would call into an object
  // mid-destruction. PackmlManagerInterface::shutdown() drains those threads, which makes tearing
  // the rig down immediately after a flood exactly the case that should be safe. A settle sleep
  // here would hide that breaking rather than avoid it.
  // See MixedSpeedEMsRepeatedCycling's own comment above: stop em_spins before em (declared
  // after rig) destructs.
  rig.em_spins.clear();
}

// ============================================================================
// A mode change immediately followed by teardown must not leave a thread inside the manager.
//
// on_change_mode() detaches one thread per request, and that thread then uses the manager through
// a fan-out and an up-to-200 ms acceptance wait before publishing status. Nothing else joins it, so
// without PackmlManagerInterface::shutdown() draining it, destroying the manager inside that window
// leaves a live thread reading freed members.
//
// Constructed rather than waited for, which is the point: each iteration sends the request and
// destroys the rig as fast as it can, so the destructor lands inside the window instead of
// hoping to. Repeated because one attempt proves very little about a race.
TEST_F(MonkeyMultiModeTest, ModeChangeThenImmediateTeardownDrainsItsFanoutThread)
{
  for (int attempt = 0; attempt < 20; ++attempt) {
    const std::string em_name = packml_ros_test::unique_node_name("mode_teardown_em");
    auto rig = begin_setup("monkey_mode_teardown", {em_name});
    // A SLOW mode handler, deliberately. With a responsive module the fan-out's acceptance wait
    // is satisfied in a few milliseconds and there is barely a window to land in -- the 200 ms is
    // a cap, not a duration. Delaying the module's own reply past that cap makes the detached
    // thread demonstrably still inside the manager when the teardown below happens.
    auto em = std::make_shared<ModeMonkeyEquipmentModule>(rig.em_nodes[0]);
    em->mode_delay = 400ms;
    finish_setup(rig);

    auto mode_client = rig.mgr_node->create_client<packml_msgs::srv::ModeChange>(
      rig.mgr_name + "/changeMode");
    ASSERT_TRUE(mode_client->wait_for_service(5s)) << "attempt " << attempt;

    // The state machine starts in STOPPED, which permits runtime mode changes.
    auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
    req->mode.val = static_cast<int8_t>(packml_modes::Maintenance);
    auto future = mode_client->async_send_request(req);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready) << "attempt " << attempt;
    ASSERT_TRUE(future.get()->success) << "attempt " << attempt;

    // The response means ACCEPTED; the fan-out thread it spawned is still running. Tear down now,
    // with no settle of any kind -- surviving that is the whole assertion, so there is nothing
    // else to check here.
    rig.em_spins.clear();
  }
}
