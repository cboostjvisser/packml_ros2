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
// "Monkey" scenarios, EM-unreliable edition: extends test_monkey_scenarios.cpp's chaos-testing
// approach to a class of misbehavior that file's own BACKLOG section groups under
// "Broader untested mechanisms" -- the Equipment Module itself failing in ways an ordinary
// slow-but-honest EM (that file's MonkeyStaleWorkEquipmentModule) does not:
//   1. an EM whose commanded-state hook never returns at all, starving its own ROS executor
//      (rather than returning fast and doing slow work on a detached background thread -- that
//      is MonkeyStaleWorkEquipmentModule's shape, not this one);
//   2. an EM whose completion report contradicts its own success flag (success=true with a
//      populated error_code/message, or success=false with an empty message);
//   3. an EM whose heartbeat-driven health flip to ABORT severity races its own genuine
//      completion report for the same coordinated state.
//
// Matches test_monkey_scenarios.cpp's own conventions: scenario-specific helper classes in this
// file's own anonymous namespace (generic scaffolding lives in test_helpers.hpp), a Rig
// struct + begin_setup()/finish_setup() pattern for a standalone manager+EM pair per test
// (extended here with optional extra manager parameter overrides and EM node options, since
// these scenarios need health-monitor configuration the plain command/completion monkeys in
// that file didn't), a wait_for_state() polling helper, and the same two flavors of assertion --
// confirmed-safe regression checks, and (were one found) a KNOWN GAP written as the correct
// expected behavior rather than a loosened one. Every test below lands in the confirmed-safe
// camp: unlike that file's RESET->ABORT->RESET gap, nothing here surfaced a currently-broken
// mechanism -- see each test's own comment for the reasoning.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
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
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// EM whose on_state_trans_req() itself blocks forever for one specific coordinated state --
/// synchronously, on whatever thread calls it (the EM's own ROS executor thread in production,
/// see begin_transition()'s comment in packml_interface.hpp) -- rather than returning fast and
/// deferring slow work to a background thread. Waits on a condition variable that nothing ever
/// notifies: deterministic, no reliance on a sleep duration outlasting the test's patience.
/// Every OTHER state behaves like a plain, instantly-approving EM.
class WedgedEquipmentModule : public PackmlNodeInterface
{
public:
  WedgedEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State wedge_state)
  : wedge_state_(wedge_state)
  {
    init(node);
  }

  std::atomic<int> wedge_entered{0};

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    if (state == wedge_state_) {
      wedge_entered.fetch_add(1);
      std::mutex wedge_mutex;
      std::condition_variable wedge_cv;
      std::unique_lock<std::mutex> lk(wedge_mutex);
      // The predicate overload (not a bare wait(lk)) is deliberate: a bare wait() is allowed
      // to wake up spuriously per the standard, which would silently un-wedge this hook. An
      // always-false predicate blocks forever regardless, starving this node's whole executor.
      wedge_cv.wait(lk, [] {return false;});
    }
    return true;
  }
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}

private:
  packml_sm::State wedge_state_;
};

/// Plain EM that defers completion for exactly one coordinated state, handing the goal's
/// completion straight to the test so it can be resolved with whatever -- including nonsensical --
/// field combination the test likes. Mirrors test_completion_integration.cpp's own
/// SimCoordinatedNode: an EM shape is scenario-specific, so it is written per file rather than
/// shared, unlike the generic scaffolding in test_helpers.hpp.
class DeferringEquipmentModule : public PackmlNodeInterface
{
public:
  DeferringEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State defer_state)
  : defer_state_(defer_state)
  {
    init(node);
  }

  packml_ros_test::PendingCompletion pending;

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == defer_state_;}
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    pending.accept(completion);
  }

private:
  packml_sm::State defer_state_;
};

/// Combines test_health_monitor_integration.cpp's SimEquipmentModule pattern (controllable
/// health via atomics, reported on every heartbeat tick) with DeferringEquipmentModule's
/// deferred-completion state above, so a single EM can race its own heartbeat-driven health
/// flip against its own completion report for the SAME coordinated state.
class RacingHealthEquipmentModule : public PackmlNodeInterface
{
public:
  RacingHealthEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State defer_state)
  : defer_state_(defer_state)
  {
    init(node);
  }

  std::atomic<int32_t> health_status{packml_msgs::msg::NodeHealth::HEALTHY};
  std::atomic<int32_t> health_action{packml_msgs::msg::NodeHealth::NONE};

  packml_ros_test::PendingCompletion pending;

  packml_msgs::msg::NodeHealth get_health_status() override
  {
    packml_msgs::msg::NodeHealth h;
    h.status = health_status.load();
    h.action = health_action.load();
    return h;
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == defer_state_;}
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    pending.accept(completion);
  }

private:
  packml_sm::State defer_state_;
};

}  // namespace

class MonkeyEdgeCasesTest : public ::testing::Test
{
protected:
  /// Every test builds its own standalone manager + EM pair (unique node names) -- see
  /// test_monkey_scenarios.cpp's own Rig doc comment for why. Extended here with optional
  /// extra manager parameter overrides and EM node options (this file's health-timing
  /// scenarios need required_nodes/heartbeat_timeout_factor/heartbeat_interval_ms that the
  /// plain command/completion monkeys never needed).
  struct Rig
  {
    std::string mgr_name;
    rclcpp::Node::SharedPtr mgr_node;
    std::unique_ptr<SMNode_new> sm_node;
    rclcpp::Node::SharedPtr em_node;
    rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client;
    std::shared_ptr<packml_ros_test::SpinHelper> mgr_spin;
    std::shared_ptr<packml_ros_test::SpinHelper> em_spin;
  };

  Rig begin_setup(
    const std::string & mgr_prefix, const std::string & em_name,
    int state_complete_timeout_ms = 5000,
    std::vector<rclcpp::Parameter> extra_mgr_params = {},
    rclcpp::NodeOptions em_options = rclcpp::NodeOptions())
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    std::vector<rclcpp::Parameter> mgr_params = {
      rclcpp::Parameter("node_names", std::vector<std::string>{em_name}),
      rclcpp::Parameter("state_complete_timeout_ms", state_complete_timeout_ms),
    };
    for (const auto & p : extra_mgr_params) {
      mgr_params.push_back(p);
    }
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides(mgr_params));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.em_node = rclcpp::Node::make_shared(em_name, em_options);
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/" + packml_ros::kChangeStateService);
    rig.mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.mgr_node);
    return rig;
  }

  void finish_setup(Rig & rig)
  {
    rig.em_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.em_node);
    ASSERT_TRUE(rig.state_client->wait_for_service(5s));
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
    std::this_thread::sleep_for(300ms);
  }
};

// ============================================================================
// Confirmed-safe (expected): the manager's existing, independent health-timeout and
// completion-timeout backstops are believed to be a real safety net even against a FULLY
// starved EM executor, not just a slow-but-still-responsive one -- this is the first test to
// exercise that specific shape, so treat a failure here as a genuine finding rather than a test
// bug. Reasoning: HealthMonitor::gate_block_reason() computes "heartbeat stale" INLINE from
// elapsed time (health_monitor.hpp), so is_node_healthy() can observe the wedge within one
// effective_timeout_ms window even before check_timeouts()'s own periodic tick formally flags
// it; that periodic tick (every 200ms) then calls fire_action_(ABORT), which calls
// CompletionTracker::notify_health_change() before attempting sm_->changeState(ABORT) --
// waking CompletionTracker::wait_for_all()'s in-flight wait promptly either way. Uses a
// deliberately short heartbeat interval/timeout-factor and a short state_complete_timeout_ms so
// both backstops have a realistic chance to fire within a few seconds.
//
// The EM's own executor is spun on a manually detached raw thread, NOT
// packml_ros_test::SpinHelper: once the wedge engages, the spin call never returns, and
// SpinHelper's destructor unconditionally joins its spin thread -- which would hang this test's
// (and the whole suite's) teardown forever. Detaching means the permanently-stuck thread simply
// outlives this test instead of blocking it.
TEST_F(MonkeyEdgeCasesTest, MonkeyWedgedEmExecutor_HealthAndCompletionTimeoutsResolveTheWait)
{
  const auto mgr_name = packml_ros_test::unique_node_name("monkey_wedged_mgr");
  const std::string em_name = "wedged_em";

  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{em_name}),
      rclcpp::Parameter("required_nodes", std::vector<std::string>{em_name}),
      rclcpp::Parameter("heartbeat_timeout_factor", 3.0),
      rclcpp::Parameter("state_complete_timeout_ms", 800),
    }));
  auto sm_node = std::make_unique<SMNode_new>(mgr_node);

  // A dedicated, separately-initialized Context for the wedged EM's node -- NOT the global
  // default context rclcpp::init()/rclcpp::shutdown() (in test_main.cpp) manage. Once the wedge
  // engages, em_executor's spin() never returns on its detached thread; global
  // rclcpp::shutdown() running at the very end of this whole test binary (after every other
  // test has already reported -- this test's own [OK]/[PASSED] lines are unaffected either way)
  // would otherwise try to tear down the SAME context that thread is still live inside of.
  // Isolating rcl/rmw participant-level resources onto their own context this way is a genuine
  // improvement, not a full fix: a residual, harmless crash (core dump, non-zero exit code)
  // AFTER every test result is already printed can still occur, traced to rcutils' own logging
  // being process-global rather than per-Context -- the wedged thread can still try to log
  // through it during the same global shutdown window. Matches this exact file's own
  // established, accepted precedent for this category of problem: test_main.cpp's
  // QCoreApplication is "intentionally never deleted to avoid a crash in its destructor during
  // atexit (it would be destroyed from the wrong thread)" -- a background thread that must
  // outlive normal per-test teardown is fundamentally in tension with fully clean process exit,
  // and this is the same trade-off, not a new or worse one.
  auto em_context = std::make_shared<rclcpp::Context>();
  em_context->init(0, nullptr);
  auto em_node = rclcpp::Node::make_shared(em_name,
    rclcpp::NodeOptions().context(em_context).parameter_overrides({
      rclcpp::Parameter("heartbeat_interval_ms", 50),
    }));
  auto em = std::make_shared<WedgedEquipmentModule>(em_node, packml_sm::State::RESETTING);

  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/" + packml_ros::kChangeStateService);

  auto mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(mgr_node);

  // See this test's own header comment: a manually detached executor thread, not SpinHelper,
  // specifically so the eventual permanent block does not hang teardown.
  auto em_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  em_executor->add_node(em_node);
  std::thread([em_executor]() {em_executor->spin();}).detach();

  ASSERT_TRUE(state_client->wait_for_service(5s));

  // Wait for real (fast, 50ms) heartbeats to actually reach the manager before the wedge
  // engages, so the health gate is open and the manager has learned the EM's real (short)
  // interval rather than the long startup-grace assumption it starts with. Waited for rather
  // than slept through: any fixed sleep flakes as "wedged_em (never seen)" when the beats have
  // not been delivered yet, and the gate is right to refuse. See wait_for_healthy_heartbeats().
  ASSERT_GE(packml_ros_test::wait_for_healthy_heartbeats(mgr_node, em_name), 2)
    << "no heartbeats from " << em_name << " reached the manager -- the health gate would "
       "block RESET for a reason that has nothing to do with this test";

  send_state_change(state_client, packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(200ms);

  auto resp = send_state_change(state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET should be accepted -- EM was healthy before the wedge";

  ASSERT_TRUE(wait_for_state(sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // From here on the EM's entire executor is starved by the wedge inside on_state_trans_req()
  // -- no more heartbeats, no cancel response, no result, ever. Only the manager's own
  // independent health-timeout (~50ms x 3 = 150ms, checked every 200ms) and completion-timeout
  // (800ms) can still move the machine.
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  bool reached_abort = false;
  packml_sm::State observed = packml_sm::State::UNDEFINED;
  while (std::chrono::steady_clock::now() < deadline) {
    observed = sm_node->getCurrentState();
    if (observed == packml_sm::State::ABORTING || observed == packml_sm::State::ABORTED) {
      reached_abort = true;
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(reached_abort)
    << "machine did not resolve the wedged EM's coordinated wait within 3s (stuck in: "
    << static_cast<int>(observed) << ") -- the health/completion timeout backstops should "
       "have fired well before this";
  EXPECT_EQ(em->wedge_entered.load(), 1) << "the wedge hook itself was never entered";

  // Deliberately leak em/em_node/em_executor -- NOT just a SpinHelper-join workaround, a
  // fundamental one: on_state_trans_req() is permanently blocked, with a live call frame
  // holding `this` on em_executor's detached thread for the rest of the process's life.
  // Destroying `em`/`em_node` normally at this scope's exit (or letting them be destroyed by
  // any later static/global teardown at process exit) would free memory a live thread is still
  // executing inside of -- confirmed by observation, not just reasoning: earlier runs of this
  // suite produced "cannot publish data"/datawriter deletion errors and a core dump at process
  // exit that trace back to exactly this. A raw new() with no matching delete is a genuine,
  // intentional leak (reclaimed by the OS at process exit; no destructor ever runs on it) --
  // the only safe option once a permanent wedge like this has actually engaged. The manager
  // side (mgr_node/sm_node/mgr_spin/state_client) is untouched by the wedge and destructs
  // normally at the end of this scope, as usual.
  new auto(em);
  new auto(em_node);
  new auto(em_executor);
  new auto(em_context);
}

// ============================================================================
// Confirmed safe: the completion report's own documented contract (packml_interface.hpp) and
// CompletionTracker::wait_for_all()'s predicate both branch ONLY on the success bool --
// error_code/message are inert pass-through fields for logging/alarms, never inspected for
// control flow. A success=true report paired with a populated error_code and a
// failure-shaped message must therefore still complete the coordinated state normally.
TEST_F(MonkeyEdgeCasesTest, MonkeyNonsensicalCompletion_SuccessTrueWithErrorCodeStillCompletesNormally)
{
  auto rig = begin_setup("monkey_nonsense_success", "nonsense_success_em", 2000);
  auto em = std::make_shared<DeferringEquipmentModule>(rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";
  // Nonsensical: success=true but paired with a nonzero error_code AND a populated
  // failure-shaped message, as if the EM couldn't decide whether it had actually failed.
  // report() waits for the fan-out to actually reach the EM rather than assuming a fixed sleep
  // was long enough, and fails here if it never did.
  ASSERT_TRUE(em->pending.report(
      packml_sm::State::RESETTING, /*success=*/true, /*error_code=*/17,
      "simulated internal fault (but success=true was still reported)"))
    << "the EM was never asked to do RESETTING's deferred work";

  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 1s))
    << "machine did not reach IDLE despite success=true -- did something start trusting "
       "error_code/message over the success flag? current state: "
    << static_cast<int>(rig.sm_node->getCurrentState());
  EXPECT_NE(rig.sm_node->getCurrentState(), packml_sm::State::ABORTING)
    << "machine aborted despite an explicit success=true completion report";

  // em (declared after rig) would otherwise destruct BEFORE rig's own em_spin member stops
  // spinning -- a still-in-flight fan-out callback calling into an object whose vtable is
  // already gone ("pure virtual method called"). Confirmed for real via gdb in
  // test_monkey_multi_mode.cpp's ClientDisconnectMidRequest test; stopping the spinner first,
  // explicitly, removes the race instead of just outrunning it.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed safe: the mirror image of the test above -- success=false governs a clean, fast
// fail-out to ABORTING even when paired with an entirely empty message (and the default,
// unset error_code=0). An empty message must not be mistaken for "the call didn't really
// happen" and must not crash whatever builds the resulting alarm/log text from it
// (report_fanout_failure() concatenates it directly into the alarm message).
TEST_F(MonkeyEdgeCasesTest, MonkeyNonsensicalCompletion_FailureWithEmptyMessageStillAbortsCleanly)
{
  auto rig = begin_setup("monkey_nonsense_failure", "nonsense_failure_em", 2000);
  auto em = std::make_shared<DeferringEquipmentModule>(rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // Nonsensical: success=false with no error_code and no message at all -- error_code/message
  // default to 0/"" here, deliberately left unset.
  ASSERT_TRUE(em->pending.report(packml_sm::State::RESETTING, /*success=*/false))
    << "the EM was never asked to do RESETTING's deferred work";

  // ABORTING is not deferred by this EM, so a failed RESETTING can ride straight through
  // ABORTING to ABORTED well inside a single 500ms poll window -- either state is proof the
  // failure was honored promptly; the point under test is "it failed out cleanly" (i.e. driven
  // by success=false alone), not which exact instant the poll happens to catch it at.
  packml_ros_test::wait_until(
    [&] {
      const auto state = rig.sm_node->getCurrentState();
      return state == packml_sm::State::ABORTING || state == packml_sm::State::ABORTED;
    }, 500ms, 10ms);
  const auto final_state = rig.sm_node->getCurrentState();
  EXPECT_TRUE(
    final_state == packml_sm::State::ABORTING || final_state == packml_sm::State::ABORTED)
    << "machine did not fail out promptly after an explicit success=false report with an empty "
       "message, current state: " << static_cast<int>(final_state);

  // See MonkeyNonsensicalCompletion_SuccessTrueWithErrorCodeStillCompletesNormally's own comment
  // above: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe resilience check -- deliberately NOT resolved in favor of one side; see the
// reasoning below for why either outcome is acceptable here, unlike the other tests in this
// file which each assert one specific, contract-mandated outcome.
//
// An EM's heartbeat-driven flip to ABORT-severity health and its own genuine
// success=true completion report for the SAME coordinated state are deliberately timed
// (via a background thread with a tuned sleep) to land within the same ~heartbeat-interval
// window, so CompletionTracker::wait_for_all()'s health cross-check and its on_result()
// callback are racing each other for real, not sequenced by this test.
//
// Both possible outcomes are sensible given the codebase never defines an ordering between
// these two signals:
//   - the health flip is observed first  -> WaitResult::ABORTED_BY_HEALTH -> the machine routes
//     to ABORTING even though the EM's own report said success=true (a health signal that wins
//     the race is allowed to preempt a completion not yet recorded).
//   - the result is recorded first       -> WaitResult::COMPLETE -> IDLE, and the EM's
//     newly-unhealthy status is instead caught by the NEXT coordinated state's own health gate,
//     the same as any ordinary post-cycle fault would be.
// What would NOT be sensible -- and is the actual point of this test -- is a hang, a crash, or
// the machine landing in neither recognized state. This asserts only that shared,
// order-independent guarantee, plus that the manager stays responsive afterward either way.
TEST_F(MonkeyEdgeCasesTest, MonkeyHealthReportRace_ConcurrentUnhealthyAndGenuineReportResolvesSafely)
{
  std::vector<rclcpp::Parameter> extra_mgr_params = {
    rclcpp::Parameter("required_nodes", std::vector<std::string>{"race_em"}),
    rclcpp::Parameter("heartbeat_timeout_factor", 3.0),
  };
  auto rig = begin_setup(
    "monkey_health_race", "race_em", /*state_complete_timeout_ms=*/3000, extra_mgr_params,
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("heartbeat_interval_ms", 100)}));
  auto em = std::make_shared<RacingHealthEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  // Wait for healthy heartbeats (100ms interval) to reach the manager before commanding RESET,
  // so the health gate is open. Waited for rather than slept through; see
  // wait_for_healthy_heartbeats().
  ASSERT_GE(packml_ros_test::wait_for_healthy_heartbeats(rig.mgr_node, "race_em"), 2)
    << "no heartbeats from race_em reached the manager -- the health gate would block RESET "
       "for a reason that has nothing to do with this test";

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET should be accepted -- EM reports healthy before the race";
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";
  // The 60ms gap below is only a tuned race if it starts from a deferral that actually exists,
  // so wait for the fan-out to reach the EM first rather than sleeping and hoping it did.
  ASSERT_TRUE(em->pending.wait_for_accept(packml_sm::State::RESETTING, 5s))
    << "the EM was never asked to do RESETTING's deferred work -- there is no race to run";

  // Fire the race from a background thread: flip health to ABORT severity (visible on the
  // manager's next ~100ms heartbeat tick), then report completion a tuned ~half an interval
  // later -- close enough that which one CompletionTracker's wait observes first is genuinely up
  // to real scheduling, not this test's own sequencing.
  std::thread([em]() {
      em->health_status.store(packml_msgs::msg::NodeHealth::ERROR);
      em->health_action.store(packml_msgs::msg::NodeHealth::ABORT);
      std::this_thread::sleep_for(60ms);
      em->pending.report(packml_sm::State::RESETTING, true);
    }).detach();

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  bool settled = false;
  packml_sm::State observed = packml_sm::State::UNDEFINED;
  while (std::chrono::steady_clock::now() < deadline) {
    observed = rig.sm_node->getCurrentState();
    if (observed == packml_sm::State::IDLE ||
      observed == packml_sm::State::ABORTING ||
      observed == packml_sm::State::ABORTED)
    {
      settled = true;
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(settled)
    << "machine did not settle into IDLE/ABORTING/ABORTED within 3s of the race -- stuck in: "
    << static_cast<int>(observed);

  // The manager must still be responsive after the race, whichever side won.
  auto follow_up =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP, 2s);
  EXPECT_NE(follow_up, nullptr) << "manager stopped responding after the health/report race";

  // See MonkeyNonsensicalCompletion_SuccessTrueWithErrorCodeStillCompletesNormally's own comment
  // above: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}
