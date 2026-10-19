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
// Integration tests: PackML manager + CompletionTracker, over the real
// ~/packml_state_transition ACTION. Run a real SMNode_new (with Qt state machine) with a real
// Equipment Module registered in node_names, and verify that:
//   - RESETTING blocks (does not SC-complete to IDLE) while the EM defers completion, until that
//     goal's own deferred completion is reported.
//   - Without a report, RESETTING times out and the machine aborts.
//   - An explicit failure report aborts fast, well before the timeout.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;

/// Equipment Module that can be told, per test, to defer completion (resolved later by the test
/// body through the handle its on_deferred_work() was given) instead of the default
/// instant-complete-on-acceptance behavior.
class SimCoordinatedNode : public PackmlNodeInterface
{
public:
  // initial_defer, not a later defer.store(), because init() (called here) probes
  // defers_completion() once, per packml_interface.hpp's own documented contract that it is
  // a static, state-shape answer -- to decide which per-state
  // deferred_completion_timeout_ms.<STATE> overrides to declare. A defer.store(true) AFTER
  // construction is invisible to that probe and declares no overrides at all.
  explicit SimCoordinatedNode(rclcpp::Node::SharedPtr node, bool initial_defer = false)
  {
    defer.store(initial_defer);
    init(node);
  }

  std::atomic<bool> defer{false};

  /// The goal this module is currently deferring, for the test body to resolve.
  packml_ros_test::PendingCompletion pending;

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State) override {return defer.load();}
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    pending.accept(completion);
  }
};

}  // namespace

class CompletionIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mgr_node_name_ = packml_ros_test::unique_node_name("completion_mgr");
    mgr_node_ = rclcpp::Node::make_shared(mgr_node_name_,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("node_names", std::vector<std::string>{"sim_completion_em_a"}),
        // Short so the timeout-path tests below don't take long, but long enough that the
        // "waits for completion" test has a comfortable window to assert "not yet done" in.
        rclcpp::Parameter("state_complete_timeout_ms", 500),
      }));

    sm_node_ = std::make_unique<SMNode_new>(mgr_node_);

    em_node_ = rclcpp::Node::make_shared("sim_completion_em_a");
    em_ = std::make_shared<SimCoordinatedNode>(em_node_);

    state_client_ = mgr_node_->create_client<packml_msgs::srv::StateChange>(
      mgr_node_name_ + "/changeState");

    spin_ = std::make_shared<packml_ros_test::SpinHelper>(mgr_node_);
    em_spin_ = std::make_shared<packml_ros_test::SpinHelper>(em_node_);

    ASSERT_TRUE(state_client_->wait_for_service(5s));

    // Drive to STOPPED. No required_nodes are configured, so the health gate is
    // trivially open — RESET does not need any heartbeat setup.
    send_state_change(state_client_, packml_msgs::srv::StateChange::Request::STOP);
    std::this_thread::sleep_for(300ms);
  }

  void TearDown() override
  {
    em_spin_.reset();
    spin_.reset();
    state_client_.reset();
    em_.reset();
    em_node_.reset();
    sm_node_.reset();
    mgr_node_.reset();
  }

  bool wait_for_sm_state(packml_sm::State target, std::chrono::milliseconds timeout)
  {
    return packml_ros_test::wait_for_state(sm_node_, target, timeout);
  }

  std::string mgr_node_name_;
  rclcpp::Node::SharedPtr mgr_node_;
  std::unique_ptr<SMNode_new> sm_node_;

  rclcpp::Node::SharedPtr em_node_;
  std::shared_ptr<SimCoordinatedNode> em_;

  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;
  std::shared_ptr<packml_ros_test::SpinHelper> spin_;
  std::shared_ptr<packml_ros_test::SpinHelper> em_spin_;
};

// RESET moves the machine into RESETTING and it STAYS there — no timer may auto-complete it to
// IDLE — while the EM defers completion, until that goal's own completion is reported; the
// machine then reaches IDLE promptly.
TEST_F(CompletionIntegrationTest, ResettingWaitsForDeferredCompletion)
{
  em_->defer.store(true);

  auto resp = send_state_change(state_client_, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET should be accepted (no required_nodes configured)";

  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // Well over twice the ~200ms an unconditional completion timer would take, so a timer-driven
  // completion would show here, and long enough for the fan-out (bounded by its own ~200ms
  // acceptance wait) to reach the EM and start its deferred wait.
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(sm_node_->getCurrentState(), packml_sm::State::RESETTING)
    << "RESETTING must not auto-complete before the EM reports completion";

  ASSERT_TRUE(em_->pending.report(packml_sm::State::RESETTING, true))
    << "the EM was never asked to do RESETTING's deferred work";

  EXPECT_TRUE(wait_for_sm_state(packml_sm::State::IDLE, 1s))
    << "machine did not reach IDLE after the EM reported completion";
}

// Without any completion report, a deferred RESETTING rides out its state_complete_timeout_ms
// and the machine aborts (ErrorEvent from the bound operation routes to ABORTING).
TEST_F(CompletionIntegrationTest, ResettingTimesOutToAbortWithoutCompletion)
{
  em_->defer.store(true);

  auto resp = send_state_change(state_client_, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  // state_complete_timeout_ms=500 for RESETTING, then again for ABORTING (also deferred,
  // also never reported) before settling — generous margin for both.
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  bool aborted = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto s = sm_node_->getCurrentState();
    if (s == packml_sm::State::ABORTING || s == packml_sm::State::ABORTED) {
      aborted = true;
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(aborted) << "machine did not abort after RESETTING's completion timeout, "
                          "current state: " << static_cast<int>(sm_node_->getCurrentState());
}

// An explicit failure report fails RESETTING FAST: the machine reaches ABORTING well before
// the 500ms completion timeout would have elapsed on its own.
TEST_F(CompletionIntegrationTest, ExplicitFailureReportAbortsFast)
{
  em_->defer.store(true);

  auto resp = send_state_change(state_client_, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  ASSERT_TRUE(wait_for_sm_state(packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // getCurrentState() reflects RESETTING as soon as the Qt state machine sets it -- BEFORE
  // on_state_changed (which fans out to the EM) has necessarily finished running. report()
  // waits for the fan-out to actually reach the EM instead of assuming a fixed number of
  // milliseconds was enough, and fails the test if it never does.
  ASSERT_TRUE(em_->pending.report(packml_sm::State::RESETTING, false, 7, "simulated failure"))
    << "the EM was never asked to do RESETTING's deferred work";

  // Tight bound, well under the 500ms completion timeout — demonstrates fail-fast rather
  // than merely "eventually aborts" (already covered by the timeout test above).
  EXPECT_TRUE(wait_for_sm_state(packml_sm::State::ABORTING, 250ms))
    << "machine did not reach ABORTING promptly after the explicit failure report, "
       "current state: " << static_cast<int>(sm_node_->getCurrentState());
}

// A node that never overrides defers_completion() (the default) completes every coordinated
// state instantly on acceptance — RESET reaches IDLE quickly, matching pre-existing behavior
// for Equipment Modules that don't participate in coordinated completion at all.
TEST_F(CompletionIntegrationTest, NonDeferringNodeCompletesInstantly)
{
  // em_->defer stays false (the default).
  auto resp = send_state_change(state_client_, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  EXPECT_TRUE(wait_for_sm_state(packml_sm::State::IDLE, 1s))
    << "machine did not reach IDLE promptly with a non-deferring EM, current state: "
    << static_cast<int>(sm_node_->getCurrentState());
}

// A node that explicitly FAILS to complete a coordinated state must not just abort the
// manager's own wait fast (see ExplicitFailureReportAbortsFast above) -- any OTHER node still
// pending for that same round must have its own outstanding goal actively canceled too, not
// silently left to ride out its own (here, deliberately much longer)
// deferred_completion_timeout_ms independently. Uses its own dedicated manager + two Equipment
// Modules rather than the shared fixture, which only registers one.
TEST_F(CompletionIntegrationTest, FailureCancelsOtherPendingNodesGoals)
{
  auto mgr_name = packml_ros_test::unique_node_name("completion_mgr_cancel");
  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{"cancel_em_a", "cancel_em_b"}),
      rclcpp::Parameter("state_complete_timeout_ms", 5000),
    }));
  auto sm_node = std::make_unique<SMNode_new>(mgr_node);

  auto em_a_node = rclcpp::Node::make_shared("cancel_em_a");
  auto em_a = std::make_shared<SimCoordinatedNode>(em_a_node);
  em_a->defer.store(true);

  // A long deferred_completion_timeout_ms on em_b: if its goal were ever merely left to time
  // out on its own (rather than actively canceled), this test's own 3s deadline below would
  // still be well short of it, so a passing assertion can only mean an explicit cancel
  // actually arrived -- not "em_b happened to finish before some unrelated short timeout".
  auto em_b_node = rclcpp::Node::make_shared("cancel_em_b",
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("deferred_completion_timeout_ms", 20000)}));
  auto em_b = std::make_shared<SimCoordinatedNode>(em_b_node);
  em_b->defer.store(true);

  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/changeState");

  auto mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(mgr_node);
  auto em_a_spin = std::make_shared<packml_ros_test::SpinHelper>(em_a_node);
  auto em_b_spin = std::make_shared<packml_ros_test::SpinHelper>(em_b_node);

  ASSERT_TRUE(state_client->wait_for_service(5s));
  send_state_change(state_client, packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(100).reliable().transient_local(),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });

  auto resp = send_state_change(state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  {
    packml_ros_test::wait_for_state(sm_node, packml_sm::State::RESETTING, 500ms);
    ASSERT_EQ(sm_node->getCurrentState(), packml_sm::State::RESETTING)
      << "machine did not enter RESETTING";
  }
  // Stop deferring before triggering the failure: defers_completion() is only consulted once,
  // at accept time, so this cannot affect the already in-flight RESETTING goals under test --
  // it only keeps the SM's own subsequent ABORTING round (routed to by the failure below) from
  // ALSO deferring on both EMs and needlessly riding out its own timeout, which would otherwise
  // add tens of seconds to this test for no assertion value.
  // BOTH EMs must have their RESETTING deferral in flight before anything below happens, and the
  // reasons differ: em_a's is what the failure is reported through, and em_b's outstanding goal is
  // the only thing the cancel assertion can be about. Waited for rather than slept through.
  //
  // This has to precede the defer.store(false) calls, not just the report. defers_completion() is
  // consulted per goal, at accept time, so flipping it off while a node's goal is still in flight
  // makes THAT node complete instantly instead of deferring -- after which there is either nothing
  // to report the failure through, or nothing outstanding to cancel. Both shapes were observed;
  // together they failed this test roughly 4 runs in 10.
  const auto wait_for_resetting_deferral =
    [](SimCoordinatedNode & em) {
      return em.pending.wait_for_accept(packml_sm::State::RESETTING, 5s);
    };
  ASSERT_TRUE(wait_for_resetting_deferral(*em_a))
    << "em_a was never asked to do RESETTING's deferred work";
  ASSERT_TRUE(wait_for_resetting_deferral(*em_b))
    << "em_b was never asked to do RESETTING's deferred work";

  em_a->defer.store(false);
  em_b->defer.store(false);

  ASSERT_TRUE(em_a->pending.report(packml_sm::State::RESETTING, false, 7, "simulated failure"))
    << "em_a was never asked to do RESETTING's deferred work";

  // em_b's goal must be actively canceled promptly -- well within its own 20s
  // deferred_completion_timeout_ms -- and specifically via a CANCELED result, distinguishing
  // this from any other way the goal might otherwise have resolved.
  const std::string expected_code =
    "result code " + std::to_string(static_cast<int>(rclcpp_action::ResultCode::CANCELED));
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  bool found_cancel_alarm = false;
  while (!found_cancel_alarm && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      for (const auto & a : alarms) {
        if (a.node_name == "cancel_em_b" && a.message.find(expected_code) != std::string::npos) {
          found_cancel_alarm = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(found_cancel_alarm)
    << "cancel_em_b's outstanding goal was not actively canceled after cancel_em_a's failure";
}

// An operator's ABORT command, accepted while a coordinated acting state (RESETTING) is
// still mid-wait, must interrupt that wait PROMPTLY -- not force the machine to ride out
// the full state_complete_timeout_ms before it can even leave RESETTING (see
// ActingState::onExit() / StateMachine::setInterruptibleStateOperation() in packml_sm).
// Also verifies the interrupted wait's own now-stale completion event does not leak into
// ABORTING: the EM here defers ABORTING too, so if a ghost StateCompleteEvent from the
// interrupted RESETTING wait were mistakenly routed to ABORTING's own SC-transition,
// ABORTING would complete instantly instead of correctly waiting for its own report.
TEST_F(CompletionIntegrationTest, AbortInterruptsInFlightWaitPromptly)
{
  auto mgr_name = packml_ros_test::unique_node_name("completion_mgr_interrupt");
  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{"interrupt_em"}),
      // Deliberately long: if ABORT merely rode out this timeout instead of
      // interrupting the wait, this test's own much shorter deadlines below would
      // catch that as a failure.
      rclcpp::Parameter("state_complete_timeout_ms", 10000),
    }));
  auto sm_node = std::make_unique<SMNode_new>(mgr_node);

  auto em_node = rclcpp::Node::make_shared("interrupt_em");
  auto em = std::make_shared<SimCoordinatedNode>(em_node);
  em->defer.store(true);  // defers every coordinated state, including ABORTING --
                          // see test comment above for why that matters here

  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/changeState");

  auto mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(mgr_node);
  auto em_spin = std::make_shared<packml_ros_test::SpinHelper>(em_node);

  ASSERT_TRUE(state_client->wait_for_service(5s));
  send_state_change(state_client, packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);

  auto resp = send_state_change(state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  {
    packml_ros_test::wait_until(
      [&] {return sm_node->getCurrentState() == packml_sm::State::RESETTING;}, 500ms, 10ms);
    ASSERT_EQ(sm_node->getCurrentState(), packml_sm::State::RESETTING)
      << "machine did not enter RESETTING";
  }
  std::this_thread::sleep_for(300ms);  // let the fan-out reach the EM, deferred wait starts

  // Issue ABORT while RESETTING's wait is still in flight (9+ seconds left on its own
  // 10s timeout).
  const auto abort_resp =
    send_state_change(state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success);

  // The machine must reach ABORTING promptly -- well under the 10s timeout that would
  // otherwise have to elapse first.
  const auto reached_aborting_deadline = std::chrono::steady_clock::now() + 2s;
  bool reached_aborting = false;
  while (std::chrono::steady_clock::now() < reached_aborting_deadline) {
    if (sm_node->getCurrentState() == packml_sm::State::ABORTING) {
      reached_aborting = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(reached_aborting)
    << "machine did not reach ABORTING promptly after ABORT was accepted, current state: "
    << static_cast<int>(sm_node->getCurrentState());

  // ABORTING itself is deferred by the same EM -- it must NOT complete on its own via a
  // stale StateCompleteEvent from the interrupted RESETTING wait. Give it a comfortable
  // margin, then confirm it is still (correctly) waiting.
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(sm_node->getCurrentState(), packml_sm::State::ABORTING)
    << "ABORTING completed on its own before being reported -- likely a stale "
       "completion event leaked from the interrupted RESETTING wait";

  // Stop deferring BEFORE reporting ABORTING complete: ABORTING -> ABORTED is itself an
  // SC-triggered transition, fanned out to every node exactly like any other (ABORTED
  // just isn't one of the manager's own coordinated states) -- with defer still true,
  // interrupt_em would defer THAT goal too, and this test never reports it, riding out
  // its own 30s deferred_completion_timeout_ms during teardown for nothing. Resetting
  // first only affects the NEXT goal's accept-time decision; the current, already
  // in-flight ABORTING wait is unaffected and still resolves via the report below.
  em->defer.store(false);

  ASSERT_TRUE(em->pending.report(packml_sm::State::ABORTING, true))
    << "the EM was never asked to do ABORTING's deferred work";

  const auto reached_aborted_deadline = std::chrono::steady_clock::now() + 1s;
  bool reached_aborted = false;
  while (std::chrono::steady_clock::now() < reached_aborted_deadline) {
    if (sm_node->getCurrentState() == packml_sm::State::ABORTED) {
      reached_aborted = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(reached_aborted)
    << "machine did not reach ABORTED after ABORTING was explicitly reported complete";
}

// A per-state override ("state_complete_timeout_ms.<STATE NAME>") governs that state's own
// coordinated wait instead of the (much longer) global default -- proving the override is
// actually read per acting-state operation, not just the flat state_complete_timeout_ms.
TEST_F(CompletionIntegrationTest, PerStateTimeoutOverrideGovernsThatStateOnly)
{
  auto mgr_name = packml_ros_test::unique_node_name("completion_mgr_per_state");
  auto mgr_node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{"per_state_em"}),
      // Deliberately long: if RESETTING fell back to this instead of its own override,
      // this test's much shorter deadline below would catch that as a failure.
      rclcpp::Parameter("state_complete_timeout_ms", 5000),
      rclcpp::Parameter("state_complete_timeout_ms.RESETTING", 300),
    }));
  auto sm_node = std::make_unique<SMNode_new>(mgr_node);

  auto em_node = rclcpp::Node::make_shared("per_state_em");
  auto em = std::make_shared<SimCoordinatedNode>(em_node);
  em->defer.store(true);  // defers every coordinated state; never reports any of them

  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/changeState");

  auto mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(mgr_node);
  auto em_spin = std::make_shared<packml_ros_test::SpinHelper>(em_node);

  ASSERT_TRUE(state_client->wait_for_service(5s));
  send_state_change(state_client, packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);

  const auto start = std::chrono::steady_clock::now();
  auto resp = send_state_change(state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  bool aborted = false;
  while (std::chrono::steady_clock::now() < deadline) {
    if (sm_node->getCurrentState() == packml_sm::State::ABORTING) {
      aborted = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  ASSERT_TRUE(aborted)
    << "machine did not abort after RESETTING's per-state timeout override elapsed";
  EXPECT_LT(elapsed, 2s)
    << "RESETTING rode out the 5s global default instead of its 300ms per-state override";

  // ABORTING is deferred too (defer=true covers every state) and has no override of its
  // own, so it would otherwise ride out the 5s global default during teardown -- report it
  // explicitly to keep teardown fast (mirrors AbortInterruptsInFlightWaitPromptly's pattern).
  em->pending.report(packml_sm::State::ABORTING, true);
}

// EM-side mirror of PerStateTimeoutOverrideGovernsThatStateOnly above: a per-state override
// ("deferred_completion_timeout_ms.<STATE NAME>") governs that state's own deferred wait
// instead of the (much longer) global default. No manager involved -- talks to the EM's
// action server directly, exactly like test_node_interface.cpp's pattern.
TEST(EmSidePerStateTimeoutOverrideTest, GovernsThatStateOnly)
{
  auto node_name = packml_ros_test::unique_node_name("em_per_state_timeout_test");
  auto node = rclcpp::Node::make_shared(node_name,
    rclcpp::NodeOptions().parameter_overrides({
      // Deliberately long: if RESETTING fell back to this instead of its own override,
      // this test's much shorter deadline below would catch that as a failure.
      rclcpp::Parameter("deferred_completion_timeout_ms", 5000),
      rclcpp::Parameter("deferred_completion_timeout_ms.RESETTING", 300),
    }));
  auto em = std::make_shared<SimCoordinatedNode>(node, /*initial_defer=*/true);

  auto state_client = rclcpp_action::create_client<packml_msgs::action::StateTransition>(
    node, node_name + "/packml_state_transition");
  auto spin = std::make_shared<packml_ros_test::SpinHelper>(node);
  ASSERT_TRUE(state_client->wait_for_action_server(5s));

  packml_msgs::action::StateTransition::Goal goal;
  goal.state.val = static_cast<int8_t>(packml_sm::State::RESETTING);

  const auto start = std::chrono::steady_clock::now();
  auto goal_handle_future = state_client->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = state_client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready)
    << "goal did not resolve within 2s -- the 300ms per-state override should have fired "
       "well before this";
  const auto elapsed = std::chrono::steady_clock::now() - start;

  auto result = result_future.get().result;
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->success);
  EXPECT_NE(result->message.find("timed out"), std::string::npos);
  EXPECT_LT(elapsed, 2s)
    << "goal rode out the 5s global default instead of its 300ms per-state override";
}

// An Equipment Module destroyed while a deferred completion is still in flight must not leave the
// waiting thread inside it — and must not take the full timeout to say so.
//
// begin_transition() hands a deferring goal to a DETACHED thread that then uses the module for as
// long as deferred_completion_timeout_ms allows. Nothing joins it, so destroying the node inside
// that window left a live thread reading freed members and resolving a goal through a destroyed
// action server. This is the same class as the manager's mode-fan-out thread, but with a far longer
// window: seconds by configuration rather than the fan-out's ~200 ms cap.
//
// Two things are asserted, and the second is what makes the first honest: destruction completes at
// all, and it completes FAST. ~PackmlNodeInterface() waits for the thread, so without the shutting-
// down flag that wakes the wait, teardown would block for the whole timeout instead — correct, but
// unusable. The timeout here is deliberately far longer than the deadline.
TEST(EmSideShutdownDrainTest, DestroyingAModuleMidDeferralIsPromptAndSafe)
{
  auto node_name = packml_ros_test::unique_node_name("em_shutdown_drain_test");
  auto node = rclcpp::Node::make_shared(node_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("deferred_completion_timeout_ms", 20000),
    }));
  auto em = std::make_shared<SimCoordinatedNode>(node, /*initial_defer=*/true);

  auto state_client = rclcpp_action::create_client<packml_msgs::action::StateTransition>(
    node, node_name + "/packml_state_transition");
  auto spin = std::make_shared<packml_ros_test::SpinHelper>(node);
  ASSERT_TRUE(state_client->wait_for_action_server(5s));

  packml_msgs::action::StateTransition::Goal goal;
  goal.state.val = static_cast<int8_t>(packml_sm::State::RESETTING);
  auto goal_handle_future = state_client->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  ASSERT_NE(goal_handle_future.get(), nullptr) << "goal was rejected, so nothing is deferring";

  // The hook has been called and the deferred wait is running. Nothing will ever report, so the
  // only two ways out are the 20 s timeout or the destructor.
  std::this_thread::sleep_for(200ms);

  const auto start = std::chrono::steady_clock::now();
  spin.reset();
  em.reset();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, 3s)
    << "destroying the module took "
    << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
    << " ms — it waited out the deferred-completion timeout instead of asking the in-flight wait "
       "to give up first";
}
