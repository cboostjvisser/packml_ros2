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
// "Monkey" scenarios, EM-reporting flavor: see test_monkey_scenarios.cpp for the shared
// rationale of this whole family of tests. That file (and its siblings covering fan-out/
// discovery and fuzzing chaos) make the OPERATOR, the manager's own command handling, or the
// EM's discoverability the chaotic party; this file makes the EM's own COMPLETION-REPORTING
// behavior the chaotic party instead -- covering the "completion/report misbehavior" group,
// plus two adjacent report-timing items:
//
//   1. Double-report: an EM reports twice for the same deferred goal (a
//      real result, then a different one moments later) -- does the redundant second call
//      corrupt whatever transition defers NEXT?
//   2. Contract-violating defers_completion(): an EM whose answer for a given state flips
//      between calls via an atomic counter, even though the framework documents this as
//      required to be a fixed, state-shape answer -- does a flip ever leave a coordinated
//      state stuck deferring forever, or does it degrade gracefully?
//   3. Chronically forgetful EM across many cycles: an EM that defers a coordinated state but
//      never reports at all, repeated over several ABORT/CLEAR/RESET recovery
//      cycles in a row -- checks for thread/timer leaks and degrading responsiveness across
//      repeated timeouts, not just a single one.
//   4. Report landing at the timeout boundary: an EM whose report is timed (via a background
//      thread sleep) to land within ~50-100ms of the manager's own completion deadline, from
//      both sides -- exactly one outcome (the report, or the timeout) must resolve the
//      transition, never both or neither, and no crash.
//   5. Cancel-vs-report simultaneity: an EM's report timed to land at approximately the same
//      moment an interrupting ABORT command arrives -- characterizes which one the framework
//      actually honors, and confirms neither hangs nor crashes.
//
// All six tests below are written "this SHOULD hold" (confirmed-safe style), based on reading
// PackmlNodeInterface's per-goal completion records and
// CompletionTracker::wait_for_all()'s own check ordering (stop_token checked before any
// per-node result) -- none are declared intentionally-red KNOWN GAP markers the way
// test_monkey_scenarios.cpp's MonkeyResetAbortReset_StaleReportResolvesLaterResetTooEarly is.
// If any of these turn red, treat that as real signal, not a mistake in the test -- do not
// loosen the assertion to make it pass (see test_monkey_scenarios.cpp's own header for the
// project's standing rule on that).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_ros/ros_names.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// EM that defers exactly one configured `monkey_state` and, once asked to transition into it,
/// reports completion TWICE from a background thread: a real success, then -- moments later --
/// a different, contradictory result (failure). Every other state behaves like a plain,
/// instantly-completing EM (defers_completion() stays false), matching
/// MonkeyStaleWorkEquipmentModule's "only monkey-behave for the one state it's built for"
/// convention in test_monkey_scenarios.cpp.
class DoubleReportEquipmentModule : public PackmlNodeInterface
{
public:
  explicit DoubleReportEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State monkey_state)
  : monkey_state_(monkey_state)
  {
    init(node);
  }

  /// Incremented once per report this EM makes -- lets a test confirm both the real and the
  /// stale report actually fired, not just that the goal resolved.
  std::atomic<int> reports_sent{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == monkey_state_;}
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    std::thread([this, completion]() {
        std::this_thread::sleep_for(50ms);
        completion.report(true);
        reports_sent.fetch_add(1);
        std::this_thread::sleep_for(50ms);
        // Stale second report: a different, contradictory result for the SAME goal, which the
        // first report has already resolved. First report wins; this one is discarded with a
        // warning and must not affect anything that comes after it.
        completion.report(false, 99, "stale second report");
        reports_sent.fetch_add(1);
      }).detach();
  }

private:
  packml_sm::State monkey_state_;
};

/// EM whose defers_completion() answer for `monkey_state` FLIPS between calls (via an atomic
/// counter), directly violating the "fixed, state-shape answer" contract documented on
/// PackmlNodeInterface::defers_completion() -- the framework never actually enforces that
/// contract. Whenever the flip does land on "defer", this EM's work reports success on its own
/// short, fixed delay: this isolates the question "does an inconsistent defers_completion()
/// answer alone ever strand a transition" from "does an EM that also fails to report ever strand
/// one" (already covered by ChronicallyForgetfulEquipmentModule below).
class FlippingDefersEquipmentModule : public PackmlNodeInterface
{
public:
  explicit FlippingDefersEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State monkey_state)
  : monkey_state_(monkey_state)
  {
    init(node);
  }

  /// Every defers_completion() query for monkey_state_ counts here (including the one-time
  /// query PackmlNodeInterface::init() makes for every state at construction) -- used only to
  /// confirm the flip was actually exercised by a test, not to predict which specific requests
  /// deferred (see FlippingDefersEquipmentModule's own class comment).
  std::atomic<int> query_count{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  // Only reached on the rounds the flip answered "defer" -- the framework asks for deferred work
  // exactly when it is going to wait for it, so the rounds that answered "no" have nothing to
  // orphan a report from.
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    std::thread([completion]() {
        std::this_thread::sleep_for(80ms);
        completion.report(true);
      }).detach();
  }
  bool defers_completion(packml_sm::State state) override
  {
    if (state != monkey_state_) {
      return false;
    }
    return (query_count.fetch_add(1) % 2) == 0;
  }

private:
  packml_sm::State monkey_state_;
};

/// EM that defers exactly one configured `monkey_state` and NEVER reports its completion -- a
/// chronic version of the timeout half of test_completion_integration.cpp's
/// ResettingTimesOutToAbortWithoutCompletion, exercised over several recovery cycles in a row
/// instead of just once.
class ChronicallyForgetfulEquipmentModule : public PackmlNodeInterface
{
public:
  explicit ChronicallyForgetfulEquipmentModule(
    rclcpp::Node::SharedPtr node, packml_sm::State monkey_state)
  : monkey_state_(monkey_state)
  {
    init(node);
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == monkey_state_;}
  // Takes the handle and drops it on the floor. Overridden rather than left to the base
  // implementation, which reports a failure immediately for a state nothing implements -- that
  // would defeat the point of this EM, which is to ride out the timeout.
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion) override {}

private:
  packml_sm::State monkey_state_;
};

/// EM that defers exactly one configured `monkey_state` and reports completion from a
/// background thread after a caller-specified, fixed delay -- used to land a report right at
/// (or on either side of) a deadline: the manager's own state_complete_timeout_ms, the EM's own
/// deferred_completion_timeout_ms, or an operator command's own timing, depending on the test.
class TimedReportEquipmentModule : public PackmlNodeInterface
{
public:
  TimedReportEquipmentModule(
    rclcpp::Node::SharedPtr node, packml_sm::State monkey_state,
    std::chrono::milliseconds report_delay)
  : monkey_state_(monkey_state), report_delay_(report_delay)
  {
    init(node);
  }

  std::atomic<int> reports_sent{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == monkey_state_;}
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    std::thread([this, completion]() {
        std::this_thread::sleep_for(report_delay_);
        completion.report(true);
        reports_sent.fetch_add(1);
      }).detach();
  }

private:
  packml_sm::State monkey_state_;
  std::chrono::milliseconds report_delay_;
};

}  // namespace

class MonkeyReportingTest : public ::testing::Test
{
protected:
  /// Same shape as test_monkey_scenarios.cpp's own Rig: every test builds its own standalone
  /// manager + EM pair (unique node names), rather than sharing fixture state.
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

  /// Extends test_monkey_scenarios.cpp's begin_setup() with an optional NodeOptions for the EM
  /// node -- a couple of tests below need to override the EM-side
  /// deferred_completion_timeout_ms parameter, which must be set before the EM's
  /// PackmlNodeInterface::init() runs (so it is passed as construction-time NodeOptions, not set
  /// afterward). Callers that don't need this just omit the argument. Does NOT construct the EM
  /// itself -- callers construct it on rig.em_node, then call finish_setup().
  Rig begin_setup(
    const std::string & mgr_prefix, const std::string & em_name,
    int state_complete_timeout_ms = 5000,
    const rclcpp::NodeOptions & em_node_options = rclcpp::NodeOptions())
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter(packml_ros::kParamNodeNames, std::vector<std::string>{em_name}),
        rclcpp::Parameter(packml_ros::kParamStateCompleteTimeoutMs, state_complete_timeout_ms),
      }));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.em_node = rclcpp::Node::make_shared(em_name, em_node_options);
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
// Confirmed-safe (expected): a redundant, contradictory second report for a goal that has
// already resolved must not corrupt anything for whatever transition defers NEXT. This
// deliberately gives the stale second report generous time (200ms) to land and settle as an
// inert orphan BEFORE driving the next cycle -- it is intentionally narrower than, and does not
// re-litigate, test_monkey_scenarios.cpp's own
// MonkeyResetAbortReset_StaleReportDoesNotResolveLaterReset, which is specifically about a stale
// report from an EARLIER, CANCELLED goal outliving its own cycle and colliding with a LATER goal
// revisiting the same state name. Here there is no cancellation and no state-name recurrence race
// in flight when the second report lands -- the question is only whether the next deferring goal
// starts from clean bookkeeping afterward.
TEST_F(MonkeyReportingTest, DoubleReport_SecondReportDoesNotCorruptNextTransition)
{
  auto rig = begin_setup("monkey_double_report", "double_report_em");
  auto em = std::make_shared<DoubleReportEquipmentModule>(rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  auto resp1 = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp1, nullptr);
  ASSERT_TRUE(resp1->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING for the first cycle";

  // The FIRST report (success=true, at ~50ms) should resolve this promptly, well before the
  // second, stale report (success=false, at ~100ms) even fires.
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 500ms))
    << "machine did not reach IDLE after the first (successful) report, current state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // Let the second, stale report land and settle as an inert orphan before moving on.
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(em->reports_sent.load(), 2) << "double-report EM did not send both reports";
  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::IDLE)
    << "the stale second report disturbed the already-settled IDLE state";

  // Drive a full ABORT->CLEAR->RESET cycle to revisit RESETTING and confirm the NEXT deferred
  // completion resolves correctly on its own merits, unaffected by the earlier double-report.
  auto abort_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1s))
    << "machine did not reach ABORTED";

  auto clear_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr);
  ASSERT_TRUE(clear_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 1s))
    << "machine did not reach STOPPED after CLEAR";

  auto resp2 = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp2, nullptr);
  ASSERT_TRUE(resp2->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING for the second cycle";
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 500ms))
    << "second RESETTING cycle did not resolve correctly on its own merits after the earlier "
       "double-report, current state: " << static_cast<int>(rig.sm_node->getCurrentState());

  // Let this cycle's own trailing stale report finish before teardown (detached thread, holds
  // a raw `this`).
  std::this_thread::sleep_for(150ms);

  // em (declared after rig) would otherwise destruct BEFORE rig's own em_spin member stops
  // spinning -- a still-in-flight fan-out callback calling into an object whose vtable is
  // already gone ("pure virtual method called"). Confirmed for real via gdb in
  // test_monkey_multi_mode.cpp's ClientDisconnectMidRequest test; stopping the spinner first,
  // explicitly, removes the race instead of just outrunning it.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected), exploratory: a contract-violating defers_completion() that flips
// its answer between calls never leaves a coordinated state stuck deferring forever -- each
// RESET cycle below reaches IDLE within a generous, bounded window regardless of which way the
// flip happened to fall that round. This degrades gracefully rather than hanging because the
// rounds that answered "no" complete synchronously in begin_transition() and are never asked for
// deferred work at all, while the rounds that answered "yes" get their report; the framework does
// not attempt to detect or reject the contract violation itself.
TEST_F(MonkeyReportingTest, ContractViolatingDefersCompletion_FlippingAnswerDegradesGracefully)
{
  auto rig = begin_setup("monkey_flip_defer", "flip_defer_em");
  auto em = std::make_shared<FlippingDefersEquipmentModule>(rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  for (int cycle = 0; cycle < 4; ++cycle) {
    auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
    ASSERT_NE(reset_resp, nullptr) << "cycle " << cycle << ": RESET got no response";
    ASSERT_TRUE(reset_resp->success) << "cycle " << cycle << ": RESET rejected";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
      << "cycle " << cycle << ": machine did not enter RESETTING";

    EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 500ms))
      << "cycle " << cycle << ": machine got stuck deferring RESETTING despite the EM's "
         "contract-violating defers_completion() flip, current state: "
      << static_cast<int>(rig.sm_node->getCurrentState());

    if (cycle < 3) {
      auto abort_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
      ASSERT_NE(abort_resp, nullptr) << "cycle " << cycle << ": ABORT got no response";
      ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1s))
        << "cycle " << cycle << ": machine did not reach ABORTED";
      auto clear_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
      ASSERT_NE(clear_resp, nullptr) << "cycle " << cycle << ": CLEAR got no response";
      ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 1s))
        << "cycle " << cycle << ": machine did not reach STOPPED after CLEAR";
    }
  }

  // Sanity check that the flip was actually exercised (init()'s own one-time probe of every
  // state already counts as 1, so more than 1 proves at least one real request re-queried it).
  EXPECT_GT(em->query_count.load(), 1)
    << "defers_completion() was not queried more than once -- test did not exercise the flip";

  std::this_thread::sleep_for(100ms);

  // See DoubleReport_SecondReportDoesNotCorruptNextTransition's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected): an EM that defers RESETTING but never reports, ridden out over 3
// ABORT/CLEAR/RESET-style recovery cycles in a row, must not degrade the manager's own
// responsiveness or drift in per-cycle timing -- either would indicate a leaked thread/timer
// accumulating across repeated timeouts. Uses a short, manager-side state_complete_timeout_ms
// (rather than the EM-side deferred_completion_timeout_ms) so this deliberately exercises the
// TIMEOUT->cancel_pending_state_goals() path: the manager's own wait_for_all() times out first,
// cancels the EM's outstanding goal (which the EM's own handle_cancel() resolves promptly), and
// the machine routes to ABORTING (not deferred by this EM, so ABORTING->ABORTED completes
// instantly every cycle).
TEST_F(MonkeyReportingTest, ChronicForgetfulness_ThreeConsecutiveTimeoutCyclesStayResponsive)
{
  auto rig = begin_setup("monkey_forgetful", "forgetful_em", /*state_complete_timeout_ms=*/300);
  auto em = std::make_shared<ChronicallyForgetfulEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  std::vector<std::chrono::milliseconds> cycle_durations;
  for (int cycle = 0; cycle < 3; ++cycle) {
    const auto cycle_start = std::chrono::steady_clock::now();

    auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
    ASSERT_NE(reset_resp, nullptr) << "cycle " << cycle << ": RESET got no response";
    ASSERT_TRUE(reset_resp->success) << "cycle " << cycle << ": RESET rejected";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
      << "cycle " << cycle << ": machine did not enter RESETTING";

    // Never reports -- the machine must still reach ABORTED via the manager's own timeout, not
    // hang indefinitely.
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
      << "cycle " << cycle << ": machine did not abort after RESETTING's completion timeout";

    cycle_durations.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - cycle_start));

    auto clear_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
    ASSERT_NE(clear_resp, nullptr) << "cycle " << cycle << ": CLEAR got no response";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 1s))
      << "cycle " << cycle << ": machine did not reach STOPPED after CLEAR";
  }

  // No drift: the last cycle's own RESET->ABORTED duration (dominated by the fixed 300ms
  // state_complete_timeout_ms) should stay close to the first cycle's -- growth here would
  // indicate a leaked thread/timer slowing later cycles down.
  ASSERT_EQ(cycle_durations.size(), 3u);
  const auto first_ms = cycle_durations.front().count();
  const auto last_ms = cycle_durations.back().count();
  const auto drift_ms = (last_ms > first_ms) ? (last_ms - first_ms) : (first_ms - last_ms);
  EXPECT_LT(drift_ms, 500)
    << "last cycle (" << last_ms << "ms) drifted far from the first (" << first_ms
    << "ms) -- possible cumulative slowdown across repeated forgetful timeout cycles";

  // Manager must still be promptly responsive to a final, clean command after all 3 cycles.
  const auto responsive_start = std::chrono::steady_clock::now();
  auto final_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR, 2s);
  const auto responsive_elapsed = std::chrono::steady_clock::now() - responsive_start;
  ASSERT_NE(final_resp, nullptr) << "manager stopped responding after 3 forgetful timeout cycles";
  EXPECT_LT(responsive_elapsed, 1s)
    << "final command took suspiciously long after repeated timeout cycles -- possible "
       "thread/timer leak";

  // See DoubleReport_SecondReportDoesNotCorruptNextTransition's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected): a report timed to land ~75ms BEFORE the EM's own
// deferred_completion_timeout_ms deadline resolves the transition via the report (IDLE),
// not the timeout (ABORTING) -- exactly one outcome, no crash.
TEST_F(MonkeyReportingTest, TimingBoundary_ReportJustBeforeDeadlineResolvesViaReport)
{
  constexpr int kDeferredTimeoutMs = 500;
  auto em_options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter(packml_ros::kParamDeferredCompletionTimeoutMs, kDeferredTimeoutMs)});
  auto rig = begin_setup("monkey_boundary_before", "boundary_before_em", 5000, em_options);
  auto em = std::make_shared<TimedReportEquipmentModule>(
    // 200 ms of margin, not 75. The two threads racing here -- the EM's report timer and the
    // deferred-completion deadline -- are separated by nothing but scheduling, so a margin near the
    // jitter of a loaded machine makes the test fail on the opposite, still-correct resolution path.
    rig.em_node, packml_sm::State::RESETTING, std::chrono::milliseconds(kDeferredTimeoutMs - 200));
  finish_setup(rig);

  const auto start = std::chrono::steady_clock::now();
  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 900ms))
    << "report landing just before the deadline did not resolve the transition, current "
       "state: " << static_cast<int>(rig.sm_node->getCurrentState());
  // Reaching IDLE at all already proves the report resolved this, not the timeout (a timeout
  // would have routed to ABORTING/ABORTED instead, never IDLE) -- this is a loose sanity bound
  // on top of that, not the primary evidence, so it stays generous to avoid flaking under load.
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 2s) << "resolving via the report took unexpectedly long";
  EXPECT_EQ(em->reports_sent.load(), 1);

  // See DoubleReport_SecondReportDoesNotCorruptNextTransition's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected): the mirror of the test above -- a report timed to land ~75ms
// AFTER the EM's own deferred_completion_timeout_ms deadline is resolved by the timeout
// (ABORTING/ABORTED), and the late report -- once it finally fires -- has no further effect:
// exactly one outcome, no crash.
TEST_F(MonkeyReportingTest, TimingBoundary_ReportJustAfterDeadlineResolvesViaTimeout)
{
  constexpr int kDeferredTimeoutMs = 500;
  auto em_options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter(packml_ros::kParamDeferredCompletionTimeoutMs, kDeferredTimeoutMs)});
  auto rig = begin_setup("monkey_boundary_after", "boundary_after_em", 5000, em_options);
  auto em = std::make_shared<TimedReportEquipmentModule>(
    // See the margin note on the sibling test above.
    rig.em_node, packml_sm::State::RESETTING, std::chrono::milliseconds(kDeferredTimeoutMs + 200));
  finish_setup(rig);

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // ABORTING is not deferred by this EM (it only monkey-behaves for RESETTING), so once the
  // deadline fails RESETTING out, ABORTING->ABORTED completes instantly.
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1500ms))
    << "machine did not abort after the deferred-completion deadline elapsed, current state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // Let the late report actually land, then confirm it had no effect: no crash, and the
  // already-settled ABORTED state must not move.
  //
  // Waited for, not slept through. The report fires a fixed margin AFTER the deadline, so any
  // constant here silently encodes that margin, and widening the margin to make the ordering
  // robust then invalidates the constant. This waits for the thing itself.
  packml_ros_test::wait_until(
    [&] {return em->reports_sent.load() >= 1;}, 3s, 10ms);
  EXPECT_EQ(em->reports_sent.load(), 1) << "the EM's late report never fired, so what follows "
                                           "would prove nothing about a late report";
  std::this_thread::sleep_for(150ms);   // give it time to do damage, if it is going to
  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::ABORTED)
    << "the late report disturbed the already-settled ABORTED state";

  // See DoubleReport_SecondReportDoesNotCorruptNextTransition's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected), based on reading CompletionTracker::wait_for_all()'s own check
// order: stop_token.stop_requested() is checked BEFORE any per-node result is examined (see
// its own implementation), so an accepted interrupt always wins over a report that arrives
// around the same time, at the MANAGER's level -- regardless of how the EM's own goal happens
// to resolve underneath at the ACTION level (CANCELED, or a race where its report narrowly
// beats the cancel notification there). This test characterizes that guarantee empirically:
// the machine must promptly leave RESETTING for ABORTING despite the near-simultaneous report,
// and must not hang or crash either way.
TEST_F(MonkeyReportingTest, CancelVsReportSimultaneity_InterruptingAbortWinsAtManagerLevel)
{
  constexpr int kReportDelayMs = 300;
  auto rig = begin_setup(
    "monkey_cancel_vs_report", "cancel_report_em", /*state_complete_timeout_ms=*/10000);
  auto em = std::make_shared<TimedReportEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING, std::chrono::milliseconds(kReportDelayMs));
  finish_setup(rig);

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // Let the fan-out reach the EM and its background report timer start, then send the
  // interrupting ABORT ~50ms before the report is due -- close enough, given fan-out/
  // scheduling slop, to count as "approximately the same moment".
  std::this_thread::sleep_for(std::chrono::milliseconds(kReportDelayMs - 50));
  auto abort_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success);

  // The interrupt must win promptly -- well under the (deliberately huge) 10s
  // state_complete_timeout_ms configured above, proving this is not just riding out the
  // ordinary timeout path instead.
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTING, 1s))
    << "machine did not reach ABORTING promptly despite the interrupting ABORT, current "
       "state: " << static_cast<int>(rig.sm_node->getCurrentState());
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1s))
    << "machine did not settle in ABORTED after the interrupt";

  // Let the near-simultaneous report actually fire and settle before teardown -- no crash, and
  // the machine must stay exactly where the interrupt left it, regardless of how the EM's own
  // goal resolved underneath.
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(em->reports_sent.load(), 1);
  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::ABORTED)
    << "state drifted after the near-simultaneous report finally landed";

  // See DoubleReport_SecondReportDoesNotCorruptNextTransition's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}
