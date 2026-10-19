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
// "Monkey" scenarios: aggressive, impatient-operator-style stress tests against the whole
// PackML state machine, rather than unit-level checks of one mechanism at a time. The whole
// packml_ros2 manager/EM pair is one big state machine reacting to commands, health events,
// and completion reports arriving in whatever order the real world delivers them -- these
// tests deliberately don't wait for things to settle before poking again, mirroring an
// operator mashing buttons on a faulting line, or an Equipment Module that is barely keeping
// up under load.
//
// Some of these are confirmed-safe regression tests (an assertion that SHOULD hold, and
// does). At least one (MonkeyResetAbortReset_StaleReportResolvesLaterResetTooEarly) is a
// KNOWN, currently-unfixed gap: it asserts the correct, intended behavior and is EXPECTED TO
// FAIL until that gap is actually closed (goal-ID-keyed completion tracking, or an
// equivalent fix). Do not "fix" a red result here by loosening its assertion; either build
// the real fix, or mark it skipped with a comment explaining why, but do not silently
// weaken it.
//
// BACKLOG -- not built yet, the EM (not just the operator/manager) as the unreliable party:
//   Completion/report misbehavior: double-report for the same goal; a defers_completion()
//     that flips its answer between calls (contract violation); a chronically forgetful EM
//     across many cycles in a row (leak/degradation check, not just one timeout).
//   Health/heartbeat chaos: rapid HEALTHY/ERROR flapping; heartbeat-sequence-reset flapping
//     COMBINED with operator mashing; scrambled/reordered/duplicated sequence numbers;
//     heartbeat_interval_ms flapping instead of a fixed value.
//   Fan-out/discovery chaos: EM action server flapping present/absent during one fan-out
//     round; EM vanishing after accepting a goal but before reporting.
//   Multi-EM chaos: several EMs registered together, one a slow-monkey, cycling repeatedly.
//   Broader untested mechanisms: mode-transition-service monkey (separate fan-out path, no
//     coverage at all yet); a TRUE concurrent multi-command burst (no gap, multiple threads,
//     sharper than the existing sequential flood test); a wedged EM executor
//     (on_state_trans_req() itself blocks forever -- no heartbeat, no cancel response, no
//     result, ever); nonsensical result content (success=true with a populated error_code);
//     a deliberately-timed health-unhealthy-vs-report-arrival simultaneity race.
//   Bigger/different shape (manager itself restarts mid-cycle, not the EM) -- worth covering,
//     but it doesn't fit this file's current shape as directly as the above.

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
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// Fire-and-don't-wait: send a command and return immediately with the pending future,
/// without blocking for the response. Used by the flood test below, where the whole point
/// is to not wait for one command to settle before sending the next.
std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr> send_state_change_async(
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client,
  int8_t command)
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = command;
  return client->async_send_request(req).future.share();
}

/// Plain, non-deferring EM -- completes every coordinated state instantly. Used by
/// scenarios that stress the MANAGER's own command handling rather than any EM-side
/// completion-report staleness.
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

/// EM whose work for a given coordinated `state` is real background work decoupled from
/// the ROS goal's own cancellation -- mirroring a "barely keeping up" node under real load:
/// the underlying work (e.g. a homing motion already issued to hardware) keeps running to
/// completion and reports whenever IT finishes, regardless of whether the ROS action that
/// requested it was cancelled in the meantime. Only defers/does background work for the one
/// state it's constructed for, so it behaves like PlainEquipmentModule for every other state.
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

  // Captures the goal's own completion by value and reports through it whenever the work
  // finishes, cancelled or not -- the "barely keeping up" node this class is modelling. The
  // handle belongs to the goal that asked for the work, so a report that arrives after that
  // goal is gone resolves nothing.
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

}  // namespace

class MonkeyScenariosTest : public ::testing::Test
{
protected:
  /// Every test builds its own standalone manager + EM pair (unique node names), rather
  /// than sharing fixture state -- each scenario needs different EM behavior and command
  /// sequencing, and sharing one manager across scenarios would make failures in one bleed
  /// into another.
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

  /// Constructs the manager/EM pair and gets both spinning, but does NOT construct the EM
  /// itself -- callers need different EM subclasses, so they construct it (on rig.em_node)
  /// and start its SpinHelper themselves, then call finish_setup().
  Rig begin_setup(
    const std::string & mgr_prefix, const std::string & em_name,
    int state_complete_timeout_ms = 5000)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("node_names", std::vector<std::string>{em_name}),
        rclcpp::Parameter("state_complete_timeout_ms", state_complete_timeout_ms),
      }));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.em_node = rclcpp::Node::make_shared(em_name);
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/changeState");
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
// "System has a problem -> ABORT. Operator slamming the reset button: CLEAR -> RESET. EM
// nodes barely keeping up, triggering another internal error -> ABORT. Operator still
// slamming the reset button." (verbatim scenario this reproduces.)
//
// A stale report from an EARLIER, cancelled RESETTING attempt (Goal A) must not resolve a LATER
// RESETTING attempt's wait (Goal C): only Goal C's own work may complete Goal C. RESETTING recurs
// here non-consecutively, through a full ABORT->CLEAR->RESET cycle, so both attempts carry the
// same state NAME -- which is all a name-keyed completion slot had to tell them apart with, and
// why it resolved the wrong one. Each goal now has its own completion record, reachable only
// through the handle handed to that goal's on_deferred_work(), so Goal A's work reports into
// Goal A's record however late it finishes.
//
// Replace that per-goal record with a shared, name-matched slot and this fails reliably: the
// machine reaches IDLE with only one of the two background work items finished.
TEST_F(MonkeyScenariosTest, MonkeyResetAbortReset_StaleReportDoesNotResolveLaterReset)
{
  auto rig = begin_setup("monkey_reset_abort_reset", "monkey_reset_em");
  auto em = std::make_shared<MonkeyStaleWorkEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING, 1500ms);
  finish_setup(rig);

  // --- Operator's first RESET press: Goal A ---
  auto resp1 = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp1, nullptr);
  ASSERT_TRUE(resp1->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING for Goal A";
  {
    packml_ros_test::wait_until(
      [&] {return em->work_started.load() > 0;}, 500ms, 10ms);
    ASSERT_EQ(em->work_started.load(), 1) << "Goal A's background work never started";
  }

  // --- "EM nodes barely keeping up, triggering another internal error -> ABORT" ---
  // Fired well before Goal A's background work (1500ms) finishes.
  std::this_thread::sleep_for(100ms);
  auto abort_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTING, 500ms))
    << "machine did not enter ABORTING";
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1s))
    << "machine did not reach ABORTED (ABORTING is not deferred by this EM)";

  // --- "Operator still slamming the reset button": CLEAR then RESET, no delay ---
  auto clear_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr);
  ASSERT_TRUE(clear_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 1s))
    << "machine did not reach STOPPED after CLEAR";

  const auto reset2_start = std::chrono::steady_clock::now();
  auto resp2 = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp2, nullptr);
  ASSERT_TRUE(resp2->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING for Goal C";

  packml_ros_test::wait_until(
    [&] {return rig.sm_node->getCurrentState() != packml_sm::State::RESETTING;}, 3s, 10ms);
  const auto elapsed_since_reset2 = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - reset2_start);

  // Goal C's OWN background work (started fresh when this second RESETTING began) needs
  // ~1500ms. work_finished is a shared counter across both goals' background threads --
  // if the machine reached IDLE while it still reads < 2, Goal C's own work had not
  // finished, so the only thing that could have resolved it is Goal A's stale report.
  const bool resolved_by_stale_report =
    rig.sm_node->getCurrentState() == packml_sm::State::IDLE && em->work_finished.load() < 2;
  EXPECT_FALSE(resolved_by_stale_report)
    << "Goal C was resolved before its own work finished, so Goal A's orphaned report resolved it "
    << "(machine state " << static_cast<int>(rig.sm_node->getCurrentState()) << ", work finished "
    << em->work_finished.load() << "/2, " << elapsed_since_reset2.count() << "ms after Goal C's "
       "RESET)";

  // The other half of the same guarantee: Goal C must still complete on its OWN work rather than
  // being stranded now that a stale report cannot resolve it. Both background items are 1500ms and
  // the loop above allowed 3s, so by here Goal C's own work has had its time.
  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::IDLE)
    << "Goal C never completed on its own work either -- state "
    << static_cast<int>(rig.sm_node->getCurrentState()) << " after "
    << elapsed_since_reset2.count() << "ms, work finished " << em->work_finished.load() << "/2";

  // Let any still-running background work finish before teardown so it doesn't outlive the
  // test (the background threads are detached and hold a raw `this`).
  std::this_thread::sleep_for(2000ms);

  // em (declared after rig) would otherwise destruct BEFORE rig's own em_spin member stops
  // spinning -- a still-in-flight fan-out callback calling into an object whose vtable is
  // already gone ("pure virtual method called"). Confirmed for real via gdb in a sibling
  // file's ClientDisconnectMidRequest test; stopping the spinner first, explicitly, removes
  // the race instead of just outrunning it.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe regression test for the ABORTING health-cross-check dependency: an
// operator mashing ABORT repeatedly while the machine is already ABORTING/ABORTED must not
// crash, hang, or corrupt the machine's state -- each redundant command should be accepted
// or safely rejected, and the manager must stay responsive throughout.
TEST_F(MonkeyScenariosTest, MonkeyAbortMashing_RedundantAbortsAreHarmless)
{
  auto rig = begin_setup("monkey_abort_mash", "mash_em");
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_node);
  finish_setup(rig);

  auto first = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
    << "machine did not reach ABORTED after the first ABORT";

  // Mash it another 10 times with no delay -- every one of these is redundant (already
  // ABORTED) and must resolve quickly (no command may hang waiting on a stuck guard).
  for (int i = 0; i < 10; ++i) {
    const auto start = std::chrono::steady_clock::now();
    auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT, 2s);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    ASSERT_NE(resp, nullptr) << "mashed ABORT #" << i << " never got a response at all";
    EXPECT_LT(elapsed, 1s) << "mashed ABORT #" << i << " took suspiciously long to answer";
  }

  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::ABORTED)
    << "machine drifted out of ABORTED after redundant ABORT mashing";

  // Manager must still be responsive to a real, different command afterward.
  auto clear_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr) << "manager stopped responding after the ABORT mashing";
  EXPECT_TRUE(clear_resp->success);
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "machine did not recover normally via CLEAR after the mashing";

  // See MonkeyResetAbortReset_StaleReportResolvesLaterResetTooEarly's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Broad crash/hang-safety stress test: fire a chaotic sequence of different commands
// without waiting for any of them to resolve first -- closer to a monkey testing tool than
// a scripted operator. Does not assert a specific end state (that's genuinely
// order/timing-dependent and not the point); asserts only that the manager survives the
// flood without hanging and is still normally responsive once it settles.
TEST_F(MonkeyScenariosTest, MonkeyCommandFlood_ManagerSurvivesAndStaysResponsive)
{
  auto rig = begin_setup("monkey_flood", "flood_em");
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_node);
  finish_setup(rig);

  static constexpr int8_t kFlood[] = {
    packml_msgs::srv::StateChange::Request::RESET,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::CLEAR,
    packml_msgs::srv::StateChange::Request::RESET,
    packml_msgs::srv::StateChange::Request::HOLD,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::CLEAR,
    packml_msgs::srv::StateChange::Request::RESET,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::CLEAR,
  };

  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> pending;
  for (const auto command : kFlood) {
    pending.push_back(send_state_change_async(rig.state_client, command));
    // A deliberately tiny, non-zero gap -- true zero-delay back-to-back sends would just
    // queue every request into the same executor tick, which exercises the service queue
    // more than the state machine. A few ms lets the manager actually start processing
    // each one before the next arrives, which is closer to real (if very fast) mashing.
    std::this_thread::sleep_for(5ms);
  }

  // Every queued request must eventually get SOME response -- none may be silently dropped
  // or leave the manager hanging on it forever.
  for (size_t i = 0; i < pending.size(); ++i) {
    ASSERT_EQ(pending[i].wait_for(5s), std::future_status::ready)
      << "flood command #" << i << " (cmd=" << static_cast<int>(kFlood[i])
      << ") never got a response -- possible hang";
  }

  // The machine must settle into SOME stable, recognized state shortly after the flood,
  // not keep oscillating or freeze mid-transition.
  EXPECT_TRUE(packml_ros_test::wait_for_settled_state(rig.sm_node, 3s, 20).has_value())
    << "machine never settled into a stable state within 3s of the flood, currently: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // The manager must still be normally responsive to a clean command after the flood.
  auto stop_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_resp, nullptr) << "manager stopped responding after the command flood";

  // See MonkeyResetAbortReset_StaleReportResolvesLaterResetTooEarly's own comment above: stop
  // em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}
