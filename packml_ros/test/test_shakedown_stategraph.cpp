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
// SHAKEDOWN suite (separate binary), state-graph group: chaos-style probes into coordinated
// states and transition paths the monkey suites (test_monkey_scenarios.cpp,
// test_monkey_edge_cases.cpp, test_monkey_health.cpp, test_monkey_multi_mode.cpp) never
// exercise -- the dark corners of the PackML state graph itself:
//   1. HOLDING recurring across a full ABORT->CLEAR->RESET->START->HOLD cycle, probing whether
//      the RESETTING-shaped stale-report KNOWN GAP (test_monkey_scenarios.cpp's
//      MonkeyResetAbortReset_StaleReportResolvesLaterResetTooEarly) generalizes to every
//      state-name-keyed recurrence, not just RESETTING;
//   2. a required node going unhealthy while CLEARING's coordinated wait is in flight --
//      CLEARING is a descendant of the `abortable` superstate, so unlike the known ABORTING
//      deadlock the error transition EXISTS here and must route;
//   3. SUSPENDING/UNSUSPENDING as deferred coordinated states (first coverage anywhere);
//   4. STOPPING as a deferred coordinated state (first coverage anywhere).
//
// State-graph facts these tests rest on, verified in packml_sm's
// states_generator.hpp::generate_all_packml_states(): the `abortable` superstate directly
// parents Clearing, Stopping, Stopped, and the `stoppable` superstate
// (`ActingState::Clearing(abortable, ...)`, `ActingState::Stopping(abortable, ...)`,
// `WaitState::Stopped(abortable)`, `PackmlSuperState::Stoppable(abortable)`); `stoppable` in
// turn parents Resetting, Idle, Starting, Execute, Holding, Held, Unholding, Suspending,
// Suspended, Unsuspending, Completing, and Complete -- so all of those are descendants of
// abortable. Aborting and Aborted are constructed with NO parent (`ActingState::Aborting(...)`,
// `WaitState::Aborted()`) -- siblings of abortable, not members. The ONLY ERROR-type
// transition in the whole graph is `abortable_aborting_on_error`, added to `abortable`
// itself, which is why an ErrorEvent posted from inside any abortable descendant routes to
// Aborting, while one posted while IN Aborting matches nothing and is silently dropped -- the
// known ABORTING deadlock.
//
// Matches the monkey suites' conventions: scenario-specific helpers in this file's own anonymous
// namespace (generic scaffolding lives in test_helpers.hpp), a Rig + begin_setup()/finish_setup()
// pair per test with unique node names, and assertions that state the
// intended CORRECT behavior with generous bounds -- a red result here is a finding to triage,
// never a cue to loosen the assertion. Test 1 below is EXPECTED TO FAIL if its timing window
// hits (a suspected twin of the known RESETTING gap); the other three assert behavior the
// contracts say must hold today.
//
// TEARDOWN RULE (confirmed via gdb on a sibling file's ClientDisconnectMidRequest test): any
// EM object declared AFTER the Rig destructs BEFORE the Rig's em_spin SpinHelper stops -- a
// still-in-flight fan-out callback then calls into an object whose vtable is already gone
// ("pure virtual method called"). Every test ends with rig.em_spin.reset() after all
// assertions, and tests whose EMs run detached background-work threads holding a raw `this`
// sleep long enough for that work to finish first.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
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

/// EM that defers completion for a configurable SET of coordinated states, doing real
/// background work for each on a detached thread decoupled from the ROS goal's own
/// cancellation -- test_monkey_scenarios.cpp's MonkeyStaleWorkEquipmentModule shape,
/// generalized from one state to several with per-state started/finished counters. The
/// decoupling is what test 1 exploits (work keeps running past its goal being cancelled and
/// reports whenever IT finishes); for tests 3 and 4 nothing ever interrupts the work, so the
/// same class simply behaves as a well-mannered deferring EM. Every state NOT in the work
/// spec behaves like a plain, instantly-approving EM.
class DeferredWorkEquipmentModule : public PackmlNodeInterface
{
public:
  using WorkSpec = std::vector<std::pair<packml_sm::State, std::chrono::milliseconds>>;

  DeferredWorkEquipmentModule(rclcpp::Node::SharedPtr node, WorkSpec work_spec)
  {
    // Fully populate the map BEFORE init() starts serving goals; afterward it is only ever
    // read (find/at), so the detached work threads and the test thread never race a rehash.
    for (const auto & [state, duration] : work_spec) {
      work_[state].duration = duration;
    }
    init(node);
  }

  int started(packml_sm::State state) const {return work_.at(state).started.load();}
  int finished(packml_sm::State state) const {return work_.at(state).finished.load();}

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return work_.count(state) > 0;}
  void on_deferred_work(packml_sm::State state, packml_ros::DeferredCompletion completion) override
  {
    auto & entry = work_.at(state);
    entry.started.fetch_add(1);
    std::thread([&entry, completion]() {
        std::this_thread::sleep_for(entry.duration);
        entry.finished.fetch_add(1);
        completion.report(true);
      }).detach();
  }

private:
  struct DeferredWork
  {
    std::chrono::milliseconds duration{0};
    std::atomic<int> started{0};
    std::atomic<int> finished{0};
  };
  std::map<packml_sm::State, DeferredWork> work_;
};

/// Combines test_monkey_edge_cases.cpp's RacingHealthEquipmentModule pattern (health
/// controllable via atomics, reported on every heartbeat tick through get_health_status())
/// with DeferredWorkEquipmentModule's detached-background-work deferral for ONE state, so a
/// test can flip the node unhealthy while that state's coordinated wait is definitely still
/// pending on it -- which is exactly what makes CompletionTracker::wait_for_all()'s health
/// cross-check (completion_tracker.hpp) see the unhealthy node instead of the round having
/// already completed on acceptance.
class FlippableHealthEquipmentModule : public PackmlNodeInterface
{
public:
  FlippableHealthEquipmentModule(
    rclcpp::Node::SharedPtr node, packml_sm::State defer_state,
    std::chrono::milliseconds work_duration)
  : defer_state_(defer_state), work_duration_(work_duration)
  {
    init(node);
  }

  std::atomic<int32_t> health_status{packml_msgs::msg::NodeHealth::HEALTHY};
  std::atomic<int32_t> health_action{packml_msgs::msg::NodeHealth::NONE};
  std::atomic<int> work_started{0};
  std::atomic<int> work_finished{0};

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
    work_started.fetch_add(1);
    std::thread([this, completion]() {
        std::this_thread::sleep_for(work_duration_);
        work_finished.fetch_add(1);
        completion.report(true);
      }).detach();
  }

private:
  packml_sm::State defer_state_;
  std::chrono::milliseconds work_duration_;
};

}  // namespace

class StateGraphMonkeyTest : public ::testing::Test
{
protected:
  /// Every test builds its own standalone manager + EM pair (unique node names) -- see
  /// test_monkey_scenarios.cpp's Rig doc comment for why. Carries the optional extra manager
  /// parameters / EM node options extension from test_monkey_edge_cases.cpp, since the
  /// health-driven CLEARING probe needs required_nodes + heartbeat configuration.
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
    // Normalizer only: the machine boots into STOPPED (abortable's initial substate, see
    // states_generator.hpp), where STOP has no matching transition (stoppable_stopping lives
    // on the `stoppable` superstate and Stopped sits OUTSIDE stoppable, directly under
    // abortable) -- so this is rejected harmlessly and never trips a STOPPING-deferring EM.
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
    std::this_thread::sleep_for(300ms);
  }

  /// Drive the machine from its post-setup STOPPED through RESET->IDLE->START->EXECUTE.
  /// None of the states crossed here are deferred by any EM in this file, so generous 2s
  /// bounds per leg are pure slack, not tuned timing.
  void drive_to_execute(Rig & rig)
  {
    auto reset_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
    ASSERT_NE(reset_resp, nullptr);
    ASSERT_TRUE(reset_resp->success) << "RESET rejected: " << reset_resp->message;
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 2s))
      << "machine did not reach IDLE after RESET";
    auto start_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::START);
    ASSERT_NE(start_resp, nullptr);
    ASSERT_TRUE(start_resp->success) << "START rejected: " << start_resp->message;
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::EXECUTE, 2s))
      << "machine did not reach EXECUTE after START";
  }
};

// ============================================================================
// SUSPECTED KNOWN-GAP TWIN, written as the correct behavior and therefore EXPECTED TO FAIL
// whenever its timing window hits -- triage a red result as a finding, do not loosen it.
//
// The HOLDING-shaped twin of test_monkey_scenarios.cpp's
// MonkeyResetAbortReset_StaleReportDoesNotResolveLaterReset. The contract under test is that a
// deferred completion belongs to ONE goal: while a name-keyed completion slot was all
// packml_interface.hpp had, any coordinated state that RECURS with the same name -- not just
// RESETTING -- could have an earlier, cancelled goal's late report resolve a later goal's wait.
// HOLDING recurs here via a full ABORT->CLEAR->RESET->START->HOLD cycle:
//
//   Goal A: EXECUTE->HOLD, machine enters HOLDING, EM starts ~1500ms decoupled background
//           work. ~100ms in, ABORT interrupts it (HOLDING is a descendant of abortable, so
//           the command transition on the superstate fires); ABORTING is not deferred by this
//           EM and completes immediately -> ABORTED. Goal A's work keeps running regardless
//           (that is the MonkeyStaleWork shape) and will report HOLDING complete at ~1500ms.
//   Goal C: CLEAR->RESET->START->HOLD drives a SECOND HOLDING, whose own work needs its own
//           ~1500ms. Goal A's stale report can land while Goal C's HOLDING is deferring, and
//           must not resolve it: each goal now has its own completion record, so Goal A's work
//           reports into Goal A's record whenever it finishes.
//
// Detection mirrors the RESETTING test exactly: finished-work counters are shared across both
// goals' background threads and each thread increments finished BEFORE reporting, so the machine
// reaching HELD while finished(HOLDING) < 2 would prove Goal C was resolved by something other
// than its own work -- only Goal A's stale report is left.
//
// This half of the pair is timing-dependent in one direction: if the intermediate
// CLEAR->RESET->START chain outlasts Goal A's remaining ~1400ms, the stale report lands mid-chain
// rather than during Goal C's own HOLDING, and the run proves nothing either way. That makes it
// weaker evidence than the RESETTING original, which lands in the window every time -- so treat a
// green run here as "did not contradict", and the RESETTING test as the one that gates.
TEST_F(StateGraphMonkeyTest, HoldingStaleReportDoesNotResolveSecondHold)
{
  const auto em_name = packml_ros_test::unique_node_name("shk_hold_stale_em");
  auto rig = begin_setup("shk_hold_stale", em_name);
  auto em = std::make_shared<DeferredWorkEquipmentModule>(
    rig.em_node,
    DeferredWorkEquipmentModule::WorkSpec{{packml_sm::State::HOLDING, 1500ms}});
  finish_setup(rig);

  drive_to_execute(rig);

  // --- Goal A: first HOLD ---
  auto hold_a = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::HOLD);
  ASSERT_NE(hold_a, nullptr);
  ASSERT_TRUE(hold_a->success) << "first HOLD rejected: " << hold_a->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::HOLDING, 500ms))
    << "machine did not enter HOLDING for Goal A";
  {
    packml_ros_test::wait_until(
      [&] {return em->started(packml_sm::State::HOLDING) > 0;}, 500ms, 10ms);
    ASSERT_EQ(em->started(packml_sm::State::HOLDING), 1)
      << "Goal A's background work never started";
  }

  // --- ABORT well before Goal A's 1500ms work finishes ---
  std::this_thread::sleep_for(100ms);
  auto abort_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 1s))
    << "machine did not reach ABORTED (ABORTING is not deferred by this EM)";

  // --- Back around the graph to a second HOLDING: CLEAR -> RESET -> START -> HOLD ---
  auto clear_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr);
  ASSERT_TRUE(clear_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 1s))
    << "machine did not reach STOPPED after CLEAR";
  drive_to_execute(rig);

  const auto hold_c_start = std::chrono::steady_clock::now();
  auto hold_c = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::HOLD);
  ASSERT_NE(hold_c, nullptr);
  ASSERT_TRUE(hold_c->success) << "second HOLD rejected: " << hold_c->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::HOLDING, 500ms))
    << "machine did not enter HOLDING for Goal C";
  {
    packml_ros_test::wait_until(
      [&] {return em->started(packml_sm::State::HOLDING) >= 2;}, 500ms, 10ms);
    ASSERT_EQ(em->started(packml_sm::State::HOLDING), 2)
      << "Goal C's background work never started";
  }

  // Observe how (and how fast) the second HOLDING resolves.
  packml_ros_test::wait_until(
    [&] {return rig.sm_node->getCurrentState() != packml_sm::State::HOLDING;}, 3s, 10ms);
  const auto elapsed_since_hold_c = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - hold_c_start);

  // Either path (Goal C's own work at ~1500ms, or the stale early resolution) must end in
  // HELD well inside the 3s observation -- never reaching HELD at all would be a different,
  // worse failure than the stale-report gap this test is aimed at.
  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::HELD)
    << "second HOLDING never completed at all within 3s -- current state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  const bool resolved_by_stale_report =
    rig.sm_node->getCurrentState() == packml_sm::State::HELD &&
    em->finished(packml_sm::State::HOLDING) < 2;
  EXPECT_FALSE(resolved_by_stale_report)
    << "the second HOLDING was resolved before its own work finished, so Goal A's orphaned report "
    << "resolved it -- machine state " << static_cast<int>(rig.sm_node->getCurrentState())
    << ", HOLDING work finished " << em->finished(packml_sm::State::HOLDING) << "/2, "
    << elapsed_since_hold_c.count() << "ms after Goal C's HOLD";

  // Let any still-running detached background work (raw `this`) finish before teardown.
  std::this_thread::sleep_for(2000ms);

  // See the file-header TEARDOWN RULE: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Correct-behavior probe: a required node going unhealthy while CLEARING's coordinated wait
// is in flight must route the machine to ABORTING -- because Clearing, unlike Aborting, IS a
// descendant of the `abortable` superstate, so the error transition exists for it.
//
// Contract chain, each link verified in source:
//   - CompletionTracker::wait_for_all() (completion_tracker.hpp) cross-checks every still-
//     pending node against the health predicate and returns ABORTED_BY_HEALTH for a required
//     node that is unhealthy -- the EM defers CLEARING with ~800ms of work precisely so the
//     round is still pending when that cross-check runs (a non-deferring EM would complete on
//     acceptance and the wait would return COMPLETE before health ever mattered).
//   - The setStateOperation-bound function (packml_interface.hpp, kCoordinatedStates loop)
//     returns nonzero for ABORTED_BY_HEALTH, and ActingState's operation (packml_sm's
//     acting_state.cpp) posts an ErrorEvent for a nonzero return.
//   - states_generator.hpp::generate_all_packml_states() constructs Clearing INSIDE abortable
//     (`ActingState::Clearing(abortable, delay_ms)`) and adds the graph's only ERROR-type
//     transition to abortable itself (`abortable->addTransition(abortable_aborting_on_error)`)
//     -- so an ErrorEvent posted while in Clearing has a matching transition to Aborting.
//
// Deliberately NOT asserted: ABORTED afterward. The node is still unhealthy when ABORTING's
// own coordinated wait runs the same cross-check, so that wait fails too -- and the resulting
// ErrorEvent is posted while IN Aborting, which is a SIBLING of abortable with no error
// transition of its own: the event is silently dropped and the machine is permanently stuck
// in ABORTING. That is the confirmed KNOWN GAP, with its own intentionally-failing test
// elsewhere; re-asserting it here would just double-count it. This
// test ends at the ABORTING assertion on purpose -- proving the error ROUTING works wherever
// the graph supports it, which sharpens the known deadlock's diagnosis: it is purely about
// Aborting's sibling position in the graph, not about error routing in general.
TEST_F(StateGraphMonkeyTest, UnhealthyDuringClearing_ErrorRoutesToAborting)
{
  const auto em_name = packml_ros_test::unique_node_name("shk_clear_health_em");
  std::vector<rclcpp::Parameter> extra_mgr_params = {
    rclcpp::Parameter("required_nodes", std::vector<std::string>{em_name}),
    rclcpp::Parameter("heartbeat_timeout_factor", 3.0),
  };
  auto rig = begin_setup(
    "shk_clear_health", em_name, /*state_complete_timeout_ms=*/5000, extra_mgr_params,
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("heartbeat_interval_ms", 100)}));
  auto em = std::make_shared<FlippableHealthEquipmentModule>(
    rig.em_node, packml_sm::State::CLEARING, 800ms);
  finish_setup(rig);

  // Let several healthy 100ms heartbeats reach the manager first, so the health monitor has
  // a live, healthy picture of the required node before anything is commanded. Waited for
  // rather than slept through; see wait_for_healthy_heartbeats().
  ASSERT_GE(packml_ros_test::wait_for_healthy_heartbeats(rig.mgr_node, em_name), 2)
    << "no heartbeats from " << em_name << " reached the manager -- without a healthy baseline "
       "the flip to unhealthy below is not a transition the monitor can observe";

  auto abort_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success) << "ABORT rejected while healthy: " << abort_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
    << "machine did not reach ABORTED while the EM was still healthy";

  // Flip the node unhealthy (ERROR / action=ABORT) and give the manager several heartbeat
  // ticks to see it. The health monitor's own fired ABORT action is a harmless no-op here --
  // the machine is already in ABORTED.
  em->health_status.store(packml_msgs::msg::NodeHealth::ERROR);
  em->health_action.store(packml_msgs::msg::NodeHealth::ABORT);
  std::this_thread::sleep_for(400ms);

  // CLEAR must be ACCEPTED despite the unhealthy node: on_change_state()'s health gate
  // (packml_interface.hpp) guards only RESET-from-STOPPED, and aborted_clearing is an
  // ordinary command transition on Aborted. The health consequence lands later, inside
  // CLEARING's own coordinated wait.
  auto clear_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr);
  ASSERT_TRUE(clear_resp->success)
    << "CLEAR from ABORTED should be accepted regardless of node health (the gate only "
       "guards RESET from STOPPED): " << clear_resp->message;

  // The machine must leave CLEARING via the error path. Accept ABORTING *or* ABORTED as proof
  // it took that path: neither intermediate state is reliably observable by polling, because
  // the unhealthy fact predates both waits and the cross-check can fail each within
  // milliseconds of entry.
  //
  // The trace is CLEARING -> ABORTING -> ABORTED, and the machine does not linger in ABORTING:
  // its own wait fails on the same unhealthy node, and the Aborting --ERROR--> Aborted edge
  // carries it straight through. Polling for ABORTING alone would miss that window and read
  // ABORTED (9) as a failure, so the full sequence is asserted below.
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    bool left_clearing_via_error = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto s = rig.sm_node->getCurrentState();
      if (s == packml_sm::State::ABORTING || s == packml_sm::State::ABORTED) {
        left_clearing_via_error = true;
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(left_clearing_via_error)
      << "machine did not route CLEARING's health-failed wait through the error path -- the "
         "error transition on the abortable superstate should have matched (Clearing is a "
         "descendant of abortable); current state: "
      << static_cast<int>(rig.sm_node->getCurrentState());
  }

  // And it must not stop in ABORTING. This is the half that was impossible before the error
  // edge: ABORTING's own coordinated wait still fails on the same unhealthy node, but its
  // ErrorEvent now has somewhere to go, so the machine reaches the safe terminal instead of
  // trapping. A failure here means the edge regressed and the wedge is back.
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
    << "machine stopped in ABORTING instead of reaching ABORTED -- the "
       "Aborting --ERROR--> Aborted edge (states_generator.hpp) is the only thing that lets a "
       "failed abort escape; current state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // Flip the node healthy again and let things quiesce so teardown is not fighting a
  // still-firing health monitor, and let the EM's ~800ms CLEARING work thread (raw `this`)
  // run out before the EM destructs.
  em->health_status.store(packml_msgs::msg::NodeHealth::HEALTHY);
  em->health_action.store(packml_msgs::msg::NodeHealth::NONE);
  std::this_thread::sleep_for(1200ms);

  // See the file-header TEARDOWN RULE: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Correct-behavior probe, first-ever coverage of SUSPENDING and UNSUSPENDING as DEFERRED
// coordinated states: both are in packml_interface.hpp's kCoordinatedStates and both are
// ordinary acting states inside `stoppable` with plain command/state-complete transitions
// (states_generator.hpp: execute_suspending, suspending_suspended, suspended_unsuspending,
// unsuspending_execute), yet no test anywhere drives them with an EM that actually defers
// them -- every existing suite either never leaves the RESET/ABORT axis or crosses the
// suspend leg with an instantly-completing EM. If deferral bookkeeping (deferral-start reset,
// state-name matching, goal completion) held for RESETTING/HOLDING but subtly broke for these
// states, nothing today would notice.
//
// A well-behaved EM defers BOTH with short (~150ms) background work, and the full
// EXECUTE->SUSPENDING->SUSPENDED->UNSUSPENDING->EXECUTE cycle is driven TWICE: recurrence of
// the same deferred states across consecutive cycles (with nothing cancelled in between) is
// exactly what the per-state counters pin down -- each leg must complete within a generous
// bound and each counter must track the cycle count exactly, no lost and no doubled work.
// SUSPEND/UNSUSPEND command values are packml_msgs::srv::StateChange::Request::SUSPEND (6)
// and ::UNSUSPEND (7) per StateChange.srv. A final ABORT->ABORTED proves the machine is still
// normally interruptible after repeated suspend cycling.
TEST_F(StateGraphMonkeyTest, SuspendCycleDeferred_SuspendingAndUnsuspendingComplete)
{
  const auto em_name = packml_ros_test::unique_node_name("shk_suspend_em");
  auto rig = begin_setup("shk_suspend", em_name);
  auto em = std::make_shared<DeferredWorkEquipmentModule>(
    rig.em_node,
    DeferredWorkEquipmentModule::WorkSpec{
      {packml_sm::State::SUSPENDING, 150ms},
      {packml_sm::State::UNSUSPENDING, 150ms}});
  finish_setup(rig);

  drive_to_execute(rig);

  for (int cycle = 1; cycle <= 2; ++cycle) {
    SCOPED_TRACE("suspend/unsuspend cycle " + std::to_string(cycle));

    auto suspend_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::SUSPEND);
    ASSERT_NE(suspend_resp, nullptr);
    ASSERT_TRUE(suspend_resp->success) << "SUSPEND rejected: " << suspend_resp->message;
    // The deferred wait pins the machine in SUSPENDING for the ~150ms of EM work, so the
    // 10ms-polling wait below observes the acting state itself, not just its outcome.
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::SUSPENDING, 1s))
      << "machine was never observed in SUSPENDING";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::SUSPENDED, 2s))
      << "deferred SUSPENDING did not complete into SUSPENDED";
    EXPECT_EQ(em->started(packml_sm::State::SUSPENDING), cycle)
      << "SUSPENDING work-started count off after cycle " << cycle;
    EXPECT_EQ(em->finished(packml_sm::State::SUSPENDING), cycle)
      << "SUSPENDING work-finished count off after cycle " << cycle
      << " -- reaching SUSPENDED requires this EM's own report, so a mismatch means a "
         "completion arrived from somewhere else";

    auto unsuspend_resp =
      send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::UNSUSPEND);
    ASSERT_NE(unsuspend_resp, nullptr);
    ASSERT_TRUE(unsuspend_resp->success) << "UNSUSPEND rejected: " << unsuspend_resp->message;
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::UNSUSPENDING, 1s))
      << "machine was never observed in UNSUSPENDING";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::EXECUTE, 2s))
      << "deferred UNSUSPENDING did not complete back into EXECUTE";
    EXPECT_EQ(em->started(packml_sm::State::UNSUSPENDING), cycle)
      << "UNSUSPENDING work-started count off after cycle " << cycle;
    EXPECT_EQ(em->finished(packml_sm::State::UNSUSPENDING), cycle)
      << "UNSUSPENDING work-finished count off after cycle " << cycle;
  }

  // Exact totals after both cycles: two of each, nothing lost, nothing doubled.
  EXPECT_EQ(em->started(packml_sm::State::SUSPENDING), 2);
  EXPECT_EQ(em->finished(packml_sm::State::SUSPENDING), 2);
  EXPECT_EQ(em->started(packml_sm::State::UNSUSPENDING), 2);
  EXPECT_EQ(em->finished(packml_sm::State::UNSUSPENDING), 2);

  // Still normally interruptible after the cycling.
  auto abort_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(abort_resp->success);
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
    << "machine did not ABORT normally after repeated suspend cycling";

  // All deferred work provably finished (finished counters asserted above); the short settle
  // only covers the tail of each detached thread exiting after its report call returns.
  std::this_thread::sleep_for(200ms);

  // See the file-header TEARDOWN RULE: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// Correct-behavior probe, first-ever coverage of STOPPING as a DEFERRED coordinated state:
// STOPPING is in packml_interface.hpp's kCoordinatedStates, and its graph position is mildly
// unusual -- the stoppable_stopping command transition lives on the `stoppable` SUPERSTATE
// (states_generator.hpp), so STOP is reachable from every stoppable descendant at once, while
// Stopping itself sits OUTSIDE stoppable, directly under abortable. No existing test drives
// STOP through an EM that actually defers it; every suite's STOPs complete on acceptance.
//
// The EM defers STOPPING with ~200ms work. Correct behavior: STOP from EXECUTE is accepted,
// the machine dwells in STOPPING until this EM's own report (observed via the acting state
// itself and a work counter of exactly 1 -- finish_setup()'s normalizing STOP is rejected
// from STOPPED, see its comment, so it contributes nothing), completes into STOPPED
// (stopping_stopped) within a generous bound, and a subsequent RESET->IDLE works -- a machine
// that can stop through a deferred STOPPING but cannot run again afterward would be a
// recoverability bug, not a stop bug.
TEST_F(StateGraphMonkeyTest, StoppingDeferred_StopFromExecuteCompletes)
{
  const auto em_name = packml_ros_test::unique_node_name("shk_stopping_em");
  auto rig = begin_setup("shk_stopping", em_name);
  auto em = std::make_shared<DeferredWorkEquipmentModule>(
    rig.em_node,
    DeferredWorkEquipmentModule::WorkSpec{{packml_sm::State::STOPPING, 200ms}});
  finish_setup(rig);

  ASSERT_EQ(em->started(packml_sm::State::STOPPING), 0)
    << "setup's normalizing STOP unexpectedly reached the EM -- it should be rejected from "
       "STOPPED (no stoppable_stopping match outside the stoppable superstate)";

  drive_to_execute(rig);

  auto stop_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_resp, nullptr);
  ASSERT_TRUE(stop_resp->success) << "STOP from EXECUTE rejected: " << stop_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPING, 1s))
    << "machine was never observed in STOPPING (deferred ~200ms dwell should be visible to "
       "10ms polling)";
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "deferred STOPPING did not complete into STOPPED";
  EXPECT_EQ(em->started(packml_sm::State::STOPPING), 1)
    << "STOPPING work should have started exactly once";
  EXPECT_EQ(em->finished(packml_sm::State::STOPPING), 1)
    << "reaching STOPPED requires this EM's own report, so a mismatch means a completion "
       "arrived from somewhere else";

  // Post-stop recoverability: the machine must run again after a deferred stop.
  auto reset_resp =
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success)
    << "RESET rejected after a deferred STOP completed: " << reset_resp->message;
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 2s))
    << "machine did not reach IDLE again after the deferred stop";

  // STOPPING work provably finished (counter asserted above); short settle for the detached
  // thread's tail, then the standard teardown order.
  std::this_thread::sleep_for(200ms);

  // See the file-header TEARDOWN RULE: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}
