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
// SHAKEDOWN suite (separate binary from the monkey tests), reentrancy/interleaving group:
// things happening DURING an in-flight coordinated wait. The existing monkey files act on a
// machine that has settled (or deliberately let one wait resolve before starting the next
// misbehavior); every scenario here injects its chaos while a CompletionTracker wait is
// genuinely in flight on the manager's Qt thread and an Equipment Module's own
// wait_for_deferred_completion() thread is blocked mid-goal:
//
//   1. A mode change fired mid-coordinated-wait (the mode path and the state path are
//      separate mechanisms sharing one manager -- does one block or corrupt the other?).
//   2. An operator command flood fired mid-coordinated-wait (the existing flood test floods
//      a SETTLED machine; here the flood races an in-flight deferred wait and its cancel/
//      stale-report machinery).
//   3. The manager itself torn down mid-coordinated-wait (the ~SMNode_new() -> shutdown()
//      -> request_shutdown() drain contract, and the orphaned EM's late report afterward).
//   4. Rapid HOLD/UNHOLD reversals (opposite transitions through coordinated states,
//      stressing begin_round()'s per-round bookkeeping being replaced mid-flight -- the
//      existing suite never reverses direction this fast).
//
// Same house style as test_monkey_scenarios.cpp: scenario-specific helpers in this file's own
// anonymous namespace (generic scaffolding lives in test_helpers.hpp), a Rig struct +
// begin_setup()/finish_setup() pattern, unique node names, 5ms gaps in send loops, and every
// test builds
// its own standalone manager + EM pair. All four tests assert intended CORRECT behavior with
// generous bounds -- none are declared KNOWN-GAP markers ahead of time; if the real build
// turns one red, treat that as signal, not a mistake in the test (see
// test_monkey_scenarios.cpp's own header for the standing rule against loosening).
//
// TEARDOWN DISCIPLINE (both learned the hard way, via gdb, in the sibling monkey files):
//   - Never let an Equipment Module object (declared after the Rig) destruct while the Rig's
//     own SpinHelper still spins its node -- a still-in-flight fan-out callback calling into
//     an object whose vtable is already gone is "pure virtual method called". Every test here
//     ends with rig.em_spin.reset() (or its explicit equivalent) AFTER all assertions.
//   - Any test that fires a ~/changeMode request must sleep ~2500ms before tearing the rig
//     down: on_change_mode() spawns a detached, unjoined fan-out thread per ACCEPTED request
//     with no shutdown synchronization (see test_monkey_multi_mode.cpp's
//     ModeChangeFlood_ManagerSurvivesAndStaysResponsive NOTE).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "packml_sm/default_modes.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// Fire-and-don't-wait: send a command and return immediately with the pending future,
/// without blocking for the response. Duplicated from test_monkey_scenarios.cpp -- used by
/// the flood/oscillation tests below, where the whole point is to not wait for one command
/// to settle before sending the next.
std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr> send_state_change_async(
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client,
  int8_t command)
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = command;
  return client->async_send_request(req).future.share();
}

/// Plain, non-deferring EM -- completes every coordinated state instantly. Duplicated from
/// test_monkey_scenarios.cpp per this test/ directory's own convention (no shared helper
/// header).
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

/// EM whose work for a given coordinated `state` is real background work decoupled from the
/// ROS goal's own cancellation: the work keeps running to completion and reports whenever IT
/// finishes, regardless of whether the goal that requested it was cancelled -- or whether the
/// manager that sent it even still exists -- in the meantime. Duplicated (same shape/
/// semantics) from test_monkey_scenarios.cpp's MonkeyStaleWorkEquipmentModule; only
/// monkey-behaves for the one state it's constructed for.
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

/// One manager "generation": a fresh rclcpp::Node + SMNode_new, spinning and ready to accept
/// ~/changeState calls. Duplicated from test_monkey_manager_restart.cpp (extended with a
/// state_complete_timeout_ms argument) -- lets the teardown test below construct, destroy,
/// and reconstruct a manager by hand, member by member, in a controlled order.
struct ManagerInstance
{
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<SMNode_new> sm_node;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client;
  std::shared_ptr<packml_ros_test::SpinHelper> spin;
};

ManagerInstance spin_up_manager(
  const std::string & mgr_name, const std::string & em_name, int state_complete_timeout_ms)
{
  ManagerInstance mgr;
  mgr.node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{em_name}),
      rclcpp::Parameter("state_complete_timeout_ms", state_complete_timeout_ms),
    }));
  mgr.sm_node = std::make_unique<SMNode_new>(mgr.node);
  mgr.state_client = mgr.node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/changeState");
  mgr.spin = std::make_shared<packml_ros_test::SpinHelper>(mgr.node);
  return mgr;
}

}  // namespace

class InterleaveMonkeyTest : public ::testing::Test
{
protected:
  /// Same shape as test_monkey_scenarios.cpp's own Rig: every test builds its own standalone
  /// manager + EM pair (unique node names), never shared fixture state.
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

  /// Constructs the manager/EM pair and gets the manager spinning, but does NOT construct
  /// the EM itself -- callers need different EM subclasses, so they construct it (on
  /// rig.em_node) themselves, then call finish_setup().
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
// ATTACKED: the mode-change path fired while a coordinated STATE wait is in flight. The two
// mechanisms are separate (on_change_mode() runs on the ROS executor thread and calls
// sm_->changeMode(); the coordinated wait blocks the Qt state-machine thread inside
// CompletionTracker::wait_for_all()), but they share one manager and one state machine --
// this probes whether one can block, corrupt, or resolve the other.
//
// CONTRACT FOUND IN SOURCE: mode changes ARE state-gated. StatesGenerator::switch_states
// (packml_sm/include/packml_sm/states_generator.hpp) contains only State::IDLE, and
// mode_switcher() rejects any changeMode from a non-IDLE state with "Cannot switch mode in
// state: ..." (the currentMode.name.empty() escape hatch only applies before the manager's
// own constructor-time initial changeMode -- to_string(ModeType) is std::to_string(int),
// never empty, so the hatch is closed for the whole life of the manager after init). On the
// rejection path on_change_mode() (packml_interface.hpp) returns success=false /
// INVALID_MODE_REQUEST synchronously, BEFORE ever spawning its detached fan-out thread.
//
// CORRECT BEHAVIOR ASSERTED (the gated variant, per the contract above): the mid-wait mode
// change is rejected promptly and cleanly -- not blocked behind the in-flight coordinated
// wait, since the rejection happens on the executor thread while the wait blocks only the Qt
// thread -- and does NOT disturb the in-flight RESETTING, which still resolves via the EM's
// own report (IDLE, work_finished == 1). A follow-up mode change from IDLE then succeeds,
// proving the earlier rejection was purely positional (the gate), not damage.
TEST_F(InterleaveMonkeyTest, ModeChangeMidCoordinatedWait_BothResolveIndependently)
{
  const auto em_name = packml_ros_test::unique_node_name("interleave_mode_mid_wait_em");
  auto rig = begin_setup("interleave_mode_mid_wait", em_name);
  auto em = std::make_shared<MonkeyStaleWorkEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING, 600ms);
  finish_setup(rig);

  auto mode_client = rig.mgr_node->create_client<packml_msgs::srv::ModeChange>(
    rig.mgr_name + "/changeMode");
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  // --- Get a coordinated wait genuinely in flight ---
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";
  {
    packml_ros_test::wait_until(
      [&] {return em->work_started.load() > 0;}, 500ms, 10ms);
    ASSERT_EQ(em->work_started.load(), 1) << "the EM's deferred background work never started";
  }

  // --- Fire the mode change mid-wait ---
  // The 400ms bound is doubly load-bearing: (a) generous for a synchronous local rejection
  // (typically single-digit ms) yet strict enough to prove the response was not serialized
  // behind the in-flight coordinated wait (600ms work, up to 5000ms manager-side timeout);
  // (b) it guarantees the still-RESETTING check right after cannot flake -- the mode request
  // went out within ~150ms of the work starting, so even a bound-maxing response lands well
  // before the EM's 600ms report could have resolved RESETTING.
  auto mode_req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  mode_req->mode.val = static_cast<int8_t>(packml_modes::Maintenance);
  auto mode_future = mode_client->async_send_request(mode_req);
  ASSERT_EQ(mode_future.wait_for(400ms), std::future_status::ready)
    << "changeMode blocked behind the in-flight coordinated wait";
  auto mode_resp = mode_future.get();
  EXPECT_FALSE(mode_resp->success)
    << "mode change from RESETTING was accepted -- switch_states (states_generator.hpp) is "
       "supposed to gate mode changes to IDLE";
  EXPECT_EQ(mode_resp->error_code, packml_msgs::srv::ModeChange::Response::INVALID_MODE_REQUEST);
  EXPECT_FALSE(mode_resp->message.empty())
    << "rejection carried no message (mode_switcher() supplies 'Cannot switch mode in state')";
  EXPECT_EQ(rig.sm_node->getCurrentState(), packml_sm::State::RESETTING)
    << "the rejected mode change disturbed the in-flight RESETTING";

  // --- The coordinated wait must still resolve on its own merits ---
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 2s))
    << "RESETTING did not resolve via the EM's own report after the mid-wait mode change, "
       "current state: " << static_cast<int>(rig.sm_node->getCurrentState());
  EXPECT_EQ(em->work_finished.load(), 1)
    << "IDLE was reached but the EM's own work had not finished -- something other than the "
       "EM's report resolved the wait";

  // --- Follow-up from IDLE: the gate is positional, not damage ---
  auto follow_up = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  follow_up->mode.val = static_cast<int8_t>(packml_modes::Maintenance);
  auto follow_up_future = mode_client->async_send_request(follow_up);
  ASSERT_EQ(follow_up_future.wait_for(2s), std::future_status::ready)
    << "manager stopped answering mode changes after the mid-wait rejection";
  EXPECT_TRUE(follow_up_future.get()->success)
    << "mode change from IDLE was rejected -- the earlier mid-wait rejection left the mode "
       "path damaged, not just gated";

  // The ACCEPTED follow-up mode change above spawned on_change_mode()'s detached, unjoined
  // fan-out thread (the rejected one did not -- rejection returns before the spawn). Settle
  // margin per test_monkey_multi_mode.cpp's confirmed-by-crash discipline; not a fix for the
  // underlying shutdown-synchronization gap, just declining to exercise it here.
  std::this_thread::sleep_for(2500ms);
  // Stop the EM node's spinner BEFORE em (declared after rig) destructs -- see this file's
  // header TEARDOWN DISCIPLINE ("pure virtual method called", confirmed via gdb in
  // test_monkey_multi_mode.cpp's ClientDisconnectMidRequest).
  rig.em_spin.reset();
}

// ============================================================================
// ATTACKED: an operator command flood arriving while a deferred coordinated wait is in
// flight. test_monkey_scenarios.cpp's MonkeyCommandFlood floods a SETTLED machine with a
// plain, instantly-completing EM; here the flood races (a) the manager's in-flight
// CompletionTracker::wait_for_all() on the Qt thread, (b) the interrupt path
// (setInterruptibleStateOperation's stop_token -> INTERRUPTED -> cancel_pending_state_goals(),
// packml_interface.hpp init()), and (c) the EM's own wait_for_deferred_completion() thread
// and its cancel/stale-report handling (begin_transition()/report_state_complete()).
//
// Interrupt-class commands (STOP/ABORT) are legitimately allowed to preempt the wait, so no
// specific end state is asserted (genuinely order/timing-dependent, not the point).
//
// CORRECT BEHAVIOR ASSERTED: every flooded command gets SOME response within 5s (command
// acceptance runs on the executor thread and must never be serialized behind the Qt-thread
// wait -- see on_change_state()'s own comment about not blocking the executor); the machine
// settles into a stable state; the manager answers a clean follow-up command; and the EM's
// detached work threads -- possibly finishing late into a cancelled goal, whose report
// report_state_complete() then discards on state mismatch or leaves as a harmless orphan --
// all drain without crashing.
TEST_F(InterleaveMonkeyTest, CommandFloodDuringDeferredWait_EveryCommandAnsweredWaitResolvesOnce)
{
  const auto em_name = packml_ros_test::unique_node_name("interleave_flood_mid_wait_em");
  auto rig = begin_setup("interleave_flood_mid_wait", em_name);
  auto em = std::make_shared<MonkeyStaleWorkEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING, 800ms);
  finish_setup(rig);

  // --- Get the deferred wait genuinely in flight ---
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";
  {
    packml_ros_test::wait_until([&] {return em->work_started.load() > 0;}, 500ms, 10ms);
    ASSERT_EQ(em->work_started.load(), 1) << "the EM's deferred background work never started";
  }

  // --- Flood mid-wait: a mix of interrupt-class and forward commands ---
  static constexpr int8_t kFlood[] = {
    packml_msgs::srv::StateChange::Request::RESET,
    packml_msgs::srv::StateChange::Request::START,
    packml_msgs::srv::StateChange::Request::HOLD,
    packml_msgs::srv::StateChange::Request::STOP,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::CLEAR,
    packml_msgs::srv::StateChange::Request::RESET,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::CLEAR,
    packml_msgs::srv::StateChange::Request::RESET,
  };

  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> pending;
  for (const auto command : kFlood) {
    pending.push_back(send_state_change_async(rig.state_client, command));
    // Deliberately tiny, non-zero gap -- see MonkeyCommandFlood's own comment in
    // test_monkey_scenarios.cpp for why true zero-delay sends exercise the service queue
    // more than the state machine.
    std::this_thread::sleep_for(5ms);
  }

  // Every queued request must get SOME response -- none silently dropped, none hung behind
  // the in-flight (or any subsequently started) coordinated wait.
  for (size_t i = 0; i < pending.size(); ++i) {
    ASSERT_EQ(pending[i].wait_for(5s), std::future_status::ready)
      << "flood command #" << i << " (cmd=" << static_cast<int>(kFlood[i])
      << ") never got a response while a coordinated wait was in flight -- possible hang";
  }

  // Settle-detection loop copied from MonkeyCommandFlood in test_monkey_scenarios.cpp; the
  // deadline is 4s rather than that test's 3s because a flood ending in RESET can leave a
  // fresh deferred RESETTING legitimately in flight for ~800ms before the machine stops
  // moving -- the stability requirement itself (20 unchanged 50ms ticks) is unchanged.
  EXPECT_TRUE(packml_ros_test::wait_for_settled_state(rig.sm_node, 4s, 20).has_value())
    << "machine never settled into a stable state within 4s of the flood, currently: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // The manager must still answer a clean follow-up command (success not asserted -- STOP
  // may legitimately be rejected depending on where the race settled; only that a real
  // response comes back, not a hang).
  auto follow_up = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(follow_up, nullptr) << "manager stopped responding after the mid-wait flood";

  // The EM must not have crashed, and its detached work threads (raw `this`, decoupled from
  // goal cancellation) must all have finished before teardown: every started work item
  // eventually finishes, however its goal was resolved/cancelled out from under it.
  {
    packml_ros_test::wait_until(
      [&] {return em->work_finished.load() >= em->work_started.load();}, 3s, 10ms);
    EXPECT_EQ(em->work_finished.load(), em->work_started.load())
      << "an EM work thread never finished -- late-into-cancelled-goal work is supposed to "
         "run to completion harmlessly";
  }
  // work_finished increments BEFORE the thread's final completion report -- a
  // short tail margin so the very last detached thread fully exits before teardown.
  std::this_thread::sleep_for(200ms);

  // See this file's header TEARDOWN DISCIPLINE: stop the EM's spinner before em (declared
  // after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// ATTACKED: the manager destroyed MID-coordinated-wait. ~SMNode_new() (packml_ros-new.hpp)
// calls shutdown() as its FIRST statement precisely for this moment: shutdown() ->
// completion_tracker_->request_shutdown() wakes the in-flight wait_for_all() (returning
// SHUTDOWN, which the setInterruptibleStateOperation binding maps to a quiet 0) BEFORE the
// member chain that owns the StateMachine starts unwinding -- because
// StateMachine::drainActingStates() (invoked from ~StateMachine()) blocks on any bound
// acting-state operation with NO internal timeout. Without that early wake, teardown would
// ride out up to the full state_complete_timeout_ms.
//
// Note what SHUTDOWN deliberately does NOT do: unlike TIMEOUT/FAILED/INTERRUPTED it never
// calls cancel_pending_state_goals(), so the EM's in-flight goal is simply orphaned -- its
// own wait_for_deferred_completion() thread keeps waiting, and when the EM's late report
// finally fires (~2s in, long after the manager died) it resolves that goal via the EM's OWN
// action server (goal_handle->succeed()); the dead manager (the action CLIENT) simply never
// reads the result. Nothing in report_state_complete() (packml_interface.hpp) touches the
// manager at all -- it is purely EM-local mutex/condition-variable state -- so the late
// report must be harmless.
//
// CORRECT BEHAVIOR ASSERTED: with a deliberately huge 10s configured
// state_complete_timeout_ms and the EM's work only ~2s, tearing the manager down ~0.5s into
// the wait completes in well under 3s (proving the destructor's drain woke the wait early
// rather than riding out either timeout); the EM survives its own late, orphaned report; and
// a FRESH generation-2 manager (same name, same config -- exactly what a supervisor restart
// produces) then drives the SAME still-running EM through a clean STOP -> RESET -> IDLE cycle.
TEST_F(InterleaveMonkeyTest, ManagerTeardownMidDeferredWait_DestructorDrainsBoundedAndEmSurvives)
{
  constexpr int kStateCompleteTimeoutMs = 10000;
  const auto mgr_name = packml_ros_test::unique_node_name("interleave_teardown_mgr");
  const auto em_name = packml_ros_test::unique_node_name("interleave_teardown_em");

  // --- Equipment Module: constructed ONCE, alive across both manager generations ---
  auto em_node = rclcpp::Node::make_shared(em_name);
  auto em = std::make_shared<MonkeyStaleWorkEquipmentModule>(
    em_node, packml_sm::State::RESETTING, 2000ms);
  auto em_spin = std::make_shared<packml_ros_test::SpinHelper>(em_node);

  // --- Generation 1: drive into the deferred wait ---
  auto mgr1 = spin_up_manager(mgr_name, em_name, kStateCompleteTimeoutMs);
  ASSERT_TRUE(mgr1.state_client->wait_for_service(5s));
  send_state_change(mgr1.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_TRUE(wait_for_state(mgr1.sm_node, packml_sm::State::STOPPED, 2s))
    << "generation 1 never reached STOPPED";

  auto reset_resp = send_state_change(mgr1.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success);
  ASSERT_TRUE(wait_for_state(mgr1.sm_node, packml_sm::State::RESETTING, 500ms))
    << "generation 1 did not enter RESETTING";
  {
    packml_ros_test::wait_until([&] {return em->work_started.load() > 0;}, 500ms, 10ms);
    ASSERT_EQ(em->work_started.load(), 1) << "the EM's deferred background work never started";
  }
  // Small beat so the EM's wait_for_deferred_completion() thread is genuinely blocked in its
  // wait (it spawns right after on_state_trans_req() approved), not still being scheduled.
  std::this_thread::sleep_for(100ms);

  // --- Teardown mid-wait, in exactly the order the restart precedent established: stop
  // spinning before destroying what it spins, then the SMNode (whose destructor holds the
  // drain contract under test), then this scope's node handle. Only sm_node.reset() is
  // timed: that call runs ~SMNode_new() -> shutdown() -> the drain. During the drain the
  // machine completes RESETTING (SHUTDOWN maps to 0) and fans out IDLE with nobody spinning
  // the manager node anymore -- that fan-out's own acceptance wait is bounded (200ms), and
  // with no executor its callbacks can never fire into freed memory. ---
  mgr1.spin.reset();
  mgr1.state_client.reset();
  const auto teardown_start = std::chrono::steady_clock::now();
  mgr1.sm_node.reset();
  const auto teardown_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - teardown_start);
  mgr1.node.reset();

  EXPECT_LT(teardown_ms.count(), 3000)
    << "destroying the manager mid-deferred-wait took " << teardown_ms.count()
    << "ms -- the destructor's shutdown() drain should have woken the in-flight wait "
       "immediately instead of riding out the configured " << kStateCompleteTimeoutMs
    << "ms state_complete_timeout_ms";

  // --- The EM's late report (fires ~2s after its work started, manager long dead) must be
  // harmless: it resolves the orphaned goal on the EM's own action server; nobody is left
  // to read the result. ---
  {
    packml_ros_test::wait_until([&] {return em->work_finished.load() >= 1;}, 3s, 10ms);
    ASSERT_GE(em->work_finished.load(), 1)
      << "the EM's background work never finished after the manager died";
  }
  // Let the report/succeed path (and the deferred-wait thread it wakes) fully run.
  std::this_thread::sleep_for(300ms);

  // --- Generation 2: same name, same config -- the EM (which never noticed the death) must
  // be drivable through a clean STOP -> RESET -> IDLE cycle. Reaching IDLE at all proves the
  // fresh RESETTING resolved via the EM's report; work counters confirm it was THIS cycle's
  // own work, not any generation-1 leftover. ---
  auto mgr2 = spin_up_manager(mgr_name, em_name, kStateCompleteTimeoutMs);
  ASSERT_TRUE(mgr2.state_client->wait_for_service(5s))
    << "generation 2's changeState service never came up";
  send_state_change(mgr2.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_TRUE(wait_for_state(mgr2.sm_node, packml_sm::State::STOPPED, 2s))
    << "generation 2 never reached STOPPED";

  auto reset2_resp = send_state_change(mgr2.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset2_resp, nullptr);
  ASSERT_TRUE(reset2_resp->success) << "generation 2 RESET rejected: " << reset2_resp->message;
  ASSERT_TRUE(wait_for_state(mgr2.sm_node, packml_sm::State::RESETTING, 500ms))
    << "generation 2 did not enter RESETTING";
  {
    packml_ros_test::wait_until(
      [&] {return em->work_started.load() >= 2;}, 500ms, 10ms);
    ASSERT_EQ(em->work_started.load(), 2)
      << "the EM's deferred work did not start fresh for generation 2's cycle";
  }
  ASSERT_TRUE(wait_for_state(mgr2.sm_node, packml_sm::State::IDLE, 4s))
    << "generation 2's RESETTING did not resolve via the surviving EM's report, current "
       "state: " << static_cast<int>(mgr2.sm_node->getCurrentState());
  EXPECT_EQ(em->work_finished.load(), 2)
    << "generation 2 reached IDLE before its own cycle's work finished";

  // Teardown: generation 2 in the same member order as generation 1, THEN the EM's spinner
  // explicitly (see this file's header TEARDOWN DISCIPLINE), then the EM itself.
  mgr2.spin.reset();
  mgr2.state_client.reset();
  mgr2.sm_node.reset();
  mgr2.node.reset();
  em_spin.reset();
  em.reset();
  em_node.reset();
}

// ============================================================================
// ATTACKED: rapid direction REVERSALS through opposite coordinated states. HOLD drives
// EXECUTE -> HOLDING (coordinated) -> HELD; UNHOLD drives HELD -> UNHOLDING (coordinated) ->
// EXECUTE. Every entry into HOLDING/UNHOLDING triggers fanout_state_transition(), whose
// completion_tracker_->begin_round() (packml_interface.hpp) unconditionally REPLACES the
// whole per-round map and clears active_state_goals_ -- so ten fast reversals replace the
// round bookkeeping ten-plus times while the previous round's goal results/cancellations may
// still be arriving. The existing suite floods same-direction commands (test_monkey_scenarios
// MonkeyCommandFlood) or lets each cycle resolve (test_monkey_multi_mode MixedSpeed cycling);
// nothing reverses direction this fast.
//
// CORRECT BEHAVIOR ASSERTED: every command answered within 5s (an out-of-turn HOLD/UNHOLD is
// a fast, clean SM-level rejection -- on_change_state() never blocks the executor on the
// coordinated wait); the machine settles into one of the four states on the HOLD/UNHOLD axis
// (EXECUTE/HOLDING/HELD/UNHOLDING -- with a plain EM the acting states resolve promptly, so
// a settled HOLDING/UNHOLDING would itself be suspicious, but the axis is the generous
// contract); and a follow-up ABORT (valid from the entire Abortable superstate, so from any
// of the four) is accepted and reaches ABORTED -- proving the round bookkeeping was left
// coherent, not wedged mid-replacement.
//
// This probe found something other than the reversal bookkeeping it was aimed at: under command
// churn the manager SILENTLY LOST commands. Two compounding causes, both since fixed.
//   1. EXECUTE self-cycled by design in ContinuousCycle mode against a placeholder operation, so a
//      machine sitting in its normal production state fanned out ~5 goal rounds per second per EM
//      forever. Covered separately -- EXECUTE does not fabricate cycles.
//   2. Under that churn, on_change_state() produced its answer inline on the executor thread,
//      blocking there until the Qt event loop answered the command. That thread is also the one
//      taking requests off the wire, and the service's reader queue is a RELIABLE KEEP_LAST
//      depth-10 default: requests past ~10 deep are OVERWRITTEN rather than rejected, so no
//      response is ever sent and the caller's future never resolves. The response is deferred
//      instead: the callback enqueues and returns, and a single FIFO worker evaluates each command
//      and sends its real answer. Answering inline costs this probe most of its 20 commands and
//      a 5 s timeout for each one lost.
// The main suite's MonkeyCommandFlood sends exactly 10 commands and its machine never dwells in
// EXECUTE, which is why this went unseen until a probe both oscillated and dwelled.
TEST_F(InterleaveMonkeyTest, HoldUnholdOscillationAnswersEveryCommand)
{
  const auto em_name = packml_ros_test::unique_node_name("interleave_hold_unhold_em");
  auto rig = begin_setup("interleave_hold_unhold", em_name);
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_node);
  finish_setup(rig);

  // --- Drive to EXECUTE, where HOLD is meaningful ---
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 1s))
    << "machine did not reach IDLE";
  auto start_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::START);
  ASSERT_NE(start_resp, nullptr);
  ASSERT_TRUE(start_resp->success);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::EXECUTE, 1s))
    << "machine did not reach EXECUTE";

  // --- Oscillate: HOLD/UNHOLD alternating, 10 of each, async, no waiting between ---
  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> pending;
  for (int i = 0; i < 10; ++i) {
    pending.push_back(send_state_change_async(
        rig.state_client, packml_msgs::srv::StateChange::Request::HOLD));
    std::this_thread::sleep_for(5ms);
    pending.push_back(send_state_change_async(
        rig.state_client, packml_msgs::srv::StateChange::Request::UNHOLD));
    std::this_thread::sleep_for(5ms);
  }

  // EVERY command must be answered. What is asserted is a response, not a success: an out-of-turn
  // HOLD or UNHOLD is expected to be REFUSED, and a refusal is a perfectly good answer. Silence is
  // the failure -- an operator mashing recovery buttons cannot tell a command that was refused from
  // one that was discarded, and the discarded one leaves the machine somewhere they did not ask for.
  //
  // Which commands are refused is deliberately NOT asserted: that depends on where each one lands
  // relative to the machine's own transitions, so pinning it would make this test flaky for a
  // reason unrelated to what it is protecting.
  std::vector<size_t> unanswered;
  size_t accepted = 0;
  for (size_t i = 0; i < pending.size(); ++i) {
    if (pending[i].wait_for(5s) != std::future_status::ready) {
      unanswered.push_back(i);
      continue;
    }
    if (pending[i].get()->success) {
      ++accepted;
    }
  }
  {
    std::string unanswered_list;
    for (const auto i : unanswered) {
      unanswered_list += (unanswered_list.empty() ? "" : ", ") + std::to_string(i);
    }
    EXPECT_TRUE(unanswered.empty())
      << unanswered.size() << " of " << pending.size() << " oscillation commands were never "
         "answered (indices " << unanswered_list << "). Commands are being taken off the wire and "
         "silently discarded -- check that on_change_state() still defers its response instead of "
         "producing it on the executor thread.";
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test"),
    accepted << " of " << pending.size() << " oscillation commands were accepted; the rest were "
    "refused as out-of-turn, which is a valid answer");

  // Settle-detection loop copied from MonkeyCommandFlood in test_monkey_scenarios.cpp.
  const auto settled = packml_ros_test::wait_for_settled_state(rig.sm_node, 3s, 20);
  EXPECT_TRUE(settled.has_value())
    << "machine never settled into a stable state within 3s of the oscillation, currently: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  static constexpr packml_sm::State kHoldAxisStates[] = {
    packml_sm::State::EXECUTE, packml_sm::State::HOLDING,
    packml_sm::State::HELD, packml_sm::State::UNHOLDING,
  };
  bool on_axis = false;
  for (const auto s : kHoldAxisStates) {
    if (settled.has_value() && s == *settled) {
      on_axis = true;
      break;
    }
  }
  EXPECT_TRUE(on_axis)
    << "machine settled OFF the HOLD/UNHOLD axis after the oscillation: "
    << static_cast<int>(settled.value_or(packml_sm::State::UNDEFINED));

  // --- Clean interrupt afterward: the per-round bookkeeping must be coherent enough to run
  // one more coordinated round (ABORTING) to completion ---
  auto abort_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr) << "manager stopped responding after the oscillation";
  EXPECT_TRUE(abort_resp->success)
    << "ABORT rejected from " << static_cast<int>(rig.sm_node->getCurrentState())
    << " -- every state on the HOLD/UNHOLD axis is Abortable";
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
    << "machine did not reach ABORTED after the post-oscillation ABORT";

  // See this file's header TEARDOWN DISCIPLINE: stop the EM's spinner before em (declared
  // after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// ATTACKED: the manager torn down with commands already accepted off the wire but not yet
// evaluated. The deferred-response accept path (on_change_state()) means a request can be sitting
// in the command queue with its caller still waiting, and the answer is owed by a worker thread
// that is about to be joined.
//
// CORRECT BEHAVIOR ASSERTED: every one of those commands is answered -- the queued ones refused
// with a message naming the shutdown, rather than abandoned. A caller left waiting forever on a
// command the manager already took off the wire is the same silent-loss failure the oscillation
// test above covers, just reached through teardown instead of through queue overflow.
//
// SCOPE, deliberately: a request still in the SERVICE's own reader queue when the service is
// destroyed cannot be answered by anyone -- the object that would answer it is what is going away.
// That was equally true before the accept path was deferred. So this burst is kept well inside the
// depth-10 reader queue and spaced enough to arrive, which makes "every command answered" a
// guarantee the design can actually keep.
//
// The EM's spinner is stopped first, and that is what makes the test work: with nobody acking the
// fan-out, entering RESETTING occupies the Qt event loop for the full acceptance wait, so the
// commands behind it are still queued when teardown starts. Without that there is no backlog and
// the test would observe nothing.
TEST_F(InterleaveMonkeyTest, TeardownRefusesCommandsItAcceptedButNeverEvaluated)
{
  const auto em_name = packml_ros_test::unique_node_name("interleave_teardown_cmd_em");
  auto rig = begin_setup("interleave_teardown_cmd", em_name);
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_node);
  finish_setup(rig);

  rig.em_spin.reset();  // see the header: this is what keeps the Qt loop busy long enough

  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> pending;
  for (int i = 0; i < 8; ++i) {
    pending.push_back(send_state_change_async(
        rig.state_client, packml_msgs::srv::StateChange::Request::RESET));
    std::this_thread::sleep_for(5ms);
  }

  rig.sm_node.reset();  // ~SMNode_new() -> shutdown() -> drains and joins the command worker

  size_t unanswered = 0;
  size_t refused_for_shutdown = 0;
  for (size_t i = 0; i < pending.size(); ++i) {
    if (pending[i].wait_for(2s) != std::future_status::ready) {
      ++unanswered;
      continue;
    }
    const auto res = pending[i].get();
    if (!res->success && res->message.find("shutting down") != std::string::npos) {
      ++refused_for_shutdown;
    }
  }

  EXPECT_EQ(unanswered, 0u)
    << unanswered << " of " << pending.size() << " commands were still owed an answer when the "
       "manager went away and never got one";
  EXPECT_GT(refused_for_shutdown, 0u)
    << "no caller was told the manager was shutting down, so the command queue was already empty "
       "at teardown and this test observed nothing -- check the burst spacing and the stopped EM "
       "spinner above, both of which exist to guarantee a backlog";

  RCLCPP_INFO_STREAM(rclcpp::get_logger("test"),
    refused_for_shutdown << " of " << pending.size() << " commands were refused for shutdown");
}
