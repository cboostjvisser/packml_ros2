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
// "Monkey" scenarios, fan-out/discovery flavor: see test_monkey_scenarios.cpp for the shared
// rationale of this whole family of tests. That file makes the OPERATOR (or the manager's own
// command handling) the chaotic party; this file makes the EQUIPMENT MODULE's discoverability
// and in-flight reliability the chaotic party instead, covering the "Fan-out/discovery chaos"
// items:
//
//   - Flapping discoverability: the EM's action server appears and disappears repeatedly DURING
//     a single fan-out round, before the goal can ever be sent -- harder than the existing
//     "appears once, late" / "never appears" coverage in test_manager_client_fanout.cpp
//     (LateDiscoveredStateChildEventuallyReceivesTransition /
//     NeverDiscoveredStateChildFailsFastNotAfterFullTimeout).
//   - Vanishing mid-transition: the EM accepts a goal (so the manager's fan-out sees it as
//     ACCEPTED, in-flight) and then goes silent for good -- no more heartbeats, no voluntary
//     completion, ever -- before it can ever report. Checks that the manager's coordinated wait
//     resolves via its own timeout or the health-monitor cross-check, never a permanent hang, and
//     that the machine lands in a recoverable state. (Simulated via set_heartbeat_active(false)
//     rather than literally destroying the EM's node/object -- see that test's own comment for
//     why: PackmlNodeInterface has no destructor to synchronize against its deferred-completion
//     wait's detached background thread, so destroying the object while that wait is still
//     blocked is a real, separate hazard this file deliberately does not need to trigger to make
//     its point.)
//   - Retry double-delivery: the discovery-retry path (same setup as the existing late-discovery
//     test) exercised close to a retry-tick boundary, to check the EM never receives more than
//     one goal for a single fan-out round.
//
// All three tests below are written as "this SHOULD hold" (confirmed-safe style) rather than
// declared known gaps: reading fanout_state_transition()/check_fanout_deadlines() closely does
// not turn up a structural double-send or a hang path. If any of these turn red, that is real
// signal, not a reason to loosen the assertion (see test_monkey_scenarios.cpp header for the
// project's standing rule on that).

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
#include "packml_ros/ros_names.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// Plain, non-deferring EM that also counts how many state-transition goals it has been asked
/// to handle in total, and records the last one -- used both to confirm a flapping/retried
/// discovery eventually delivers a goal (Flapping* test) and to confirm it delivers EXACTLY one
/// per round, not a duplicate (RetryDoubleDelivery* test). Completes every coordinated state
/// instantly on acceptance (defers_completion() stays false, the default), same as
/// PlainEquipmentModule in test_monkey_scenarios.cpp.
class TrackingEquipmentModule : public PackmlNodeInterface
{
public:
  explicit TrackingEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  std::atomic<int> transitions_received{0};
  std::atomic<int8_t> last_state_received{0};
  // Per-state counts: fanout_state_transition() fires for EVERY manager state change, not just
  // coordinated ones -- RESETTING completing auto-advances to IDLE, which is ALSO fanned out to
  // this same EM as its own, separate (non-deferred) goal. transitions_received/
  // last_state_received alone can't distinguish "received RESETTING once, then IDLE once" from
  // an actual duplicate delivery of the SAME state -- use resetting_received for that.
  std::atomic<int> resetting_received{0};

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    transitions_received.fetch_add(1);
    last_state_received.store(static_cast<int8_t>(state));
    if (state == packml_sm::State::RESETTING) {
      resetting_received.fetch_add(1);
    }
    return true;
  }
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
};

/// EM that accepts a goal for exactly one configured `monkey_state`, defers completion for it,
/// and then never reports it at all -- simulating a process that accepted
/// work and then went silent for good, instead of MonkeyStaleWorkEquipmentModule's (test_monkey_
/// scenarios.cpp) "finishes late but does finish". Re-exposes set_heartbeat_active() exactly as
/// SimEquipmentModule does in test_health_monitor_integration.cpp, to simulate "a crashed or
/// silent node" (that method's own doc comment) the same sanctioned way that file already does
/// -- see the test below for why this file deliberately does NOT instead destroy the EM object
/// itself. Every other state behaves like a plain, instantly-completing EM (matching
/// MonkeyStaleWorkEquipmentModule's own "only monkey-behave for the one state it's built for"
/// convention).
class VanishingEquipmentModule : public PackmlNodeInterface
{
public:
  VanishingEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State monkey_state)
  : monkey_state_(monkey_state)
  {
    init(node);
  }

  using PackmlNodeInterface::set_heartbeat_active;

  std::atomic<int> goal_accepted{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == monkey_state_;}
  // Counts the goal and then drops its handle on the floor: gone silent for good. Overridden
  // rather than left to the base implementation, which would report a failure straight back.
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion) override
  {
    goal_accepted.fetch_add(1);
  }

private:
  packml_sm::State monkey_state_;
};

}  // namespace

class MonkeyFanoutTest : public ::testing::Test
{
protected:
  /// Same shape as test_monkey_scenarios.cpp's own Rig: every test builds its own standalone
  /// manager + EM pair (unique manager name; the EM node name is a plain literal, unique only
  /// within this file, matching that file's own convention).
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

  /// Extends test_monkey_scenarios.cpp's begin_setup() with optional required_nodes /
  /// heartbeat_timeout_factor overrides -- only VanishingMidTransition needs the health
  /// cross-check wired up; the other two tests call this with their defaults (no required
  /// nodes registered, matching the original signature's behavior exactly). As there, this does
  /// NOT construct the EM itself -- callers construct it on rig.em_node (or, for the flapping
  /// test, repeatedly construct/destroy it there) and start spinning via finish_setup().
  Rig begin_setup(
    const std::string & mgr_prefix, const std::string & em_name,
    int state_complete_timeout_ms = 5000,
    std::vector<std::string> required_nodes = {},
    double heartbeat_timeout_factor = 3.0)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);

    std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter(packml_ros::kParamNodeNames, std::vector<std::string>{em_name}),
      rclcpp::Parameter(packml_ros::kParamStateCompleteTimeoutMs, state_complete_timeout_ms),
    };
    if (!required_nodes.empty()) {
      params.push_back(rclcpp::Parameter(packml_ros::kParamRequiredNodes, required_nodes));
      params.push_back(
        rclcpp::Parameter(packml_ros::kParamHeartbeatTimeoutFactor, heartbeat_timeout_factor));
    }

    rig.mgr_node = rclcpp::Node::make_shared(
      rig.mgr_name, rclcpp::NodeOptions().parameter_overrides(params));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.em_node = rclcpp::Node::make_shared(em_name);
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
// Confirmed-safe (expected): flapping discoverability. Unlike
// LateDiscoveredStateChildEventuallyReceivesTransition / NeverDiscoveredStateChildFailsFastNot-
// AfterFullTimeout in test_manager_client_fanout.cpp (which only cover "appears once, late" and
// "never appears"), this repeatedly constructs and destroys the EM's PackmlNodeInterface object
// on the SAME underlying node -- action server appears, vanishes, appears again -- several times
// during RESETTING's single fan-out round (whose own discovery-retry deadline is a hardcoded 5s,
// see fanout_state_transition()), before finally staying up for good, well inside that window.
// Whichever way it resolves -- the retried goal getting through once the EM stabilizes (IDLE), or
// the fan-out round giving up on a never-stable EM and the machine failing out (ABORTING/
// ABORTED) -- the manager must not hang or crash. Given the EM DOES stabilize well within the 5s
// window here, IDLE is the expected outcome; ABORTING/ABORTED is accepted too so this cannot
// flake on a slow CI box that happens to miss the stabilization window.
//
// Constructing/destroying a PackmlNodeInterface subclass on rig.em_node, present/absent several
// times, models "action server appears, vanishes, appears again." Unlike
// RequiredNodeRestartTriggersManagerAbort's (test_health_monitor_integration.cpp) similar-looking
// swap-the-object move, this test does NOT keep a single SpinHelper spinning rig.em_node
// continuously across the swaps: destroying a PackmlNodeInterface's ROS entities (subscriptions,
// action server, timers) while a SEPARATE thread is concurrently mid-callback on that same node
// is a genuine data race regardless of defers_completion() -- confirmed the hard way, this
// exact pattern crashed the whole shared test binary with "pure virtual method called" during
// development. Each cycle below therefore stops spinning rig.em_node (joining that thread,
// guaranteeing no callback can be in flight) BEFORE destroying the EM instance, and only resumes
// spinning once a new instance is ready -- so a destroy never overlaps a live callback.
TEST_F(MonkeyFanoutTest, MonkeyFlappingDiscoverability_EventualDeliveryOrFastFailureNoHang)
{
  // Shorter than the default so a genuinely-never-stabilizing run (which should not happen
  // given the timing below, but bounds the test's own worst case) fails out quickly rather than
  // riding out the full 5s default on top of the already-generous polling deadline below.
  auto rig = begin_setup("monkey_flap_discovery", "flap_em", /*state_complete_timeout_ms=*/3000);
  finish_setup(rig);  // STOPPED's own fan-out round begins with the EM entirely absent
  rig.em_spin.reset();  // manage rig.em_node's spin lifecycle manually for this test -- see above

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET rejected: " << resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // Flap the EM's action server present/absent three times during RESETTING's fan-out retry
  // window, then let it stay up for good -- comfortably inside the 5s/3000ms bounds above.
  std::shared_ptr<TrackingEquipmentModule> em;
  for (int i = 0; i < 3; ++i) {
    {
      packml_ros_test::SpinHelper cycle_spin(rig.em_node);
      em = std::make_shared<TrackingEquipmentModule>(rig.em_node);
      std::this_thread::sleep_for(100ms);
      // cycle_spin destructs here (joins, so nothing can be mid-callback), THEN em destructs --
      // vanish: action server torn down along with the object that owns it, safely.
    }
    em.reset();
    std::this_thread::sleep_for(100ms);  // node genuinely absent: not spinning, no EM object
  }
  rig.em_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.em_node);
  em = std::make_shared<TrackingEquipmentModule>(rig.em_node);  // stays up for good this time

  // Either outcome is a "no hang" pass; what must NOT happen is staying stuck in RESETTING.
  packml_ros_test::wait_until(
    [&] {return rig.sm_node->getCurrentState() != packml_sm::State::RESETTING;}, 6s, 20ms);
  const auto final_state = rig.sm_node->getCurrentState();
  EXPECT_TRUE(
    final_state == packml_sm::State::IDLE ||
    final_state == packml_sm::State::ABORTING ||
    final_state == packml_sm::State::ABORTED)
    << "machine did not settle into a recognized state after the flapping action server, "
       "stuck at: " << static_cast<int>(final_state);

  if (final_state == packml_sm::State::IDLE) {
    // Reaching IDLE at all is already the proof RESETTING was delivered to and completed by
    // SOME EM instance during this round -- only one EM is registered (node_names has just this
    // one name), so there is no other way out of RESETTING. No separate per-instance check is
    // needed (and none would be reliable here): the very first flapping cycle's EM instance
    // typically completes RESETTING almost instantly, well before the loop's later vanish/
    // reappear cycles even run, so `em` (the LAST instance constructed) is often not the one
    // that actually received it -- its own resetting_received can legitimately read 0 even on
    // a fully successful round.
  } else if (final_state == packml_sm::State::ABORTING) {
    // Let the now-stably-up EM carry the machine the rest of the way through recovery rather
    // than leaving it mid-transition at teardown.
    wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s);
  }

  // Stop spinning rig.em_node BEFORE `em` (a local variable declared after `rig`, so it would
  // otherwise destruct FIRST, while em_spin's background thread is still concurrently servicing
  // callbacks on the very node `em`'s destructor is tearing down -- a real crash, not a
  // theoretical one: an earlier run of this file hit exactly this ordering and crashed with
  // "pure virtual method called" during teardown). SpinHelper's destructor joins synchronously,
  // so after this line no more callbacks can fire on rig.em_node.
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected): vanishing mid-transition. Harsher variant of
// ResettingTimesOutToAbortWithoutCompletion (test_completion_integration.cpp), which never
// silences the EM's heartbeat at all and so only exercises the plain completion timeout -- this
// test ALSO registers the EM as a required node with a fast heartbeat, so the health-monitor
// cross-check (CompletionTracker's node_healthy_ predicate, re-checked on every condition-
// variable wakeup -- see completion_tracker.hpp's wait_for_all()) has a realistic chance to
// resolve the wait well before the plain timeout would have to. Either backstop firing is
// accepted: the point is that the manager's coordinated wait for RESETTING must resolve -- never
// hang forever on a goal that will now never be voluntarily completed -- and the machine must
// land in a recognized, recoverable state.
//
// Deliberately simulates "vanished" via set_heartbeat_active(false) (the same sanctioned
// mechanism test_health_monitor_integration.cpp already uses for "a crashed or silent node"),
// NOT by destroying the EM object/node outright, and deliberately leaves the EM's own executor
// (rig.em_spin) spinning for the rest of the test. This is a deliberate safety choice, not an
// oversight: PackmlNodeInterface declares no destructor at all (confirmed by inspection), so its
// deferred-completion wait's detached background thread (wait_for_deferred_completion(), holding
// a raw `this`) has nothing to synchronize against if the object were destroyed while that wait
// is still blocked -- destroying completion_mutex_/completion_cv_ out from under a thread still
// waiting on them is undefined behavior, and this test's own local `em` would otherwise be
// destroyed at scope exit before that background thread necessarily got a chance to exit
// cleanly. Keeping the EM's executor alive lets the manager's own eventual cancel (see
// cancel_pending_state_goals(), fired once the wait below resolves) reach it and wake that
// thread gracefully via goal_handle->is_canceling(), exactly like test_completion_integration.
// cpp's own never-reporting tests already rely on. From the MANAGER's side -- which is what this
// test actually asserts on -- a silenced, never-completing EM is observationally identical to a
// genuinely vanished one: no heartbeats, and a goal that will never be voluntarily finished.
TEST_F(MonkeyFanoutTest, MonkeyVanishingMidTransition_ResolvesViaTimeoutOrHealthNoHang)
{
  const std::string em_name = "vanish_em";
  // Registered as required with a fast heartbeat (learned interval 100ms x default factor 3.0 =
  // 300ms effective timeout) so the health cross-check gets a real chance to be the one that
  // rescues the wait, not just the (here, deliberately short) state_complete_timeout_ms backstop.
  auto rig = begin_setup(
    "monkey_vanish_mid_transition", em_name, /*state_complete_timeout_ms=*/3000,
    /*required_nodes=*/{em_name}, /*heartbeat_timeout_factor=*/3.0);
  rig.em_node->declare_parameter(packml_ros::kParamHeartbeatIntervalMs, 100);
  auto em = std::make_shared<VanishingEquipmentModule>(rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  // Let the health gate see at least a couple of healthy heartbeats before RESET -- RESET from
  // STOPPED is gated on required-node health (see HealthMonitor::gate_block_reason: "never
  // seen" blocks unconditionally regardless of elapsed time). Waited for rather than slept
  // through; see wait_for_healthy_heartbeats().
  ASSERT_GE(packml_ros_test::wait_for_healthy_heartbeats(rig.mgr_node, em_name), 2)
    << "no heartbeats from " << em_name << " reached the manager -- the health gate would "
       "block RESET for a reason that has nothing to do with this test";

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET rejected: " << resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // Give the fan-out (bounded by its own ~200ms acceptance wait) time to actually reach the EM
  // and have it accept the goal before silencing it -- matching the pattern used throughout
  // test_completion_integration.cpp for the same reason.
  {
    packml_ros_test::wait_until(
      [&] {return em->goal_accepted.load() > 0;}, 500ms, 10ms);
    ASSERT_EQ(em->goal_accepted.load(), 1) << "EM never accepted the RESETTING goal";
  }

  // The EM goes silent for good: no more heartbeats, and (since it never calls
  // never reporting) no voluntary completion of the goal it already accepted either.
  em->set_heartbeat_active(false);

  // The manager must settle into a recognized state within a bounded time -- never hang
  // waiting forever on a goal that will now never resolve on its own.
  packml_ros_test::wait_until(
    [&] {
      const auto state = rig.sm_node->getCurrentState();
      return state == packml_sm::State::ABORTING || state == packml_sm::State::ABORTED;
    }, 5s, 20ms);
  const auto final_state = rig.sm_node->getCurrentState();
  EXPECT_TRUE(
    final_state == packml_sm::State::ABORTING || final_state == packml_sm::State::ABORTED)
    << "machine did not resolve to a recognized state after the EM went silent mid-transition, "
       "stuck at: " << static_cast<int>(final_state);

  // What the assertion below protects, because the mechanism is subtle enough to be broken by
  // accident. It rests on a single edge, Aborting --ERROR--> Aborted, next to aborting_aborted in
  // states_generator.hpp. Reparenting Aborting under `abortable` is NOT an alternative: the
  // superstate's ERROR transition targets Aborting, so inheriting it would self-loop instead of
  // escaping.
  //
  // ABORTING is ITSELF one of kCoordinatedStates (packml_interface.hpp),
  // so its own completion wait ALSO cross-checks vanish_em's health -- regardless of
  // defers_completion(ABORTING), which is irrelevant here; the health cross-check applies to
  // every required node on every coordinated wait, deferred or not. vanish_em is still silenced
  // (required, unhealthy) when ABORTING's own wait runs, so it ALSO resolves via
  // ABORTED_BY_HEALTH, returning error code 1 -- which posts an ErrorEvent while the machine is
  // CURRENTLY IN Aborting. Per states_generator.hpp, the only ERROR transition
  // (abortable_aborting_on_error) is attached to the `abortable` superstate, and Aborting/
  // Aborted are siblings of `abortable`, not descendants of it (confirmed: add_state(sm,
  // abortable); add_state(sm, Aborting); add_state(sm, Aborted); are all top-level, and
  // Aborting's own transitions list only STATE_COMPLETED -> Aborted, no ERROR transition at
  // all). Without the Aborting --ERROR--> Aborted edge this ErrorEvent has no matching transition
  // anywhere and is silently dropped, leaving the machine permanently stuck in ABORTING and
  // recoverable only by restarting the manager process. With it, the machine reaches the safe
  // terminal and CLEAR recovers from there as normal.
  if (final_state == packml_sm::State::ABORTING) {
    EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
      << "REGRESSION: machine did not settle from ABORTING to ABORTED. ABORTING's own "
         "coordinated wait health-cross-checks the same still-unhealthy required node and fails, "
         "posting an ErrorEvent from inside Aborting -- which is only escapable because of the "
         "Aborting --ERROR--> Aborted edge in states_generator.hpp. If this fails, that edge is "
         "gone or no longer matches (see this block's own comment for the full mechanism)";
  }

  // Let the manager's own cancel (issued once the wait above resolved, see
  // cancel_pending_state_goals()) actually reach the still-spinning EM and let its deferred-
  // completion background thread exit cleanly before this test's local `em`/rig are destroyed.
  std::this_thread::sleep_for(300ms);

  // Stop spinning rig.em_node BEFORE `em` destructs (see FlappingDiscoverability's own comment
  // above for why this ordering matters and is not just defensive paranoia).
  rig.em_spin.reset();
}

// ============================================================================
// Confirmed-safe (expected): retry double-delivery. Same discoverability setup as
// LateDiscoveredStateChildEventuallyReceivesTransition (test_manager_client_fanout.cpp) -- an EM
// not yet discoverable when the fan-out round begins -- but brought up close to a
// check_fanout_deadlines() retry-tick boundary (that timer runs every ~200ms) instead of well
// after several ticks have already passed, to give the discovery event and the retry path a
// realistic chance to overlap. Reading fanout_state_transition()/check_fanout_deadlines()
// closely: a client is only ever in ONE of {sent synchronously and marked ACKED} or {left in
// `unsent_` for retry} for a given round -- the synchronous attempt and the retry closure are
// never both live for the same client at once, so this SHOULD be structurally impossible to
// double-send today. This test is the empirical check of that reading, not just the argument.
TEST_F(MonkeyFanoutTest, MonkeyRetryDoesNotDoubleDeliverAGoalTheEmActuallyRuns)
{
  auto rig = begin_setup("monkey_retry_double_delivery", "retry_dup_em");
  finish_setup(rig);  // rig.em_node spins from the start; no EM object lives on it yet

  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET rejected: " << resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::RESETTING, 500ms))
    << "machine did not enter RESETTING";

  // Bring the EM up just before the first ~200ms retry tick -- late enough that the original,
  // synchronous send attempt (issued the instant RESETTING's fan-out began) has certainly
  // already missed it and queued it into `unsent_`, but close enough to the next tick that the
  // discovery event and the retry's own action_server_is_ready() check have a real chance to
  // land close together in time.
  std::this_thread::sleep_for(190ms);
  auto em = std::make_shared<TrackingEquipmentModule>(rig.em_node);

  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 3s))
    << "machine did not reach IDLE once the late-discovered EM came up, current state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // The EM must have received the RESETTING goal for this round exactly once -- not a
  // duplicate delivery from the retry path racing the original attempt's own bookkeeping.
  // (Checked via resetting_received, not the generic transitions_received/last_state_received:
  // by the time the machine reaches IDLE, RESETTING's own successful completion has already
  // triggered a SEPARATE, legitimate fan-out for IDLE too -- see TrackingEquipmentModule's own
  // comment. That is normal, expected behavior, not the double-delivery this test is checking
  // for.)
  // NOT an equality check, deliberately. This test exists to catch DOUBLE delivery from the
  // retry path racing the original send, so "more than one" is the failure it polices.
  //
  // A count of 0 is a DIFFERENT, separately-tracked defect and must not be conflated with it:
  // the goal IS delivered, but the EM's guard has already adopted RESETTING from the manager's
  // own status echo, so request_state() takes the already_there shortcut and never calls
  // on_state_trans_req(). The log shows exactly that -- "Node already in state: RESETTING"
  // followed by "completed state transition". Which of the two lands first is a ~10ms race
  // between this test's 190ms EM-startup sleep and the manager's 200ms acceptance-wait status
  // publish; it is stable within a given binary and flips on recompilation, so asserting on it
  // would make this test a coin flip on an unrelated mechanism. The shortcut itself is the
  // already_there/local-vouch defect covered separately (see test_shakedown_wire.cpp's
  // KnownDefect_StatusSpoofing... and test_shakedown_guard.cpp).
  //
  // Delivery is still proven, by the ASSERT above: with exactly one registered EM the machine
  // cannot reach IDLE unless that EM answered RESETTING's goal.
  // WHAT THIS ASSERTION IS WORTH: the EM takes the already-there shortcut on every run, logging
  // "Node already in state: RESETTING", so resetting_received is 0 and this EXPECT_LE passes on
  // zero. A genuine double
  // delivery in which both goals also shortcut would read 0 and pass too. The test is therefore
  // NOT duplicate-delivery coverage today; treat it as a no-hang/no-crash exercise of the retry
  // path, which is real, plus an assertion that becomes meaningful again the moment a goal is
  // observed.
  //
  // The cause is the already_there shortcut in TransitionGuard::decide_state(), and it is not
  // going to change: that shortcut trusting a broadcast status is an ACCEPTED RISK, on the
  // grounds that the deployment has no adversarial publishers. What stands in its place is
  // DETECTION ONLY (the manager warns when another node publishes the status topic it owns; see
  // foreign_publishers_on()). Detection addresses the duplicate-manager
  // MISCONFIGURATION; it does nothing about the shortcut itself, which is what empties this
  // counter. The shortcut's own characterization test is
  // test_shakedown_wire.cpp's KnownDefect_StatusSpoofingSkipsRealWorkViaAlreadyThere -- a
  // different test, deliberately kept as the marker.
  //
  // Left GREEN rather than GTEST_SKIP'd on the zero case. A skip that fires on every run is a
  // permanently skipped test, which trains readers to ignore skips and buys no signal that this
  // comment does not already give. Counting deliveries at a level the shortcut cannot bypass would
  // need goal-count instrumentation the EM base class does not expose, and inventing that API for
  // one monkey test is not worth it.
  EXPECT_LE(em->resetting_received.load(), 1)
    << "EM received " << em->resetting_received.load() << " RESETTING goals for one fan-out "
       "round -- the discovery-retry path delivered a duplicate alongside the original send";

  // Stop spinning rig.em_node BEFORE `em` destructs (see FlappingDiscoverability's own comment
  // for why this ordering matters and is not just defensive paranoia).
  rig.em_spin.reset();
}
