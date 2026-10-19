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
// SHAKEDOWN suite (separate binary -- see CMakeLists.txt's ${PROJECT_NAME}_shakedown target and
// its own comment for why these probes live apart from the polished main suite), POISONED
// CONFIGURATION group: nonsensical parameter/request values the system should contain
// gracefully -- duplicated node_names entries, unknown mode values on the wire, a zero
// state_complete_timeout_ms, and heartbeats advertising heartbeat_interval_ms = 0. Unlike the
// monkey files (which stress TIMING with well-formed inputs), every scenario here feeds the
// system a configuration or request value that is nonsense on its face and asserts the intended
// CORRECT containment behavior, with generous timing bounds. Assertions are deliberately NOT
// pre-weakened: a red result from one of these is exactly the finding the probe exists to make,
// and goes to triage rather than being loosened away (see test_monkey_scenarios.cpp's standing
// rule on that).
//
// House style follows test_monkey_scenarios.cpp: anonymous-namespace helper classes/functions
// in this file's own anonymous namespace (generic scaffolding lives in test_helpers.hpp), a Rig
// struct + begin_setup()/
// finish_setup() pair building a standalone manager + EM per test with unique node names and
// construction-time parameter_overrides, a wait_for_state() polling helper, and a
// send_state_change() helper. The raw-heartbeat-publisher scenario copies its
// matching-wait/make_heartbeat patterns from test_monkey_health.cpp; the EM-side NodeOptions
// override pattern comes from test_monkey_reporting.cpp's begin_setup().
//
// TEARDOWN RULE (confirmed the hard way, via gdb, in test_monkey_multi_mode.cpp's
// ClientDisconnectMidRequest): any Equipment Module object (PackmlNodeInterface subclass)
// declared AFTER the Rig destructs BEFORE the Rig's own em_spin member stops spinning -- a
// still-in-flight fan-out callback then calls into an object whose vtable is already gone
// ("pure virtual method called", SIGABRT, takes the whole shared binary down). Every test here
// therefore ends with rig.em_spin.reset() after all assertions, before returning.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_ros/ros_names.hpp"
#include "packml_msgs/srv/mode_change.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/msg/node_heartbeat.hpp"
#include "packml_sm/common.hpp"
#include "packml_sm/default_modes.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

packml_msgs::srv::ModeChange::Response::SharedPtr send_mode_change(
  rclcpp::Client<packml_msgs::srv::ModeChange>::SharedPtr client,
  int8_t mode_val,
  std::chrono::seconds timeout = 5s)
{
  auto req = std::make_shared<packml_msgs::srv::ModeChange::Request>();
  req->mode.val = mode_val;
  auto future = client->async_send_request(req);
  if (future.wait_for(timeout) == std::future_status::ready) {
    return future.get();
  }
  return nullptr;
}

/// Like wait_for_state(), but satisfied by ANY of several targets -- used by the zero-timeout
/// probe below, where the machine can legitimately be observed in either ABORTING or (if the
/// race resolves the friendly way) already in ABORTED by the first poll.
bool wait_for_any_state(
  const std::unique_ptr<SMNode_new> & sm_node,
  const std::vector<packml_sm::State> & targets,
  std::chrono::milliseconds timeout)
{
  const auto is_target = [&targets](packml_sm::State s) {
      return std::find(targets.begin(), targets.end(), s) != targets.end();
    };
  return packml_ros_test::wait_until(
    [&sm_node, &is_target] {return is_target(sm_node->getCurrentState());}, timeout, 10ms);
}

/// Non-deferring EM that counts, per state, how many times its on_state_trans_req() hook is
/// invoked -- the duplicate-node-names probe uses this to prove a deduped fan-out reaches the
/// one real EM exactly once per acting state, never twice (double fan-out) or zero times.
class CountingEquipmentModule : public PackmlNodeInterface
{
public:
  explicit CountingEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  int count_for(packml_sm::State state) const
  {
    std::lock_guard<std::mutex> lk(counts_mutex_);
    const auto it = counts_.find(state);
    return it == counts_.end() ? 0 : it->second;
  }

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    std::lock_guard<std::mutex> lk(counts_mutex_);
    ++counts_[state];
    return true;
  }
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}

private:
  mutable std::mutex counts_mutex_;
  std::map<packml_sm::State, int> counts_;
};

/// Non-deferring EM that records every ModeType value its on_mode_trans_req() hook is asked to
/// approve (and approves all of them) -- the unknown-mode probe uses this to observe whether a
/// garbage mode value the manager should have rejected was instead fanned out to a registered
/// child as if it were real.
class ModeRecordingEquipmentModule : public PackmlNodeInterface
{
public:
  explicit ModeRecordingEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  bool saw_mode(packml_sm::ModeType mode) const
  {
    std::lock_guard<std::mutex> lk(modes_mutex_);
    return std::find(modes_.begin(), modes_.end(), mode) != modes_.end();
  }

  std::vector<packml_sm::ModeType> modes_seen() const
  {
    std::lock_guard<std::mutex> lk(modes_mutex_);
    return modes_;
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    std::lock_guard<std::mutex> lk(modes_mutex_);
    modes_.push_back(mode);
    return true;
  }
  void on_status_changed() override {}

private:
  mutable std::mutex modes_mutex_;
  std::vector<packml_sm::ModeType> modes_;
};

/// EM that defers exactly one configured `monkey_state` and NEVER reports its completion -- same
/// shape as test_monkey_reporting.cpp's ChronicallyForgetfulEquipmentModule, written per file per
/// this directory's no-shared-header convention. The zero-timeout probe uses it to guarantee
/// RESETTING's coordinated wait cannot resolve via a completion, only via the (instant) timeout.
class NeverReportingEquipmentModule : public PackmlNodeInterface
{
public:
  NeverReportingEquipmentModule(rclcpp::Node::SharedPtr node, packml_sm::State monkey_state)
  : monkey_state_(monkey_state)
  {
    init(node);
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override {return state == monkey_state_;}
  // Takes the handle and drops it, rather than leaving the base implementation to report a
  // failure for an unimplemented state -- riding out the timeout is the whole point here.
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion) override {}

private:
  packml_sm::State monkey_state_;
};

/// Build a NodeHeartbeat for direct publication by a plain rclcpp::Publisher, bypassing
/// PackmlNodeInterface's own heartbeat plumbing -- copied from test_monkey_health.cpp, used by
/// the zero-advertised-interval probe, which needs to control heartbeat_interval_ms directly
/// (a real PackmlNodeInterface clamps its own parameter to a positive value in init()).
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

class ConfigMonkeyTest : public ::testing::Test
{
protected:
  /// Same shape as test_monkey_scenarios.cpp's Rig: every test builds its own standalone
  /// manager + EM pair (unique node names), never shared fixture state. em_node/em_spin stay
  /// null for the manager-only heartbeat rig (begin_health_setup below).
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

  /// Fan-out rig: the manager's node_names parameter is taken VERBATIM from `node_names`
  /// (it may deliberately contain duplicates -- that is the whole point of the first probe),
  /// while exactly ONE real EM node (`em_name`) is created regardless. Extends the
  /// test_monkey_scenarios.cpp original with test_monkey_reporting.cpp's optional EM-side
  /// NodeOptions (EM parameters must be set at construction, before PackmlNodeInterface::init()
  /// reads them). Does NOT construct the EM itself -- callers construct their subclass on
  /// rig.em_node, then call finish_setup().
  Rig begin_setup(
    const std::string & mgr_prefix, const std::vector<std::string> & node_names,
    const std::string & em_name,
    int state_complete_timeout_ms = 5000,
    const rclcpp::NodeOptions & em_node_options = rclcpp::NodeOptions())
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter(packml_ros::kParamNodeNames, node_names),
        rclcpp::Parameter(packml_ros::kParamStateCompleteTimeoutMs, state_complete_timeout_ms),
      }));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.em_node = rclcpp::Node::make_shared(em_name, em_node_options);
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/" + packml_ros::kChangeStateService);
    rig.mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.mgr_node);
    return rig;
  }

  /// Manager-only heartbeat rig, mirroring test_monkey_health.cpp's begin_setup(): exactly one
  /// REQUIRED node registered for the health monitor, node_names deliberately left unset (no
  /// action-based fan-out EM at all, so state transitions resolve instantly once the health
  /// gate allows them), and a small heartbeat_startup_grace_ms so the liveness timeout is
  /// observable in seconds: startup_interval = grace / heartbeat_timeout_factor(3.0), and the
  /// effective per-node timeout after the clamp is startup_interval x 3.0 = grace itself
  /// (see PackmlManagerInterface::init() and HealthMonitor::set_max_expected_interval_ms()).
  Rig begin_health_setup(
    const std::string & mgr_prefix, const std::string & required_node_name,
    int heartbeat_startup_grace_ms)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name);
    rig.mgr_node->declare_parameter(
      packml_ros::kParamRequiredNodes, std::vector<std::string>{required_node_name});
    rig.mgr_node->declare_parameter(
      packml_ros::kParamHeartbeatStartupGraceMs, heartbeat_startup_grace_ms);
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/" + packml_ros::kChangeStateService);
    rig.mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.mgr_node);
    return rig;
  }

  void finish_setup(Rig & rig)
  {
    if (rig.em_node) {
      rig.em_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.em_node);
    }
    ASSERT_TRUE(rig.state_client->wait_for_service(5s));
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
    std::this_thread::sleep_for(300ms);
  }
};

// ============================================================================
// ATTACK: node_names = {"dup_em", "dup_em"} -- the same Equipment Module listed twice in the
// manager's fan-out roster, a copy-paste-grade config mistake any deployment could ship.
//
// WHY IT COULD BREAK: PackmlManagerInterface::init() (packml_interface.hpp) builds client_map_
// with `client_map_[node_name] = make_shared<PackmlClientInterface>(...)` in a loop -- a
// std::map keyed by name, so the second entry OVERWRITES the first (constructing, then
// destroying, a redundant client set along the way). fanout_state_transition() then derives the
// round's roster from client_map_, and CompletionTracker::begin_round() keys its own map by
// name too. If any of those layers instead kept a phantom second entry, the machine would
// either double-fan-out to the same EM (its on_state_trans_req() hook firing twice per acting
// state) or hang every coordinated wait forever on a second completion that can never arrive
// (only one real node exists to answer).
//
// CORRECT BEHAVIOR: the duplicate dedupes benignly END TO END -- a full RESET->IDLE cycle
// completes within a normal bound, the EM's RESETTING hook fired exactly once, and
// ABORT->ABORTED then CLEAR->STOPPED work normally (with ABORTING/CLEARING also fanned out
// exactly once each).
//
// CONTRACT SOURCES: packml_interface.hpp init() (client_map_ construction) and
// fanout_state_transition(); completion_tracker.hpp begin_round().
TEST_F(ConfigMonkeyTest, DuplicateNodeNames_DedupedFanoutCompletesCleanly)
{
  const auto em_name = packml_ros_test::unique_node_name("shakedown_dup_em");
  auto rig = begin_setup("shakedown_dup_names", {em_name, em_name}, em_name);
  auto em = std::make_shared<CountingEquipmentModule>(rig.em_node);
  finish_setup(rig);

  // --- Full RESET -> IDLE cycle: must complete, not hang on a phantom second completion. ---
  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr) << "RESET got no response";
  ASSERT_TRUE(reset_resp->success) << "RESET rejected: " << reset_resp->message;
  // Deliberately no wait for the transient RESETTING state itself: with a non-deferring EM it
  // can resolve faster than the 10ms poll (flake, not signal); the RESETTING fan-out counter
  // below is the real evidence it happened, and happened once.
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 2s))
    << "machine did not reach IDLE -- a duplicated node_names entry must not stall the "
       "coordinated wait on a phantom second completion";

  EXPECT_EQ(em->count_for(packml_sm::State::RESETTING), 1)
    << "the single real EM saw RESETTING fanned out " << em->count_for(packml_sm::State::RESETTING)
    << " time(s) -- a duplicated node_names entry must dedupe (client_map_ is keyed by name), "
       "never double-fan-out";

  // --- ABORT -> ABORTED then CLEAR -> STOPPED must also work, each fanned out exactly once. ---
  auto abort_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr) << "ABORT got no response";
  ASSERT_TRUE(abort_resp->success) << "ABORT rejected: " << abort_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 2s))
    << "machine did not reach ABORTED";

  auto clear_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr) << "CLEAR got no response";
  ASSERT_TRUE(clear_resp->success) << "CLEAR rejected: " << clear_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "machine did not reach STOPPED after CLEAR";

  EXPECT_EQ(em->count_for(packml_sm::State::ABORTING), 1)
    << "ABORTING was fanned out " << em->count_for(packml_sm::State::ABORTING)
    << " time(s) to the single real EM, expected exactly 1";
  EXPECT_EQ(em->count_for(packml_sm::State::CLEARING), 1)
    << "CLEARING was fanned out " << em->count_for(packml_sm::State::CLEARING)
    << " time(s) to the single real EM, expected exactly 1";
  EXPECT_EQ(em->count_for(packml_sm::State::RESETTING), 1)
    << "RESETTING count drifted after the ABORT/CLEAR recovery -- a stale duplicate entry "
       "must not replay earlier fan-outs";

  // See this file's header: stop the EM's spinner before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// ATTACK: ~/changeMode with mode.val = 99, then mode.val = -1 -- values no modes YAML defines
// (the generated packml_sm/default_modes.hpp knows Invalid=0, Production=1, Maintenance=2,
// Manual=3 -- and its to_string() just stringifies anything else).
//
// WHERE THE VALUE IS CHECKED, and why it has to be there: on_change_mode()
// (packml_interface.hpp) now rejects any value is_known_mode() does not recognise, before the
// request reaches the state machine. Nothing further down can do it. ModeType is a bare `int`
// (common.hpp); StatesGenerator::mode_switcher() (states_generator.hpp) checks only that the
// CURRENT STATE permits mode switching (switch_states == {IDLE}), never that the mode exists; and
// StateMachine::changeMode(ModeType)'s single-argument overload builds an ALL-STATES-AVAILABLE
// mask for whatever number it is handed.
//
// That last one is why this matters more than "garbage propagates": an undeclared mode admitted
// this far arrives with no state mask at all, so every command restriction the configured modes
// impose is silently absent until the next valid mode change -- a state that correctly refuses a
// command in Production would accept it. It is also latched into the manager's current_mode,
// published on the latched packml_status topic, and fanned out to every registered EM's
// on_mode_trans_req() as an approved switch.
//
// WHAT IS ASSERTED: both probes are refused with INVALID_MODE_REQUEST, neither reaches the child
// EM's hook, and the containment floor still holds -- no crash, no hang, and a subsequent VALID
// mode change still succeeds. The hard ASSERTs are that floor; a failure there means the probes
// wedged the mode machinery rather than being turned away by it.
//
// CONTRACT SOURCES: packml_interface.hpp on_change_mode(); packml_sm/src/state_machine.cpp
// changeMode(); packml_sm/include/packml_sm/states_generator.hpp mode_switcher() +
// switch_states; generated packml_sm/default_modes.hpp (is_known_mode()).
//
// Was KnownDefect_UnknownModeValueAcceptedAndPropagated, a characterization test asserting the
// accepted-and-propagated behaviour so the suite could still gate. It failed the moment the
// validation landed, which is what it was built to do.
TEST_F(ConfigMonkeyTest, UnknownModeValueRejectedAndNeverPropagated)
{
  const auto em_name = packml_ros_test::unique_node_name("shakedown_mode_em");
  auto rig = begin_setup("shakedown_unknown_mode", {em_name}, em_name);
  auto em = std::make_shared<ModeRecordingEquipmentModule>(rig.em_node);
  finish_setup(rig);

  // The machine starts in STOPPED, a valid state for runtime mode changes. Any
  // rejection below therefore concerns the requested mode value.
  auto mode_client = rig.mgr_node->create_client<packml_msgs::srv::ModeChange>(
    rig.mgr_name + "/" + packml_ros::kChangeModeService);
  ASSERT_TRUE(mode_client->wait_for_service(5s));

  // --- Probe 1: mode.val = 99. Hard floor: a response arrives at all (no hang, no crash). ---
  auto resp99 = send_mode_change(mode_client, static_cast<int8_t>(99));
  ASSERT_NE(resp99, nullptr) << "changeMode(99) never got a response -- possible hang";
  EXPECT_FALSE(resp99->success)
    << "changeMode(99) was ACCEPTED. An undeclared mode reaches changeMode()'s single-argument "
       "overload, which builds an ALL-STATES-AVAILABLE mask for it, so every command restriction "
       "the configured modes impose is silently gone until the next valid mode change";
  EXPECT_EQ(resp99->error_code, resp99->INVALID_MODE_REQUEST);

  // --- Probe 2: mode.val = -1. Same containment floor, same rejection. ---
  auto resp_neg = send_mode_change(mode_client, static_cast<int8_t>(-1));
  ASSERT_NE(resp_neg, nullptr) << "changeMode(-1) never got a response -- possible hang";
  EXPECT_FALSE(resp_neg->success)
    << "changeMode(-1) was ACCEPTED -- see the note on the 99 probe above";

  // Give the (detached) mode fan-out threads time to deliver before inspecting what the EM saw.
  std::this_thread::sleep_for(500ms);
  {
    std::string seen;
    for (const auto m : em->modes_seen()) {
      seen += (seen.empty() ? "" : ", ") + std::to_string(m);
    }
    // The propagation half, checked at the child's end of the wire: a rejected mode must never
    // have been fanned out at all, so no registered Equipment Module's on_mode_trans_req() ever
    // sees it as an approved switch. Both probes are asserted here: a request rejected before it
    // reaches the state machine cannot reach a child by any route, so neither value has a window
    // in which it propagates intermittently.
    EXPECT_FALSE(em->saw_mode(static_cast<packml_sm::ModeType>(99)))
      << "the undeclared mode 99 reached the EM as an approved switch (EM saw: [" << seen << "])";
    EXPECT_FALSE(em->saw_mode(static_cast<packml_sm::ModeType>(-1)))
      << "the undeclared mode -1 reached the EM as an approved switch (EM saw: [" << seen << "])";
  }

  // --- Containment floor: a subsequent VALID mode change must still succeed, whatever the
  // garbage probes did. Hard ASSERT: failing this means the poison wedged the mode machinery.
  auto valid_resp = send_mode_change(mode_client, static_cast<int8_t>(packml_modes::Production));
  ASSERT_NE(valid_resp, nullptr)
    << "changeMode(Production) never got a response after the garbage probes -- possible hang";
  EXPECT_TRUE(valid_resp->success)
    << "a valid Production mode change failed after the garbage probes: " << valid_resp->message;

  // The manager must also still answer plain state commands (response arriving is the check;
  // acceptance is not the point here).
  auto probe_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(probe_resp, nullptr) << "manager stopped answering state commands after mode probes";

  // No settle sleep before teardown. on_change_mode() detaches an unjoined thread per request, so
  // tearing the rig down under one of those would crash the shared binary; a sleep here would only
  // paper over that. PackmlManagerInterface::shutdown() drains them instead, which is what makes
  // the margin unnecessary rather than merely absent.
  // See this file's header: stop the EM's spinner before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// ATTACK: state_complete_timeout_ms = 0 -- "wait zero milliseconds for coordinated
// completion" -- combined with one EM that defers RESETTING and never reports.
//
// TIMEOUT=0 SEMANTICS (read before writing this; 0 is NOT special-cased anywhere, so 0 is used
// here as specified rather than falling back to 1): PackmlManagerInterface::init()
// (packml_interface.hpp) reads the parameter as a plain int and hands
// std::chrono::milliseconds(0) straight to CompletionTracker::wait_for_all()
// (completion_tracker.hpp), whose cv_.wait_for(lk, 0ms, pred) evaluates the predicate exactly
// once and returns TIMEOUT if any node lacks a result. Ordering note: the fan-out (and its
// begin_round()) runs SYNCHRONOUSLY inside the acting state's onEntry -- PackmlState::onEntry()
// emits stateEntered (state.cpp) -> on_state_changed -> fanout_state_transition() BEFORE
// ActingState::onEntry() (acting_state.cpp) schedules operation() on QtConcurrent -- so the
// instant timeout is evaluated against the REAL current round, not a stale one.
//
// THE EXPECTED CHAIN, step by step:
//   1. RESET from STOPPED enters RESETTING; its wait times out essentially instantly (the
//      deferring EM could never have reported in ~0ms) -> the bound operation returns 1
//      (init()'s TIMEOUT arm, after cancel_pending_state_goals()) -> ActingState::operation()
//      posts ErrorEvent (acting_state.cpp). RESETTING sits inside the `abortable` superstate,
//      which owns the graph's ONLY ErrorTransition (states_generator.hpp,
//      generate_all_packml_states()) -> routes to ABORTING. This half is expected to WORK.
//   2. ABORTING is ITSELF a coordinated state with the SAME 0ms timeout: its own wait also
//      times out instantly -> returns 1 -> ErrorEvent posted while ALREADY IN Aborting.
//      Aborting/Aborted are SIBLINGS of
//      `abortable` (added directly to the SM), so that ErrorEvent matches NO transition and is
//      silently dropped -- and the TIMEOUT path, unlike a completed wait, never posts
//      STATE_COMPLETED, so nothing else can drive Aborting -> Aborted. The machine would be
//      PERMANENTLY stuck in ABORTING: a timeout-driven twin of the known health-driven
//      deadlock, requiring NO unhealthy node at all -- just an aggressive timeout config plus
//      one slow EM.
//   (Benign-race caveat: ABORTING's single predicate pass COULD, rarely, find the EM's
//   instant ABORTING result already recorded -- the EM does not defer ABORTING -- in which
//   case the wait returns COMPLETE and the machine reaches ABORTED normally. TIMEOUT is
//   expected to win nearly always; either way the assertion below states correct behavior.)
//
// CORRECT BEHAVIOR: the machine reaches ABORTED within a generous 3s bound. If this FAILS,
// that is exactly the finding this probe exists to make -- the assertion stays strict per this
// suite's standing rule; do not loosen it, triage it.
//
// CONTRACT SOURCES: packml_interface.hpp init() (setInterruptibleStateOperation binding, the
// kParamStateCompleteTimeoutMs doc comment in ros_names.hpp); completion_tracker.hpp
// wait_for_all(); packml_sm acting_state.cpp operation(); packml_sm states_generator.hpp
// generate_all_packml_states().
TEST_F(ConfigMonkeyTest, ZeroStateCompleteTimeout_InstantTimeoutsDoNotWedgeTheMachine)
{
  const auto em_name = packml_ros_test::unique_node_name("shakedown_zero_timeout_em");
  // EM-side deferred_completion_timeout_ms is shortened so the EM's own never-resolved
  // RESETTING wait (a detached thread holding a raw `this`) is guaranteed to fall back on its
  // own safety net within this test's lifetime even if the manager's cancel never lands --
  // teardown hygiene for the EM, irrelevant to the manager-side 0ms behavior under test.
  auto em_options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter(packml_ros::kParamDeferredCompletionTimeoutMs, 1500)});
  auto rig = begin_setup(
    "shakedown_zero_timeout", {em_name}, em_name,
    /*state_complete_timeout_ms=*/0, em_options);
  auto em = std::make_shared<NeverReportingEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  auto reset_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr) << "RESET got no response";
  ASSERT_TRUE(reset_resp->success) << "RESET rejected: " << reset_resp->message;

  // Step 1 of the chain must work: the instant RESETTING timeout errors out to ABORTING (the
  // ErrorTransition on `abortable` exists and RESETTING is inside it). Accept ABORTED too --
  // with a 0ms timeout the machine can fly through ABORTING between 10ms polls when the
  // benign race resolves the friendly way.
  ASSERT_TRUE(wait_for_any_state(
      rig.sm_node, {packml_sm::State::ABORTING, packml_sm::State::ABORTED}, 2s))
    << "RESETTING's instant timeout did not route to ABORTING at all (stuck in state "
    << static_cast<int>(rig.sm_node->getCurrentState())
    << ") -- even the abortable-superstate ErrorTransition leg failed";

  // Step 2, THE probe: ABORTING's own instantly-timed-out wait must still get the machine to
  // ABORTED.
  //
  // The timeout-driven twin of the health-driven ABORTING deadlock, reachable by
  // MISCONFIGURATION ALONE -- no unhealthy node anywhere, just an aggressive timeout and one
  // slow EM. Both twins rest on the same edge, Aborting --ERROR--> Aborted
  // (states_generator.hpp), which is what gives the ErrorEvent the timeout path posts from
  // inside Aborting somewhere to go. The TIMEOUT path never posts STATE_COMPLETED, which is
  // why an error edge rather than a completion edge is the right shape here.
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 3s))
    << "ABORTING WEDGE REGRESSION (timeout-driven): ABORTING's own 0ms coordinated wait timed "
       "out and its ErrorEvent, posted from inside Aborting, went nowhere -- machine stuck in "
       "state "
    << static_cast<int>(rig.sm_node->getCurrentState())
    << " with no unhealthy node anywhere, just an aggressive timeout config and one slow EM. "
       "The Aborting --ERROR--> Aborted edge in states_generator.hpp is what makes this "
       "recoverable; if this fails, that edge is gone or no longer matches.";

  // Whatever the state machine did, the MANAGER's service layer must still answer (a wedged
  // graph must not become a wedged process). Acceptance is not asserted -- from a stuck
  // ABORTING every command is legitimately rejectable -- only that a response arrives.
  auto probe_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(probe_resp, nullptr)
    << "manager stopped answering state commands entirely after the zero-timeout run";

  // Let the EM's deferred-wait safety net (1500ms, see em_options above) resolve its detached
  // thread before the EM object destructs.
  std::this_thread::sleep_for(2000ms);
  // See this file's header: stop the EM's spinner before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// ATTACK: a required node whose every heartbeat advertises heartbeat_interval_ms = 0 --
// "I promise a beat every zero milliseconds" -- published raw (SensorDataQoS, per-node topic
// "/<name>/heartbeat", matching-wait before publishing; pattern copied from
// test_monkey_health.cpp), since a real PackmlNodeInterface forces its own interval parameter
// positive in init() and so can never emit this poison itself.
//
// WHY IT COULD BREAK: HealthMonitor::on_heartbeat() (health_monitor.hpp) learns a node's
// interval from what the node advertises; if 0 were learned, effective_timeout_ms() would
// become 0 x factor = 0 and every liveness/staleness comparison (elapsed >= timeout) would be
// instantly true -- the node permanently "stale" the moment each beat lands, the RESET health
// gate permanently shut, ABORT firing every check tick. The source guards this: only an
// advertised interval > 0 is learned (`if (msg.heartbeat_interval_ms > 0)`), so an advertised
// 0 is IGNORED and the startup-derived interval stays in force (registered at
// startup_interval_ms = heartbeat_startup_grace_ms / heartbeat_timeout_factor, see
// PackmlManagerInterface::init()).
//
// CORRECT BEHAVIOR, both directions of "liveness stays bounded":
//   1. Beats advertising 0 still COUNT as liveness: with beats flowing (and one published
//      fresh, right before the check -- gate_block_reason()'s staleness comparison is
//      real-time against the last accepted beat, not the periodic timer), RESET from STOPPED
//      succeeds.
//   2. Advertising 0 does not EXTEND liveness either: once the publisher goes silent past the
//      effective timeout (grace = 1500ms here), the node is detected absent and a RESET from
//      STOPPED is rejected. Detection can surface two ways, both valid evidence: the gate
//      itself rejects with "Health gate blocked", or check_timeouts() already fired ABORT
//      (valid from STOPPED via the abortable superstate) and the machine has left STOPPED, so
//      the RESET is rejected from ABORTING/ABORTED.
//
// CONTRACT SOURCES: health_monitor.hpp on_heartbeat() (the `> 0` learn guard),
// effective_timeout_ms(), gate_block_reason(), check_timeouts();
// packml_interface.hpp init() (startup_interval derivation + heartbeat subscription) and
// on_change_state() (the RESET-from-STOPPED health gate).
TEST_F(ConfigMonkeyTest, ZeroAdvertisedHeartbeatInterval_LivenessStaysBounded)
{
  const auto em_name = packml_ros_test::unique_node_name("shakedown_zero_interval_em");
  // grace 1500ms -> startup interval 500ms -> effective timeout 500 x 3.0 = 1500ms: small
  // enough to observe absence within seconds (same derivation test_monkey_health.cpp's
  // interval-flapping test uses).
  auto rig = begin_health_setup(
    "shakedown_zero_interval", em_name, /*heartbeat_startup_grace_ms=*/1500);
  finish_setup(rig);

  auto pub_node = rclcpp::Node::make_shared(
    packml_ros_test::unique_node_name("shakedown_zero_interval_pub"));
  auto publisher = pub_node->create_publisher<packml_msgs::msg::NodeHeartbeat>(
    "/" + em_name + "/" + std::string(packml_ros::kHeartbeatTopic), rclcpp::SensorDataQoS());
  {
    packml_ros_test::wait_until(
      [&] {return publisher->get_subscription_count() > 0;}, 2s, 10ms);
    ASSERT_GT(publisher->get_subscription_count(), 0u)
      << "heartbeat publisher never matched the manager's subscription";
  }

  // --- Direction 1: beats advertising interval=0, every beat, still count as liveness. ---
  uint64_t seq = 1;
  for (int i = 0; i < 12; ++i) {
    publisher->publish(make_heartbeat(em_name, seq++, /*interval_ms=*/0));
    std::this_thread::sleep_for(100ms);
  }
  // Normalize to STOPPED before the gate check: if discovery/matching above ate into the
  // 1500ms startup grace, the required node may have briefly timed out BEFORE the first beat
  // landed, firing ABORT and parking the machine in ABORTED -- a startup artifact unrelated to
  // this probe. CLEAR is a harmless no-op from STOPPED and recovers from ABORTED; the node is
  // no longer timed out (beats are flowing), so re-entering STOPPED does not re-fire ABORT.
  send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "machine not in STOPPED before the liveness gate check, state: "
    << static_cast<int>(rig.sm_node->getCurrentState());
  // One fresh beat immediately before the gate check: the staleness comparison is real-time,
  // so "beats are flowing" must be true at the RESET instant, not merely recently.
  publisher->publish(make_heartbeat(em_name, seq++, /*interval_ms=*/0));
  std::this_thread::sleep_for(50ms);

  auto reset_ok = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_ok, nullptr) << "RESET got no response while zero-interval beats were flowing";
  EXPECT_TRUE(reset_ok->success)
    << "RESET was blocked while fresh heartbeats were flowing -- an advertised interval of 0 "
       "must be IGNORED (only >0 learns), not learned as a 0ms timeout that marks every beat "
       "instantly stale: " << reset_ok->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 2s))
    << "machine did not reach IDLE (no fan-out EMs are registered; this should be immediate)";

  // Back to STOPPED so the second gate check below is a genuine RESET-from-STOPPED.
  auto stop_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_resp, nullptr);
  ASSERT_TRUE(stop_resp->success) << "STOP rejected: " << stop_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "machine did not return to STOPPED";

  // --- Direction 2: silence past the effective timeout (1500ms) must be detected. ---
  std::this_thread::sleep_for(2500ms);

  auto reset_blocked = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_blocked, nullptr) << "RESET got no response after the node went silent";
  EXPECT_FALSE(reset_blocked->success)
    << "RESET succeeded despite 2.5s of silence from a required node whose beats all "
       "advertised interval=0 -- the startup-derived liveness bound did not hold";
  {
    // Corroborate that the rejection is genuinely absence detection, whichever surface it
    // took (see this test's header comment): the gate's own message, or the timeout-ABORT
    // having already moved the machine out of STOPPED.
    const bool gate_message =
      reset_blocked->message.find("Health gate blocked") != std::string::npos;
    const auto state_now = rig.sm_node->getCurrentState();
    const bool timeout_abort_fired =
      state_now == packml_sm::State::ABORTING || state_now == packml_sm::State::ABORTED;
    EXPECT_TRUE(gate_message || timeout_abort_fired)
      << "RESET failed but neither via the health gate nor via a timeout-driven ABORT "
         "(message: '" << reset_blocked->message << "', state: "
      << static_cast<int>(state_now) << ") -- the failure is not absence detection";
  }

  // Manager must still be alive and answering after the silence-driven detection.
  auto probe_resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(probe_resp, nullptr) << "manager stopped responding after the silence phase";

  // No PackmlNodeInterface EM exists in this rig (raw publisher only), so the header's
  // teardown rule is trivially satisfied; reset() on the null spinner keeps the convention
  // uniform across every test in this file.
  rig.em_spin.reset();
}
