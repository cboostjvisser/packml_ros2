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
// SHAKEDOWN suite, ROGUE WIRE ACTORS group. This file belongs to the packml_ros_shakedown
// binary (see CMakeLists.txt), deliberately NOT the polished packml_ros_tests binary: these
// are adversarial probes hunting for crashes, deadlocks and trust violations, and a probe
// that kills the process must not destabilize the main suite. Every probe here is written
// against the INTENDED contract as read out of the source (locations cited per test); a red
// result is a triage input, not something to weaken. Probes may be promoted into the main
// suite or dropped entirely once triaged.
//
// The attacker modelled by this file is a node that is NOT part of this PackML system but can
// reach its topics: it publishes on the manager's own status topic, impersonates a required
// Equipment Module on that module's heartbeat topic, and puts values on the wire that no
// well-behaved participant would ever emit. Nothing here uses a privileged back door -- every
// message is one any ROS node on the same graph can publish, which is exactly why the system's
// reaction to it is worth pinning down.
//
// House style is inherited from test_monkey_scenarios.cpp / test_monkey_health.cpp: helper
// classes and functions live in this file's own anonymous namespace (duplicated per file
// rather than shared through a header), each test builds a standalone manager (+ EM and/or
// raw publisher) with unique node names via a Rig struct and begin_setup()/finish_setup(),
// and any send loop keeps a small non-zero gap between sends (a true zero-delay loop overruns
// the service's QoS queue before the single-threaded executor can drain any of it -- an
// already-documented lesson from MonkeyCommandFlood_ManagerSurvivesAndStaysResponsive).
//
// TEARDOWN RULE (this cost a real SIGABRT, confirmed via gdb backtrace in a sibling file):
// an Equipment Module object declared after the Rig destructs BEFORE the Rig's own
// em_spin member stops spinning, so an in-flight fan-out callback can call into an object
// whose vtable is already gone -- "pure virtual method called". Every test below therefore
// ends with rig.em_spin.reset() after its last assertion.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_ros/ros_names.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/msg/node_heartbeat.hpp"
#include "packml_msgs/msg/status.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;
using packml_ros_test::wait_for_state;

/// Fire-and-don't-wait: send a command and return immediately with the pending future.
/// Used by the heartbeat-identity-theft probe, which fires probe commands INTO the chaos
/// window rather than waiting for each to settle first -- "did every command get answered"
/// is checked afterwards, from the collected futures.
std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr> send_state_change_async(
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client,
  int8_t command)
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = command;
  return client->async_send_request(req).future.share();
}

/// Blocks until `publisher` sees at least one matched subscription, or `timeout` elapses.
/// Returns whether it matched. Discovery is asynchronous, so publishing before this returns
/// silently drops the message -- which for a rogue publisher would look exactly like the
/// system correctly ignoring it, i.e. a false pass.
template<typename PublisherT>
bool wait_for_subscriber(
  const PublisherT & publisher, std::chrono::milliseconds timeout = 2s)
{
  packml_ros_test::wait_until(
    [&] {return publisher->get_subscription_count() > 0;}, timeout, 10ms);
  return publisher->get_subscription_count() > 0;
}

/// Same wait, from the other side: a subscription that has not yet matched its publisher
/// would silently miss the alarms this file wants to inspect.
template<typename SubscriptionT>
bool wait_for_publisher(
  const SubscriptionT & subscription, std::chrono::milliseconds timeout = 2s)
{
  packml_ros_test::wait_until(
    [&] {return subscription->get_publisher_count() > 0;}, timeout, 10ms);
  return subscription->get_publisher_count() > 0;
}

/// Plain, non-deferring EM -- completes every coordinated state instantly. Duplicated from
/// test_monkey_scenarios.cpp (same shape/semantics); used where the probe's variable is the
/// rogue traffic, not the EM's own behavior.
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

/// Non-deferring EM that counts, per state, how many times its own on_state_trans_req() hook
/// was invoked. The count is the observable that distinguishes "the EM really did the work
/// this cycle asked for" from "the EM's TransitionGuard thought it was already in that state
/// and short-circuited the hook" -- the exact failure the status-spoofing probe hunts for,
/// which is otherwise invisible from the outside (the machine reaches the target state either
/// way, because the goal succeeds either way).
class StateCountingEquipmentModule : public PackmlNodeInterface
{
public:
  /// `deferred_state`/`work_duration`: optionally defer ONE state with real background work
  /// (MonkeyStaleWorkEquipmentModule shape, reporting when the work finishes). The spoofing
  /// probe needs this: with a purely instant EM the whole double cycle finishes in ~80ms and
  /// the spoofer physically cannot land enough garbage inside the window that matters --
  /// confirmed empirically on the first shakedown run (10 messages, all outside the
  /// RESETTING window). Deferring RESETTING holds each cycle's vulnerable window open for
  /// real, which is also exactly when a spoofed status is most dangerous to the guard.
  explicit StateCountingEquipmentModule(
    rclcpp::Node::SharedPtr node,
    std::optional<packml_sm::State> deferred_state = std::nullopt,
    std::chrono::milliseconds work_duration = std::chrono::milliseconds(150))
  : deferred_state_(deferred_state), work_duration_(work_duration)
  {
    init(node);
  }

  int count(packml_sm::State state) const
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
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    std::thread([this, completion]() {
        std::this_thread::sleep_for(work_duration_);
        completion.report(true);
      }).detach();
  }
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
  bool defers_completion(packml_sm::State state) override
  {
    return deferred_state_ && state == *deferred_state_;
  }

private:
  mutable std::mutex counts_mutex_;
  std::map<packml_sm::State, int> counts_;
  std::optional<packml_sm::State> deferred_state_;
  std::chrono::milliseconds work_duration_;
};

/// Build a NodeHeartbeat for direct publication by a plain rclcpp::Publisher, bypassing
/// PackmlNodeInterface's heartbeat plumbing entirely -- the only way to control node_name,
/// sequence_number and the embedded health independently of any real node's HeartbeatState.
/// Mirrors test_monkey_health.cpp's own make_heartbeat, extended with error_code/message
/// because the garbage-action probe asserts on how those propagate into an Alarm.
packml_msgs::msg::NodeHeartbeat make_heartbeat(
  const std::string & node_name, uint64_t seq, uint32_t interval_ms,
  int32_t status = packml_msgs::msg::NodeHealth::HEALTHY,
  int32_t action = packml_msgs::msg::NodeHealth::NONE,
  int32_t error_code = 0,
  const std::string & message = "")
{
  packml_msgs::msg::NodeHeartbeat hb;
  hb.node_name = node_name;
  hb.sequence_number = seq;
  hb.heartbeat_interval_ms = interval_ms;
  hb.health.status = status;
  hb.health.action = action;
  hb.health.error_code = error_code;
  hb.health.message = message;
  return hb;
}

}  // namespace

class WireMonkeyTest : public ::testing::Test
{
protected:
  /// Every probe builds its own standalone manager (plus, where needed, one real EM node and
  /// one rogue publisher node) with unique node names, rather than sharing fixture state --
  /// see test_monkey_scenarios.cpp's own Rig for the rationale. em_node/em_spin stay null for
  /// probes whose "Equipment Module" is only a raw publisher with no real node behind it.
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

  /// Constructs the manager and gets it spinning, and (when `em_node_name` is non-empty) the
  /// node a real EM will be built on -- but never the EM itself: callers need different EM
  /// subclasses, so they construct it on rig.em_node and then call finish_setup().
  ///
  /// The two node lists are deliberately independent: `coordinated_nodes` (node_names) drives
  /// the state-transition ACTION fan-out, `required_nodes` drives the health monitor and its
  /// RESET-from-STOPPED gate. A probe that wants a coordinated EM without a health gate in the
  /// way passes only the former; one that wants a health-monitored heartbeat source without
  /// any fan-out dependency passes only the latter.
  Rig begin_setup(
    const std::string & mgr_prefix,
    const std::vector<std::string> & coordinated_nodes,
    const std::vector<std::string> & required_nodes,
    const std::string & em_node_name = "",
    int em_heartbeat_interval_ms = -1,
    int state_complete_timeout_ms = 5000)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter(packml_ros::kParamNodeNames, coordinated_nodes),
        rclcpp::Parameter(packml_ros::kParamRequiredNodes, required_nodes),
        rclcpp::Parameter(packml_ros::kParamStateCompleteTimeoutMs, state_complete_timeout_ms),
      }));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    if (!em_node_name.empty()) {
      rig.em_node = rclcpp::Node::make_shared(em_node_name);
      if (em_heartbeat_interval_ms > 0) {
        // Declared before the EM is constructed on this node, since PackmlNodeInterface::init()
        // reads it once to size its heartbeat timer.
        rig.em_node->declare_parameter(
          packml_ros::kParamHeartbeatIntervalMs, em_heartbeat_interval_ms);
      }
    }
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
// PROBE 1 -- identity/trust attack on the STATUS topic.
//
// What is attacked: packml_status (packml_ros::kStatusTopic -- a bare, deliberately
// un-namespaced global topic name, see ros_names.hpp) is a plain ROS topic with no notion of
// who is allowed to publish it. A rogue node offering the same QoS the manager offers
// (QoS(1).transient_local().reliable(), see where status_pub_ is created in
// packml_interface.hpp's init()) is indistinguishable, at the wire level, from the real
// manager.
//
// Why it could break: every Equipment Module subscribes to that topic (status_sub_ in
// PackmlNodeInterface::init()) and feeds whatever arrives straight into
// TransitionGuard::on_status_update(), which unconditionally adopts it as current_state_
// (transition_guard.cpp). current_state_ is exactly what TransitionGuard::request_state()
// consults to take its already_there shortcut -- and that shortcut returns success to the
// manager WITHOUT calling the node's own on_state_trans_req()/defers_completion() hooks
// (see begin_transition() in packml_interface.hpp). note_goal_admitted() protects against a
// REAL status echo racing ahead of its own goal, but its protection is keyed on
// current_state_ having changed to the target only AFTER admission; a spoofed status that
// poisons current_state_ BEFORE the goal is admitted looks to the guard exactly like a
// genuinely pre-existing already-there. So a rogue publisher spraying the coordinated state
// names could, in principle, make an EM report "already done" for work it never started --
// silently skipping a homing sequence, a purge, a safe-position move.
//
// What correct looks like: the operator's real cycle is unaffected. RESET completes to IDLE,
// and the EM's own RESETTING hook ran exactly once for that cycle (not zero times, which is
// the spoofed-past failure). A second, full ABORT -> CLEAR -> RESET cycle behaves the same, so
// this is not a one-shot survival. The manager answers every command throughout. The spoof
// runs continuously across all of it.
//
// Health is deliberately out of the picture (required_nodes empty -> the RESET-from-STOPPED
// health gate is trivially open), so nothing but the status spoofing can influence the result.
//
// An OPEN defect, and PROBE 1b below is its control: this test fails while that control --
// identical double cycle, identical deferring EM, NO rogue publisher -- passes, which is what
// makes the rogue causal rather than incidental. The mechanism: on the second cycle the manager
// logs "Node already in state: RESETTING" and the EM's on_state_trans_req() is never called,
// leaving the hook count at 1 across two real RESET cycles. A spoofed status adopted into
// current_state_ BEFORE the goal is admitted
// is indistinguishable, to request_state(), from a genuinely pre-existing already-there --
// note_goal_admitted()'s snapshot cannot separate them, because at admission time the poisoned
// value is already the "before" picture. Impact: any node able to publish on the bare, global
// packml_status topic can make Equipment Modules report success for coordinated work they never
// performed (a homing sequence, a purge, a move to safe position). Kept red to mark that the
// defect is open. Candidate directions (undecided): authenticate/namespace the status topic so
// only the owning manager can write it; or stop trusting a passive broadcast for already_there and
// require the EM's own local adoption (mark_state_locally_reached()) as the only source of
// current_state_ for shortcut purposes.
// CHARACTERIZATION TEST -- the final assertion below asserts the CURRENT, DEFECTIVE behaviour so
// this suite can gate. A permanently-red test cannot distinguish a new regression from the known
// failure. Passing here does NOT mean the work-skip is fixed; it means it still happens. Note the
// first cycle's assertion is left as a genuine correct-behaviour check, because it holds.
TEST_F(WireMonkeyTest, KnownDefect_StatusSpoofingSkipsRealWorkViaAlreadyThere)
{
  const auto em_name = packml_ros_test::unique_node_name("spoof_victim_em");
  auto rig = begin_setup("shakedown_status_spoof", {em_name}, {}, em_name);
  // RESETTING deferred with 150ms of real background work per cycle -- see the class comment:
  // this holds each cycle's vulnerable window open so the continuous spoof genuinely overlaps
  // the in-flight goal, instead of the whole cycle outrunning the spoofer.
  auto em = std::make_shared<StateCountingEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "machine never reached STOPPED after setup -- probe cannot start from a known state";

  // --- The rogue actor: a node that is not part of this system at all, publishing the
  // manager's own status topic with the manager's own QoS.
  auto rogue_node = rclcpp::Node::make_shared(
    packml_ros_test::unique_node_name("rogue_status_pub"));
  auto rogue_pub = rogue_node->create_publisher<packml_msgs::msg::Status>(
    packml_ros::kStatusTopic, rclcpp::QoS(1).transient_local().reliable());
  ASSERT_TRUE(wait_for_subscriber(rogue_pub))
    << "rogue status publisher never matched the Equipment Module's status subscription -- "
       "the spoof would not have been delivered at all";

  // States the machine is NOT in, plus raw values that are not states at all. RESETTING (15) is
  // interleaved between every other value ON PURPOSE, and that weighting is load-bearing rather
  // than cosmetic.
  //
  // The shortcut fires only when a spoofed RESETTING is the LAST status the Equipment Module
  // processed before its goal is admitted. As one of eight rotating values RESETTING occupies
  // ~1/8 of the timeline, which makes whether the spoof "takes" a coin flip for the whole run
  // rather than an independent draw per cycle -- so adding cycles does not converge. Interleaving
  // RESETTING to ~50% of samples keeps the poison present at the critical instant.
  //
  // This is not a weaker attack -- it is a more faithful one. A duplicate manager stuck
  // reporting RESETTING is exactly the realistic shape of this failure; the rotating garbage
  // around it still exercises the "malformed values do not crash anything" half.
  // mode.val stays 0 (the real current mode) so this probe varies exactly one thing: state.
  static constexpr int8_t kSpoofedStates[] = {
    15,   // RESETTING -- the dangerous one; interleaved throughout
    6,    // EXECUTE
    15,
    9,    // ABORTED
    15,
    99,   // not a State at all
    15,
    11,   // HELD
    15,
    4,    // IDLE -- also a state the real cycle moves through
    15,
    17,   // COMPLETE
    15,
    -5,   // negative garbage
  };

  std::atomic<bool> spoofing{true};
  std::atomic<int> spoofed_count{0};
  std::thread spoofer([&]() {
      while (spoofing.load()) {
        for (const auto raw_state : kSpoofedStates) {
          if (!spoofing.load()) {
            break;
          }
          packml_msgs::msg::Status msg;
          msg.state.val = raw_state;
          msg.mode.val = 0;
          msg.error = 99;        // garbage payload, ignored by every real consumer today
          msg.sub_error = -1;
          rogue_pub->publish(msg);
          spoofed_count.fetch_add(1);
          // Rapid fire, but not zero-delay -- see this file's header comment.
          std::this_thread::sleep_for(5ms);
        }
      }
    });

  // --- The operator's real RESET, driven straight through the spoof: STOPPED -> RESETTING
  // -> IDLE.
  auto reset_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr) << "manager did not answer RESET while status was being spoofed";
  ASSERT_TRUE(reset_resp->success) << "RESET rejected during status spoofing: "
    << reset_resp->message;
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 5s))
    << "RESET cycle did not complete to IDLE while a rogue node spoofed the status topic; "
       "last seen state: " << static_cast<int>(rig.sm_node->getCurrentState());
  // NOT asserted per-cycle. With RESETTING interleaved to ~50% of spoofed samples (see the
  // weighting note above) ANY cycle can be the skipped one, including the first -- an earlier
  // revision asserted "exactly once" here and started failing with 0 the moment the spoof became
  // reliable, which is the spoof succeeding, not a new problem. The aggregate assertion at the
  // end of the cycle loop is the characterization; only the machine reaching IDLE matters here.
  EXPECT_LE(em->count(packml_sm::State::RESETTING), 1)
    << "the EM's RESETTING hook ran " << em->count(packml_sm::State::RESETTING)
    << " times for a single cycle -- more than one goal per cycle would mean a fan-out or retry "
       "problem, unrelated to the spoof this test characterizes";

  // --- A second, full recovery cycle under the same continuous spoof: ABORT -> CLEAR ->
  // RESET. Surviving one cycle could be luck of the interleaving; the hook must count up
  // again on a state name the machine has already visited once (which is also when a stale
  // current_state_ is most plausible).
  auto abort_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr) << "manager did not answer ABORT while status was being spoofed";
  EXPECT_TRUE(abort_resp->success) << "ABORT rejected during status spoofing: "
    << abort_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 5s))
    << "machine did not reach ABORTED during status spoofing; last seen state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  auto clear_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr) << "manager did not answer CLEAR while status was being spoofed";
  EXPECT_TRUE(clear_resp->success) << "CLEAR rejected during status spoofing: "
    << clear_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 5s))
    << "machine did not return to STOPPED after CLEAR during status spoofing; last seen state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  auto reset2_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset2_resp, nullptr)
    << "manager did not answer the second RESET while status was being spoofed";
  EXPECT_TRUE(reset2_resp->success) << "second RESET rejected during status spoofing: "
    << reset2_resp->message;
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 5s))
    << "second RESET cycle did not complete to IDLE during status spoofing; last seen state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  // --- Further recovery cycles, because WINNING the race is itself intermittent.
  //
  // Whether a spoofed sample lands in the window between goal admission and request_state() is a
  // genuine race, so a single trial wins it only about two times in three and asserting on one
  // would flake. Running several cycles and asserting "at least one was skipped" converts that
  // into a stable multi-trial assertion without weakening what is being characterized -- one
  // silent skip is the defect, regardless of which cycle it lands on.
  static constexpr int kTotalCycles = 5;
  for (int cycle = 3; cycle <= kTotalCycles; ++cycle) {
    ASSERT_NE(send_state_change(
        rig.state_client, packml_msgs::srv::StateChange::Request::ABORT), nullptr)
      << "cycle " << cycle << ": manager did not answer ABORT";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 5s))
      << "cycle " << cycle << ": machine did not reach ABORTED";
    ASSERT_NE(send_state_change(
        rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR), nullptr)
      << "cycle " << cycle << ": manager did not answer CLEAR";
    ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 5s))
      << "cycle " << cycle << ": machine did not return to STOPPED";
    ASSERT_NE(send_state_change(
        rig.state_client, packml_msgs::srv::StateChange::Request::RESET), nullptr)
      << "cycle " << cycle << ": manager did not answer RESET";
    EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 5s))
      << "cycle " << cycle << ": did not reach IDLE; last seen state: "
      << static_cast<int>(rig.sm_node->getCurrentState());
  }

  // CHARACTERIZED DEFECT. Correct behaviour is that the hook count EQUALS the cycle count: every
  // real RESET must run the EM's own RESETTING work. Under the spoof at least one cycle is
  // skipped via the already_there shortcut, which is the whole finding. PROBE 1b (the no-rogue
  // control) is what proves the rogue causes this rather than ordinary cycling -- it must stay
  // green, and if BOTH change together, suspect the test rig rather than the product.
  EXPECT_LT(em->count(packml_sm::State::RESETTING), kTotalCycles)
    << "All " << kTotalCycles << " RESET cycles ran the EM's own RESETTING hook despite the "
       "status spoof -- the silent work-skip appears to be FIXED. That is the correct behaviour: "
       "rewrite this as EXPECT_EQ(em->count(packml_sm::State::RESETTING), kTotalCycles) and drop "
       "the KnownDefect_ prefix. Likely cause: already_there now requires the node's own local "
       "completion rather than trusting a passive broadcast.";

  spoofing.store(false);
  spoofer.join();
  EXPECT_GE(spoofed_count.load(), 20)
    << "the spoofer only managed " << spoofed_count.load() << " messages -- too few for this "
       "probe to mean anything about spoofing resistance";

  // The manager must still answer normally once the spoof stops.
  auto stop_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_resp, nullptr) << "manager stopped answering after the status-spoofing run";

  // Let the last cycle's detached work thread (holds a raw `this`) fully exit before teardown.
  std::this_thread::sleep_for(200ms);

  // See this file's header comment: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// PROBE 1b -- THE CONTROL for probe 1. Byte-for-byte the same double cycle, the same deferring
// counting Equipment Module, the same timings -- with NO rogue publisher anywhere.
//
// Why this exists: probe 1 came back red with "Node already in state: RESETTING" on the second
// cycle, meaning the EM's goal took TransitionGuard's already_there shortcut and skipped its own
// work. That is exactly the failure probe 1 predicted, but a red probe alone cannot tell WHO
// poisoned current_state_: the rogue publisher (a security finding -- an untrusted node on the
// bus can make Equipment Modules skip real work) or the manager's own status echo racing its
// paired goal (a plain product bug in the already_there/note_goal_admitted() logic under
// ABORT -> CLEAR -> RESET cycling, needing no attacker at all -- a far more serious finding,
// since it would mean ordinary recovery cycles can silently skip an EM's work in production).
//
// This control decides it. It PASSES (count == 2) while probe 1 fails on the same build, which
// is what establishes that the rogue publisher is causal and that ordinary recovery cycling is
// sound. Keep this test green alongside probe 1's red -- together they are the proof, and if a
// change makes THIS one fail, the already_there logic has broken without any attacker.
TEST_F(WireMonkeyTest, StatusSpoofingControl_SameDoubleCycleWithoutAnyRoguePublisher)
{
  const auto em_name = packml_ros_test::unique_node_name("spoof_control_em");
  auto rig = begin_setup("shakedown_status_control", {em_name}, {}, em_name);
  auto em = std::make_shared<StateCountingEquipmentModule>(
    rig.em_node, packml_sm::State::RESETTING);
  finish_setup(rig);

  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 2s))
    << "machine never reached STOPPED after setup";

  // --- Cycle 1: RESET -> IDLE
  auto reset_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success) << "RESET rejected: " << reset_resp->message;
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 5s))
    << "first RESET cycle did not reach IDLE; last seen state: "
    << static_cast<int>(rig.sm_node->getCurrentState());
  EXPECT_EQ(em->count(packml_sm::State::RESETTING), 1)
    << "first cycle's RESETTING hook did not run exactly once";

  // --- Recovery: ABORT -> CLEAR -> STOPPED
  auto abort_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(abort_resp, nullptr);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::ABORTED, 5s))
    << "machine did not reach ABORTED";
  auto clear_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR);
  ASSERT_NE(clear_resp, nullptr);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 5s))
    << "machine did not return to STOPPED after CLEAR";

  // --- Cycle 2: the one probe 1 saw skipped.
  auto reset2_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset2_resp, nullptr);
  EXPECT_TRUE(reset2_resp->success) << "second RESET rejected: " << reset2_resp->message;
  EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 5s))
    << "second RESET cycle did not reach IDLE; last seen state: "
    << static_cast<int>(rig.sm_node->getCurrentState());
  EXPECT_EQ(em->count(packml_sm::State::RESETTING), 2)
    << "NO ROGUE PUBLISHER INVOLVED: the EM's RESETTING hook ran "
    << em->count(packml_sm::State::RESETTING) << " time(s) across two real RESET cycles instead "
       "of twice. If this fails alongside probe 1, the already_there shortcut skips an EM's real "
       "work during ordinary ABORT -> CLEAR -> RESET recovery with no attacker at all -- the "
       "manager's own status echo is the poisoner, and probe 1's rogue is a red herring.";

  std::this_thread::sleep_for(200ms);
  rig.em_spin.reset();
}

// ============================================================================
// PROBE 2 -- identity theft on a required Equipment Module's HEARTBEAT topic.
//
// What is attacked: /<node>/heartbeat (packml_ros::kHeartbeatTopic), subscribed by the manager
// once per required node (see the heartbeat_subs_ loop in packml_interface.hpp's init()). The
// subscription is per-TOPIC; nothing ties a heartbeat to the process that should have sent it,
// and NodeHeartbeat::node_name is self-declared. A rogue publisher on that topic is, to
// HealthMonitor, the same node.
//
// Why it could break: HealthMonitor::on_heartbeat() (health_monitor.hpp) disambiguates a
// backward/equal sequence number by OBSERVED ABSENCE -- a counter reset while the node was
// absent (timed out, or silent for at least its own timeout) is classified RESTART and
// re-baselined; anything else is DROPPED_STALE and ignored. That logic is written for exactly
// this threat ("a second 'zombie' publisher on the same node_name"), and its own comment says
// so. Two publishers interleaving on one topic drive it against the ambiguous middle: the real
// stream climbs while the rogue keeps re-asserting low values, and any rogue beat that IS
// accepted also refreshes the node's last_stamp/timed_out state as a side effect (a dropped one
// returns before that, leaving liveness state untouched). A RESTART classification fires ABORT
// (the fire_packml_action(ABORT) call in that same subscription callback) -- that is ALLOWED
// here, not a bug: an unannounced restart of a required node genuinely routes through
// CLEAR/RESET recovery. What must not happen is a crash, a hang, an unanswered command, or a
// machine that cannot be recovered afterwards.
//
// What correct looks like: nothing hangs (every probe command fired into the chaos gets some
// response), and once the rogue stops and the real EM's clean heartbeats have continued for
// comfortably longer than its effective timeout (100ms interval x heartbeat_timeout_factor 3.0
// = 300ms), the machine is recoverable from wherever it landed: CLEAR (tolerated if rejected,
// since it is only legal from ABORTED) followed by a RESET that is accepted inside a bounded
// window. The window is poll-retried for ~3s on purpose: the manager's health tick runs every
// 200ms, and the gate's own staleness check (gate_block_reason()) is a fresh real-time
// comparison against the LAST accepted heartbeat, so recovery needs a couple of fresh beats to
// be visible, not just one. Which intermediate states occurred is deliberately not asserted --
// that is genuinely interleaving-dependent.
TEST_F(WireMonkeyTest, HeartbeatIdentityTheft_SecondPublisherSameNodeStaysRecoverable)
{
  const auto em_name = packml_ros_test::unique_node_name("theft_victim_em");
  // Required (health-monitored) but not coordinated: this probe is about heartbeat identity,
  // so no fan-out dependency should be able to influence whether recovery is possible.
  // interval 100ms -> effective timeout 300ms, above the manager's 200ms health tick, so a
  // continuously-beating real EM never falsely times out.
  static constexpr uint32_t kIntervalMs = 100;
  auto rig = begin_setup(
    "shakedown_hb_theft", {}, {em_name}, em_name, static_cast<int>(kIntervalMs));
  auto em = std::make_shared<PlainEquipmentModule>(rig.em_node);
  finish_setup(rig);

  // Let a few genuine, climbing heartbeats land first, so the rogue's low sequence numbers are
  // genuinely backward relative to an established baseline rather than the first thing seen.
  std::this_thread::sleep_for(400ms);

  // --- The impostor: same topic, same claimed node_name, same advertised interval.
  auto rogue_node = rclcpp::Node::make_shared(
    packml_ros_test::unique_node_name("rogue_heartbeat_pub"));
  auto rogue_pub = rogue_node->create_publisher<packml_msgs::msg::NodeHeartbeat>(
    "/" + em_name + "/" + std::string(packml_ros::kHeartbeatTopic), rclcpp::SensorDataQoS());
  ASSERT_TRUE(wait_for_subscriber(rogue_pub))
    << "rogue heartbeat publisher never matched the manager's heartbeat subscription -- "
       "the impersonation would not have been delivered at all";

  // Low, conflicting, repeatedly re-asserted sequence numbers while the real EM climbs past
  // them. Some of these will look like duplicates, some like counter resets; whether any given
  // one is read as RESTART depends on whether the real stream happened to leave a gap at that
  // instant, which is exactly the ambiguity being probed.
  static constexpr uint64_t kRogueSeq[] = {1, 2, 1, 3, 2, 1, 4, 1};
  std::vector<std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr>> probes;
  for (int round = 0; round < 4; ++round) {
    for (const auto seq : kRogueSeq) {
      rogue_pub->publish(make_heartbeat(em_name, seq, kIntervalMs));
      std::this_thread::sleep_for(25ms);
    }
    // Probe commands fired INTO the chaos, not after it: CLEAR and STOP are both harmless
    // from a health-recovery standpoint (neither depends on the health gate), so their only
    // job here is to prove the manager is still answering while it is being lied to.
    probes.push_back(send_state_change_async(
        rig.state_client,
        round % 2 == 0
        ? packml_msgs::srv::StateChange::Request::CLEAR
        : packml_msgs::srv::StateChange::Request::STOP));
    std::this_thread::sleep_for(5ms);
  }

  for (size_t i = 0; i < probes.size(); ++i) {
    ASSERT_EQ(probes[i].wait_for(5s), std::future_status::ready)
      << "probe command #" << i << " never got a response during heartbeat identity theft -- "
         "possible hang";
  }

  // --- The impostor goes away; the real EM keeps beating cleanly. 1500ms is five times its
  // 300ms effective timeout, so "the real node is alive and healthy" is unambiguous by the
  // time recovery is attempted.
  std::this_thread::sleep_for(1500ms);

  bool reset_accepted = false;
  std::string last_reset_message;
  const auto recovery_deadline = std::chrono::steady_clock::now() + 3s;
  while (!reset_accepted && std::chrono::steady_clock::now() < recovery_deadline) {
    // CLEAR unconditionally, tolerating rejection: it is only legal from ABORTED, and this
    // probe deliberately does not assume where the interleaving left the machine.
    send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::CLEAR, 2s);
    std::this_thread::sleep_for(100ms);
    auto reset_resp = send_state_change(
      rig.state_client, packml_msgs::srv::StateChange::Request::RESET, 2s);
    if (reset_resp && reset_resp->success) {
      reset_accepted = true;
      break;
    }
    if (reset_resp) {
      last_reset_message = reset_resp->message;
    } else {
      last_reset_message = "no response from the manager at all";
    }
    std::this_thread::sleep_for(200ms);
  }

  EXPECT_TRUE(reset_accepted)
    << "machine never became RESET-able within 3s of clean heartbeats resuming after the "
       "heartbeat identity theft (state: " << static_cast<int>(rig.sm_node->getCurrentState())
    << ", last RESET rejection: " << last_reset_message << ")";
  if (reset_accepted) {
    EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 3s))
      << "RESET was accepted but the machine never reached IDLE; last seen state: "
      << static_cast<int>(rig.sm_node->getCurrentState());
  }

  // See this file's header comment: stop em_spin before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// PROBE 3 -- garbage payload: a health action value that is not in the protocol.
//
// What is attacked: NodeHealth::action, as carried inside a NodeHeartbeat for a required node.
// The defined constants are NONE/WARN/HOLD/SUSPEND/ABORT (0..4, NodeHealth.msg); this probe
// puts 42 on the wire, with status=ERROR, error_code=7 and a message, from a rogue publisher
// speaking for a required node that has no real process behind it at all.
//
// Why it could break: the value flows through HealthMonitor::process_action_transition()
// (health_monitor.hpp), which compares actions by MAGNITUDE (escalation/de-escalation) with no
// validation that the value is a defined constant -- 42 is "greater than ABORT", so it is
// treated as a new, escalating event and handed to the fire_action_ callback. It also lands in
// node.last_action, where gate_block_reason()'s `last_action > WARN` check closes the health
// gate. If the manager's side of that callback mapped unknown values loosely (a default that
// picked some command, or a cast into TransitionCmd), a rogue could drive the machine with a
// value nobody defined.
//
// INTENDED CONTRACT, as found in the source: fire_packml_action() in packml_interface.hpp
// switches on the action and handles ONLY HOLD/SUSPEND/ABORT; its default branch logs at DEBUG
// ("fire_packml_action: ignoring action %d") and RETURNS WITHOUT TOUCHING THE STATE MACHINE.
// So the intended behavior for an unknown action is: ignore it as a command. The assertion
// below is written to that contract -- the machine's state must be unchanged across the garbage
// window. Separately, compute_alarm() in health_monitor.hpp treats any non-NONE action as
// alarm-worthy and copies msg.health.message/error_code straight into the AlarmEvent, and
// on_alarm_event() publishes it (unenriched, as no error catalog is loaded here) -- so an alarm
// IS expected, and it must carry the node's own message and error_code through rather than
// being swallowed or rewritten. (Observation for triage, not asserted: Alarm::severity is a
// uint8 copy of the raw action, so an out-of-protocol 42 reaches alarm consumers verbatim,
// outside the WARN..ABORT scale Alarm.msg documents.)
//
// What correct looks like: no crash; the machine does not spontaneously do anything; the
// manager still answers a STOP probe; the alarm carries the node's message; and a subsequent
// HEALTHY beat restores RESET-ability (process_action_transition()'s rule 1 clears last_action
// on status==HEALTHY, reopening the gate).
//
// One deliberate deviation from a single garbage beat: the garbage health is REPEATED on every
// beat for the duration of the observation window, and clean beats resume afterwards. A single
// beat followed by silence would trip the node's own liveness timeout mid-window and fire a
// real, correct ABORT -- which would make "did the machine stay put" unanswerable. Repeating is
// also harmless to the mechanism under test: rule 2 (same action already active) suppresses
// re-firing, so the unknown action is still evaluated exactly once as a new event.
TEST_F(WireMonkeyTest, GarbageHealthAction_UnknownActionValueContainedSafely)
{
  // No real node behind this name -- a rogue publisher is the only thing that ever speaks for
  // it, which is itself part of the attack (nothing in the manager checks that a heartbeat's
  // claimed node exists).
  const auto phantom_name = packml_ros_test::unique_node_name("garbage_action_em");
  auto rig = begin_setup("shakedown_garbage_action", {}, {phantom_name});
  finish_setup(rig);

  // Advertised 1000ms interval -> 3000ms effective timeout (factor 3.0), well under the
  // startup-grace-derived clamp (30000/3 = 10000ms) so nothing is clamped. Beating every
  // 200ms keeps the node comfortably live while leaving room for sub-second observation
  // windows that a 300ms timeout would have made impossible.
  static constexpr uint32_t kAdvertisedIntervalMs = 1000;
  static constexpr auto kBeatPeriod = 200ms;
  static constexpr int32_t kUndefinedAction = 42;   // not any NodeHealth action constant
  static constexpr int32_t kGarbageErrorCode = 7;
  static const std::string kGarbageMessage = "garbage action";

  auto rogue_node = rclcpp::Node::make_shared(
    packml_ros_test::unique_node_name("garbage_action_pub"));
  auto rogue_pub = rogue_node->create_publisher<packml_msgs::msg::NodeHeartbeat>(
    "/" + phantom_name + "/" + std::string(packml_ros::kHeartbeatTopic),
    rclcpp::SensorDataQoS());
  ASSERT_TRUE(wait_for_subscriber(rogue_pub))
    << "rogue heartbeat publisher never matched the manager's heartbeat subscription";

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = rig.mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(50).reliable().transient_local(),
    [&alarms, &alarms_mutex, &phantom_name](packml_msgs::msg::Alarm::SharedPtr msg) {
      if (msg->node_name != phantom_name) {
        return;
      }
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  ASSERT_TRUE(wait_for_publisher(alarm_sub)) << "Alarm subscription never matched";

  std::atomic<bool> beating{true};
  std::atomic<bool> garbage{false};
  std::atomic<uint64_t> seq{0};
  std::thread beater([&]() {
      while (beating.load()) {
        const uint64_t next_seq = seq.fetch_add(1) + 1;
        if (garbage.load()) {
          rogue_pub->publish(make_heartbeat(
              phantom_name, next_seq, kAdvertisedIntervalMs,
              packml_msgs::msg::NodeHealth::ERROR, kUndefinedAction, kGarbageErrorCode,
              kGarbageMessage));
        } else {
          rogue_pub->publish(make_heartbeat(phantom_name, next_seq, kAdvertisedIntervalMs));
        }
        std::this_thread::sleep_for(kBeatPeriod);
      }
    });

  // --- Phase A: healthy beats only. The gate must open, proving the rogue publisher is
  // accepted as this required node's health source in the first place (otherwise the rest of
  // the probe would pass for the wrong reason).
  std::this_thread::sleep_for(700ms);
  auto reset_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr) << "manager did not answer the initial RESET";
  ASSERT_TRUE(reset_resp->success)
    << "health gate never opened on the rogue publisher's HEALTHY heartbeats: "
    << reset_resp->message;
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 3s))
    << "machine did not reach IDLE after the initial RESET";
  auto stop_back = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(stop_back, nullptr) << "manager did not answer STOP";
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 3s))
    << "machine did not return to STOPPED before the garbage-action window";

  // --- Phase B: the undefined action. Nothing else drives the machine for this window.
  const auto baseline_state = rig.sm_node->getCurrentState();
  garbage.store(true);
  std::this_thread::sleep_for(600ms);

  EXPECT_EQ(rig.sm_node->getCurrentState(), baseline_state)
    << "an undefined NodeHealth action (" << kUndefinedAction << ") moved the machine from "
    << static_cast<int>(baseline_state) << " to "
    << static_cast<int>(rig.sm_node->getCurrentState())
    << " -- fire_packml_action()'s default branch is supposed to ignore an action it does not "
       "recognize (see packml_interface.hpp), so an unknown value must never act as a command";

  auto probe_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_NE(probe_resp, nullptr)
    << "manager stopped answering after an undefined health action was injected";

  {
    std::lock_guard<std::mutex> lk(alarms_mutex);
    bool found_raise = false;
    for (const auto & alarm : alarms) {
      if (!alarm.trigger || alarm.error_code != static_cast<uint32_t>(kGarbageErrorCode)) {
        continue;
      }
      found_raise = true;
      EXPECT_NE(alarm.message.find(kGarbageMessage), std::string::npos)
        << "the alarm raised for the undefined action carried message '" << alarm.message
        << "' instead of the node's own '" << kGarbageMessage
        << "' -- compute_alarm()/on_alarm_event() are supposed to pass the node's message "
           "through verbatim when no error catalog is loaded";
      break;
    }
    EXPECT_TRUE(found_raise)
      << "no alarm was raised for the ERROR/error_code=" << kGarbageErrorCode
      << " heartbeat (" << alarms.size() << " alarm(s) seen for this node) -- an actionable "
         "health condition must still be reported even when its action value is unknown";
  }

  // --- Phase C: clean beats resume. The unknown action must not have left the node
  // permanently unhealthy: process_action_transition()'s rule 1 clears last_action on a
  // HEALTHY status, which reopens the gate.
  garbage.store(false);
  std::this_thread::sleep_for(800ms);
  ASSERT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::STOPPED, 3s))
    << "machine was not in STOPPED before the recovery RESET; last seen state: "
    << static_cast<int>(rig.sm_node->getCurrentState());

  auto recovery_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(recovery_resp, nullptr) << "manager did not answer the recovery RESET";
  EXPECT_TRUE(recovery_resp->success)
    << "RESET stayed blocked after healthy heartbeats resumed following an undefined health "
       "action -- the garbage value left the node permanently unhealthy: "
    << recovery_resp->message;
  if (recovery_resp->success) {
    EXPECT_TRUE(wait_for_state(rig.sm_node, packml_sm::State::IDLE, 3s))
      << "recovery RESET was accepted but the machine never reached IDLE; last seen state: "
      << static_cast<int>(rig.sm_node->getCurrentState());
  }

  beating.store(false);
  beater.join();

  // No real EM in this probe (em_spin is null), but the convention is kept unconditionally --
  // see this file's header comment.
  rig.em_spin.reset();
}
