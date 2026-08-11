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
// Fuzzing-harness "monkey" scenario: unlike test_monkey_scenarios.cpp's other files (where
// either the OPERATOR or the MANAGER's own command handling is the chaotic party, against a
// well-behaved Equipment Module), here the EM itself is the unreliable one, and it is
// unreliable in a SEED-based, combinable, randomized way rather than one hard-wired
// misbehavior per test. ChaosEquipmentModule bundles several independently-toggleable
// knobs (deferred-completion work that finishes late or never, outright transition
// rejection, flapping heartbeat health) behind simple atomic setters, and ONE bounded fuzz
// test drives a manager + two such EMs through a randomized command sequence, all derived
// from a single fixed literal seed so a failing run is exactly reproducible.
//
// This intentionally does NOT assert a specific end state -- the whole point of a fuzz
// run is to explore combinations no single scripted monkey test enumerates. It only
// asserts crash/hang-safety: every sent command gets a response, and the manager is still
// normally answering a clean command once the storm is over.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_ros/ros_names.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;

using NodeHealth = packml_msgs::msg::NodeHealth;

/// Fire-and-don't-wait: send a command and return immediately with the pending future.
/// Used by the fuzz loop below, which paces itself with a short fixed sleep rather than
/// blocking on each response before deciding the next command.
std::shared_future<packml_msgs::srv::StateChange::Response::SharedPtr> send_state_change_async(
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client,
  int8_t command)
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = command;
  return client->async_send_request(req).future.share();
}

/// Configurable "chaos EM" -- the fuzz target itself. Every knob below is independently
/// toggleable (a simple atomic field) so a test can enable any combinable subset:
///   (a) defer_state / defer_delay_*_ms : defer completion of ONE named coordinated state
///       and finish its "work" from a detached background thread after a randomized short
///       delay, instead of completing the instant on_state_trans_req() approves it.
///   (b) forget_probability : chance that, for a deferred state, the background thread
///       simply never calls report_state_complete() at all -- bounded only by the
///       manager's own state_complete_timeout_ms, not by anything in this class.
///   (c) reject_probability : chance that on_state_trans_req() rejects the transition
///       outright (returns false), regardless of which state is being requested.
///   (d) health_flap_enabled / health_flap_period_ms : flap reported health between
///       HEALTHY/NONE and ERROR/HOLD on a fixed period, independent of (a)-(c).
/// UNDEFINED (State value 0) is used as defer_state's "defer nothing" sentinel -- it is
/// never itself a coordinated acting state fanned out to an EM (see
/// PackmlManagerInterface::init()'s kCoordinatedStates), so it can never collide with a
/// real target.
class ChaosEquipmentModule : public PackmlNodeInterface
{
public:
  /// `rng_seed` seeds this instance's OWN internal randomness (delay jitter and the
  /// probability rolls for knobs (b)/(c)) -- kept separate from, but just as fixed and
  /// deterministic as, the fuzz test's own command-sequence RNG, so the whole run
  /// (including exactly which forgets/rejects fire) is reproducible from one literal seed.
  ChaosEquipmentModule(rclcpp::Node::SharedPtr node, uint32_t rng_seed)
  : rng_(rng_seed)
  {
    init(node);
    health_flap_thread_ = std::thread(&ChaosEquipmentModule::health_flap_loop, this);
  }

  // PackmlNodeInterface has no virtual destructor -- safe here because every instance is
  // owned via std::shared_ptr<ChaosEquipmentModule> (the concrete type), never through a
  // PackmlNodeInterface* / shared_ptr<PackmlNodeInterface>, matching every other EM
  // subclass in this test/ directory.
  ~ChaosEquipmentModule()
  {
    health_flap_running_.store(false);
    if (health_flap_thread_.joinable()) {
      health_flap_thread_.join();
    }
  }

  // --- knobs (independently toggleable; set before spinning starts to avoid racing this
  // node's own executor thread) ---
  std::atomic<packml_sm::State> defer_state{packml_sm::State::UNDEFINED};
  std::atomic<int> defer_delay_min_ms{20};
  std::atomic<int> defer_delay_max_ms{150};
  std::atomic<double> forget_probability{0.0};
  std::atomic<double> reject_probability{0.0};
  std::atomic<bool> health_flap_enabled{false};
  std::atomic<int> health_flap_period_ms{150};

  NodeHealth get_health_status() override
  {
    NodeHealth h;
    if (health_flap_enabled.load() && !flap_is_healthy_.load()) {
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::HOLD;
    } else {
      h.status = NodeHealth::HEALTHY;
      h.action = NodeHealth::NONE;
    }
    return h;
  }

protected:
  // Runs synchronously on this node's own single executor thread (see begin_transition()'s
  // own comment in packml_interface.hpp) -- so rng_ needs no locking here.
  bool on_state_trans_req(packml_sm::State) override
  {
    return !roll(reject_probability.load());
  }

  // Same thread and the same rng_ reasoning as on_state_trans_req() above: the framework calls
  // this synchronously, right after that hook accepted.
  void on_deferred_work(packml_sm::State, packml_ros::DeferredCompletion completion) override
  {
    const int min_ms = std::max(0, defer_delay_min_ms.load());
    const int max_ms = std::max(min_ms, defer_delay_max_ms.load());
    std::uniform_int_distribution<int> delay_dist(min_ms, max_ms);
    const int delay_ms = delay_dist(rng_);
    const bool forget = roll(forget_probability.load());
    std::thread([completion, delay_ms, forget]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        if (!forget) {
          completion.report(true);
        }
        // else: (b) -- silently never reports; the manager's own
        // state_complete_timeout_ms is the only thing that resolves this goal.
      }).detach();
  }

  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}

  bool defers_completion(packml_sm::State state) override
  {
    const auto target = defer_state.load();
    return target != packml_sm::State::UNDEFINED && state == target;
  }

private:
  /// Returns true with probability `p` (clamped to [0,1]).
  bool roll(double p)
  {
    const double clamped = std::min(1.0, std::max(0.0, p));
    return uniform01_(rng_) < clamped;
  }

  void health_flap_loop()
  {
    while (health_flap_running_.load()) {
      const int period_ms = std::max(10, health_flap_period_ms.load());
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
      if (!health_flap_enabled.load()) {
        continue;
      }
      flap_is_healthy_.store(!flap_is_healthy_.load());
    }
  }

  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform01_{0.0, 1.0};

  std::thread health_flap_thread_;
  std::atomic<bool> health_flap_running_{true};
  std::atomic<bool> flap_is_healthy_{true};
};

}  // namespace

class MonkeyFuzzerTest : public ::testing::Test
{
protected:
  /// Every test builds its own standalone manager + EM(s) (unique node names), mirroring
  /// test_monkey_scenarios.cpp's Rig pattern but generalized to more than one child, since
  /// the fuzz run below exercises two ChaosEquipmentModule instances together.
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

  /// Builds the manager (registered for every name in `em_names`, and also treating each as
  /// a required node so knob (d)'s health flapping actually reaches HealthMonitor) and one
  /// bare rclcpp::Node per EM name -- but does NOT construct any ChaosEquipmentModule itself
  /// (the caller needs to configure each one's knobs before its SpinHelper starts, see
  /// finish_setup()). Each EM node gets a fast heartbeat_interval_ms so heartbeat-driven
  /// health flapping and the gate settle within this test's short bounded runtime.
  Rig begin_setup(const std::string & mgr_prefix, const std::vector<std::string> & em_names)
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    rig.mgr_node = rclcpp::Node::make_shared(rig.mgr_name,
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter(packml_ros::kParamNodeNames, em_names),
        rclcpp::Parameter(packml_ros::kParamRequiredNodes, em_names),
        rclcpp::Parameter(packml_ros::kParamHeartbeatTimeoutFactor, 3.0),
        // Bounded so a forgotten deferred completion (knob (b)) resolves well within this
        // test's own overall time budget rather than the much longer production default.
        rclcpp::Parameter(packml_ros::kParamStateCompleteTimeoutMs, 2000),
      }));
    rig.sm_node = std::make_unique<SMNode_new>(rig.mgr_node);
    for (const auto & name : em_names) {
      auto em_node = rclcpp::Node::make_shared(name);
      em_node->declare_parameter(packml_ros::kParamHeartbeatIntervalMs, 100);
      rig.em_nodes.push_back(em_node);
    }
    rig.state_client = rig.mgr_node->create_client<packml_msgs::srv::StateChange>(
      rig.mgr_name + "/" + packml_ros::kChangeStateService);
    rig.mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(rig.mgr_node);
    return rig;
  }

  /// Starts every EM's SpinHelper (only after the caller has finished configuring each
  /// ChaosEquipmentModule's knobs -- see this class's own header comment) and settles the
  /// machine to STOPPED before the fuzz sequence begins.
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
// Confirmed-safe: crash/hang-safety only. This is the ONE bounded fuzz run this file
// builds (deliberately not a battery of per-behavior unit tests -- see this file's header
// comment): a fixed literal seed drives a single std::mt19937 through two decisions, in
// order, so the ENTIRE run -- which knobs are active on each ChaosEquipmentModule, and
// every command sent afterward -- is reproducible byte-for-byte just from the seed
// printed in the log below.
//
// Deliberately does NOT assert any specific end state (which state the machine lands in
// after 20 rounds of combined rejects/forgotten-completions/health-flapping/random
// commands is exactly the kind of thing this harness exists to explore, not pin down).
// Asserts only: every command sent gets a response (no silent drop, no hang), and the
// manager is still normally answering afterward.
TEST_F(MonkeyFuzzerTest, BoundedFuzzRun_EveryCommandAnsweredAndManagerRecoversAfterward)
{
  static constexpr uint32_t kFuzzSeed = 20260728u;  // fixed literal -- reproducible run
  std::mt19937 rng(kFuzzSeed);
  std::uniform_real_distribution<double> prob01(0.0, 1.0);

  auto rig = begin_setup("monkey_fuzzer", {"chaos_em_1", "chaos_em_2"});

  // Coordinated states a ChaosEquipmentModule might be assigned to defer -- picked from
  // the SAME rng sequence used for the command loop below, so which state (if any) gets
  // the deferred-completion treatment this run is itself part of the reproducible sequence.
  static constexpr packml_sm::State kDeferrableStates[] = {
    packml_sm::State::RESETTING, packml_sm::State::ABORTING, packml_sm::State::CLEARING,
    packml_sm::State::HOLDING, packml_sm::State::UNHOLDING, packml_sm::State::STOPPING,
  };
  static constexpr size_t kNumDeferrableStates =
    sizeof(kDeferrableStates) / sizeof(kDeferrableStates[0]);
  std::uniform_int_distribution<size_t> defer_state_pick(0, kNumDeferrableStates - 1);

  auto em1 = std::make_shared<ChaosEquipmentModule>(rig.em_nodes[0], kFuzzSeed + 1);
  auto em2 = std::make_shared<ChaosEquipmentModule>(rig.em_nodes[1], kFuzzSeed + 2);

  // --- This run's combinable subset of active knobs, chosen from the seeded sequence.
  // Configured BEFORE finish_setup() starts each EM's SpinHelper, so there is no race with
  // this node's own executor thread reading these atomics from on_state_trans_req(). ---
  auto configure_chaos = [&](ChaosEquipmentModule & em, const std::string & name) {
    std::string summary = name + "[";
    if (prob01(rng) < 0.5) {
      const auto state = kDeferrableStates[defer_state_pick(rng)];
      em.defer_state.store(state);
      summary += "defer=" + packml_sm::to_string(state) + ",";
    } else {
      summary += "defer=none,";
    }
    const double forget_p = (prob01(rng) < 0.5) ? 0.35 : 0.0;
    em.forget_probability.store(forget_p);
    summary += "forget_p=" + std::to_string(forget_p) + ",";
    const double reject_p = (prob01(rng) < 0.5) ? 0.2 : 0.0;
    em.reject_probability.store(reject_p);
    summary += "reject_p=" + std::to_string(reject_p) + ",";
    const bool flap = prob01(rng) < 0.5;
    em.health_flap_enabled.store(flap);
    summary += flap ? "health=flapping]" : "health=stable]";
    return summary;
  };

  const std::string em1_summary = configure_chaos(*em1, "chaos_em_1");
  const std::string em2_summary = configure_chaos(*em2, "chaos_em_2");
  RCLCPP_INFO(rclcpp::get_logger("packml_ros_test"),
    "[MonkeyFuzzer] seed=%u knobs -- %s %s",
    kFuzzSeed, em1_summary.c_str(), em2_summary.c_str());

  finish_setup(rig);

  // Fixed, hardcoded command palette -- indexed via the RNG each iteration.
  static constexpr int8_t kCommands[] = {
    packml_msgs::srv::StateChange::Request::RESET,
    packml_msgs::srv::StateChange::Request::ABORT,
    packml_msgs::srv::StateChange::Request::CLEAR,
    packml_msgs::srv::StateChange::Request::HOLD,
    packml_msgs::srv::StateChange::Request::UNHOLD,
    packml_msgs::srv::StateChange::Request::STOP,
  };
  static constexpr size_t kNumCommands = sizeof(kCommands) / sizeof(kCommands[0]);
  std::uniform_int_distribution<size_t> command_pick(0, kNumCommands - 1);

  static constexpr int kIterations = 20;
  std::string sequence_log;
  for (int i = 0; i < kIterations; ++i) {
    const int8_t command = kCommands[command_pick(rng)];
    sequence_log += std::to_string(static_cast<int>(command)) + " ";

    auto future = send_state_change_async(rig.state_client, command);
    ASSERT_EQ(future.wait_for(2s), std::future_status::ready)
      << "iteration " << i << " (cmd=" << static_cast<int>(command)
      << ") never got a response -- possible hang. Sequence so far: " << sequence_log;

    std::this_thread::sleep_for(200ms);
  }

  RCLCPP_INFO(rclcpp::get_logger("packml_ros_test"),
    "[MonkeyFuzzer] seed=%u command sequence (StateChange::Request values): %s",
    kFuzzSeed, sequence_log.c_str());

  // Quiesce every knob before the closing check: a forgotten completion or an outright
  // reject would make the FINAL command itself flaky, which is not what this last
  // assertion is testing (that the manager is still alive and responsive, not that it
  // survives fresh chaos on the very last call).
  for (auto * em : {em1.get(), em2.get()}) {
    em->reject_probability.store(0.0);
    em->forget_probability.store(0.0);
    em->health_flap_enabled.store(false);
    em->defer_state.store(packml_sm::State::UNDEFINED);
  }
  // Let any still in-flight deferred background work from the last few iterations finish
  // before the final check (bounded by defer_delay_max_ms, well under this sleep).
  std::this_thread::sleep_for(300ms);

  // Final, clean command: the manager must still answer normally after the whole storm.
  // Deliberately not asserting `success` here -- which transition is valid from wherever
  // this run's chaos happened to land the machine is exactly the "specific end state"
  // this fuzz run does not pin down; only that the manager is alive and answering.
  auto final_resp = send_state_change(
    rig.state_client, packml_msgs::srv::StateChange::Request::STOP, 3s);
  ASSERT_NE(final_resp, nullptr)
    << "manager did not respond to the final clean STOP after the fuzz run";

  // em1/em2 (declared after rig) would otherwise destruct BEFORE rig's own em_spins stop
  // spinning -- a still-in-flight fan-out callback calling into an object whose vtable is
  // already gone ("pure virtual method called"). Confirmed for real via gdb in
  // test_monkey_multi_mode.cpp's ClientDisconnectMidRequest test; stopping the spinners
  // first, explicitly, removes the race instead of just outrunning it.
  rig.em_spins.clear();
}
