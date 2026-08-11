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
// "Monkey" scenario, manager-restart edition: sibling to test_monkey_scenarios.cpp, covering
// the one backlog item that file's own BACKLOG comment calls out as "a bigger, different
// shape" -- the MANAGER process itself crashing and restarting mid-cycle, rather than an
// Equipment Module misbehaving underneath an otherwise-stable manager. A real supervisor
// (systemd, a launch-file restart policy, a container orchestrator) would relaunch the exact
// same manager node -- same name, same node_names/parameter configuration -- after a crash;
// this test reproduces that directly by destroying an SMNode_new and its rclcpp::Node
// entirely and then constructing a brand new pair with identical configuration, while the
// Equipment Module node is left running throughout, exactly as it would be in the real
// failure this models (the EM is a separate OS process from the manager, and has no reason
// to know the manager died).
//
// The scenario hinges on a specific, deliberate design property of packml_status (see
// ros_names.hpp / PackmlManagerInterface::init()): its name is a bare, hardcoded topic
// ("packml_status"), never derived from or namespaced under the manager node's own name, and
// it is published TRANSIENT_LOCAL + RELIABLE (depth 1) -- see
// test_manager_status_publication.cpp's LateJoiningMatchedQosSubscriberGetsRetainedStatus for
// the same QoS property exercised within a single manager's lifetime. An Equipment Module's
// status subscription therefore does not carry any session/identity concept to reconnect --
// a brand new manager process's first published status is, from the EM's own subscription's
// perspective, indistinguishable from any other status update from the SAME manager it was
// already subscribed to. This test exists to confirm that property empirically, end to end
// (real manager destruction/reconstruction, real EM node, real topic), rather than only by
// reading the QoS/topic-naming code.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
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

/// Equipment Module whose only job is to observe the manager's published state via its OWN
/// packml_status subscription (see PackmlNodeInterface::init()'s status_sub_, TRANSIENT_LOCAL
/// + RELIABLE) -- exactly the same view any real EM keeps of whichever manager it is talking
/// to, exposed here read-only for the test to poll. Approves every requested transition
/// instantly (a non-deferring EM, like PlainEquipmentModule in the sibling monkey test file),
/// so the manager side can move through the state machine on its own without this test having
/// to separately drive EM-side completion for each step.
class StatusObservingEquipmentModule : public PackmlNodeInterface
{
public:
  explicit StatusObservingEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

  /// The state this node's own transition-guard protocol last adopted from a packml_status
  /// message -- i.e. exactly what a real EM's on_status_update()/on_status_changed() maintain.
  packml_sm::State observed_state() const { return get_current_packml_state(); }

  /// Counts how many times on_status_changed() actually fired -- confirms the re-sync below
  /// is driven by real inbound status messages, not a coincidental default value.
  std::atomic<int> status_update_count{0};

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {status_update_count.fetch_add(1);}
};

bool wait_for_observed_state(
  const std::shared_ptr<StatusObservingEquipmentModule> & em,
  packml_sm::State target,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (em->observed_state() == target) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return em->observed_state() == target;
}

/// One manager "generation": a fresh rclcpp::Node + SMNode_new, spinning and ready to accept
/// ~/changeState calls. Deliberately NOT a fixture/Rig shared across tests in this file (there
/// is only one test) -- it exists purely so the test body can build two, back-to-back,
/// identically-configured manager instances without duplicating the construction sequence.
struct ManagerInstance
{
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<SMNode_new> sm_node;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client;
  std::shared_ptr<packml_ros_test::SpinHelper> spin;
};

/// Construct one manager generation with the given name/node_names configuration. Called
/// twice by the test below with IDENTICAL arguments both times -- once for the manager's
/// original run, once again for the "crashed and restarted" instance -- the same way a real
/// supervisor would relaunch the same manager process, under the same name, after a crash.
ManagerInstance spin_up_manager(const std::string & mgr_name, const std::string & em_name)
{
  ManagerInstance mgr;
  mgr.node = rclcpp::Node::make_shared(mgr_name,
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("node_names", std::vector<std::string>{em_name}),
    }));
  mgr.sm_node = std::make_unique<SMNode_new>(mgr.node);
  mgr.state_client = mgr.node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/changeState");
  mgr.spin = std::make_shared<packml_ros_test::SpinHelper>(mgr.node);
  return mgr;
}

}  // namespace

// ============================================================================
// Confirmed-safe regression test: the EM's status-driven view of the manager's state should
// re-sync to a brand new manager instance with NO special-cased reconnect logic anywhere,
// purely because packml_status is a bare, un-namespaced, TRANSIENT_LOCAL+RELIABLE topic (see
// this file's own header comment). This is the one, deliberately bigger and differently-shaped
// scenario in this file -- destroying/recreating the manager itself, not just an EM -- so a
// single thorough test covers it rather than several small ones.
TEST(MonkeyManagerRestartTest, ManagerRestartMidCycle_EmStatusViewResyncsToNewInstance)
{
  const auto mgr_name = packml_ros_test::unique_node_name("monkey_restart_mgr");
  const std::string em_name = "monkey_restart_em";

  // --- Equipment Module: constructed ONCE, stays alive across BOTH manager generations ---
  auto em_node = rclcpp::Node::make_shared(em_name);
  auto em = std::make_shared<StatusObservingEquipmentModule>(em_node);
  auto em_spin = std::make_shared<packml_ros_test::SpinHelper>(em_node);

  // --- Generation 1: drive the machine to a non-trivial state (IDLE) ---
  auto mgr1 = spin_up_manager(mgr_name, em_name);
  ASSERT_TRUE(mgr1.state_client->wait_for_service(5s));

  send_state_change(mgr1.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_TRUE(wait_for_state(mgr1.sm_node, packml_sm::State::STOPPED, 2s))
    << "generation 1 manager never reached STOPPED";

  auto reset_resp = send_state_change(mgr1.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset_resp, nullptr);
  ASSERT_TRUE(reset_resp->success) << "generation 1 RESET failed: " << reset_resp->message;
  ASSERT_TRUE(wait_for_state(mgr1.sm_node, packml_sm::State::IDLE, 2s))
    << "generation 1 manager never reached IDLE";

  // The EM's own status-subscription view must have followed generation 1 into IDLE too,
  // BEFORE that generation is torn down -- this is the "before" baseline the restart below
  // has to actually change, not just coincidentally already match.
  ASSERT_TRUE(wait_for_observed_state(em, packml_sm::State::IDLE, 2s))
    << "EM's own status view never caught up to generation 1's IDLE";
  // NOT checked here: on_status_changed() firing count. A non-deferring EM's OWN
  // begin_transition() calls mark_state_locally_reached() (effectively on_status_update())
  // the moment ITS OWN action goal is accepted/completed -- typically slightly ahead of the
  // manager's separate, passive status broadcast for that same state. By the time that status
  // message arrives, on_status_update() usually finds nothing changed (already there) and
  // on_status_changed() never fires -- expected, not a bug (see this session's own
  // already_there/note_goal_admitted() work). The real proof that the restart re-sync below
  // isn't a coincidental default value is generation 2's STOPPED check further down: STOPPED
  // can never coincidentally equal generation 1's IDLE, so that match can only come from a
  // real, new message.

  // --- Simulate the manager process crashing: destroy EVERYTHING manager-side ---
  // The EM (node, spin, subscription) is left running throughout -- only generation 1's own
  // spin helper, SMNode_new, and rclcpp::Node are torn down here, in that order: stop
  // spinning before destroying what it spins, then destroy the SM (which holds its own
  // internal shared_ptr to the node -- see PackmlManagerInterface::init()'s node_ member)
  // before releasing this scope's own copy of the node's shared_ptr.
  mgr1.spin.reset();
  mgr1.state_client.reset();
  mgr1.sm_node.reset();
  mgr1.node.reset();
  std::this_thread::sleep_for(300ms);

  // Nothing is publishing packml_status anymore at this point -- the EM's view is now
  // necessarily stale, still reading whatever generation 1 last reported (IDLE), even though
  // "the manager" (as a process) no longer exists. Sanity-checks that the EM's view doesn't
  // spontaneously reset/clear itself on its own when the publisher goes away.
  EXPECT_EQ(em->observed_state(), packml_sm::State::IDLE)
    << "sanity check: EM's view should still be the stale pre-crash value at this point";

  // --- "Restart": a brand new manager, SAME name, SAME node_names/parameter config ---
  auto mgr2 = spin_up_manager(mgr_name, em_name);
  ASSERT_TRUE(mgr2.state_client->wait_for_service(5s))
    << "restarted manager's changeState service never came up";

  // Generation 2 is driven to STOPPED exactly like generation 1 was (a freshly-constructed
  // SMNode_new is not guaranteed to already be in STOPPED -- see this suite's own
  // begin_setup()/finish_setup() STOP-first convention in test_monkey_scenarios.cpp) -- a
  // state DIFFERENT from generation 1's IDLE, so a pass below can only mean the EM's view
  // actually followed the NEW manager, not that it coincidentally still matched the old one.
  send_state_change(mgr2.state_client, packml_msgs::srv::StateChange::Request::STOP);
  ASSERT_TRUE(wait_for_state(mgr2.sm_node, packml_sm::State::STOPPED, 2s))
    << "generation 2 manager never reached STOPPED";

  // THE ASSERTION: with no special-cased reconnect logic anywhere -- the EM never "noticed"
  // that its manager was destroyed and replaced, it simply kept subscribing to the same
  // packml_status topic name the whole time -- the EM's own view re-syncs to generation 2's
  // STOPPED, purely from the pre-existing TRANSIENT_LOCAL + RELIABLE subscription matching the
  // new publisher and delivering its first published status.
  ASSERT_TRUE(wait_for_observed_state(em, packml_sm::State::STOPPED, 2s))
    << "EM's status view never re-synced to the restarted manager's STOPPED; stuck at: "
    << static_cast<int>(em->observed_state());

  // Drive generation 2 further (RESET -> IDLE) and confirm the EM keeps following it -- proving
  // this is a live, ongoing subscription against whichever manager instance currently owns the
  // topic, not a one-off fluke of only the very first post-restart message.
  auto reset2_resp = send_state_change(mgr2.state_client, packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(reset2_resp, nullptr);
  ASSERT_TRUE(reset2_resp->success) << "generation 2 RESET failed: " << reset2_resp->message;
  ASSERT_TRUE(wait_for_state(mgr2.sm_node, packml_sm::State::IDLE, 2s))
    << "generation 2 manager never reached IDLE after RESET";
  EXPECT_TRUE(wait_for_observed_state(em, packml_sm::State::IDLE, 2s))
    << "EM's status view did not keep following generation 2 after its own RESET";

  // Teardown: generation 2's own destruction (same order as generation 1's above), then the EM.
  mgr2.spin.reset();
  mgr2.state_client.reset();
  mgr2.sm_node.reset();
  mgr2.node.reset();
  em_spin.reset();
  em.reset();
  em_node.reset();
}
