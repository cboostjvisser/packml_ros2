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
// "Monkey" scenarios, alarm-path group: continues test_monkey_scenarios.cpp's house style
// (anonymous-namespace helpers, a Rig + begin_setup()/finish_setup() pair per test, a
// standalone manager+EM pair per test with unique node names) but makes the EQUIPMENT MODULE's
// own health/heartbeat reporting the unreliable party, exercised specifically through the
// packml_alarms path. Covers:
//   1. Alarm flapping: an EM raising and clearing the same fault rapidly, checking that
//      Alarm::stop_event_id correlation (see PackmlManagerInterface's stop-episode doc comment
//      in packml_interface.hpp) stays sane under fast cycling.
//   2. Garbage/out-of-range error codes: an EM reporting a heartbeat error_code with no catalog
//      entry, checking the error-catalog lookup path fails open (alarm still published,
//      unenriched) rather than crashing.
//   3. A slow packml_alarms subscriber during a burst of real alarms, checking that a slow
//      OBSERVER of the reliable, depth-100 alarm topic cannot create backpressure that blocks
//      the manager's own, unrelated command handling.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/msg/alarm.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "packml_sm/common.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;

namespace {

using packml_ros_test::send_state_change;

/// Minimal Equipment Module whose health is entirely test-controlled via the public
/// post_event() (see PackmlNodeInterface) rather than a periodically-polled getter --
/// gives the test direct, synchronous control over exactly when each heartbeat/health
/// change is published, without depending on heartbeat_interval_ms timing at all.
class SimpleHealthEquipmentModule : public PackmlNodeInterface
{
public:
  explicit SimpleHealthEquipmentModule(rclcpp::Node::SharedPtr node)
  {
    init(node);
  }

protected:
  bool on_state_trans_req(packml_sm::State) override {return true;}
  bool on_mode_trans_req(packml_sm::ModeType) override {return true;}
  void on_status_changed() override {}
};

/// Write content to a unique temp YAML file; removed on destruction. Mirrors
/// test_error_catalog.cpp's own helper of the same name/shape (duplicated here rather than
/// shared, matching this test/ directory's existing no-shared-header convention).
class TempYaml
{
public:
  explicit TempYaml(const std::string & content)
  {
    static int counter = 0;
    path_ = std::filesystem::temp_directory_path() /
      ("monkey_alarms_catalog_" + std::to_string(::getpid()) + "_" +
       std::to_string(counter++) + ".yaml");
    std::ofstream f(path_);
    f << content;
    f.close();
  }

  ~TempYaml()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
};

}  // namespace

class MonkeyAlarmsTest : public ::testing::Test
{
protected:
  /// Every test builds its own standalone manager + EM pair (unique node names) -- see
  /// test_monkey_scenarios.cpp's own Rig for the rationale. Extended with an
  /// extra_mgr_params hook (absent from the original) so a test can set required_nodes,
  /// error_catalog_file, etc. on top of the always-present node_names override.
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
  /// the EM itself -- callers need different EM subclasses/behavior, so they construct it
  /// (on rig.em_node) and then call finish_setup().
  Rig begin_setup(
    const std::string & mgr_prefix, const std::string & em_name,
    std::vector<rclcpp::Parameter> extra_mgr_params = {})
  {
    Rig rig;
    rig.mgr_name = packml_ros_test::unique_node_name(mgr_prefix);
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("node_names", std::vector<std::string>{em_name}));
    for (const auto & p : extra_mgr_params) {
      params.push_back(p);
    }
    rig.mgr_node = rclcpp::Node::make_shared(
      rig.mgr_name, rclcpp::NodeOptions().parameter_overrides(params));
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
// CONFIRMED SAFE (with a deliberately loose, tolerant bound -- see the comment inline
// below for why exact single-id equality isn't asserted): rapid raise/clear alarm
// flapping from one required Equipment Module must not mint a fresh Alarm::stop_event_id
// on every single flap -- the manager's episode-close check only samples the health gate
// on its own 200ms health_timeout_timer_ tick, not synchronously with every heartbeat (see
// on_alarm_event()'s stop-episode doc comment in packml_interface.hpp) -- and once the gate
// has genuinely, stably reopened, the next raise must get a fresh id rather than reusing
// whatever the flapping phase last handed out.
TEST_F(MonkeyAlarmsTest, AlarmFlapping_StopEventIdStaysCorrelatedNotStale)
{
  auto rig = begin_setup(
    "monkey_alarm_flap", "flap_em",
    {rclcpp::Parameter("required_nodes", std::vector<std::string>{"flap_em"})});
  auto em = std::make_shared<SimpleHealthEquipmentModule>(rig.em_node);
  finish_setup(rig);

  // Must match alarm_pub_'s own QoS (packml_interface.hpp: reliable().transient_local()) -- a
  // bare rclcpp::QoS(N) defaults to VOLATILE durability, which discovers/matches the publisher
  // fine (get_publisher_count() below still reports > 0) but can silently drop delivery under
  // load, exactly the QoS-incompatibility class of bug already flagged elsewhere in this
  // codebase's own test comments. Traced to this after this test flaked under full-suite load
  // (0 alarms received despite the log showing them genuinely being raised/cleared) while
  // passing standalone every time beforehand.
  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = rig.mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(100).reliable().transient_local(),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u) << "alarm subscription never matched";
  }

  packml_msgs::msg::NodeHealth fault;
  fault.status = packml_msgs::msg::NodeHealth::ERROR;
  fault.action = packml_msgs::msg::NodeHealth::HOLD;
  fault.error_code = 55;
  fault.message = "flapping fault";
  packml_msgs::msg::NodeHealth clear;
  clear.status = packml_msgs::msg::NodeHealth::HEALTHY;
  clear.action = packml_msgs::msg::NodeHealth::NONE;

  // Many raise/clear cycles in about a second -- much faster than the 200ms episode-close
  // poll, so most individual flaps should land inside the SAME episode.
  static constexpr int kFlaps = 20;
  for (int i = 0; i < kFlaps; ++i) {
    em->post_event(fault);
    std::this_thread::sleep_for(20ms);
    em->post_event(clear);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // let the last few events fully propagate

  std::vector<packml_msgs::msg::Alarm> flap_alarms;
  {
    std::lock_guard<std::mutex> lk(alarms_mutex);
    for (const auto & a : alarms) {
      if (a.node_name == "flap_em") {
        flap_alarms.push_back(a);
      }
    }
  }
  ASSERT_GE(flap_alarms.size(), static_cast<size_t>(kFlaps))
    << "expected at least one alarm event per flap, got " << flap_alarms.size();

  std::set<uint64_t> distinct_ids;
  for (const auto & a : flap_alarms) {
    EXPECT_NE(a.stop_event_id, 0u)
      << "every alarm from a HOLD-severity flap must belong to SOME stop episode";
    distinct_ids.insert(a.stop_event_id);
  }
  // Sane, not exact: an occasional extra id from a flap that happened to straddle a
  // 200ms poll is expected and not itself a bug, but minting a brand-new id for
  // essentially every single flap (distinct_ids.size() ~= flap_alarms.size()) would mean
  // stop_event_id isn't correlating the episode at all -- that's the actual gap this
  // guards against.
  EXPECT_LT(distinct_ids.size(), flap_alarms.size() / 2)
    << "stop_event_id looks uncorrelated with the flapping episode -- " << distinct_ids.size()
    << " distinct ids across " << flap_alarms.size() << " alarm events";

  const uint64_t last_flap_id = flap_alarms.back().stop_event_id;

  // Let the gate settle stably OPEN, well over the 200ms poll period, before raising a
  // genuinely separate, later fault.
  std::this_thread::sleep_for(600ms);

  fault.error_code = 56;  // a distinct condition id, so this can't be confused with a repeat
  em->post_event(fault);

  packml_msgs::msg::Alarm new_raise;
  bool found_new_raise = false;
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!found_new_raise && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(alarms_mutex);
        for (auto it = alarms.rbegin(); it != alarms.rend(); ++it) {
          if (it->node_name == "flap_em" && it->trigger && it->error_code == 56u) {
            new_raise = *it;
            found_new_raise = true;
            break;
          }
        }
      }
      if (!found_new_raise) {
        std::this_thread::sleep_for(10ms);
      }
    }
  }
  ASSERT_TRUE(found_new_raise) << "the post-settle raise never appeared on packml_alarms";
  EXPECT_NE(new_raise.stop_event_id, 0u);
  EXPECT_NE(new_raise.stop_event_id, last_flap_id)
    << "a fresh raise after the gate genuinely reopened reused the flapping episode's stale id";

  em->post_event(clear);  // leave healthy so no background state outlives the test

  // em (declared after rig) would otherwise destruct BEFORE rig's own em_spin member stops
  // spinning -- a still-in-flight fan-out callback calling into an object whose vtable is
  // already gone ("pure virtual method called"). Confirmed for real via gdb in
  // test_monkey_multi_mode.cpp's ClientDisconnectMidRequest test; stopping the spinner first,
  // explicitly, removes the race instead of just outrunning it.
  rig.em_spin.reset();
}

// ============================================================================
// CONFIRMED SAFE: the error-catalog lookup fails open. An EM reporting a heartbeat
// error_code with no matching catalog entry -- here, deliberately the largest value
// NodeHealth::error_code (int32) can carry on the wire, 0x7FFFFFFF / INT32_MAX, well above
// any real catalog entry (the aggregation tool rejects any code above this at build time,
// so a real node can never legitimately advertise one) -- must not crash the manager or
// the lookup path. on_alarm_event() must
// still publish the alarm, simply without enrichment (global_code stays 0, message stays
// exactly what the node sent), and the manager must remain fully responsive afterward.
TEST_F(MonkeyAlarmsTest, GarbageErrorCodeDegradesGracefullyWithoutEnrichment)
{
  const std::string catalog_yaml = R"(
languages: [en]
nodes:
  garbage_code_em:
    base: 100
    codes:
      1:
        global: 101
        name: KNOWN_FAULT
        severity: ERROR
        action: HOLD
        descriptions:
          en: "A known, catalogued fault"
)";
  TempYaml catalog(catalog_yaml);

  auto rig = begin_setup(
    "monkey_alarm_garbage_code", "garbage_code_em",
    {rclcpp::Parameter("required_nodes", std::vector<std::string>{"garbage_code_em"}),
     rclcpp::Parameter("error_catalog_file", catalog.path())});
  auto em = std::make_shared<SimpleHealthEquipmentModule>(rig.em_node);
  finish_setup(rig);

  std::mutex alarms_mutex;
  std::vector<packml_msgs::msg::Alarm> alarms;
  auto alarm_sub = rig.mgr_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(50).reliable().transient_local(),
    [&alarms, &alarms_mutex](packml_msgs::msg::Alarm::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(alarms_mutex);
      alarms.push_back(*msg);
    });
  {
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u) << "alarm subscription never matched";
  }

  // Control: the catalogued code (1) IS enriched -- proves the catalog genuinely loaded,
  // so this test isn't trivially passing because has_error_catalog_ stayed false.
  packml_msgs::msg::NodeHealth known_fault;
  known_fault.status = packml_msgs::msg::NodeHealth::ERROR;
  known_fault.action = packml_msgs::msg::NodeHealth::HOLD;
  known_fault.error_code = 1;
  em->post_event(known_fault);

  packml_msgs::msg::Alarm known_alarm;
  bool found_known = false;
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!found_known && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(alarms_mutex);
        for (const auto & a : alarms) {
          if (a.node_name == "garbage_code_em" && a.trigger && a.error_code == 1u) {
            known_alarm = a;
            found_known = true;
            break;
          }
        }
      }
      if (!found_known) {
        std::this_thread::sleep_for(10ms);
      }
    }
  }
  ASSERT_TRUE(found_known) << "the catalogued fault never appeared on packml_alarms";
  EXPECT_EQ(known_alarm.global_code, 101u) << "the catalog should have enriched a known code";

  // Clear before raising the garbage code, so the garbage alarm below is a genuinely NEW
  // (raised) event rather than a suppressed repeat of the same still-active action.
  packml_msgs::msg::NodeHealth clear;
  clear.status = packml_msgs::msg::NodeHealth::HEALTHY;
  clear.action = packml_msgs::msg::NodeHealth::NONE;
  em->post_event(clear);
  std::this_thread::sleep_for(200ms);

  // The garbage code itself: valid ON THE WIRE (NodeHealth::error_code is int32, and this
  // is exactly INT32_MAX) but nonsensical -- no real catalog entry can ever reach it.
  packml_msgs::msg::NodeHealth garbage_fault;
  garbage_fault.status = packml_msgs::msg::NodeHealth::ERROR;
  garbage_fault.action = packml_msgs::msg::NodeHealth::ABORT;
  garbage_fault.error_code = 0x7FFFFFFF;
  garbage_fault.message = "garbage code monkey test";
  em->post_event(garbage_fault);

  packml_msgs::msg::Alarm garbage_alarm;
  bool found_garbage = false;
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!found_garbage && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lk(alarms_mutex);
        for (const auto & a : alarms) {
          if (a.node_name == "garbage_code_em" && a.trigger && a.error_code == 0x7FFFFFFFu) {
            garbage_alarm = a;
            found_garbage = true;
            break;
          }
        }
      }
      if (!found_garbage) {
        std::this_thread::sleep_for(10ms);
      }
    }
  }
  ASSERT_TRUE(found_garbage)
    << "the garbage-code alarm never appeared on packml_alarms -- the lookup path may have "
       "thrown or otherwise prevented publication instead of failing open";
  EXPECT_EQ(garbage_alarm.global_code, 0u)
    << "an out-of-range error_code must not be enriched with an unrelated catalog entry";
  EXPECT_EQ(garbage_alarm.message, "garbage code monkey test")
    << "an unmatched error_code must leave the node's own message untouched, not substitute "
       "or prefix a lookup result";
  EXPECT_EQ(garbage_alarm.severity, packml_msgs::msg::NodeHealth::ABORT);

  // Manager must still be fully responsive afterward -- the garbage lookup must not have
  // wedged the executor or corrupted any shared state.
  auto resp = send_state_change(rig.state_client, packml_msgs::srv::StateChange::Request::ABORT, 2s);
  ASSERT_NE(resp, nullptr) << "manager stopped responding after the garbage error_code alarm";

  // See AlarmFlapping_StopEventIdStaysCorrelatedNotStale's own comment above: stop em_spin
  // before em (declared after rig) destructs.
  rig.em_spin.reset();
}

// ============================================================================
// CONFIRMED SAFE: packml_alarms is reliable QoS with depth 100, not fire-and-forget (see
// ros_names.hpp / PackmlManagerInterface::init()) specifically so a slow consumer doesn't
// need to be fire-and-forget-tolerant -- but that intent is only real if the manager's OWN,
// unrelated command handling (~/changeState here) truly keeps working promptly while a
// separate, slow packml_alarms subscriber lags behind a burst of real alarms. The slow
// subscriber runs on its own dedicated node + executor thread specifically so ITS sleeping
// callback can only ever stall its own thread, never the manager's -- isolating whether the
// manager's publish() path itself experiences backpressure from a slow reader (the actual
// property under test) from a trivial "one thread can't run two callbacks at once" artifact.
TEST_F(MonkeyAlarmsTest, SlowAlarmSubscriberDoesNotBlockManagerCommandHandling)
{
  auto mgr_name = packml_ros_test::unique_node_name("monkey_alarm_slow_sub");
  auto mgr_node = rclcpp::Node::make_shared(
    mgr_name, rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter(
        "required_nodes",
        std::vector<std::string>{"slow_sub_em_a", "slow_sub_em_b", "slow_sub_em_c"})}));
  auto sm_node = std::make_unique<SMNode_new>(mgr_node);

  auto em_a_node = rclcpp::Node::make_shared("slow_sub_em_a");
  auto em_b_node = rclcpp::Node::make_shared("slow_sub_em_b");
  auto em_c_node = rclcpp::Node::make_shared("slow_sub_em_c");
  auto em_a = std::make_shared<SimpleHealthEquipmentModule>(em_a_node);
  auto em_b = std::make_shared<SimpleHealthEquipmentModule>(em_b_node);
  auto em_c = std::make_shared<SimpleHealthEquipmentModule>(em_c_node);

  auto state_client = mgr_node->create_client<packml_msgs::srv::StateChange>(
    mgr_name + "/changeState");

  auto mgr_spin = std::make_shared<packml_ros_test::SpinHelper>(mgr_node);
  auto em_a_spin = std::make_shared<packml_ros_test::SpinHelper>(em_a_node);
  auto em_b_spin = std::make_shared<packml_ros_test::SpinHelper>(em_b_node);
  auto em_c_spin = std::make_shared<packml_ros_test::SpinHelper>(em_c_node);

  ASSERT_TRUE(state_client->wait_for_service(5s));
  send_state_change(state_client, packml_msgs::srv::StateChange::Request::STOP);
  std::this_thread::sleep_for(300ms);

  // Slow subscriber: its own dedicated node + spinner (see this test's own header comment
  // for why), sleeping in every callback to simulate a backed-up alarm journal/HMI.
  auto sub_node = rclcpp::Node::make_shared(
    packml_ros_test::unique_node_name("monkey_alarm_slow_subscriber"));
  std::atomic<int> slow_received{0};
  auto alarm_sub = sub_node->create_subscription<packml_msgs::msg::Alarm>(
    packml_ros::kAlarmsTopic, rclcpp::QoS(100).reliable().transient_local(),
    [&slow_received](packml_msgs::msg::Alarm::SharedPtr) {
      slow_received.fetch_add(1);
      std::this_thread::sleep_for(250ms);
    });
  auto sub_spin = std::make_shared<packml_ros_test::SpinHelper>(sub_node);

  {
    // Ensure the slow subscriber has actually matched the manager's publisher before the
    // burst below, else the burst could race ahead of discovery and never exercise it.
    packml_ros_test::wait_until(
      [&] {return alarm_sub->get_publisher_count() > 0;}, 2s, 10ms);
    ASSERT_GT(alarm_sub->get_publisher_count(), 0u)
      << "slow subscriber never matched the manager's alarm publisher";
  }

  // Burst: fault all three EMs in sequence -- several real alarms hit packml_alarms in a
  // short window, right as the slow subscriber above starts (slowly) draining them.
  packml_msgs::msg::NodeHealth fault;
  fault.status = packml_msgs::msg::NodeHealth::ERROR;
  fault.action = packml_msgs::msg::NodeHealth::HOLD;
  fault.message = "burst fault";
  em_a->post_event(fault);
  em_b->post_event(fault);
  em_c->post_event(fault);

  // While the burst is still being (slowly) drained, hammer the manager's OWN, unrelated
  // command handling -- every one of these must resolve promptly. ABORT is redundant-safe
  // from any state the machine may currently be in (see MonkeyAbortMashing_
  // RedundantAbortsAreHarmless in test_monkey_scenarios.cpp for the same pattern).
  for (int i = 0; i < 5; ++i) {
    const auto start = std::chrono::steady_clock::now();
    auto resp = send_state_change(state_client, packml_msgs::srv::StateChange::Request::ABORT, 2s);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    ASSERT_NE(resp, nullptr)
      << "command #" << i << " never got a response while the alarm subscriber was backed up";
    EXPECT_LT(elapsed, 500ms)
      << "command #" << i << " took suspiciously long -- possible backpressure from the "
         "slow alarm subscriber";
    std::this_thread::sleep_for(50ms);
  }

  // Sanity: the slow subscriber did eventually see the burst's alarms (proves it was
  // genuinely subscribed and processing, not silently disconnected this whole time).
  packml_ros_test::wait_until(
    [&] {return slow_received.load() >= 3;}, 3s, 50ms);
  EXPECT_GE(slow_received.load(), 3) << "slow subscriber never received the burst's alarms";
}
