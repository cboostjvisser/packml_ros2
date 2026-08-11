// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// Tests for PackmlManagerInterface status publication on /packml_status topic.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <vector>

#include "packml_ros/packml_ros-new.hpp"
#include "packml_msgs/msg/status.hpp"
#include "packml_msgs/msg/state.hpp"
#include "packml_msgs/srv/state_change.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;
using StateMsg = packml_msgs::msg::State;

class ManagerStatusPublicationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_name_ = packml_ros_test::unique_node_name("mgr_status_test");
    node_ = rclcpp::Node::make_shared(node_name_);
    sm_node_ = std::make_unique<SMNode_new>(node_);

    state_client_ = node_->create_client<packml_msgs::srv::StateChange>(
      node_name_ + "/changeState");

    last_status_state_.store(StateMsg::UNDEFINED);
    status_received_.store(false);

    // Deliberately mismatched vs. the manager's publisher (TRANSIENT_LOCAL + RELIABLE):
    // ROS 2 QoS compatibility still connects this (BEST_EFFORT/VOLATILE only requests
    // less than what's offered), so it observes any FUTURE publish just fine — every
    // test below triggers a transition first. It would NOT receive a status published
    // before this subscription existed; see LateJoiningMatchedQosSubscriberGetsRetainedStatus
    // below for a subscriber using matching QoS that verifies that retained-status behavior.
    status_sub_ = node_->create_subscription<packml_msgs::msg::Status>(
      "packml_status", rclcpp::SensorDataQoS(),
      [this](const packml_msgs::msg::Status & msg) {
        last_status_state_.store(msg.state.val);
        last_status_mode_.store(msg.mode.val);
        status_received_.store(true);
      });

    spinner_ = std::make_unique<packml_ros_test::SpinHelper>(node_);

    ASSERT_TRUE(state_client_->wait_for_service(5s));
    // Wait for SM to activate (starts in STOPPED)
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    spinner_.reset();
    status_sub_.reset();
    state_client_.reset();
    sm_node_.reset();
    node_.reset();
  }

  packml_msgs::srv::StateChange::Response::SharedPtr send_state(int8_t cmd)
  {
    auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
    req->command = cmd;
    auto future = state_client_->async_send_request(req);
    if (future.wait_for(5s) == std::future_status::ready) {
      return future.get();
    }
    return nullptr;
  }

  bool wait_for_status(int8_t expected_state, std::chrono::milliseconds timeout = 2000ms)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (last_status_state_.load() == expected_state) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::string node_name_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SMNode_new> sm_node_;
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr state_client_;
  rclcpp::Subscription<packml_msgs::msg::Status>::SharedPtr status_sub_;
  std::unique_ptr<packml_ros_test::SpinHelper> spinner_;
  std::atomic<int8_t> last_status_state_;
  std::atomic<int8_t> last_status_mode_;
  std::atomic<bool> status_received_;
};

TEST_F(ManagerStatusPublicationTest, StatusPublishedAfterReset)
{
  // Drive SM from STOPPED -> RESET -> IDLE
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "RESET failed: " << resp->message;

  // After RESET, SM goes through RESETTING -> IDLE; status should be published
  EXPECT_TRUE(wait_for_status(StateMsg::IDLE))
    << "Expected IDLE status publication, got state=" << (int)last_status_state_.load();
}

TEST_F(ManagerStatusPublicationTest, StatusPublishedAfterAbort)
{
  // SM starts in STOPPED; ABORT transitions to ABORTING -> ABORTED
  auto resp = send_state(packml_msgs::srv::StateChange::Request::ABORT);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "ABORT failed: " << resp->message;

  EXPECT_TRUE(wait_for_status(StateMsg::ABORTED))
    << "Expected ABORTED status publication, got state=" << (int)last_status_state_.load();
}

TEST_F(ManagerStatusPublicationTest, StatusContainsCorrectStateValue)
{
  // Drive to IDLE
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_status(StateMsg::IDLE));

  EXPECT_EQ(last_status_state_.load(), StateMsg::IDLE);
}

TEST_F(ManagerStatusPublicationTest, StatusTopicReceivesMessages)
{
  // Trigger a transition to get a publish
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  // Wait for any status message
  auto start = std::chrono::steady_clock::now();
  while (!status_received_.load() && std::chrono::steady_clock::now() - start < 3s) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(status_received_.load()) << "No status message received on packml_status topic";
}

// A subscriber using QoS matching the publisher (TRANSIENT_LOCAL + RELIABLE) that joins
// AFTER a status was already published must immediately receive that RETAINED status,
// with NO further transition triggered.
// Expected: the late joiner observes IDLE without the test triggering any new transition.
TEST_F(ManagerStatusPublicationTest, LateJoiningMatchedQosSubscriberGetsRetainedStatus)
{
  // Drive a transition so the manager publishes (and retains) at least one status.
  auto resp = send_state(packml_msgs::srv::StateChange::Request::RESET);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);
  ASSERT_TRUE(wait_for_status(StateMsg::IDLE))
    << "Setup transition never published — cannot test late-join behavior";

  // NOW create the late-joining subscriber, with QoS matching the manager's publisher.
  std::atomic<int8_t> late_state{StateMsg::UNDEFINED};
  std::atomic<bool> late_received{false};
  auto late_sub = node_->create_subscription<packml_msgs::msg::Status>(
    "packml_status", rclcpp::QoS(1).transient_local().reliable(),
    [&late_state, &late_received](const packml_msgs::msg::Status & msg) {
      late_state.store(msg.state.val);
      late_received.store(true);
    });

  // No further transition here — the retained sample must arrive on its own.
  auto start = std::chrono::steady_clock::now();
  while (!late_received.load() && std::chrono::steady_clock::now() - start < 2s) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(late_received.load())
    << "Late-joining subscriber with matching QoS never received the retained status";
  EXPECT_EQ(late_state.load(), StateMsg::IDLE);
}

// ---------------------------------------------------------------------------
// Duplicate-publisher detection on the status topic.
//
// packml_status is a bare global name and the manager is only its owner by convention. A second
// publisher -- a duplicated launch entry, two managers started against one machine -- makes an
// Equipment Module adopt a state nobody commanded and possibly skip coordinated work it believes
// is already done, with nothing in either node's log to say so. foreign_publishers_on() is what
// makes it visible; it reports, it does not prevent.
// ---------------------------------------------------------------------------

namespace {
/// The GID of a publisher, in the shape foreign_publishers_on() compares against.
std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid_of(
  const rclcpp::Publisher<packml_msgs::msg::Status>::SharedPtr & pub)
{
  const auto & gid = pub->get_gid();
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> out{};
  std::copy(std::begin(gid.data), std::end(gid.data), out.begin());
  return out;
}

/// Poll until `pred` holds for the reported publisher set, or give up. Discovery is what is being
/// waited on here, and how long a participant takes to announce itself is an RMW question rather
/// than a fixed number of milliseconds -- asserting on a single read would be asserting on timing.
std::vector<std::string> await_publishers(
  rclcpp::Node::SharedPtr observer,
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & own_gid,
  const std::function<bool(const std::vector<std::string> &)> & pred)
{
  std::vector<std::string> seen;
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline) {
    seen = packml_ros::foreign_publishers_on(*observer, "/packml_status", own_gid);
    if (pred(seen)) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  return seen;
}

std::string join(const std::vector<std::string> & names)
{
  std::string joined;
  for (const auto & name : names) {
    if (!joined.empty()) { joined += ", "; }
    joined += name;
  }
  return joined;
}
}  // namespace

// Self-exclusion is by ENDPOINT, and this is the test that says so.
//
// The probe publisher below lives on the SAME NODE as the manager -- same name, same namespace,
// different endpoint. That is the shape of a duplicated launch entry, which is the likeliest way
// two publishers ever end up on this topic, and it is precisely what an exclusion keyed on
// (node_name, node_namespace) cannot see: it would discard the manager's publisher as "self" and
// report nothing. Excluding by GID reports it.
TEST_F(ManagerStatusPublicationTest, StatusTopicSelfExclusionIsByEndpointNotByNodeName)
{
  auto probe_pub = node_->create_publisher<packml_msgs::msg::Status>(
    "/packml_status", rclcpp::QoS(1).transient_local().reliable());

  const auto seen = await_publishers(
    node_, gid_of(probe_pub), [](const std::vector<std::string> & p) {return !p.empty();});

  ASSERT_FALSE(seen.empty())
    << "the manager's publisher was not reported to a probe sharing its node name and namespace -- "
       "self-exclusion is matching on the node, not the endpoint, so a duplicated launch entry is "
       "invisible to it";
  const std::string expected = std::string(node_->get_namespace()) == "/"
    ? "/" + std::string(node_->get_name())
    : std::string(node_->get_namespace()) + "/" + node_->get_name();
  EXPECT_NE(std::find(seen.begin(), seen.end(), expected), seen.end())
    << "expected the manager's own node (" << expected << ") among: " << join(seen);
}

TEST_F(ManagerStatusPublicationTest, StatusTopicReportsAPublisherOnAnotherNode)
{
  auto rogue_name = packml_ros_test::unique_node_name("rogue_status_publisher");
  auto rogue = rclcpp::Node::make_shared(rogue_name);
  auto rogue_pub = rogue->create_publisher<packml_msgs::msg::Status>(
    "/packml_status", rclcpp::QoS(1).transient_local().reliable());
  packml_ros_test::SpinHelper rogue_spinner(rogue);

  // Observed from the probe publisher's point of view, so both the manager's publisher and the
  // rogue one are foreign to it.
  auto probe_pub = node_->create_publisher<packml_msgs::msg::Status>(
    "/packml_status", rclcpp::QoS(1).transient_local().reliable());
  const std::string rogue_fq = "/" + rogue_name;

  const auto seen = await_publishers(
    node_, gid_of(probe_pub), [&rogue_fq](const std::vector<std::string> & p) {
      return std::find(p.begin(), p.end(), rogue_fq) != p.end();
    });

  EXPECT_NE(std::find(seen.begin(), seen.end(), rogue_fq), seen.end())
    << "the rogue publisher on " << rogue_fq << " was never reported; saw: " << join(seen);
}

// Nothing but this manager writes the topic, so a probe that excludes only itself still sees the
// manager, and a check keyed on the manager's OWN publisher sees nobody. The latter is what
// check_status_topic_ownership() does every tick in a healthy system, and it must stay quiet.
TEST_F(ManagerStatusPublicationTest, StatusTopicIsQuietWhenTheManagerIsItsOnlyWriter)
{
  // Excluding a GID that belongs to no endpoint, so every publisher on the topic is reported.
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> nobody{};

  std::vector<std::string> seen;
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < deadline) {
    seen = packml_ros::foreign_publishers_on(*node_, "/packml_status", nobody);
    if (seen.size() == 1u) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(seen.size(), 1u)
    << "expected exactly the manager's own publisher on the topic, saw: " << join(seen);
}
