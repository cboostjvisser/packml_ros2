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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "packml_sm/common.hpp"
#include "packml_ros/ros_names.hpp"
#include "packml_ros/deferred_completion.hpp"
#include "packml_msgs/msg/node_health.hpp"
#include "packml_msgs/msg/node_heartbeat.hpp"
#include "packml_msgs/srv/state_change.hpp"

namespace packml_ros_test {

/// Spin a node in a background thread for test lifetime.
///
/// Must spin blocking, not poll with spin_some(). An executor learns about an entity created after
/// it started spinning only from the node's notify guard condition, and a poll loop -- mostly
/// outside the middleware wait -- can lose that one trigger for the entity's whole life, after
/// which its messages arrive at the middleware and are never taken while the node's other entities
/// keep working. A blocking spin() is parked in the wait when the trigger arrives.
///
/// Holds the node it spins. An executor keeps only weak references, so without this a node released
/// while its helper is still alive leaves a thread servicing something already destroyed. That
/// covers the node and nothing built ON it: a PackmlNodeInterface subclass has its own
/// lifetime, and a test destroying one must still stop the spinner first or a fan-out callback
/// lands in a freed vtable.
class SpinHelper
{
public:
  explicit SpinHelper(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)),
    exec_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    exec_->add_node(node_);
    thread_ = std::thread([this]() {exec_->spin(); stopped_.store(true);});
    // Wait for spin() to claim the executor before returning. This says nothing about entities
    // having been collected yet -- the flag is set on entry, ahead of that -- it is what makes the
    // destructor's cancel able to land at all. stopped_ covers a context already shut down, where
    // spin() returns without ever spinning and this wait would never end.
    while (!exec_->is_spinning() && !stopped_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  ~SpinHelper()
  {
    stop();
  }

  /// Stop servicing the node and return once the spin thread is out of the executor. Idempotent, so
  /// a test can stop the spinner at the point in its own teardown where that has to happen --
  /// before destroying anything a still-in-flight callback can call into -- without also destroying
  /// the helper, and the destructor can then call it again.
  void stop()
  {
    // cancel() is documented as throwing when it cannot trigger the executor's guard condition, and
    // an exception leaving the destructor's call while the thread member is still joinable ends the
    // process. Retry rather than propagate: a cancel that never lands means spin() never returns,
    // so the join would not finish either.
    while (!stopped_.load()) {
      try {
        exec_->cancel();
      } catch (const std::exception &) {
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  /// Declared first so it is destroyed LAST: the spin thread is joined in stop() before any member
  /// goes, and this keeps the node alive for whatever the executor still holds on the way out.
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::atomic<bool> stopped_{false};
  std::thread thread_;
};

/// Poll `predicate` until it holds or `timeout` elapses; returns what it last answered.
///
/// Every wait in this suite is a precondition wait rather than a timing assertion, so they all want
/// the same shape: ask, sleep, ask again, and report whether it ever became true. Writing that loop
/// per site is how they drift into asserting on the deadline instead of the condition.
template<typename Predicate>
bool wait_until(
  Predicate predicate,
  std::chrono::milliseconds timeout = std::chrono::seconds(5),
  std::chrono::milliseconds poll = std::chrono::milliseconds(5))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return predicate();
    }
    std::this_thread::sleep_for(poll);
  }
  return true;
}

/// Issue one ~/changeState command and return its response, or nullptr if none arrived in time.
///
/// The response says the command was ACCEPTED, not that the transition finished (see
/// StateChange.srv), so a test that cares about the outcome still waits for the state -- which is
/// wait_for_state() below.
inline packml_msgs::srv::StateChange::Response::SharedPtr send_state_change(
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr client,
  int8_t command,
  std::chrono::seconds timeout = std::chrono::seconds(5))
{
  auto req = std::make_shared<packml_msgs::srv::StateChange::Request>();
  req->command = command;
  auto future = client->async_send_request(req);
  if (future.wait_for(timeout) == std::future_status::ready) {
    return future.get();
  }
  return nullptr;
}

/// Block until the machine reports `target`, or `timeout` elapses; returns whether it got there.
///
/// Templated on the holder rather than naming SMNode_new, so this header stays independent of the
/// node classes under test -- anything with `->getCurrentState()` works.
template<typename SmNodeT>
bool wait_for_state(
  const SmNodeT & sm_node,
  packml_sm::State target,
  std::chrono::milliseconds timeout)
{
  return wait_until(
    [&sm_node, target] {return sm_node->getCurrentState() == target;},
    timeout, std::chrono::milliseconds(10));
}

/// Block until the machine has reported the SAME state for `required_unchanged_ticks` consecutive
/// polls, or `window` elapses; returns the state it settled on, or nullopt if it never settled.
///
/// A different question from wait_for_state(), and not expressible as one: this asks whether the
/// machine stopped moving, without saying where. Which state a flood or a fault settles into is
/// genuinely order-dependent, so naming one up front would be asserting on scheduling -- but a
/// caller that wants to check the answer against a set of acceptable states needs it handed back,
/// which is why this returns the state rather than a bool.
template<typename SmNodeT>
std::optional<packml_sm::State> wait_for_settled_state(
  const SmNodeT & sm_node,
  std::chrono::milliseconds window,
  int required_unchanged_ticks = 20,
  std::chrono::milliseconds poll = std::chrono::milliseconds(50))
{
  const auto deadline = std::chrono::steady_clock::now() + window;
  packml_sm::State last_seen = sm_node->getCurrentState();
  int unchanged_ticks = 0;
  while (std::chrono::steady_clock::now() < deadline &&
    unchanged_ticks < required_unchanged_ticks)
  {
    std::this_thread::sleep_for(poll);
    const auto now_state = sm_node->getCurrentState();
    if (now_state == last_seen) {
      ++unchanged_ticks;
    } else {
      unchanged_ticks = 0;
      last_seen = now_state;
    }
  }
  if (unchanged_ticks >= required_unchanged_ticks) {
    return last_seen;
  }
  return std::nullopt;
}

/// Block until `count` heartbeats from `em_node_name` that leave the health gate OPEN have reached
/// `observer`'s participant, or `timeout` elapses; returns how many arrived.
///
/// Counts only beats the gate would accept -- `health.action` no worse than WARN, which is what
/// HealthMonitor::gate_block_reason() calls "unhealthy" -- because every caller uses this to
/// establish that a health-gated command will not be refused for a reason unrelated to its test.
/// A node publishing beats that carry an actionable fault satisfies "heartbeats are flowing" and
/// still leaves the gate shut, so counting those would report a precondition that does not hold.
///
/// Two beats by default, and the number is load-bearing. This subscription and the manager's own
/// live on the same node and are dispatched by the same executor in no defined order, so seeing
/// beat N here does not mean HealthMonitor has processed beat N; it takes a second beat to make
/// "the manager has processed at least one" reliable.
inline int wait_for_healthy_heartbeats(
  rclcpp::Node::SharedPtr observer,
  const std::string & em_node_name,
  int count = 2,
  std::chrono::milliseconds timeout = std::chrono::seconds(5))
{
  const std::string topic = "/" + em_node_name + "/" + std::string(packml_ros::kHeartbeatTopic);
  auto healthy_seen = std::make_shared<std::atomic<int>>(0);
  auto sub = observer->create_subscription<packml_msgs::msg::NodeHeartbeat>(
    topic, rclcpp::SensorDataQoS(),
    [healthy_seen](packml_msgs::msg::NodeHeartbeat::SharedPtr msg) {
      if (msg->health.action <= packml_msgs::msg::NodeHealth::WARN) {
        healthy_seen->fetch_add(1);
      }
    });

  wait_until([healthy_seen, count] {return healthy_seen->load() >= count;}, timeout);

  // A count short of the target has two very different causes and the caller's assertion message
  // can only guess at one of them. Nobody publishing this topic at all means the node name is wrong
  // or has drifted from the node that was actually created, which is not a timing problem and no
  // longer wait will fix.
  if (healthy_seen->load() < count && 0 == observer->count_publishers(topic)) {
    RCLCPP_ERROR_STREAM(observer->get_logger(),
      "Nothing publishes " << topic << " -- check the node name, not the timeout");
  }
  return healthy_seen->load();
}

/// Keeps the DeferredCompletions an Equipment Module was handed, KEYED BY STATE, so a test body
/// can resolve that module's in-flight goal for a particular state itself, from outside the
/// module.
///
/// Keyed by state, and the key is not optional. A module that defers every state accumulates
/// handles: the manager fans a goal out on every state change, including the one it boots into, so
/// a module can easily be holding an unreported STOPPED deferral from boot when a test wants to
/// resolve RESETTING. Reporting through "the latest handle" then resolves the wrong goal.
///
/// A handle is good for exactly one report and is TAKEN by report(), for the same reason. Leaving a
/// spent handle behind makes "a goal for this state has arrived" true forever, so a later visit to
/// the same state reports into the earlier, already-resolved goal, the real one is never answered,
/// and it rides out the completion timeout as a manager fault.
class PendingCompletion
{
public:
  /// Call from on_deferred_work(). Replaces any unreported handle FOR THAT STATE: a module that
  /// receives a second goal for the same state has moved on, and the older handle is exactly what
  /// must NOT resolve the newer goal.
  void accept(packml_ros::DeferredCompletion completion)
  {
    {
      std::lock_guard<std::mutex> lk(pending_mutex_);
      pending_.insert_or_assign(completion.state(), completion);
      ++accepted_[completion.state()];
    }
    arrived_.notify_all();
  }

  /// Block until a goal for `state` is held that has not been reported through yet.
  ///
  /// Use this, never `accepted(state) > 0`, to wait for the fan-out to reach the module: the count
  /// is cumulative and so is already true on any second visit to the same state.
  bool wait_for_accept(
    packml_sm::State state, std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    std::unique_lock<std::mutex> lk(pending_mutex_);
    return arrived_.wait_for(lk, timeout, [this, state] {return pending_.count(state) > 0;});
  }

  /// Resolve the goal held for `state`, waiting for one to arrive first, and take the handle.
  ///
  /// Waits rather than assuming, because the module only holds a handle once the manager's fan-out
  /// actually reached it, and how long that takes is a discovery-and-round-trip question rather
  /// than a fixed number of milliseconds. The default deadline is generous on purpose: this is a
  /// PRECONDITION wait, not a timing assertion.
  ///
  /// Returns whether the report reached a live goal. False means either that no handle ever arrived
  /// or that the goal behind it was already gone -- cancelled, timed out, or its node shutting
  /// down, in which case DeferredCompletion discards the report. A test asserting that it resolved
  /// something cannot tell those from success any other way.
  bool report(
    packml_sm::State state, bool success, int32_t error_code = 0, const std::string & message = "",
    std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    std::optional<packml_ros::DeferredCompletion> claimed;
    {
      std::unique_lock<std::mutex> lk(pending_mutex_);
      arrived_.wait_for(lk, timeout, [this, state] {return pending_.count(state) > 0;});
      const auto it = pending_.find(state);
      if (it == pending_.end()) {
        return false;
      }
      claimed = it->second;
      pending_.erase(it);
    }
    return claimed->report(success, error_code, message);
  }

  /// How many goals this module has been asked to do deferred work for, for `state`, over its whole
  /// life. Cumulative, so it answers "how many times" and never "has the current one arrived" --
  /// that is wait_for_accept().
  int accepted(packml_sm::State state) const
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    const auto it = accepted_.find(state);
    return it == accepted_.end() ? 0 : it->second;
  }

private:
  mutable std::mutex pending_mutex_;
  std::condition_variable arrived_;
  std::map<packml_sm::State, packml_ros::DeferredCompletion> pending_;
  std::map<packml_sm::State, int> accepted_;
};

/// Generate a unique node name to avoid conflicts between tests
inline std::string unique_node_name(const std::string & prefix)
{
  static std::atomic<int> counter{0};
  return prefix + "_" + std::to_string(counter.fetch_add(1));
}

}  // namespace packml_ros_test
