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

#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>

#include "packml_sm/common.hpp"

class PackmlNodeInterface;

namespace packml_ros
{

namespace detail
{

/// One wakeup shared by every deferral a node has in flight, and co-owned by the node itself.
///
/// Shared rather than per-deferral so that a node going away can wake all of its waiters with a
/// single notify, and co-owned (shared_ptr, not a node member) because a subclass's own work thread
/// can outlive the node: a handle held by that thread keeps this alive, so reporting into a
/// destroyed node's deferral is a live object doing nothing rather than a use-after-free.
struct CompletionSignal
{
  /// Guards every in-flight DeferralState's fields as well as shutting_down below.
  std::mutex deferrals_mutex;
  /// Signals anything that could end a deferred wait: a goal reported, or the node shutting down.
  std::condition_variable deferral_progress_cv;
  bool shutting_down{false};
};

/// One goal's deferred completion. Created per accepted goal that defers, and reachable only
/// through the single DeferredCompletion handed to that goal's on_deferred_work() -- which is what
/// makes a stale report harmless: an earlier goal's handle refers to an earlier goal's record, so
/// there is no shared slot for it to resolve the wrong wait through.
///
/// Every field except `signal` and `state` is guarded by signal->deferrals_mutex.
struct DeferralState
{
  DeferralState(std::shared_ptr<CompletionSignal> signal_in, packml_sm::State state_in)
  : signal(std::move(signal_in)), state(state_in) {}

  const std::shared_ptr<CompletionSignal> signal;
  const packml_sm::State state;

  bool reported{false};
  bool success{false};
  int32_t error_code{0};
  std::string message;

  /// Set once the waiting thread has stopped waiting on this record without a report (cancelled,
  /// timed out, or the node is going away). Only a diagnostic: a report arriving afterwards is
  /// already inert, and this is what lets it say so instead of vanishing.
  bool abandoned{false};
};

}  // namespace detail

/// The one way to resolve a deferred state transition: handed to on_deferred_work() for the goal it
/// belongs to, and carried by the subclass into whatever thread does that goal's work.
///
/// Copyable and cheap, so it can be captured by value into a lambda or stored on the module.
/// Deliberately has no public constructor -- a handle can only come from the interface, for the
/// goal the interface is actually waiting on.
///
/// A handle rather than a std::future returned from the hook, which would express the same
/// per-goal identity: a future obtained from std::async -- the idiom an implementor reaches for
/// first -- blocks in its own destructor until the task finishes, so dropping one on cancellation
/// or on node teardown would stall for the full length of the abandoned work. Reporting through a
/// handle leaves abandoned work running and unwaited-for, which is the whole point of abandoning
/// it.
class DeferredCompletion
{
public:
  /// Resolve this goal: `success` false aborts it, carrying error_code/message back to the manager.
  ///
  /// Safe to call at any time, including after the goal is gone -- a report for a goal nothing is
  /// waiting on is discarded with a warning, never applied to whatever goal came next. First report
  /// wins; a second one for the same goal is discarded the same way.
  ///
  /// Returns whether this report was applied. A caller that only reports has no use for it and may
  /// ignore it; one that asserts a report LANDED cannot tell a discarded report from an applied one
  /// any other way, since both leave the handle valid and neither throws.
  bool report(bool success, int32_t error_code = 0, const std::string & message = "") const
  {
    auto & deferral = *deferral_;
    {
      std::lock_guard<std::mutex> lk(deferral.signal->deferrals_mutex);
      if (deferral.abandoned) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"),
          "Deferred completion reported for " << to_string(deferral.state) << " after its goal was "
          "cancelled, timed out, or the node started shutting down -- discarding. The work this "
          "reports on kept running past the goal that asked for it; check whether it should be "
          "observing abandoned() and stopping early.");
        return false;
      }
      if (deferral.reported) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"),
          "Deferred completion for " << to_string(deferral.state) << " reported more than once -- "
          "keeping the first report and discarding this one.");
        return false;
      }
      deferral.reported = true;
      deferral.success = success;
      deferral.error_code = error_code;
      deferral.message = message;
    }
    deferral.signal->deferral_progress_cv.notify_all();
    return true;
  }

  /// True once nothing is waiting on this goal any more. Long-running work can poll this to stop
  /// early instead of finishing into a report that will be discarded.
  bool abandoned() const
  {
    std::lock_guard<std::mutex> lk(deferral_->signal->deferrals_mutex);
    return deferral_->abandoned;
  }

  /// The state this goal is completing, so work shared across several states can tell them apart
  /// without the caller having to thread it through separately.
  packml_sm::State state() const {return deferral_->state;}

private:
  friend class ::PackmlNodeInterface;

  explicit DeferredCompletion(std::shared_ptr<detail::DeferralState> deferral)
  : deferral_(std::move(deferral)) {}

  std::shared_ptr<detail::DeferralState> deferral_;
};

}  // namespace packml_ros
