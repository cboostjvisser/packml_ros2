// Copyright (c) 2017 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#pragma  once

// #include <packml_msgs/msg/detail/status__struct.hpp>
// #include <packml_msgs/srv/detail/mode_change__struct.hpp>
// #include <packml_msgs/srv/detail/mode_transition__struct.hpp>
// #include <packml_msgs/srv/detail/state_transition__struct.hpp>
#include <qglobal.h>
#include <rmw/qos_profiles.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <condition_variable>
#include <optional>
#include <stop_token>
#include <thread>

#include "packml_sm/modes_config.hpp"
// is_known_mode() -- what on_change_mode() validates against when the deployment declared no
// modes of its own. Deliberately NOT a generated modes header: including one here would impose
// its mode values on every consumer of this header and collide with the consumer's own.
#include "packml_sm/modes_registry.hpp"
#include "packml_ros/detached_worker_gate.hpp"
#include "packml_ros/deferred_completion.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <packml_sm/common.hpp>
#include <packml_sm/state_machine.hpp>
#include <packml_ros/transition_guard.hpp>

#include <packml_msgs/action/state_transition.hpp>
#include <packml_msgs/srv/mode_transition.hpp>
#include <packml_msgs/msg/status.hpp>

#include <packml_msgs/msg/state.hpp>
#include <packml_msgs/msg/node_health.hpp>
#include <packml_msgs/msg/node_heartbeat.hpp>
#include <packml_msgs/msg/alarm.hpp>
#include <packml_msgs/srv/all_status.hpp>
#include <packml_msgs/srv/mode_change.hpp>
#include <packml_msgs/srv/state_change.hpp>
#include "packml_ros/health_monitor.hpp"
#include "packml_ros/completion_tracker.hpp"
#include "packml_ros/error_catalog.hpp"

namespace packml_ros {
  inline packml_sm::TransitionCmd to_transition_cmd(packml_msgs::srv::StateChange::Request::_command_type command)
  {
    switch (command) {
      case packml_msgs::srv::StateChange::Request::ABORT:
        return packml_sm::TransitionCmd::ABORT;
      case packml_msgs::srv::StateChange::Request::STOP:
        return packml_sm::TransitionCmd::STOP;
      case packml_msgs::srv::StateChange::Request::CLEAR:
        return packml_sm::TransitionCmd::CLEAR;
      case packml_msgs::srv::StateChange::Request::HOLD:
        return packml_sm::TransitionCmd::HOLD;
      case packml_msgs::srv::StateChange::Request::RESET:
        return packml_sm::TransitionCmd::RESET;
      case packml_msgs::srv::StateChange::Request::START:
        return packml_sm::TransitionCmd::START;
      case packml_msgs::srv::StateChange::Request::SUSPEND:
        return packml_sm::TransitionCmd::SUSPEND;
      case packml_msgs::srv::StateChange::Request::UNHOLD:
        return packml_sm::TransitionCmd::UNHOLD;
      case packml_msgs::srv::StateChange::Request::UNSUSPEND:
        return packml_sm::TransitionCmd::UNSUSPEND;
      default:
        return packml_sm::TransitionCmd::NO_COMMAND;
    }
  }

}  // namespace packml_ros

class PackmlNodeInterface
{
public:
  using StateTransitionAction = packml_msgs::action::StateTransition;
  using StateTransitionGoalHandle = rclcpp_action::ServerGoalHandle<StateTransitionAction>;

  /// Do not return while a detached deferred-completion thread is still inside this object.
  ///
  /// begin_transition() hands a goal off to a detached thread that then uses this object for as
  /// long as deferred_completion_timeout_ms allows -- configurable, and realistically seconds.
  /// Nothing joined it, so a node destroyed inside that window (a launch shutdown, a Ctrl-C, a
  /// supervisor restart) left a live thread reading freed members and resolving a goal through a
  /// destroyed action server.
  ///
  /// Two steps, and both are needed: the flag plus the notify make an in-flight wait give up now
  /// instead of riding out its timeout, and the gate is what actually holds this destructor until
  /// the thread has left. Only the flag would still race; only the gate would stall teardown for
  /// the full timeout.
  ///
  /// Virtual because subclasses are what this class exists to be -- deleting one through a base
  /// pointer was undefined before this declaration existed. Worth knowing: the worker touches only
  /// this class's own members, so by the time a subclass destructor has already run, waiting here
  /// is still safe.
  virtual ~PackmlNodeInterface()
  {
    {
      std::lock_guard<std::mutex> lk(completion_signal_->deferrals_mutex);
      completion_signal_->shutting_down = true;
    }
    completion_signal_->deferral_progress_cv.notify_all();
    deferred_workers_.await_idle();
  }

private:
  /// Action server for ~/packml_state_transition — replaces a plain service because the
  /// RESULT is not "accepted" (that happens fast, same as before) but "this node's own
  /// commanded work for the requested state has finished (or failed)", which can take far
  /// longer than an accept decision. The action's own goal id is the manager's correlation
  /// mechanism for "an answer to the cycle I'm currently waiting on" -- no separate
  /// generation/sequence field is carried on the wire.
  rclcpp_action::Server<StateTransitionAction>::SharedPtr trans_action_server_;

  /**
  * @brief Pointer for state and elapsed time status update service server
  */
  rclcpp::Subscription<packml_msgs::msg::Status>::SharedPtr status_sub_;

  rclcpp::Service<packml_msgs::srv::ModeTransition>::SharedPtr mode_server_;

  rclcpp::Service<packml_msgs::srv::AllStatus>::SharedPtr status_server_;

  /// Shared protocol logic — single source of truth for both C++ and Python.
  packml_ros::PackmlNodeProtocol protocol_;

  /// Heartbeat publisher — started by init(), publishes NodeHeartbeat at heartbeat_interval_ms.
  rclcpp::Publisher<packml_msgs::msg::NodeHeartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  /// Serializes post_event() against the periodic heartbeat timer: HeartbeatState's
  /// latch_mutex_ (transition_guard.hpp) only protects the latch fields from a torn
  /// read, not the read-decide-publish sequence around it. Without this, a timer tick
  /// that already snapshotted a clear latch can be overtaken by a full post_event(fault)
  /// — set_latch() + publish() — and then publish its stale HEALTHY at a HIGHER
  /// sequence number, which the manager reads as an immediate (false) clear of the
  /// fault it just raised.
  /// SYNC: keep in lockstep with packml_ros_py/packml_ros_py/packml_node.py's
  /// `_heartbeat_lock` — both guard the identical race on the shared protocol_.heartbeat.
  std::mutex heartbeat_publish_mutex_;

  /// Deferred-completion synchronization: the wakeup shared by every deferral this node has in
  /// flight (see CompletionSignal). Each deferral's own payload -- reported/success/error_code/
  /// message -- lives in that goal's DeferralState instead of here, which is what keeps one goal's
  /// completion out of another's wait: only the goal's own handle can reach its own record.
  ///
  /// A single shared slot was enough while only one ~/packml_state_transition goal ran at a time
  /// (the manager's wait for the previous one resolves before it sends the next), but "one at a
  /// time" is not "one ever": a cancelled goal's work can still be running, and still reporting,
  /// while its successor defers. Keyed by nothing but the state NAME, that late report resolved the
  /// successor's wait -- and a state name recurs (RESETTING via ABORT->CLEAR->RESET, HOLDING via a
  /// full cycle), so the name matched and the manager advanced on work that had not finished.
  const std::shared_ptr<packml_ros::detail::CompletionSignal> completion_signal_{
    std::make_shared<packml_ros::detail::CompletionSignal>()};

  /// EM-side safety net: how long a node that defers completion for a state will wait for
  /// its own on_deferred_work() to report before giving up on itself. Default, configurable
  /// via the deferred_completion_timeout_ms parameter (see init()); some states genuinely
  /// need a different bound than others (a homing RESETTING vs. a near-instant ABORTING), so
  /// deferred_completion_timeout_ms_by_state_ (also populated in init(), one entry per state
  /// this node's own defers_completion() returns true for) is checked first.
  int deferred_completion_timeout_ms_{30000};
  std::map<packml_sm::State, int> deferred_completion_timeout_ms_by_state_;

  /// The detached wait_for_deferred_completion() threads currently inside this object. Read the
  /// destructor for what this is for; in short, nothing joined those threads and they use this
  /// object for as long as the timeout above allows, so destroying the node under one of them used
  /// freed memory. The flag that tells them to stop waiting is completion_signal_->shutting_down.
  packml_ros::DetachedWorkerGate deferred_workers_;

  /// Update this node's own view of its current state the moment it locally commits
  /// to succeeding a transition, rather than waiting for the manager's status-topic
  /// echo to arrive. waiting_for_state_ is otherwise only cleared by on_status_update()
  /// (see TransitionGuard::request_state()'s "already in progress" rejection) — when
  /// the manager moves through two coordinated states in quick succession (e.g. the
  /// IDLE waypoint on the way to STARTING), this node's own next goal can arrive
  /// before that echo does, and get rejected as a conflicting in-flight request even
  /// though nothing actually conflicts: this node already knows, locally, that it
  /// just finished the previous one. Reuses on_status_update() (rather than a new
  /// TransitionGuard method) since the effect needed -- clear the waiting flag, adopt
  /// the new current_state_ -- is exactly what it already does for a real echo; a
  /// later, genuinely-redundant echo of the same state is then just a no-op.
  void mark_state_locally_reached(packml_sm::State state)
  {
    protocol_.transitions.on_status_update(state, protocol_.transitions.current_mode());
  }

  /// Runs SYNCHRONOUSLY on the ROS executor thread from handle_accepted (below) — deliberately
  /// NOT on a spawned thread. This matters for a real race, not just style: the manager fans out
  /// the transition request BEFORE publishing the latched packml_status update specifically so an
  /// EM sees "you're being asked to transition" before "the machine is now in that state" (see
  /// packml_ros-new.hpp's on_state_changed) — an EM that saw status FIRST would hit the
  /// already_there shortcut and skip this node's own on_state_trans_req()/defers_completion()
  /// entirely. Deferring this decision onto a spawned thread reopens that race: thread
  /// creation/scheduling latency is enough for the status topic — delivered on this same executor
  /// thread's queue — to be processed first. Only the (potentially long) deferred WAIT below is
  /// moved onto its own thread; the entire accept/reject/instant-complete decision is made here,
  /// immediately, with no thread hop at all.
  void begin_transition(const std::shared_ptr<StateTransitionGoalHandle> & goal_handle)
  {
    const auto state = static_cast<packml_sm::State>(goal_handle->get_goal()->state.val);
    auto result = std::make_shared<StateTransitionAction::Result>();

    // The arm this claim carries is released by its destructor, so every exit below -- the
    // already-there shortcut, the rejection, the node refusing the switch, the immediate
    // success, and both outcomes of the deferred wait -- gives it back without having to
    // remember to. Nothing else releases it: a status echo cannot, because it has no way to
    // know whose request it would be ending.
    auto claimed = protocol_.transitions.claim_state(state);
    const auto & guard_result = claimed.result;

    if (guard_result.already_there) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "Node already in state: " << to_string(state));
      result->success = true;
      goal_handle->succeed(result);
      return;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "Node State changing to: " << to_string(state));

    if (!guard_result.error.empty()) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), guard_result.error);
    }

    if (!guard_result.accepted) {
      result->success = false;
      result->error_code = StateTransitionAction::Result::INVALID_STATE_REQUEST;
      result->message = guard_result.error;
      goal_handle->abort(result);
      return;
    }

    if (!on_state_trans_req(state)) {
      const std::string error_string = "Node did not approve state switch";
      RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), error_string);
      result->success = false;
      result->message = error_string;
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("packml_ros"), "Node approved state switch");

    // defers_completion() is a static, state-shape answer (does this node defer THIS state at
    // all), not a per-request decision -- so asking it here, after the accept, costs nothing.
    if (!defers_completion(state)) {
      result->success = true;
      goal_handle->succeed(result);
      mark_state_locally_reached(state);
      return;
    }

    // This goal's own completion record, and the only handle that can reach it. Nothing else
    // shares it, so a report from an earlier goal's still-running work has no path to this wait.
    auto deferral = std::make_shared<packml_ros::detail::DeferralState>(completion_signal_, state);

    // Synchronously on this (ROS executor) thread, for the same reason the accept decision above
    // runs here: this is what dispatches the node's own commanded work, and the work has to be
    // under way before the manager's status echo can arrive. Reporting from inside the hook is
    // fine -- the record already exists, so an instant report is simply already there when the
    // wait below starts.
    on_deferred_work(state, packml_ros::DeferredCompletion(deferral));

    // Deferred: hand off to a background thread that only waits for that report — the one part of
    // this that can legitimately take a long time.
    //
    // The arm moves with the work. The transition is still in flight until that thread resolves
    // the goal, so releasing it here -- when this frame returns -- would let a second goal in
    // while the first is still waiting.
    // Counted BEFORE the spawn, not inside the new thread: teardown slipping between the two is
    // exactly the window ~PackmlNodeInterface() has to be able to close.
    deferred_workers_.enter();
    std::thread(
      &PackmlNodeInterface::wait_for_deferred_completion, this, goal_handle, result,
      std::move(claimed.arm), std::move(deferral)).detach();
  }

  void wait_for_deferred_completion(
    const std::shared_ptr<StateTransitionGoalHandle> goal_handle,
    std::shared_ptr<StateTransitionAction::Result> result,
    // Named but never read: this parameter exists so the arm's lifetime spans this function, and
    // its destructor is what releases it on every exit below.
    [[maybe_unused]] packml_ros::InFlightToken arm,
    std::shared_ptr<packml_ros::detail::DeferralState> deferral)
  {
    // Every return below leaves the gate, which is what lets ~PackmlNodeInterface() know this
    // thread is no longer inside the object.
    packml_ros::DetachedWorkerGate::Scope worker_scope(deferred_workers_);

    const auto state = deferral->state;
    const auto override_it = deferred_completion_timeout_ms_by_state_.find(state);
    const int timeout_ms = override_it != deferred_completion_timeout_ms_by_state_.end()
      ? override_it->second
      : deferred_completion_timeout_ms_;

    // Bounded so a subclass that never reports can't hang the goal forever. A report or the node
    // shutting down both wake this wait through completion_signal_->deferral_progress_cv.
    //
    // Cancellation cannot be woken for, which is why this is a re-checking loop and not one
    // wait_for. handle_cancel() runs BEFORE rclcpp_action moves the goal to CANCELING, so its
    // notify arrives while is_canceling() is still false, and nothing notifies again once the
    // transition does happen. A single wait_for therefore slept through every cancel and gave the
    // goal back only when the timeout expired -- 30 s by default. The interval below is a
    // re-check cadence, not a deadline: a report or a shutdown still wakes the wait instantly, and
    // the overall bound is still timeout_ms. Same shape, and for the same reason, as
    // StateMachine::postCommand()'s liveness re-check.
    static constexpr auto kCancelRecheckInterval = std::chrono::milliseconds(50);
    bool reported = false;
    bool shutting_down = false;
    {
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
      std::unique_lock<std::mutex> lk(completion_signal_->deferrals_mutex);
      while (!deferral->reported && !goal_handle->is_canceling() &&
        !completion_signal_->shutting_down)
      {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
          break;
        }
        completion_signal_->deferral_progress_cv.wait_for(
          lk, std::min<std::chrono::steady_clock::duration>(
            kCancelRecheckInterval, deadline - now));
      }

      reported = deferral->reported;
      shutting_down = completion_signal_->shutting_down;
      if (reported) {
        result->success = deferral->success;
        result->error_code = deferral->error_code;
        result->message = deferral->message;
      } else {
        // Nothing waits on this goal from here on. Marking it says so to a report that arrives
        // later -- work that kept running past its own cancellation -- which is discarded with a
        // warning instead of silently going nowhere. It cannot reach any OTHER goal's wait: this
        // record is the only thing its handle refers to.
        deferral->abandoned = true;
      }
    }

    // Teardown started while this wait was in flight. The destructor is already blocked waiting
    // for this thread, so riding out the rest of the configured timeout would hold up the whole
    // process; give the goal a terminal state and get out. Checked before the branches below
    // because "not reported" is true here too, and "timed out" would be the wrong thing to tell
    // a client about a node that is going away.
    if (shutting_down && !reported && !goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Equipment Module shutting down while deferring completion";
      goal_handle->abort(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
    if (!reported) {
      result->success = false;
      result->message =
        "Deferred completion for " + to_string(state) + " timed out with no report";
      RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), result->message);
      goal_handle->abort(result);
      return;
    }
    if (result->success) {
      goal_handle->succeed(result);
      mark_state_locally_reached(state);
    } else {
      goal_handle->abort(result);
    }
  }

public:
  /// Returns the current health of this Equipment Module.
  /// Override to report real conditions; base implementation returns HEALTHY / NONE.
  ///
  /// IMPORTANT: This method MUST be a pure getter — do NOT call post_event()
  /// from within it.  For event-driven faults, call post_event() (which latches
  /// the state); while a latch is active the periodic publisher repeats the
  /// latched health and does NOT call this method, so you do not need to mirror
  /// the fault here.
  virtual packml_msgs::msg::NodeHealth get_health_status()
  {
    packml_msgs::msg::NodeHealth h;
    h.status = packml_msgs::msg::NodeHealth::HEALTHY;
    h.action = packml_msgs::msg::NodeHealth::NONE;
    return h;
  }

  /// Immediately publish a heartbeat with the given health state, bypassing the
  /// periodic timer.  Use for safety-critical events (e.g. E-stop) where waiting
  /// up to heartbeat_interval_ms for the next tick is unacceptable.
  ///
  /// The event is *latched*: the periodic publisher repeats this health on every
  /// subsequent tick (instead of calling get_health_status()), so a transient
  /// getter cannot flap the alarm/state. Posting a healthy/NONE event clears the
  /// latch and resumes get_health_status()-driven publishing.
  void post_event(const packml_msgs::msg::NodeHealth & health)
  {
    if (!heartbeat_publisher_) {
      return;  // init() not yet called
    }
    std::lock_guard<std::mutex> lk(heartbeat_publish_mutex_);  // see heartbeat_publish_mutex_
    if (health.action == packml_msgs::msg::NodeHealth::NONE) {
      protocol_.heartbeat.clear_latch();
    } else {
      protocol_.heartbeat.set_latch(
        health.status, health.action, health.error_code, health.message, health.instance_id);
    }
    heartbeat_publisher_->publish(make_heartbeat(health));
  }

  /// Override to return true for states where your own commanded work finishes later than the
  /// transition is approved -- the states you implement on_deferred_work() for. The default, for
  /// every state, is to complete the moment on_state_trans_req() approves the transition. This is
  /// how a node declares -- in its OWN code, not manager-side configuration -- that it participates
  /// in coordinated completion for a given state.
  ///
  /// Must be a static, state-shape answer (does this node defer THIS state at all), not a per-
  /// request decision: init() probes it once per state to decide which
  /// deferred_completion_timeout_ms.<STATE> overrides to declare.
  virtual bool defers_completion(packml_sm::State state)
  {
    (void)state;
    return false;
  }

  /// Start this node's own commanded work for `state`, and resolve `completion` when that work has
  /// genuinely finished or definitively failed. Called once per accepted goal, for exactly the
  /// states defers_completion() returns true for.
  ///
  /// Runs on the ROS executor thread and must not block: put long work on a thread of your own and
  /// capture `completion` into it (it is copyable and safe to hold for as long as you need).
  /// Reporting inline, before returning, is also fine for work that is already done.
  ///
  /// The handle is per-goal, which is the point of the shape: identity travels with the work
  /// instead of being re-derived when the report arrives. Work that outlives its own goal -- a
  /// homing motion already issued to hardware, still running after an operator's ABORT cancelled
  /// the goal that asked for it -- reports into a record nothing is waiting on, and can see that
  /// coming via completion.abandoned(). It cannot resolve whatever goal is deferring by then, not
  /// even when that goal is for the same state under a different name for the same press of the
  /// same button.
  virtual void on_deferred_work(packml_sm::State state, packml_ros::DeferredCompletion completion)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("packml_ros"),
      "defers_completion(" << to_string(state) << ") returned true but on_deferred_work() is not "
      "implemented, so nothing can ever complete this state. Implement it, or stop deferring "
      "this state.");
    completion.report(
      false, StateTransitionAction::Result::INVALID_STATE_REQUEST,
      "Node defers completion for " + to_string(state) +
      " but implements no on_deferred_work()");
  }

  protected:

  /// Pause or resume heartbeat publishing.  Protected: intended for derived test/demo
  /// Equipment Modules to simulate a crashed or silent node (no heartbeat = timeout in
  /// the HealthMonitor) — not part of the public API.
  void set_heartbeat_active(bool active) { protocol_.heartbeat.set_active(active); }

  /// Assemble a NodeHeartbeat with the standard header (node_name, next sequence,
  /// interval) and the given health. Single source for both post_event() and the
  /// periodic timer; calls next_sequence() exactly once per published heartbeat.
  packml_msgs::msg::NodeHeartbeat make_heartbeat(const packml_msgs::msg::NodeHealth & health)
  {
    packml_msgs::msg::NodeHeartbeat hb;
    hb.node_name = protocol_.heartbeat.node_name();
    hb.sequence_number = protocol_.heartbeat.next_sequence();
    hb.heartbeat_interval_ms = protocol_.heartbeat.interval_ms();
    hb.health = health;
    return hb;
  }


  inline auto get_current_packml_mode() const -> packml_sm::ModeType { return protocol_.transitions.current_mode(); }

  inline auto get_current_packml_state() const -> packml_sm::State { return protocol_.transitions.current_state(); }

  inline bool is_switching_mode() const { return protocol_.transitions.is_switching_mode(); }

  inline bool is_switching_state() const { return protocol_.transitions.is_switching_state(); }

  template <typename NodeT>
  inline void init(std::shared_ptr<NodeT> node) {

    auto onModeTransReq =
      [this](const std::shared_ptr<packml_msgs::srv::ModeTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::ModeTransition::Response> res)-> void {
            auto mode = static_cast<packml_sm::ModeType>(req->mode.val);
            auto result = protocol_.transitions.request_mode(mode);

            if (result.already_there) {
              RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "Node already in mode: " << packml_sm::to_string(mode));
              res->success = true;
              return;
            }

            RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "Node Mode changing to: " << packml_sm::to_string(mode));

            if (!result.error.empty()) {
              RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), result.error);
            }

            if (result.accepted && on_mode_trans_req(mode)) {
              RCLCPP_INFO(rclcpp::get_logger("packml_ros"), "Node approved mode switch");
              res->success = true;
            } else {
              std::string error_string = "Node did not approve mode switch";
              res->message = error_string;
              res->success = false;
              RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), error_string);
            }
        };

    auto onStatusChanged =
      [this](const packml_msgs::msg::Status& status) -> void {
        auto state = static_cast<packml_sm::State>(status.state.val);
        auto mode = static_cast<packml_sm::ModeType>(status.mode.val);

        if (protocol_.transitions.on_status_update(state, mode)) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"),
            "Status changed - State: " << to_string(state) << ", Mode: " << packml_sm::to_string(mode));
          on_status_changed();
        }
      };

    // --- State-transition action server ---
    if (!node->has_parameter(packml_ros::kParamDeferredCompletionTimeoutMs)) {
      node->template declare_parameter<int>(packml_ros::kParamDeferredCompletionTimeoutMs, 30000);
    }
    deferred_completion_timeout_ms_ =
      node->get_parameter(packml_ros::kParamDeferredCompletionTimeoutMs).as_int();

    // Per-state override: declared only for states this node's own defers_completion()
    // actually returns true for (a static, state-shape answer -- see that method's own
    // doc comment), so a node that defers nothing, or only one or two states, does not
    // clutter its parameter list with overrides it will never use. Named
    // "deferred_completion_timeout_ms.<STATE NAME>", defaulting to the value above.
    static constexpr packml_sm::State kAllStates[] = {
      packml_sm::State::UNDEFINED, packml_sm::State::CLEARING, packml_sm::State::STOPPED,
      packml_sm::State::STARTING, packml_sm::State::IDLE, packml_sm::State::SUSPENDED,
      packml_sm::State::EXECUTE, packml_sm::State::STOPPING, packml_sm::State::ABORTING,
      packml_sm::State::ABORTED, packml_sm::State::HOLDING, packml_sm::State::HELD,
      packml_sm::State::UNHOLDING, packml_sm::State::SUSPENDING, packml_sm::State::UNSUSPENDING,
      packml_sm::State::RESETTING, packml_sm::State::COMPLETING, packml_sm::State::COMPLETE,
    };
    for (const auto state : kAllStates) {
      if (!defers_completion(state)) {
        continue;
      }
      const std::string state_timeout_param =
        std::string(packml_ros::kParamDeferredCompletionTimeoutMs) + "." +
        packml_sm::to_string(state);
      if (!node->has_parameter(state_timeout_param)) {
        node->template declare_parameter<int>(state_timeout_param, deferred_completion_timeout_ms_);
      }
      deferred_completion_timeout_ms_by_state_[state] =
        node->get_parameter(state_timeout_param).as_int();
    }

    auto handle_goal =
      [this](const rclcpp_action::GoalUUID &,
        std::shared_ptr<const StateTransitionAction::Goal> goal) -> rclcpp_action::GoalResponse {
        // Pure admission control -- always accept. Business-logic accept/reject (the
        // on_state_trans_req() hook) happens in the execute thread below as a normal, fast
        // succeeded/aborted result, so a rejection still carries a proper Result message
        // (error_code/message) rather than the goal simply vanishing with no result at all.
        //
        // admit_state() runs here, not in handle_accepted below: this callback's GoalResponse is
        // what the SendGoal service response is built from, and that response cannot reach the
        // manager (unblocking its own bounded acceptance wait, which gates publish_status())
        // before this call returns. So current_state_ here is still this goal's PRE-echo view,
        // and it is the only moment at which "am I already in the requested state" can be
        // answered without the answer being poisoned by the very request being answered.
        protocol_.transitions.admit_state(
          static_cast<packml_sm::State>(goal->state.val));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel =
      [this](const std::shared_ptr<StateTransitionGoalHandle> &) -> rclcpp_action::CancelResponse {
        // Wake a deferred wait rather than leaving it to its own timeout. This notify is an
        // OPTIMISATION and cannot be the mechanism, which is why wait_for_deferred_completion()
        // re-checks on an interval instead of trusting it.
        //
        // rclcpp_action calls this callback FIRST and only transitions the goal to CANCELING
        // afterwards (Server::call_handle_cancel_callback runs handle_cancel_, then
        // goal_handle->_cancel_goal(); rcl_action_process_cancel_request explicitly does not change
        // goal state). So is_canceling() is still false for the duration of this call, and a wait
        // woken here re-evaluates a predicate that is still entirely false and parks itself again.
        // There is no second notification: without the interval re-check that wait would ride out
        // the whole deferred_completion_timeout_ms.
        completion_signal_->deferral_progress_cv.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted =
      [this](const std::shared_ptr<StateTransitionGoalHandle> & goal_handle) {
        // Synchronous on this (ROS executor) thread — see begin_transition()'s own comment for
        // why: the fan-out-before-status ordering the manager relies on depends on it. Only a
        // genuinely deferred completion spawns a background thread, and only for the wait.
        begin_transition(goal_handle);
      };

    trans_action_server_ = rclcpp_action::create_server<StateTransitionAction>(
      node, "~/" + std::string(packml_ros::kStateTransitionAction),
      handle_goal, handle_cancel, handle_accepted);

    mode_server_ = node->template create_service<packml_msgs::srv::ModeTransition>("~/" + std::string(packml_ros::kModeTransitionService), onModeTransReq);
    // Match the latched status publisher (TRANSIENT_LOCAL + RELIABLE, see where
    // status_pub_ is created) so a module that (re)starts after the packml node has
    // already published immediately receives the retained current state instead of
    // waiting for the next transition.
    status_sub_ = node->template create_subscription<packml_msgs::msg::Status>(
      packml_ros::kStatusTopic, rclcpp::QoS(1).transient_local().reliable(), onStatusChanged);

    // --- Heartbeat publisher ---
    // Declare parameter so callers can override via YAML or command line.
    if (!node->has_parameter(packml_ros::kParamHeartbeatIntervalMs)) {
      node->template declare_parameter<int>(packml_ros::kParamHeartbeatIntervalMs, 1000);
    }
    const int interval_ms_param = node->get_parameter(packml_ros::kParamHeartbeatIntervalMs).as_int();
    const uint32_t interval_ms =
      static_cast<uint32_t>(interval_ms_param > 0 ? interval_ms_param : 1000);

    protocol_.heartbeat.init(std::string(node->get_name()), interval_ms);

    // Sensor-style QoS (best-effort, keep-last): heartbeats are periodic liveness
    // telemetry — only the latest matters, and a dropped beat is fine. The manager
    // subscription MUST use the same profile or QoS-incompatibility silently drops it.
    heartbeat_publisher_ = node->template create_publisher<packml_msgs::msg::NodeHeartbeat>(
      "~/" + std::string(packml_ros::kHeartbeatTopic), rclcpp::SensorDataQoS());

    heartbeat_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      [this]() {
        if (!protocol_.heartbeat.is_active()) {
          return;  // silent mode: timer ticks but no message is published
        }
        packml_msgs::msg::NodeHealth health;
        // Locking makes snapshot -> getter -> publish atomic against a concurrent
        // post_event() (see heartbeat_publish_mutex_); the snapshot itself is
        // additionally torn-read-safe via HeartbeatState's own latch mutex.
        std::lock_guard<std::mutex> lk(heartbeat_publish_mutex_);
        const auto latch = protocol_.heartbeat.latch_snapshot();
        if (latch.active) {
          // A post_event() fault is latched — repeat it instead of polling the getter.
          health.status     = latch.status;
          health.action     = latch.action;
          health.error_code = latch.error_code;
          health.message    = latch.message;
          health.instance_id = latch.instance_id;
        } else {
          health = get_health_status();
        }
        heartbeat_publisher_->publish(make_heartbeat(health));
      });

    RCLCPP_INFO(rclcpp::get_logger("packml_ros"), "Services created!");
  }

  virtual bool on_state_trans_req(packml_sm::State switching_state) = 0;

  virtual bool on_mode_trans_req(packml_sm::ModeType switching_mode) = 0;

  virtual void on_status_changed() = 0;
};

class PackmlClientInterface {
  public:
  rclcpp_action::Client<packml_msgs::action::StateTransition>::SharedPtr state_tr_client;
  rclcpp::Client<packml_msgs::srv::ModeTransition>::SharedPtr mode_tr_client;
  rclcpp::Subscription<packml_msgs::msg::Status>::SharedPtr status_sub;

  /// Clients live in the node's default callback group: fan-out responses arrive as
  /// asynchronous callbacks delivered by whatever executor spins the manager node. No
  /// per-client callback group or separately spun executor is needed, because nothing blocks
  /// inside a service callback waiting for those responses.
  PackmlClientInterface(std::string name, rclcpp::Node::SharedPtr parent_node) {
    auto mode_tr_service_name = name + "/" + packml_ros::kModeTransitionService;
    mode_tr_client = parent_node->create_client<packml_msgs::srv::ModeTransition>(mode_tr_service_name);

    auto state_tr_action_name = name + "/" + packml_ros::kStateTransitionAction;
    state_tr_client = rclcpp_action::create_client<packml_msgs::action::StateTransition>(
      parent_node, state_tr_action_name);
  }
};

/// Fully-qualified names of publishers on `topic` other than `own_publisher`.
///
/// The manager owns the status topic in the sense that it is the only thing meant to write it, but
/// packml_status is a bare global name with nothing enforcing that. A second publisher -- a
/// duplicated launch entry, two managers started against one machine -- makes an Equipment Module
/// adopt a state nobody commanded, and can make it shortcut coordinated work it never did on the
/// grounds that it is "already there". Nothing about that is visible from either node's own logs,
/// which is what this exists to change; it is a report, not a defence, and a deployment that needs
/// the topic to be genuinely unwritable wants secure ROS instead.
///
/// SELF IS EXCLUDED BY ENDPOINT GID, NOT BY NODE NAME, and the distinction is the whole point.
/// Matching on (node name, namespace) also excludes a second node that happens to share this one's
/// name and namespace -- which is exactly a duplicated launch entry, the first case named above and
/// the likeliest one in practice. ROS 2 permits duplicate node names, so that filter was blind to
/// the scenario the function exists for. A GID identifies the individual endpoint.
///
/// Returns an empty vector on a graph-query failure, which reads the same as "no duplicates". That
/// is the right direction for a diagnostic: a discovery hiccup must not produce a warning about a
/// second manager that does not exist.
namespace packml_ros {
inline std::vector<std::string> foreign_publishers_on(
  const rclcpp::Node & node,
  const std::string & topic,
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & own_gid)
{
  std::vector<rclcpp::TopicEndpointInfo> publishers;
  try {
    publishers = node.get_publishers_info_by_topic(topic);
  } catch (const std::exception &) {
    return {};
  }

  std::vector<std::string> others;
  for (const auto & publisher : publishers) {
    if (publisher.endpoint_gid() == own_gid) {
      continue;
    }
    const std::string ns = publisher.node_namespace();
    others.push_back((ns == "/" ? ns : ns + "/") + publisher.node_name());
  }
  return others;
}
}  // namespace packml_ros

class PackmlManagerInterface
{
  /// Client name and client interface object
  std::map<std::string, std::shared_ptr<PackmlClientInterface>> client_map_;

  rclcpp::Service<packml_msgs::srv::ModeChange>::SharedPtr mode_server_;
  rclcpp::Service<packml_msgs::srv::StateChange>::SharedPtr state_server_;

  /// One state command accepted but not yet evaluated. Carries the service and request id because
  /// the response is sent later, from the command worker -- see on_change_state(). Both are null for
  /// a command nobody is waiting on an answer for: a health-raised action (fire_packml_action()) is
  /// queued the same way and through the same FIFO, so operator commands and health actions cannot
  /// reach the machine out of the order they were raised in.
  struct QueuedCommand
  {
    std::shared_ptr<rclcpp::Service<packml_msgs::srv::StateChange>> service;
    std::shared_ptr<rmw_request_id_t> request_id;
    packml_sm::TransitionCmd command{packml_sm::TransitionCmd::NO_COMMAND};
  };

  /// Depth at which the backlog is worth telling an operator about. Not a limit -- see
  /// on_change_state() for why the queue has none.
  static constexpr size_t kCommandBacklogWarnDepth = 16;

  /// Answer one deferred ~/changeState request, surviving a caller that is no longer there.
  ///
  /// `send_response()` throws on any middleware failure other than a timeout, and a client that
  /// vanished between issuing a command and being answered is an ordinary event rather than a
  /// manager fault. This mostly runs on the command worker, which has no exception handler above
  /// it, so an escaping throw would take the whole process down.
  static void send_command_response(
    const std::shared_ptr<rclcpp::Service<packml_msgs::srv::StateChange>> & service,
    rmw_request_id_t & request_id,
    packml_msgs::srv::StateChange::Response & res)
  {
    try {
      service->send_response(request_id, res);
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"),
        "Could not deliver the answer to a state command: " << e.what() <<
        " -- the caller is most likely gone");
    }
  }

  std::mutex command_queue_mutex_;
  std::condition_variable command_queue_cv_;
  std::deque<QueuedCommand> command_queue_;
  bool command_worker_stopping_{false};
  std::thread command_worker_;
  rclcpp::Service<packml_msgs::srv::AllStatus>::SharedPtr status_server_;

  rclcpp::Publisher<packml_msgs::msg::Status>::SharedPtr status_pub_;

  packml_sm::ModeType switching_mode;

  /// Embedded health monitor — subscribes to Equipment Module heartbeats and
  /// fires PackML actions (Hold/Suspend/Abort) on new health events.
  std::unique_ptr<HealthMonitor> health_monitor_;

  /// Subscriptions to required-node heartbeat topics.
  std::vector<rclcpp::Subscription<packml_msgs::msg::NodeHeartbeat>::SharedPtr> heartbeat_subs_;

  /// Periodic timer that drives heartbeat timeout checks.
  rclcpp::TimerBase::SharedPtr health_timeout_timer_;

  /// Periodic timer that watches for a second publisher on the status topic — see
  /// foreign_publishers_on(). Separate from health_timeout_timer_ and far slower: a graph query
  /// walks every discovered endpoint, and a duplicate manager is a launch-time mistake that stays
  /// wrong, not something worth checking five times a second.
  rclcpp::TimerBase::SharedPtr status_owner_timer_;

  /// The last set of foreign status publishers reported, so the warning repeats only when the
  /// answer changes rather than every tick forever.
  std::vector<std::string> reported_status_publishers_;

  /// Embedded completion-signal aggregator — structurally parallel to health_monitor_ but
  /// independent of it: completion is a progress fact, not a health fact. Aggregates the
  /// ~/packml_state_transition action's per-node results for every node in node_names — there
  /// is no separate opt-in list: a node's own defers_completion() override is what decides
  /// whether it participates meaningfully, not manager-side configuration.
  std::unique_ptr<CompletionTracker> completion_tracker_;

  /// The currently in-flight state-transition goal handle per client node (this fan-out
  /// round only) — needed to issue a cancel if completion_tracker_ times out waiting on it.
  /// Guarded by its own mutex: written from goal_response_callback/result_callback (ROS
  /// executor thread), read from the setStateOperation-bound function (QtConcurrent thread).
  std::map<std::string,
    rclcpp_action::ClientGoalHandle<packml_msgs::action::StateTransition>::SharedPtr>
    active_state_goals_;
  std::mutex active_state_goals_mutex_;

  /// Publisher for alarm events — one message per alarm raise/update/clear.
  rclcpp::Publisher<packml_msgs::msg::Alarm>::SharedPtr alarm_pub_;

  /// Loaded once at init() if `error_catalog_file` is set (see init()); left
  /// default-constructed/unused otherwise. Fail-open: an absent or unloadable
  /// catalog just means on_alarm_event() doesn't enrich alarm messages.
  packml_ros::MachineCatalog machine_catalog_;
  bool has_error_catalog_{false};
  std::string catalog_language_{"en"};

  /// A "stop episode" is the span from the first gate-blocking alarm (HOLD/
  /// SUSPEND/ABORT severity, or a heartbeat timeout — WARN never blocks the
  /// gate) until the health gate reopens (every required node healthy again).
  /// active_stop_event_id_ is 0 outside an episode; on_alarm_event() mints a
  /// new id from next_stop_event_id_ on the first gate-blocking alarm, every
  /// alarm raised or cleared during the episode (from any node) carries that
  /// same id, and the health_timeout_timer_ tick resets it to 0 once the gate
  /// reopens (see init()). Lets a consumer of Alarm::stop_event_id group
  /// everything that happened during one stop and find the first
  /// (earliest-timestamp) fault that caused it.
  uint64_t next_stop_event_id_{1};
  uint64_t active_stop_event_id_{0};

  rclcpp::Node::SharedPtr node_;

protected:
  /// Health-gate bypass config (see init()). A RESET from STOPPED with a required
  /// EM in ERROR is bypassed ONLY when bypass is explicitly enabled AND the machine
  /// is in the configured manual/maintenance mode. Default: no bypass — the gate
  /// enforces health in every mode. (Heartbeat TIMEOUT is never bypassable.)
  bool manual_mode_allows_health_bypass_{false};
  int64_t health_bypass_mode_{-1};   // mode value that permits bypass; -1 = none
  bool all_required_seen_{false};    // latches true once every required node reported

  /// TODO: This should be private!
  /// Also this should be in state machine class?
  /// Read by publish_status() from the Qt SM thread (via on_mode_changed/on_state_changed)
  /// and written/read on the ROS executor thread → atomic to avoid a data race.
  std::atomic<packml_sm::ModeType> current_mode{0};
  /// Per-mode AvailableStates masks parsed from modes_config_file, loaded once in init().
  /// Empty when no config was supplied, in which case changeMode()'s all-open overload is the
  /// correct behaviour rather than a silent wipe. See on_change_mode()'s use of it.
  std::map<packml_sm::ModeType, packml_sm::AvailableStates> mode_masks_;

  /// Mode values a deployment's modes_config_file declared, which narrow what ~/changeMode will
  /// accept. Empty when no config declared any, in which case is_known_mode() -- the vocabulary of
  /// whatever generated modes headers this program links -- is the authority instead.
  std::set<packml_sm::ModeType> declared_modes_;

  /// TODO: This should be private!
  /// Written by on_state_changed on the Qt SM thread and read by the health gate +
  /// publish_status on the ROS executor thread → atomic so the gate never reads a
  /// torn/stale current_state (a stale read could skip the RESET health check).
  std::atomic<packml_sm::State> current_state{packml_sm::State::UNDEFINED};
  packml_sm::State switching_state;

  static rclcpp::Client<packml_msgs::srv::ModeTransition>::SharedPtr get_mode_client(std::shared_ptr<PackmlClientInterface> client) {
    return client->mode_tr_client;
  }

  // -------------------------------------------------------------------------
  // Asynchronous client fan-out with acknowledgement tracking
  // -------------------------------------------------------------------------

  /// Fan-out kind labels — used in logs and in the alarm message
  /// ("Equipment Module did not acknowledge <kind> transition: ...").
  static constexpr const char * kStateFanoutKind = "state";
  static constexpr const char * kModeFanoutKind  = "mode";

  /// Tracks one asynchronous state/mode fan-out to the registered child clients.
  /// Owned by active_fanouts_ (and by in-flight response callbacks) until finalized.
  struct ClientFanout
  {
    enum class ClientStatus { PENDING, ACKED, FAILED };
    std::string kind;                               ///< "state" or "mode" — for logs/alarms
    /// Fan-out round this object belongs to (state fan-out only; 0 for mode, which has no
    /// per-round completion tracking). A ClientFanout deliberately outlives a round -- its 5s
    /// deadline versus the round's much longer completion budget -- so it must carry the round
    /// it came from rather than assuming it is still the live one.
    uint64_t round{0};
    std::chrono::steady_clock::time_point deadline;
    std::map<std::string, ClientStatus> clients;
    std::map<std::string, int64_t> request_ids;     ///< to prune unanswered SENT requests
    /// Clients whose service was not yet discovered at fan-out time; retried every
    /// deadline-check tick until it appears or the deadline expires. The closures
    /// capture this tracker (shared_ptr) — finalize_fanout() MUST clear this map to
    /// break the resulting ownership cycle.
    std::map<std::string, std::function<bool()>> unsent_;
    std::function<void(const std::string &, int64_t)> prune_request;
    /// Optional: called once per client that is still PENDING when this fan-out is
    /// finalized (deadline expired with no ack — including one that was never even
    /// sent). Lets a caller with its own, separate completion-tracking (the state
    /// fan-out's completion_tracker_) learn "this client will never answer" promptly
    /// instead of independently riding out its own, usually much longer, timeout.
    /// Unused (nullptr) by mode fan-out, which has no such separate tracker.
    std::function<void(const std::string &)> on_client_failed;
    bool finalized{false};
  };

  std::mutex fanouts_mutex_;   // guards active_fanouts_ and every ClientFanout's fields
  std::condition_variable fanouts_cv_;   // notified whenever any ClientFanout's clients map changes
  std::vector<std::shared_ptr<ClientFanout>> active_fanouts_;

  /// The detached mode-fan-out threads currently inside this object, and the flag that tells them
  /// to stop waiting and go home. Drained by shutdown(); see it for why.
  packml_ros::DetachedWorkerGate mode_fanout_workers_;
  std::atomic<bool> mode_fanout_shutting_down_{false};
  /// Mirrors CompletionTracker's own round counter so "am I still the live round?" is answerable
  /// from a callback without locking the tracker (and when there is no tracker). Written only by
  /// fanout_state_transition() on the Qt thread; read from ROS executor callbacks and retry
  /// closures, hence atomic.
  std::shared_ptr<std::atomic<uint64_t>> state_fanout_round_ =
    std::make_shared<std::atomic<uint64_t>>(0);
  std::mutex status_publish_mutex_;  // see publish_status()

  /// DESTRUCTION ORDER IS LOAD-BEARING: sm_ must stay the LAST declared data member
  /// of this class. Members are destroyed in reverse declaration order, and
  /// ~StateMachine synchronously stops the Qt state-machine thread and drains its
  /// callbacks — the on_state_changed callback (Qt thread) touches the fan-out,
  /// status, and health members of this class, so the state machine must be torn
  /// down FIRST, while everything the callback uses is still alive. Declare any
  /// new data member ABOVE this line.
  std::shared_ptr<packml_sm::StateMachine> sm_;

  /// Surface one client's fan-out failure out-of-band: an event-style WARN Alarm on
  /// packml_alarms (trigger=true with no matching clear — it records an occurrence,
  /// not a persistent condition) plus the WARN log on_alarm_event() already emits.
  void report_fanout_failure(
    const std::string & kind, const std::string & client_name, const std::string & reason)
  {
    AlarmEvent ev;
    ev.trigger    = true;
    ev.node_name  = client_name;
    ev.severity   = packml_msgs::msg::NodeHealth::WARN;
    ev.error_code = 0;
    ev.is_timeout = false;
    ev.message    = "Equipment Module did not acknowledge " + kind + " transition: " + reason;
    on_alarm_event(ev);
  }

  // -------------------------------------------------------------------------
  // State-transition action fan-out
  // -------------------------------------------------------------------------
  // Deliberately NOT built on the generic ClientFanout/fanout_transition_to_clients<T>
  // machinery above (still used for mode changes): the state transition is an ACTION, whose
  // RESULT feeds completion_tracker_ directly rather than a WARN-alarm-on-timeout tracker,
  // since a coordinated completion failure already drives the state machine to ABORTING via
  // ErrorEvent — a much stronger, more visible signal than a log-only alarm.

  using StateTransitionAction = packml_msgs::action::StateTransition;
  using StateClientGoalHandle = rclcpp_action::ClientGoalHandle<StateTransitionAction>;

  /// Fan out a state-transition ACTION goal to every registered client, feeding
  /// completion_tracker_ so the setStateOperation-bound function (see init()) can wait for
  /// every node's real completion.
  ///
  /// Blocks the calling (Qt state-machine) thread BRIEFLY, bounded by kAcceptanceWaitMs, for
  /// every client's goal to be ACCEPTED (each node's own handle_goal decision — a fast, local
  /// check, not the node's actual acting-state work) before returning. This is a real fix, not
  /// just a style choice: this manager fans out BEFORE publishing packml_status specifically
  /// so a node sees "you are being asked to transition" before "the machine is now in that
  /// state" — an node that saw status FIRST hits its own already_there shortcut and skips
  /// on_state_trans_req()/defers_completion() entirely. An action's goal-request/response
  /// handshake has measurably more overhead than a single topic publish, so without this wait
  /// the status topic can consistently win that race even though the fan-out is issued first
  /// in program order — silently defeating defers_completion() for every node, every time.
  /// Waiting only for ACCEPTANCE (not the full, potentially-deferred completion) keeps this
  /// bounded and fast — nothing like the old, deliberately-removed multi-second wait for full
  /// acknowledgement; an unresponsive node simply falls back to today's best-effort ordering
  /// once the short bound elapses, rather than stalling the machine.
  /// Attempt one client's state-transition goal send for the current fan-out round.
  /// Returns false if the client's action server is not yet discovered — mirroring
  /// try_send_to_client<T>()'s discovery-retry contract for mode fan-out, this closure is
  /// then kept in `fanout->unsent_` and retried on every check_fanout_deadlines() tick until
  /// the server appears or the fan-out deadline expires. Prior to this, an EM whose action
  /// server was not yet discoverable at fan-out time (e.g. mid-startup, or mid-restart) got
  /// no goal at all, ever — no retry, no failure signal — and would silently ride out the
  /// whole, separate state_complete_timeout_ms with completion_tracker_ waiting on a node
  /// that was never actually asked.
  ///
  /// On a successful send, marks the client ACKED in `fanout` — meaning only "the goal was
  /// handed to the middleware", not "the node answered"; actual completion is entirely
  /// completion_tracker_'s job via the goal-response/result callbacks installed here (the
  /// same ones fanout_state_transition() always installed).
  /// `round` identifies the fan-out round this send belongs to; every callback installed below
  /// carries it so a late answer from a superseded round cannot be mistaken for an answer to
  /// the live one. See CompletionTracker::begin_round() for why the id is required.
  bool try_send_state_goal(
    const std::shared_ptr<ClientFanout> & fanout,
    const std::string & name,
    const std::shared_ptr<PackmlClientInterface> & client,
    const StateTransitionAction::Goal & goal,
    const std::shared_ptr<std::atomic<size_t>> & pending_acceptance,
    const std::shared_ptr<std::condition_variable> & acceptance_cv,
    const std::shared_ptr<std::mutex> & acceptance_mutex,
    uint64_t round)
  {
    if (!client->state_tr_client->action_server_is_ready()) {
      return false;
    }
    typename rclcpp_action::Client<StateTransitionAction>::SendGoalOptions options;
    options.goal_response_callback =
      [this, name, pending_acceptance, acceptance_cv, acceptance_mutex, round,
        round_counter = state_fanout_round_](
        StateClientGoalHandle::SharedPtr handle) {
        {
          std::lock_guard<std::mutex> lk(*acceptance_mutex);
          pending_acceptance->fetch_sub(1);
        }
        acceptance_cv->notify_all();
        if (!handle) {
          report_fanout_failure(kStateFanoutKind, name, "goal rejected by node");
          if (completion_tracker_) {
            completion_tracker_->on_goal_rejected(name, "goal rejected by node", round);
          }
          return;
        }
        {
          std::lock_guard<std::mutex> lk(active_state_goals_mutex_);
          // Only the live round may install a handle. A superseded round's acceptance
          // arriving now would otherwise overwrite the current round's handle for this
          // node, and cancel_pending_state_goals() would then cancel the wrong goal.
          if (round == round_counter->load()) {
            active_state_goals_[name] = handle;
          }
        }
        if (completion_tracker_) {
          completion_tracker_->on_goal_accepted(name, handle->get_goal_id(), round);
        }
      };
    options.result_callback =
      [this, name, round, round_counter = state_fanout_round_](
        const StateClientGoalHandle::WrappedResult & wrapped) {
        const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        const int32_t error_code = wrapped.result ? wrapped.result->error_code : 0;
        const std::string message = wrapped.result ? wrapped.result->message : std::string();
        if (!success) {
          report_fanout_failure(
            kStateFanoutKind, name,
            "result code " + std::to_string(static_cast<int>(wrapped.code)) + ": " + message);
        } else {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"),
            name << " completed state transition");
        }
        if (completion_tracker_) {
          completion_tracker_->on_result(
            name, wrapped.goal_id, success, error_code, message, round);
        }
        std::lock_guard<std::mutex> lk(active_state_goals_mutex_);
        // Same reasoning as the acceptance path: a stale result's unconditional erase(name)
        // would remove the CURRENT round's handle, after which cancel_pending_state_goals()
        // silently cancels nothing for that node.
        if (round == round_counter->load()) {
          active_state_goals_.erase(name);
        }
      };
    client->state_tr_client->async_send_goal(goal, options);
    bool all_acked = false;
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      if (!fanout->finalized) {
        fanout->clients[name] = ClientFanout::ClientStatus::ACKED;
        // Finalize as soon as every client has been sent to, rather than always sitting out
        // the full 5s deadline. Without this a state fan-out occupied active_fanouts_ for 5s
        // regardless of how fast every node answered -- and since a fan-out happens on EVERY
        // state change, a machine self-cycling in EXECUTE kept ~25 live fan-outs at all times,
        // each replaying its own retry closures on every 200ms tick.
        all_acked = fanout->unsent_.empty() &&
          std::none_of(fanout->clients.begin(), fanout->clients.end(),
            [](const auto & e) {return e.second == ClientFanout::ClientStatus::PENDING;});
      }
    }
    fanouts_cv_.notify_all();
    if (all_acked) {
      finalize_fanout(fanout);
    }
    return true;
  }

  void fanout_state_transition(packml_sm::State state)
  {
    std::vector<std::string> names;
    names.reserve(client_map_.size());
    for (const auto & [name, client] : client_map_) {
      (void)client;
      names.push_back(name);
    }

    // One monotonic identity for this round, shared by the tracker, this fan-out object and
    // every callback and retry closure installed below. state_fanout_round_ mirrors the
    // tracker's own counter so the currency check stays available even where the tracker is
    // not consulted (and when there is no tracker at all).
    const uint64_t round = completion_tracker_
      ? completion_tracker_->begin_round(names)
      : state_fanout_round_->load() + 1;
    state_fanout_round_->store(round);

    // Retire any state fan-out from a previous round BEFORE this one starts. Their deadlines
    // would otherwise expire later and call on_client_failed against a round that no longer
    // exists -- the round check in the tracker now rejects that, but leaving zombie fan-outs
    // alive also means their retry closures keep re-sending a dead round's goal on every tick.
    // Retired quietly: a superseded round has no one left to report to.
    {
      std::vector<std::shared_ptr<ClientFanout>> superseded;
      {
        std::lock_guard<std::mutex> lk(fanouts_mutex_);
        for (const auto & existing : active_fanouts_) {
          if (existing->kind == kStateFanoutKind && !existing->finalized &&
            existing->round != round)
          {
            existing->on_client_failed = nullptr;  // nothing to report; the round is gone
            superseded.push_back(existing);
          }
        }
      }
      for (const auto & existing : superseded) {
        finalize_fanout(existing);
      }
    }
    {
      std::lock_guard<std::mutex> lk(active_state_goals_mutex_);
      active_state_goals_.clear();
    }

    if (client_map_.empty()) {
      return;
    }

    StateTransitionAction::Goal goal;
    goal.state.val = static_cast<int8_t>(state);

    auto pending_acceptance = std::make_shared<std::atomic<size_t>>(client_map_.size());
    auto acceptance_cv = std::make_shared<std::condition_variable>();
    auto acceptance_mutex = std::make_shared<std::mutex>();

    // Tracks send (not completion) per client for this round, purely to drive the
    // discovery-retry below — completion stays entirely completion_tracker_'s job. A
    // client still PENDING when this fan-out's own 5s deadline expires never got its goal
    // sent at all; completion_tracker_ is told immediately (on_client_failed) so its own,
    // separate and much longer state_complete_timeout_ms wait doesn't have to find out the
    // slow way.
    auto fanout = std::make_shared<ClientFanout>();
    fanout->kind = kStateFanoutKind;
    fanout->round = round;
    fanout->deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    // Captures `round` so an expiry that lands after this round has been superseded is
    // rejected by the tracker instead of failing whatever round is live: this fan-out's 5s
    // deadline is far shorter than the round's completion budget, so it routinely outlives it.
    fanout->on_client_failed = [this, round](const std::string & name) {
        if (completion_tracker_) {
          completion_tracker_->on_goal_rejected(
            name, "action server not available for the entire fan-out deadline", round);
        }
      };
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      for (const auto & name : names) {
        fanout->clients[name] = ClientFanout::ClientStatus::PENDING;
      }
      active_fanouts_.push_back(fanout);
    }

    for (const auto & [name, client] : client_map_) {
      if (!try_send_state_goal(
          fanout, name, client, goal, pending_acceptance, acceptance_cv, acceptance_mutex, round))
      {
        std::lock_guard<std::mutex> lk(fanouts_mutex_);
        if (!fanout->finalized) {
          fanout->unsent_[name] = [this, fanout, name, client, goal,
            pending_acceptance, acceptance_cv, acceptance_mutex, round,
            round_counter = state_fanout_round_]() {
              // A retry must never send a superseded round's goal. Without this check a
              // late-discovered node received a burst of stale commands naming states the
              // machine had already left -- observed as two goals 1.24ms apart, the first for
              // a state abandoned 1.6s earlier.
              if (round != round_counter->load()) {
                return true;  // treat as handled so it stops being retried
              }
              return try_send_state_goal(
                fanout, name, client, goal, pending_acceptance, acceptance_cv, acceptance_mutex,
                round);
            };
        }
      }
    }

    // Bounded wait for acceptance — see this method's own doc comment for why. The ROS
    // executor thread (separate from this Qt thread) keeps servicing the goal-response
    // callbacks above while this thread blocks here, exactly like CompletionTracker's own
    // cross-thread bridge. A client still undiscovered at this point never decrements
    // pending_acceptance, so this simply rides out the full bound — the same outcome as an
    // accepted-but-silent node already produced before this method had any retry at all.
    static constexpr auto kAcceptanceWaitMs = std::chrono::milliseconds(200);
    std::unique_lock<std::mutex> lk(*acceptance_mutex);
    acceptance_cv->wait_for(
      lk, kAcceptanceWaitMs, [pending_acceptance] { return pending_acceptance->load() == 0; });
  }

  /// Cancel every still-outstanding state-transition goal from the current round — called
  /// whenever completion_tracker_->wait_for_all() ends the wait early for a reason that
  /// leaves OTHER nodes still pending (TIMEOUT, FAILED, or ABORTED_BY_HEALTH; never COMPLETE
  /// or SHUTDOWN, see setStateOperation's binding above), so a coordinated node that never
  /// answered is told honestly to stop rather than being silently abandoned mid-goal to ride
  /// out its own, potentially much longer, deferred_completion_timeout_ms independently.
  /// pending_nodes() naturally excludes whichever node actually triggered the early exit (it
  /// already has a result, by definition), so only genuinely still-outstanding goals are
  /// touched.
  void cancel_pending_state_goals()
  {
    if (!completion_tracker_) {
      return;
    }
    const auto pending = completion_tracker_->pending_nodes();
    std::lock_guard<std::mutex> lk(active_state_goals_mutex_);
    for (const auto & name : pending) {
      auto goal_it = active_state_goals_.find(name);
      if (goal_it == active_state_goals_.end()) {
        continue;  // never got as far as an accepted goal handle
      }
      auto client_it = client_map_.find(name);
      if (client_it != client_map_.end()) {
        client_it->second->state_tr_client->async_cancel_goal(goal_it->second);
      }
    }
  }

  /// Finalize a fan-out: clients still PENDING are failed (deadline expired), their
  /// unanswered requests pruned from the client, a summary logged, and the tracker
  /// retired. Safe to call from a response callback and the deadline check
  /// concurrently — only the first caller acts.
  void finalize_fanout(const std::shared_ptr<ClientFanout> & fanout)
  {
    std::vector<std::pair<std::string, int64_t>> unanswered;   // sent, no response
    std::vector<std::string> undiscovered;                     // never became available
    size_t acked = 0;
    size_t total = 0;
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      if (fanout->finalized) {
        return;
      }
      fanout->finalized = true;
      total = fanout->clients.size();
      for (auto & [name, status] : fanout->clients) {
        if (status == ClientFanout::ClientStatus::PENDING) {
          status = ClientFanout::ClientStatus::FAILED;
          auto id_it = fanout->request_ids.find(name);
          if (id_it != fanout->request_ids.end()) {
            unanswered.emplace_back(name, id_it->second);
          } else {
            undiscovered.push_back(name);
          }
        } else if (status == ClientFanout::ClientStatus::ACKED) {
          ++acked;
        }
      }
      // The retry closures capture this tracker — clear them to break the
      // shared_ptr ownership cycle (fanout -> unsent_ -> closure -> fanout).
      fanout->unsent_.clear();
      active_fanouts_.erase(
        std::remove(active_fanouts_.begin(), active_fanouts_.end(), fanout),
        active_fanouts_.end());
    }

    for (const auto & [name, request_id] : unanswered) {
      if (fanout->prune_request) {
        fanout->prune_request(name, request_id);
      }
      report_fanout_failure(fanout->kind, name, "no response within the fan-out deadline");
      if (fanout->on_client_failed) {
        fanout->on_client_failed(name);
      }
    }
    for (const auto & name : undiscovered) {
      report_fanout_failure(fanout->kind, name, "service unavailable for the entire fan-out deadline");
      if (fanout->on_client_failed) {
        fanout->on_client_failed(name);
      }
    }
    if (acked == total) {
      RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
        "%s fan-out complete: %zu/%zu Equipment Module(s) acknowledged",
        fanout->kind.c_str(), acked, total);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
        "%s fan-out incomplete: %zu/%zu Equipment Module(s) acknowledged",
        fanout->kind.c_str(), acked, total);
    }
  }

  /// Retry sends whose service was not yet discovered, then expire fan-outs whose
  /// deadline passed with acknowledgements still missing. Driven by the manager's
  /// periodic wall timer (see init()). Retrying first gives a service that appeared
  /// just before the deadline one last chance in the same tick.
  void check_fanout_deadlines()
  {
    std::vector<std::pair<std::shared_ptr<ClientFanout>, std::function<bool()>>> retries;
    std::vector<std::pair<std::shared_ptr<ClientFanout>, std::string>> retry_names;
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      for (const auto & fanout : active_fanouts_) {
        for (const auto & [name, try_send] : fanout->unsent_) {
          retries.emplace_back(fanout, try_send);
          retry_names.emplace_back(fanout, name);
        }
      }
    }
    for (size_t i = 0; i < retries.size(); ++i) {
      if (retries[i].second()) {   // sends outside the lock; records its own id
        std::lock_guard<std::mutex> lk(fanouts_mutex_);
        retry_names[i].first->unsent_.erase(retry_names[i].second);
      }
    }

    std::vector<std::shared_ptr<ClientFanout>> expired;
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      const auto now = std::chrono::steady_clock::now();
      for (const auto & fanout : active_fanouts_) {
        if (now >= fanout->deadline) {
          expired.push_back(fanout);
        }
      }
    }
    for (const auto & fanout : expired) {
      finalize_fanout(fanout);
    }
  }

  /// Send `request` to every registered child's state/mode-transition service WITHOUT
  /// blocking the calling thread (state fan-outs run on the Qt state-machine thread,
  /// mode fan-outs on the executor thread). Acknowledgements arrive as asynchronous
  /// response callbacks on the node's executor; a client that is offline, rejects, or
  /// stays silent past the deadline is surfaced out-of-band via WARN log + WARN Alarm.
  /// A failed fan-out does NOT roll back the machine state: the state machine is the
  /// source of truth and the manager's status has already been published.
  ///
  /// Returns the ClientFanout tracker so the caller can (optionally, briefly) wait for
  /// initial acknowledgement before publishing status — see on_change_mode() for why
  /// that ordering matters (the same race documented on fanout_state_transition()).
  template <typename T>
  std::shared_ptr<ClientFanout> fanout_transition_to_clients(
    const std::string & kind,
    std::function<typename rclcpp::Client<T>::SharedPtr(std::shared_ptr<PackmlClientInterface>)> get_client,
    typename T::Request::SharedPtr request)
  {
    if (client_map_.empty()) {
      return nullptr;
    }

    auto fanout = std::make_shared<ClientFanout>();
    fanout->kind = kind;
    fanout->deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    fanout->prune_request = [this, get_client](const std::string & name, int64_t request_id) {
        auto it = client_map_.find(name);
        if (it != client_map_.end()) {
          get_client(it->second)->remove_pending_request(request_id);
        }
      };

    // Pass 1: register EVERY client in the tracker before any request is sent, so a
    // fast response to the first request can never observe a partially-populated set
    // and declare the fan-out complete prematurely.
    std::vector<std::pair<std::string, typename rclcpp::Client<T>::SharedPtr>> targets;
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      for (const auto & [client_name, client] : client_map_) {
        fanout->clients[client_name] = ClientFanout::ClientStatus::PENDING;
        targets.emplace_back(client_name, get_client(client));
      }
      active_fanouts_.push_back(fanout);
    }

    // Pass 2: send. A client whose service is not yet discovered (e.g. a fan-out
    // fired milliseconds after startup, or an Equipment Module mid-restart) is NOT
    // failed immediately: it is retried on every deadline-check tick until the
    // service appears or the deadline expires — a non-blocking replacement for the
    // old 1s wait_for_service grace. The deadline covers the never-appears case.
    for (const auto & [client_name, srv] : targets) {
      if (!try_send_to_client<T>(fanout, client_name, srv, request)) {
        const std::string name = client_name;
        auto service = srv;
        std::lock_guard<std::mutex> lk(fanouts_mutex_);
        if (!fanout->finalized) {
          fanout->unsent_[name] = [this, fanout, name, service, request]() {
              return try_send_to_client<T>(fanout, name, service, request);
            };
        }
      }
    }
    return fanout;
  }

  /// Attempt one client's send. Returns false if the service is not yet discovered
  /// (caller keeps it for retry). On send, installs the response callback that
  /// records the acknowledgement and finalizes the fan-out when it is the last one
  /// outstanding.
  template <typename T>
  bool try_send_to_client(
    const std::shared_ptr<ClientFanout> & fanout,
    const std::string & client_name,
    typename rclcpp::Client<T>::SharedPtr srv,
    typename T::Request::SharedPtr request)
  {
    if (!srv->service_is_ready()) {
      return false;
    }
    const std::string name = client_name;
    auto future_and_id = srv->async_send_request(request,
      [this, fanout, name](typename rclcpp::Client<T>::SharedFuture response_future) {
        const auto response = response_future.get();
        bool complete = false;
        {
          std::lock_guard<std::mutex> lk(fanouts_mutex_);
          if (fanout->finalized) {
            return;  // the deadline check already reported this fan-out
          }
          fanout->clients[name] = response->success
            ? ClientFanout::ClientStatus::ACKED
            : ClientFanout::ClientStatus::FAILED;
          complete = std::none_of(fanout->clients.begin(), fanout->clients.end(),
            [](const auto & entry) {return entry.second == ClientFanout::ClientStatus::PENDING;});
        }
        fanouts_cv_.notify_all();
        if (response->success) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"),
            name << " acknowledged " << fanout->kind << " transition");
        } else {
          report_fanout_failure(fanout->kind, name, "rejected: " + response->message);
        }
        if (complete) {
          finalize_fanout(fanout);
        }
      });
    // Record the id and re-check finalized in ONE critical section: if the deadline
    // finalized this fan-out while the request was being handed to the middleware,
    // finalize could not have known this id — prune the request ourselves so it
    // cannot linger in the client's pending map for a never-responding EM.
    bool finalized_meanwhile = false;
    {
      std::lock_guard<std::mutex> lk(fanouts_mutex_);
      if (fanout->finalized) {
        finalized_meanwhile = true;
      } else {
        fanout->request_ids[client_name] = future_and_id.request_id;
      }
    }
    if (finalized_meanwhile) {
      srv->remove_pending_request(future_and_id.request_id);
    }
    return true;
  }


  void publish_status()
  {
    // Serialize snapshot+publish: this is called from BOTH the Qt state-machine
    // thread (on_state_changed) and the executor thread (on_change_mode). Without
    // the lock, two racing calls can publish out of order and the depth-1
    // TRANSIENT_LOCAL topic would retain the OLDER snapshot indefinitely (each
    // caller stores its own atomic before calling, so under the lock the last
    // publisher always emits a snapshot at least as fresh as its own change).
    std::lock_guard<std::mutex> status_lk(status_publish_mutex_);

    // One consistent snapshot of the atomics for this publication.
    const packml_sm::State state_now = current_state.load();
    const packml_sm::ModeType mode_now = current_mode.load();

    RCLCPP_DEBUG(rclcpp::get_logger("packml_ros"), "Borrowing message");
    auto msg = status_pub_->borrow_loaned_message();

    auto state = packml_msgs::msg::State();
    // TODO: make mapping between packml_msgs::msg::State constant declarations and packml_sm::State
    state.val = static_cast<signed char>(state_now);
    // state.set__val(current_state);
    msg.get().state = state;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"), "Current state: " << state_now);

    auto mode = packml_msgs::msg::Mode();
    // TODO: make mapping between packml_msgs::msg::Mode constant declarations and packml_sm::Mode
    mode.val = static_cast<signed char>(mode_now);
    msg.get().mode = mode;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"), "Current mode: " << packml_sm::to_string(mode_now));

    RCLCPP_DEBUG(rclcpp::get_logger("packml_ros"), "publising message");
    status_pub_->publish(std::move(msg));
  }

private:
  void on_change_mode(
    // const std::shared_ptr<rmw_request_id_t> request_header,
std::shared_ptr<packml_msgs::srv::ModeChange::Request> req,
        std::shared_ptr<packml_msgs::srv::ModeChange::Response> res) {

      // Reject a mode nobody declared, before it touches the state machine.
      //
      // Nothing else on this path can: ModeType is a bare int, mode_switcher() checks only that
      // the CURRENT STATE permits switching and never that the mode exists, and the
      // single-argument changeMode() builds an ALL-STATES-AVAILABLE mask for whatever number it
      // is handed. So an unrecognised value was not merely accepted and fanned out to every
      // equipment module as an approved switch -- it arrived with no state mask at all, silently
      // removing every command restriction the configured modes impose until the next valid mode
      // change. A masked state that correctly refuses a command in Production would accept it.
      //
      // Two authorities for whether a mode EXISTS, in that order. A modes_config_file that
      // declares modes wins, because it is the narrower and more specific statement: a program
      // may link a mode vocabulary far wider than the machine in front of it is commissioned for,
      // and accepting the surplus is what leaves a mode running with no mask. Where no config
      // declared any, is_known_mode() answers from the vocabulary of the generated modes headers
      // this program links.
      //
      // Deliberately not mode_masks_: that table says which states a mode allows, and a declared
      // mode with no configured mask is legitimate -- it means fully open, which is the documented
      // fail-open behaviour below.
      const auto requested_mode = static_cast<packml_sm::ModeType>(req->mode.val);
      const bool mode_exists = declared_modes_.empty()
        ? packml_sm::is_known_mode(requested_mode)
        : declared_modes_.count(requested_mode) != 0;
      if (!mode_exists) {
        // static_cast<int>: mode.val is int8_t, which streams as a CHARACTER -- 99 logged as 'c'
        // and -1 as a stray byte before this cast.
        RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"),
          "Rejected mode change to " << static_cast<int>(req->mode.val) <<
            ": not a mode this deployment declares");
        res->success = false;
        res->error_code = res->INVALID_MODE_REQUEST;
        res->message = "Unknown mode " + std::to_string(req->mode.val) +
          (declared_modes_.empty()
            ? "; declared modes come from the generated modes header this program links"
            : "; declared modes come from modes_config_file");
        return;
      }

      // TODO: make mapping between packml_msgs::msg::Mode constant declarations and packml_sm::Mode
      switching_mode = requested_mode;

      // Apply this mode's CONFIGURED state mask if the deployment declared one.
      //
      // The single-argument changeMode() must NOT be used here: it builds an all-states-available
      // mask for whatever value it is given (state_machine.cpp), which would replace the mask
      // parsed from modes_config_file with a fully-open one and never restore it. Machine
      // behaviour would then depend on HOW a mode was set (boot vs. ~/changeMode) rather than on
      // which mode it was. The parsed table is owned by this class instead -- see mode_masks_.
      const auto mask_it = mode_masks_.find(switching_mode);
      auto change_result = (mask_it != mode_masks_.end())
        ? sm_->changeMode(switching_mode, mask_it->second)
        : sm_->changeMode(switching_mode);

      if (!change_result.has_value()) {
        res->success = false;
        res->error_code = res->INVALID_MODE_REQUEST;
        res->message = change_result.error();
        return;
      }

      // The state machine accepted the mode change and IS the source of truth, so the
      // manager's view and the latched status update immediately. The response means
      // the request was *accepted* (mirroring ~/changeState): Equipment Module
      // acknowledgements are collected asynchronously by the fan-out below and
      // surfaced as WARN logs/alarms — never as a synchronous failure here. (The old
      // blocking wait starved this executor's health-timeout checks for up to 5s and
      // reported failure for a mode the state machine had already switched.)
      current_mode.store(switching_mode);

      // Fan out and publish status on their own thread, not this callback's own ROS
      // executor thread: an EM that sees the new mode on the latched status topic
      // before its own mode-transition request arrives would "already there"-
      // shortcut past on_mode_trans_req(), the same race fanout_state_transition()
      // already guards against for state (see its own comment). Waiting for that
      // ordering INSIDE this callback would deadlock a single-threaded executor,
      // since the mode-transition response the wait needs is itself delivered by
      // this same thread. So, like wait_for_deferred_completion() on the Equipment
      // Module side, the wait (and the status publish it guards) runs on its own
      // detached thread instead — bounded to a short grace window, acceptable here
      // since mode changes are rare, operator-driven actions rather than a hot path
      // (unlike state transitions, which the Qt-thread wait keeps responsive for).
      auto request = std::make_shared<packml_msgs::srv::ModeTransition::Request>();
      request->mode = req->mode;
      // Counted BEFORE the spawn (see DetachedWorkerGate::enter) so shutdown() cannot slip
      // between creating this thread and knowing about it.
      mode_fanout_workers_.enter();
      std::thread([this, request]() {
          packml_ros::DetachedWorkerGate::Scope worker_scope(mode_fanout_workers_);

          auto fanout = fanout_transition_to_clients<packml_msgs::srv::ModeTransition>(
            kModeFanoutKind, &PackmlManagerInterface::get_mode_client, request);

          if (fanout) {
            static constexpr auto kAcceptanceWaitMs = std::chrono::milliseconds(200);
            std::unique_lock<std::mutex> lk(fanouts_mutex_);
            fanouts_cv_.wait_for(
              lk, kAcceptanceWaitMs,
              [this, &fanout] {
                return mode_fanout_shutting_down_.load() ||
                  std::none_of(fanout->clients.begin(), fanout->clients.end(),
                  [](const auto & entry) {return entry.second == ClientFanout::ClientStatus::PENDING;});
              });
          }

          // Nothing to publish on the way out the door, and status_pub_ is about to go: the
          // manager is being torn down, so this thread's only remaining job is to leave.
          if (mode_fanout_shutting_down_.load()) {
            return;
          }
          publish_status();
        }).detach();

      res->success = true;
      res->error_code = res->SUCCESS;
  };

  /// Take a ~/changeState request off the wire and hand it to the command worker, without
  /// evaluating it here.
  ///
  /// Evaluating it here silently loses commands. `evaluate_state_change()` below blocks in
  /// `sm_->changeState()` until the Qt event loop answers the command -- deliberately unbounded (see
  /// `StateMachine::postCommand()`: the command stays queued and still takes effect, so a wall-clock
  /// false would be a lie). An executor thread parked in that wait takes nothing else off the wire,
  /// and the service's reader queue is a RELIABLE KEEP_LAST depth-10 default nobody chose: requests
  /// past ~10 deep are OVERWRITTEN. Not rejected -- overwritten, so no response is ever sent and
  /// the caller's future never resolves.
  ///
  /// Silence is the part that matters. An operator mashing recovery buttons has no way to tell a
  /// command that was refused from one that was discarded, and the discarded one leaves the machine
  /// somewhere they did not ask for. A deeper queue only moves the depth at which that starts.
  ///
  /// Returning immediately keeps the reader queue draining, so every request reaches the worker and
  /// every request gets an answer. The response is deferred, not skipped: rclcpp's
  /// defer-response callback form hands us the service and the request id, and the worker sends the
  /// real accepted/rejected answer through them once the machine has actually answered. Nothing is
  /// predicted or guessed -- the client still learns the true outcome, just not on this thread.
  void on_change_state(
    std::shared_ptr<rclcpp::Service<packml_msgs::srv::StateChange>> service,
    std::shared_ptr<rmw_request_id_t> request_id,
    packml_msgs::srv::StateChange::Request::SharedPtr req)
  {
    size_t depth = 0;
    {
      std::lock_guard<std::mutex> lk(command_queue_mutex_);
      if (command_worker_stopping_) {
        // Answer rather than drop: this is the one path that cannot wait for the worker.
        packml_msgs::srv::StateChange::Response res;
        res.success = false;
        res.error_code = res.INVALID_TRANSITION_REQUEST;
        res.message = "Manager is shutting down; command not evaluated";
        send_command_response(service, *request_id, res);
        return;
      }
      // Mapped here rather than in the worker: it is a pure lookup whose answer cannot change while
      // the command waits, unlike the health gate and current-state checks the worker keeps.
      command_queue_.push_back(
        {std::move(service), std::move(request_id), packml_ros::to_transition_cmd(req->command)});
      depth = command_queue_.size();
    }
    command_queue_cv_.notify_one();

    // The queue is deliberately unbounded -- a bound would put back the thing being removed, a
    // depth at which commands stop being answered. What it cannot hide is that a backlog means
    // commands are being applied well after they were issued, so say so where an operator's log
    // will show it.
    if (depth > kCommandBacklogWarnDepth) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"),
        "State-command backlog is " << depth << " deep: commands are being answered slower than "
        "they arrive, so each one now takes effect noticeably later than it was issued");
    }
  }

  /// Evaluate one command against the state machine and build its response. Runs on the command
  /// worker thread, never on a ROS executor thread.
  ///
  /// The health gate and current-state checks are evaluated HERE rather than at arrival, so a
  /// command is judged against the machine as it is when the machine actually acts on it. For a
  /// command that sat in the queue behind a slow transition, arrival-time answers would be about a
  /// machine that has since moved on.
  packml_msgs::srv::StateChange::Response evaluate_state_change(packml_sm::TransitionCmd command)
  {
    packml_msgs::srv::StateChange::Response response;
    packml_msgs::srv::StateChange::Response * res = &response;
    std::string error_message;

    if (command == packml_sm::TransitionCmd::NO_COMMAND) {
      error_message =  "Unrecognized transition request command: " + to_string(command);
      res->success = false;
      res->error_code = res->UNRECOGNIZED_REQUEST;
      res->message = error_message;
      return response;
    }

    // Health gate: block RESET from STOPPED if any required Equipment Module
    // is unhealthy or has timed out.
    if (command == packml_sm::TransitionCmd::RESET &&
        current_state.load() == packml_sm::State::STOPPED)
    {
      // Bypass ERROR only in an explicitly-configured manual/maintenance mode with
      // bypass enabled (params manual_mode_allows_health_bypass + manual_mode).
      // TIMEOUT / never-seen are never bypassable.
      const bool manual = manual_mode_allows_health_bypass_ &&
                          health_bypass_mode_ >= 0 &&
                          static_cast<int64_t>(current_mode.load()) == health_bypass_mode_;
      if (!health_monitor_->can_transition_from_stopped(manual)) {
        error_message = "Health gate blocked: required Equipment Module(s) not healthy ["
                        + health_monitor_->gate_block_summary(manual)
                        + "]. All required nodes must send a healthy heartbeat "
                          "before RESET is allowed.";
        RCLCPP_WARN(rclcpp::get_logger("packml_ros"), "%s", error_message.c_str());
        res->success = false;
        res->error_code = res->INVALID_TRANSITION_REQUEST;
        res->message = error_message;
        return response;
      }
    }

    auto change_result = sm_->changeState(command);

    if (!change_result.has_value()) {
      res->success = false;
      res->error_code = res->INVALID_TRANSITION_REQUEST;
      res->message = change_result.error();
    }
    else {
      // Per StateChange.srv, the response means the request was *accepted*, not that
      // the transition has completed. The Qt state machine runs on its own thread and
      // applies the transition asynchronously; clients await the outcome on the
      // packml_status topic. Blocking here for the real outcome would starve the
      // single-threaded executor's heartbeat and timeout callbacks for the duration
      // of every transition.
      res->success = true;
      res->error_code = res->SUCCESS;
    }
    return response;
  }

  /// One thread, strictly FIFO, for every ~/changeState command.
  ///
  /// One rather than a pool because the order commands are evaluated in is the order the operator
  /// issued them. HOLD then UNHOLD is not the same instruction as UNHOLD then HOLD, and a pool
  /// would let the second overtake the first whenever the first happens to take longer.
  void run_command_worker()
  {
    for (;;) {
      QueuedCommand queued;
      {
        std::unique_lock<std::mutex> lk(command_queue_mutex_);
        command_queue_cv_.wait(
          lk, [this] {return !command_queue_.empty() || command_worker_stopping_;});

        if (command_worker_stopping_) {
          // Answer everything still owed, then leave. Deliberately NOT evaluated: the machine is
          // being torn down, so a refusal naming that is more honest than an answer about a
          // machine that is going away -- and it keeps teardown bounded by at most the one
          // command already inside evaluate_state_change(), rather than by the whole backlog.
          auto owed = std::move(command_queue_);
          command_queue_.clear();
          lk.unlock();
          for (auto & abandoned : owed) {
            if (nullptr == abandoned.service) {
              continue;   // health-raised: nobody is waiting for an answer
            }
            packml_msgs::srv::StateChange::Response res;
            res.success = false;
            res.error_code = res.INVALID_TRANSITION_REQUEST;
            res.message = "Manager is shutting down; command not evaluated";
            send_command_response(abandoned.service, *abandoned.request_id, res);
          }
          return;
        }

        queued = std::move(command_queue_.front());
        command_queue_.pop_front();
      }

      auto response = evaluate_state_change(queued.command);
      if (nullptr != queued.service) {
        send_command_response(queued.service, *queued.request_id, response);
      } else if (!response.success) {
        // A health-raised action the machine will not take from where it currently is (HOLD while
        // STOPPED, ABORT while already ABORTED) is the expected case, not a fault: the monitor asks
        // and the machine decides. See fire_packml_action().
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"),
          "[HealthMonitor] queued action not applicable from the current state: " <<
            response.message);
      }
    }
  }

    // auto change_result = sm_->setState(switching_state, QString name)

    //     bool command_rtn = false;
    //     bool command_valid = true;
    //     auto command_int = static_cast<int>(req->command);
    //     std::stringstream ss;
    //     std::cout << "Evaluating transition request command: " << command_int << std::endl;
    //     switch (command_int) {
          // case packml_msgs::srv::StateChange::Request::ABORT:
    //         command_rtn = sm->abort();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::STOP:
    //         command_rtn = sm->stop();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::CLEAR:
    //         command_rtn = sm->clear();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::HOLD:
    //         command_rtn = sm->hold();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::RESET:
    //         command_rtn = sm->reset();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::START:
    //         command_rtn = sm->start();
    //         break;
    //       // case packml_msgs::srv::StateChange::Request::STOP:
    //       //   command_rtn = sm->stop();
    //       //   break;
    //       case packml_msgs::srv::StateChange::Request::SUSPEND:
    //         command_rtn = sm->suspend();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::UNHOLD:
    //         command_rtn = sm->unhold();
    //         break;
    //       case packml_msgs::srv::StateChange::Request::UNSUSPEND:
    //         command_rtn = sm->unsuspend();
    //         break;
    //       default:
    //         command_valid = false;
    //         break;
    //     }
    //     if (command_valid) {
    //       if (command_rtn) {
    //         ss << "Successful transition request command: " << command_int;
    //         res->success = true;
    //         res->error_code = res->SUCCESS;
    //         res->message = ss.str();
    //       } else {
    //         ss << "Invalid transition request command: " << command_int;
    //         res->success = false;
    //         res->error_code = res->INVALID_TRANSITION_REQUEST;
    //         res->message = ss.str();
    //       }
    //     } else {
    //       ss << "Unrecognized transition request command: " << command_int;
    //       res->success = false;
    //       res->error_code = res->UNRECOGNIZED_REQUEST;
    //       res->message = ss.str();
    //     }


  void on_all_status(
    std::shared_ptr<packml_msgs::srv::AllStatus::Request> req,
    std::shared_ptr<packml_msgs::srv::AllStatus::Response> res)
  {
    (void)req;
    (void)res;
    // TODO: change the packml_msgs::srv::AllStatus to just contain packml_msgs::msg::Status.
  }

  /// Publish an alarm event from a HealthMonitor AlarmEvent. When an error
  /// catalog is loaded (see init()), a raise's message is enriched with the
  /// catalog's description ahead of the node's own free-text message.
  void on_alarm_event(const AlarmEvent & ev)
  {
    if (!alarm_pub_) {
      return;
    }

    std::string message = ev.message;
    uint32_t global_code = 0;
    if (has_error_catalog_ && ev.trigger) {
      const packml_ros::MachineEntry * entry = ev.is_timeout
        ? machine_catalog_.find_global(machine_catalog_.reserved("heartbeat_timeout"))
        : machine_catalog_.find(ev.node_name, ev.error_code);
      if (entry) {
        const std::string resolved =
          machine_catalog_.resolve_message(entry->entry, catalog_language_, ev.instance_id);
        message = ev.message.empty() ? resolved : resolved + ": " + ev.message;
        global_code = static_cast<uint32_t>(entry->global);

        // The catalog's `action` is documentation only — the SM only ever
        // reacts to the runtime NodeHealth.action a node actually emits
        // (ev.severity here). If they disagree, the fault sheet/HMI would
        // show one reaction while the machine does another — surface it
        // instead of letting the two silently drift apart.
        if (entry->entry.action != ev.severity) {
          RCLCPP_WARN_THROTTLE(rclcpp::get_logger("packml_ros"), *node_->get_clock(), 5000,
            "[ErrorCatalog] node '%s' error_code=%d emitted action %d but the catalog "
            "documents action %d for this fault — the fault sheet/HMI and the machine's "
            "actual reaction disagree; update the catalog or the node",
            ev.node_name.c_str(), ev.error_code, ev.severity, entry->entry.action);
        }
      } else if (!ev.is_timeout && ev.error_code != 0) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("packml_ros"), *node_->get_clock(), 5000,
          "[ErrorCatalog] node '%s' error_code=%d has no catalog entry — "
          "alarm message not enriched",
          ev.node_name.c_str(), ev.error_code);
      }
    }

    // Stop-episode correlation: mint a new id on the first gate-blocking alarm (WARN
    // never blocks the gate — see HealthMonitor::gate_block_reason — so it doesn't
    // start an episode); the timer tick in init() resets it once the gate reopens.
    // Approximate: it doesn't check whether the raising node is itself *required*,
    // so a non-required node's HOLD/ABORT can start an episode the gate never
    // actually blocked on.
    const bool blocking_severity =
      ev.is_timeout || ev.severity > packml_msgs::msg::NodeHealth::WARN;
    if (ev.trigger && blocking_severity && active_stop_event_id_ == 0) {
      active_stop_event_id_ = next_stop_event_id_++;
    }

    if (ev.trigger) {
      RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
        "[Alarm] RAISED  node='%s' id=%d severity=%d%s: %s",
        ev.node_name.c_str(), ev.error_code, ev.severity,
        ev.is_timeout ? " (timeout)" : "", message.c_str());
    } else {
      RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
        "[Alarm] CLEARED node='%s' id=%d", ev.node_name.c_str(), ev.error_code);
    }
    packml_msgs::msg::Alarm msg;
    msg.trigger    = ev.trigger;
    msg.stamp      = node_->get_clock()->now();
    msg.node_name  = ev.node_name;
    msg.severity   = static_cast<uint8_t>(ev.severity);
    msg.error_code = static_cast<uint32_t>(ev.error_code);
    msg.is_timeout = ev.is_timeout;
    msg.message    = message;
    msg.global_code = global_code;
    msg.instance_id = ev.instance_id;
    msg.state.val  = static_cast<int8_t>(current_state.load());
    msg.stop_event_id = active_stop_event_id_;
    // msg.source_key intentionally left empty — see Alarm.msg.
    alarm_pub_->publish(msg);
  }

  /// Convert a NodeHealth action constant into a PackML TransitionCmd and
  /// issue it to the state machine.  Called by HealthMonitor when a new
  /// or escalating health event is detected on a required Equipment Module.
  void fire_packml_action(int32_t health_action)
  {
    using NodeHealth = packml_msgs::msg::NodeHealth;
    packml_sm::TransitionCmd cmd = packml_sm::TransitionCmd::NO_COMMAND;

    switch (health_action) {
      case NodeHealth::HOLD:    cmd = packml_sm::TransitionCmd::HOLD;    break;
      case NodeHealth::SUSPEND: cmd = packml_sm::TransitionCmd::SUSPEND; break;
      case NodeHealth::ABORT:   cmd = packml_sm::TransitionCmd::ABORT;   break;
      default:
        RCLCPP_DEBUG(rclcpp::get_logger("packml_ros"),
          "fire_packml_action: ignoring action %d", health_action);
        return;
    }

    // Wake any in-flight CompletionTracker wait immediately: a health-triggered action can
    // make a node newly unhealthy, and the wait's own predicate re-checks health on every
    // wakeup — no need to wait for the condition variable's next natural wakeup (an action
    // result or the timeout) to notice.
    if (completion_tracker_) {
      completion_tracker_->notify_health_change();
    }

    // Queued for the command worker, NOT applied here. sm_->changeState() blocks until the Qt event
    // loop answers, which is as long as the transition in progress takes; this runs on the ROS
    // executor thread (HealthMonitor calls fire_action_ straight out of on_heartbeat() and
    // check_timeouts()), and an executor parked there takes nothing else off the wire for the
    // duration — no operator command, no heartbeat, no alarm. That is the same starvation
    // on_change_state() exists to avoid, and the worker already evaluates a command against the
    // machine as it is when it acts, which for a health event is the honest moment.
    //
    // The state machine stays the single source of truth for legality: the command is offered and
    // the machine rejects it if it is not valid from where it is (HOLD/SUSPEND while STOPPED, ABORT
    // while already ABORTED). See run_command_worker() for where that rejection is reported.
    {
      std::lock_guard<std::mutex> lk(command_queue_mutex_);
      if (command_worker_stopping_) {
        return;
      }
      // check_timeouts() re-raises ABORT on every tick while a node stays timed out, so without
      // this an unreachable machine accumulates one queued action every 200 ms. Collapsing them is
      // exact rather than approximate: the actions are identical and none carries a response.
      const bool already_queued = std::any_of(command_queue_.begin(), command_queue_.end(),
        [cmd](const QueuedCommand & queued) {
          return nullptr == queued.service && queued.command == cmd;
        });
      if (already_queued) {
        return;
      }
      command_queue_.push_back({nullptr, nullptr, cmd});
    }
    command_queue_cv_.notify_one();

    RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
      "[HealthMonitor] Raised PackML action %d from Equipment Module health event",
      health_action);
  }

protected:

  /// Re-arm timed-out required nodes so they trigger ABORT again on the next
  /// check_timeouts() cycle.  Call this when the machine returns to STOPPED
  /// (e.g. after an operator CLEAR) to avoid leaving the machine silently
  /// stuck in STOPPED while required Equipment Modules are still offline.
  void rearm_health_timeouts()
  {
    if (health_monitor_) {
      health_monitor_->rearm_timed_out_nodes();
    }
  }

  /// Wake any in-flight CompletionTracker wait immediately. MUST be called as the
  /// FIRST statement of the derived manager's destructor body (e.g.
  /// SMNode_new::~SMNode_new()), before the StateMachine shared_ptr chain that
  /// owns sm_ starts unwinding — StateMachine::drainActingStates() (called from
  /// ~StateMachine()) blocks on any bound acting-state operation with NO internal
  /// timeout, so an in-flight completion wait would otherwise stall process
  /// teardown for up to state_complete_timeout_ms.
  void shutdown()
  {
    if (completion_tracker_) {
      completion_tracker_->request_shutdown();
    }

    // The MODE side needs this drain as much as the state side does: on_change_mode() detaches one
    // thread per request, and that thread uses this object through a fan-out and an up-to-200 ms
    // wait before publishing status. A manager destroyed inside that window -- an operator mode
    // change shortly before a launch shutdown or a Ctrl-C -- leaves a live thread in freed memory.
    //
    // Flag first so a waiting thread gives up now rather than riding out its wait, notify to wake
    // it, then hold here until it has actually left.
    mode_fanout_shutting_down_.store(true);
    fanouts_cv_.notify_all();
    mode_fanout_workers_.await_idle();

    // The command worker holds a raw `this` and answers requests through the service, so it has to
    // be gone before either is. Stopping it also answers whatever is still queued -- see
    // run_command_worker() -- so a client waiting on a command that arrived just before teardown
    // gets a refusal instead of a future that never resolves.
    {
      std::lock_guard<std::mutex> lk(command_queue_mutex_);
      command_worker_stopping_ = true;
    }
    command_queue_cv_.notify_all();
    if (command_worker_.joinable()) {
      command_worker_.join();
    }
  }

  /// Backstop for the derived destructor's shutdown() call, NOT a replacement for it.
  ///
  /// The derived class must still call shutdown() first, because by the time this runs the derived
  /// part is already gone and anything the worker might have needed from it is unreachable. What
  /// this covers is the case where the derived destructor never runs at all: a throw anywhere
  /// between init() starting command_worker_ and the end of the derived constructor destroys a
  /// partially-constructed object, so ~PackmlManagerInterface is reached but ~SMNode_new is not.
  /// Without a join here that path met ~std::thread on a joinable thread, and std::terminate ends
  /// the process. It is reachable from an ordinary mistake, not just an exotic one -- a bad value
  /// for a declared parameter throws out of init(), and SMNode_new throws deliberately when
  /// activate() fails -- so a launch-file typo aborted instead of reporting itself.
  ///
  /// shutdown() is safe to run twice (the flags are already set, the thread is no longer joinable,
  /// the gate is already idle) and safe to run when init() never got far enough to start anything.
  ///
  /// Protected and non-virtual on purpose. Nothing deletes a manager through a base pointer --
  /// SMNode_new inherits privately -- so protected is what enforces that, and this is the class's
  /// only virtual candidate, so declaring it virtual would add a vtable for nothing.
  ~PackmlManagerInterface()
  {
    shutdown();
  }

  void init(rclcpp::Node::SharedPtr node, std::shared_ptr<packml_sm::StateMachine> sm) {

    node_ = node;
    sm_ = sm;

    if (!node->has_parameter(packml_ros::kParamNodeNames)) {
      node->declare_parameter(packml_ros::kParamNodeNames, std::vector<std::string>{});
    }
    std::vector<std::string> node_names_;

    node_names_= node->get_parameter(packml_ros::kParamNodeNames).as_string_array();

    current_mode = 0;
    current_state = packml_sm::State::UNDEFINED;
    switching_mode = 0;
    switching_state = packml_sm::State::UNDEFINED;

    // Create clients for all nodes
    for (auto & node_name : node_names_) {
      client_map_[node_name] = std::make_shared<PackmlClientInterface>(node_name, node);
    }

    mode_server_ = node->create_service<packml_msgs::srv::ModeChange>("~/" + std::string(packml_ros::kChangeModeService), [this](const std::shared_ptr<packml_msgs::srv::ModeChange::Request>& req, const std::shared_ptr<packml_msgs::srv::ModeChange::Response>& res){on_change_mode(req, res); });
    // Defer-response form: the callback returns without an answer and the command worker sends it
    // once the machine has really answered. See on_change_state() for the commands this lost when
    // the answer was produced inline on the executor thread.
    state_server_ = node->create_service<packml_msgs::srv::StateChange>(
      "~/" + std::string(packml_ros::kChangeStateService),
      [this](
        std::shared_ptr<rclcpp::Service<packml_msgs::srv::StateChange>> service,
        std::shared_ptr<rmw_request_id_t> request_id,
        packml_msgs::srv::StateChange::Request::SharedPtr req) {
        on_change_state(std::move(service), std::move(request_id), std::move(req));
      });
    command_worker_ = std::thread(&PackmlManagerInterface::run_command_worker, this);
    status_server_ = node->create_service<packml_msgs::srv::AllStatus>("~/" + std::string(packml_ros::kAllStatusService), [this](const std::shared_ptr<packml_msgs::srv::AllStatus::Request>& req, const std::shared_ptr<packml_msgs::srv::AllStatus::Response>& res){on_all_status(req, res); });
    // Status is a latched state topic: it is only published on state change.
    // Use TRANSIENT_LOCAL + RELIABLE (depth 1) so late-joining subscribers
    // (e.g. the rviz panel, which starts after this node) immediately receive
    // the current state instead of waiting for the next transition.
    status_pub_ = node->create_publisher<packml_msgs::msg::Status>(
      packml_ros::kStatusTopic, rclcpp::QoS(1).transient_local().reliable());

    // -----------------------------------------------------------------------
    // Health Monitor setup
    // -----------------------------------------------------------------------

    // Declare parameters used by the health subsystem.
    if (!node->has_parameter(packml_ros::kParamRequiredNodes)) {
      node->declare_parameter(packml_ros::kParamRequiredNodes, std::vector<std::string>{});
    }
    if (!node->has_parameter(packml_ros::kParamHeartbeatTimeoutFactor)) {
      node->declare_parameter(packml_ros::kParamHeartbeatTimeoutFactor, 3.0);
    }
    // Health-gate bypass: disabled by default (gate enforces ERROR in every mode). Set
    // manual_mode_allows_health_bypass=true AND manual_mode=<the manual mode value> to let an
    // operator command RESET past an EM ERROR for diagnostics.
    //
    // The health bypass affects RESET admission only. During coordinated completion,
    // HealthMonitor::is_node_healthy() checks heartbeat liveness without applying
    // actionable-error policy.
    // Runtime mode switches are allowed from STOPPED, and from ABORTED when the target is the
    // manual_mode configured below.
    if (!node->has_parameter(packml_ros::kParamManualModeAllowsHealthBypass)) {
      node->declare_parameter(packml_ros::kParamManualModeAllowsHealthBypass, false);
    }
    if (!node->has_parameter(packml_ros::kParamManualMode)) {
      node->declare_parameter(packml_ros::kParamManualMode, -1);
    }

    const auto required_nodes =
      node->get_parameter(packml_ros::kParamRequiredNodes).as_string_array();
    const double timeout_factor =
      node->get_parameter(packml_ros::kParamHeartbeatTimeoutFactor).as_double();
    manual_mode_allows_health_bypass_ =
      node->get_parameter(packml_ros::kParamManualModeAllowsHealthBypass).as_bool();
    health_bypass_mode_ = node->get_parameter(packml_ros::kParamManualMode).as_int();
    // manual_mode identifies the mode admitted from ABORTED. The default value, -1,
    // disables this mode-entry exception.
    sm_->set_manual_mode(health_bypass_mode_);

    // Optional startup grace period: how long to wait before timing out a
    // node that has not yet sent its first heartbeat.  Default 30 s.
    if (!node->has_parameter(packml_ros::kParamHeartbeatStartupGraceMs)) {
      node->declare_parameter(packml_ros::kParamHeartbeatStartupGraceMs, 30000);
    }
    const int startup_grace_ms =
      node->get_parameter(packml_ros::kParamHeartbeatStartupGraceMs).as_int();
    // startup_interval = grace / factor so that: startup_interval × factor = grace
    const uint32_t startup_interval_ms =
      static_cast<uint32_t>(startup_grace_ms > 0
        ? static_cast<double>(startup_grace_ms) / timeout_factor
        : 1000.0);

    // -----------------------------------------------------------------------
    // Mode masks: parse ONCE and KEEP the table.
    // -----------------------------------------------------------------------
    // Parsing the table for the boot changeMode() call and then discarding it would leave
    // ~/changeMode nothing to apply, falling back to the all-open overload. Owned here instead,
    // so both the boot path and every runtime mode change consult the same source. Fail-open on
    // a bad/missing file, matching how error_catalog_file and the rest of this init() handle
    // configuration problems.
    if (!node->has_parameter(packml_ros::kParamModesConfigFile)) {
      node->declare_parameter(packml_ros::kParamModesConfigFile, std::string(""));
    }
    const auto modes_config_path =
      node->get_parameter(packml_ros::kParamModesConfigFile).as_string();
    if (!modes_config_path.empty()) {
      const auto declared = packml_sm::parse_declared_modes(modes_config_path);
      for (const auto & [name, value] : declared) {
        declared_modes_.insert(value);
      }
      // Registering gives the deployment's own names to every to_string(ModeType) in the process,
      // including for modes no generated header knows. It happens long after static
      // initialisation, so these names win over any a linked modes header registered.
      packml_sm::register_modes(declared);

      mode_masks_ = packml_sm::parse_modes_config(modes_config_path);

      if (declared.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
          "[Modes] %s declared no modes; ~/changeMode keeps validating against the %zu mode(s) "
          "this program links", modes_config_path.c_str(), packml_sm::known_modes().size());
      } else {
        RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
          "[Modes] loaded %zu declared mode(s) and %zu per-mode state mask(s) from %s",
          declared_modes_.size(), mode_masks_.size(), modes_config_path.c_str());
      }
    }

    if (declared_modes_.empty() && packml_sm::known_modes().empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("packml_ros"),
        "[Modes] no modes are declared, so ~/changeMode will reject every request. Link a header "
        "from packml_sm_generate_modes() or point modes_config_file at a file declaring them.");
    }

    // Mirror each state-machine mode change into the manager's atomic snapshot.
    // publish_status() and health-bypass admission both read current_mode. The
    // callback also emits the standard mode-change log.
    sm->on_mode_changed = [this](packml_sm::ModeType value) {
        PACKML_INFO_STREAM("packml_sm", "Default callback; Mode changed to: " << value);
        current_mode.store(value);
      };

    health_monitor_ = std::make_unique<HealthMonitor>(
      [this](int32_t action) { fire_packml_action(action); },
      0,  // no internal thread; timeout checked by health_timeout_timer_
      [this](const AlarmEvent & ev) { on_alarm_event(ev); });

    health_monitor_->set_global_timeout_factor(timeout_factor);
    // Cap the interval a node may advertise to the startup-grace-derived bound, so a
    // node cannot disable its own liveness check by claiming a very long interval.
    // (max × factor == startup grace ⇒ a running node's timeout never exceeds it.
    //  Keep heartbeat_startup_grace_ms ≥ slowest node interval × factor.)
    health_monitor_->set_max_expected_interval_ms(startup_interval_ms);

    // Reliable + transient_local so a late-joining alarm journal/HMI receives recent
    // history (depth 100). Journals should subscribe transient_local to get the backlog.
    alarm_pub_ = node->create_publisher<packml_msgs::msg::Alarm>(
      packml_ros::kAlarmsTopic, rclcpp::QoS(100).reliable().transient_local());

    // Register required nodes and subscribe to their heartbeat topics.
    for (const auto & req_node : required_nodes) {
      health_monitor_->register_required_node(req_node, 0.0, startup_interval_ms);

      const std::string topic = "/" + req_node + "/" + packml_ros::kHeartbeatTopic;
      heartbeat_subs_.push_back(
        node->create_subscription<packml_msgs::msg::NodeHeartbeat>(
          topic, rclcpp::SensorDataQoS(),
          [this, req_node, startup_interval_ms](packml_msgs::msg::NodeHeartbeat::SharedPtr msg) {
            RCLCPP_DEBUG(rclcpp::get_logger("packml_ros"),
              "[HealthMonitor] Heartbeat from '%s': status=%d action=%d seq=%lu interval=%ums",
              req_node.c_str(),
              msg->health.status, msg->health.action,
              static_cast<unsigned long>(msg->sequence_number),
              msg->heartbeat_interval_ms);
            // A node cannot extend its own liveness timeout: HealthMonitor clamps an
            // advertised interval above the cap. Surface it (throttled) so a misconfig —
            // or a node trying to evade liveness detection — is visible, not silent.
            if (startup_interval_ms > 0 && msg->heartbeat_interval_ms > startup_interval_ms) {
              RCLCPP_WARN_THROTTLE(rclcpp::get_logger("packml_ros"), *node_->get_clock(), 5000,
                "[HealthMonitor] Node '%s' advertised heartbeat_interval_ms=%u above the cap %u — "
                "clamping; its liveness timeout stays bounded (check the node's config)",
                req_node.c_str(), msg->heartbeat_interval_ms, startup_interval_ms);
            }
            const auto res = health_monitor_->on_heartbeat(*msg);
            if (res == HealthMonitor::HeartbeatResult::DROPPED_STALE) {
              RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("packml_ros"), *node_->get_clock(), 5000,
                "[HealthMonitor] Out-of-order/duplicate heartbeat from '%s' (seq=%lu) — "
                "possible duplicate publisher; ignoring",
                req_node.c_str(), static_cast<unsigned long>(msg->sequence_number));
            } else if (res == HealthMonitor::HeartbeatResult::RESTART) {
              RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
                "[HealthMonitor] Node '%s' restarted (heartbeat sequence reset)",
                req_node.c_str());
              // A required node's own state after an unannounced restart is unknown, and
              // PackML has no "reconcile silently" path -- recovery already goes through
              // CLEAR→RESET regardless of what triggers it. Route the
              // restart through the same fire_packml_action() path as any other health
              // escalation rather than inventing a parallel one; it's idempotent, so a
              // restart detected while already ABORTING/ABORTED is a harmless no-op.
              fire_packml_action(packml_msgs::msg::NodeHealth::ABORT);
            }
          }));

      RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
        "[HealthMonitor] Monitoring required node: %s (topic: %s)",
        req_node.c_str(), topic.c_str());
    }

    // -----------------------------------------------------------------------
    // Completion Tracker setup
    // -----------------------------------------------------------------------
    // No separate opt-in node list: every node in node_names is fanned out to via the
    // ~/packml_state_transition ACTION (see fanout_state_transition()), and that action's
    // own result IS the completion signal -- a node that never overrides
    // defers_completion() simply resolves its goal the instant it's accepted, matching
    // today's behavior exactly and costing this wait nothing. Whether a node meaningfully
    // participates in coordination is therefore that node's OWN choice, in its own code,
    // never manager-side configuration.
    if (!node->has_parameter(packml_ros::kParamStateCompleteTimeoutMs)) {
      node->declare_parameter(packml_ros::kParamStateCompleteTimeoutMs, 30000);
    }
    const int state_complete_timeout_ms_default =
      node->get_parameter(packml_ros::kParamStateCompleteTimeoutMs).as_int();

    completion_tracker_ = std::make_unique<CompletionTracker>(
      [this](const std::string & name) { return health_monitor_->is_node_healthy(name); });

    // Bind every coordinated acting state EXCEPT EXECUTE: ContinuousCycle::init()
    // (packml_sm/src/state_machine.cpp) removes the generated EXECUTE→COMPLETING
    // transition and installs its own EXECUTE→EXECUTE self-loop operation method BEFORE
    // this init() runs (continuousCycleSM() constructs and initializes the state machine
    // first — see packml_ros-new.hpp). setOperationMethod() is an unconditional overwrite,
    // so binding EXECUTE here would silently replace that self-loop with a blocking
    // multi-node wait on every production iteration. COMPLETING stays bound: it is never
    // entered under ContinuousCycle, so binding it there is inert dead code, and it
    // becomes live for a future SingleCycle deployment.
    static constexpr packml_sm::State kCoordinatedStates[] = {
      packml_sm::State::CLEARING, packml_sm::State::STOPPING, packml_sm::State::RESETTING,
      packml_sm::State::STARTING, packml_sm::State::HOLDING, packml_sm::State::UNHOLDING,
      packml_sm::State::SUSPENDING, packml_sm::State::UNSUSPENDING,
      packml_sm::State::COMPLETING, packml_sm::State::ABORTING,
    };
    for (const auto coordinated_state : kCoordinatedStates) {
      // Per-state override: some coordinated states genuinely need a different bound than
      // others (e.g. RESETTING's homing sequence vs. ABORTING's near-instant e-stop), so one
      // global state_complete_timeout_ms for all ten was too coarse. Declared as
      // "state_complete_timeout_ms.<STATE NAME>" (e.g. state_complete_timeout_ms.RESETTING),
      // defaulting to state_complete_timeout_ms_default so a deployment that never sets an
      // override behaves exactly as before.
      const std::string state_timeout_param =
        std::string(packml_ros::kParamStateCompleteTimeoutMs) + "." +
        packml_sm::to_string(coordinated_state);
      if (!node->has_parameter(state_timeout_param)) {
        node->declare_parameter(state_timeout_param, state_complete_timeout_ms_default);
      }
      const int state_complete_timeout_ms =
        node->get_parameter(state_timeout_param).as_int();

      // setInterruptibleStateOperation (not plain setStateOperation): passes this acting
      // state's own std::stop_token through to wait_for_all(), so an operator's HOLD/
      // SUSPEND/ABORT/STOP accepted while this wait is in flight wakes it immediately
      // instead of riding out the full state_complete_timeout_ms before the state machine
      // can actually leave this state -- see ActingState::onExit() in packml_sm for why
      // that blocking mattered (it holds up every OTHER node's fan-out too, since the
      // interrupting transition can't complete, and thus can't fan out, until this
      // acting state's own operation returns).
      sm_->setInterruptibleStateOperation(coordinated_state,
        [this, coordinated_state, state_complete_timeout_ms](std::stop_token stop_token) -> int {
        // ABORTING opts out of the health cross-check: it is already the terminal response to an
        // unhealthy node, and the node that caused the abort is by definition still unhealthy
        // while it runs, so cross-checking there fails the wait with nowhere left to escalate.
        // See wait_for_all()'s own doc comment.
        const bool health_cross_check = (coordinated_state != packml_sm::State::ABORTING);
        const auto result = completion_tracker_->wait_for_all(
          std::chrono::milliseconds(state_complete_timeout_ms), stop_token, health_cross_check);
        switch (result) {
          case CompletionTracker::WaitResult::COMPLETE:
            return 0;
          case CompletionTracker::WaitResult::SHUTDOWN:
            // Let teardown proceed quietly -- do not route into ErrorEvent/ABORTING
            // while the manager is already being destroyed.
            return 0;
          case CompletionTracker::WaitResult::INTERRUPTED:
            // The state machine already decided to leave this state early (see
            // ActingState::operation()'s own comment on why it will not post a stale
            // completion/error event for it either way) -- the return value here is
            // moot to the machine, but still cancel this round's other outstanding
            // goals so nodes that hadn't yet answered are told to stop rather than
            // silently abandoned.
            cancel_pending_state_goals();
            return 1;
          case CompletionTracker::WaitResult::TIMEOUT:
            RCLCPP_ERROR(rclcpp::get_logger("packml_ros"),
              "[CompletionTracker] Timed out waiting for node(s) to report %s complete",
              to_string(coordinated_state).c_str());
            cancel_pending_state_goals();
            return 1;
          case CompletionTracker::WaitResult::FAILED:
            RCLCPP_ERROR(rclcpp::get_logger("packml_ros"),
              "[CompletionTracker] A node reported failure completing %s",
              to_string(coordinated_state).c_str());
            cancel_pending_state_goals();
            return 1;
          case CompletionTracker::WaitResult::ABORTED_BY_HEALTH:
            RCLCPP_ERROR(rclcpp::get_logger("packml_ros"),
              "[CompletionTracker] A node became unhealthy while waiting for %s to complete",
              to_string(coordinated_state).c_str());
            cancel_pending_state_goals();
            return 1;
        }
        return 1;
      });
    }

    // Optional: load the aggregated error catalog so on_alarm_event() can
    // enrich an alarm's message with the catalog's description. Mirrors
    // modes_config_file: a path parameter, loaded once here, fail-open on any
    // problem — the machine runs unaffected, alarms just stay unenriched.
    if (!node->has_parameter(packml_ros::kParamErrorCatalogFile)) {
      node->declare_parameter(packml_ros::kParamErrorCatalogFile, std::string(""));
    }
    if (!node->has_parameter(packml_ros::kParamLanguage)) {
      node->declare_parameter(packml_ros::kParamLanguage, std::string("en"));
    }
    catalog_language_ = node->get_parameter(packml_ros::kParamLanguage).as_string();
    const auto error_catalog_path =
      node->get_parameter(packml_ros::kParamErrorCatalogFile).as_string();

    if (!error_catalog_path.empty()) {
      const auto catalog_result = packml_ros::load_machine_catalog_from_yaml(error_catalog_path);
      for (const auto & warning : catalog_result.warnings) {
        RCLCPP_WARN(rclcpp::get_logger("packml_ros"), "[ErrorCatalog] %s", warning.c_str());
      }
      if (catalog_result.ok) {
        machine_catalog_ = catalog_result.catalog;
        has_error_catalog_ = true;
        RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
          "[ErrorCatalog] Loaded '%s' (%zu node code(s))",
          error_catalog_path.c_str(), machine_catalog_.size());

        // Bidirectional drift check: a required node missing from the catalog,
        // or a catalog node that isn't required, usually means the error map
        // and the bringup config were edited independently. Surfaced as a
        // startup WARN, not a load failure — the catalog is purely
        // informational (fail-open), so drift degrades text, never behavior.
        for (const auto & req_node : required_nodes) {
          if (!machine_catalog_.has_node(req_node)) {
            RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
              "[ErrorCatalog] required node '%s' has no entries in the error catalog",
              req_node.c_str());
          }
        }
        for (const auto & catalog_node : machine_catalog_.node_names()) {
          if (std::find(required_nodes.begin(), required_nodes.end(), catalog_node) ==
            required_nodes.end())
          {
            RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
              "[ErrorCatalog] catalog node '%s' is not in required_nodes",
              catalog_node.c_str());
          }
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("packml_ros"),
          "[ErrorCatalog] Failed to load '%s': %s — continuing without it",
          error_catalog_path.c_str(), catalog_result.error.c_str());
      }
    }

    // Periodic timeout checker (every 200 ms by default). Also expires client
    // fan-out deadlines — both are "did the thing we're waiting on go silent?"
    // checks on the same cadence.
    health_timeout_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        health_monitor_->check_timeouts();
        check_fanout_deadlines();
        // Close out the current stop episode once the health gate reopens (see
        // Alarm::stop_event_id and on_alarm_event()) — mirrors the bypass
        // computation in on_change_state().
        if (active_stop_event_id_ != 0) {
          const bool manual = manual_mode_allows_health_bypass_ &&
                              health_bypass_mode_ >= 0 &&
                              static_cast<int64_t>(current_mode.load()) == health_bypass_mode_;
          if (health_monitor_->can_transition_from_stopped(manual)) {
            active_stop_event_id_ = 0;
          }
        }
        // Startup readiness: log (throttled) which required nodes are still unseen,
        // so a misconfigured/missing node name is visible rather than a silent block.
        // Once every required node has reported once, stop doing this work — the
        // steady state (which lasts forever) then pays nothing per tick.
        if (!all_required_seen_) {
          const auto unseen = health_monitor_->unseen_required_nodes();
          if (unseen.empty()) {
            all_required_seen_ = true;
          } else {
            std::string list;
            for (const auto & n : unseen) { if (!list.empty()) { list += ", "; } list += n; }
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("packml_ros"), *node_->get_clock(), 3000,
              "[HealthMonitor] Waiting for required node(s): %s", list.c_str());
          }
        }
      });

    RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
      "[HealthMonitor] Monitoring %zu required node(s), timeout_factor=%.1f",
      required_nodes.size(), timeout_factor);

    // Runs for the life of the node rather than once after discovery settles: a second manager
    // can be launched at any time, and "settled" has no observable moment in a ROS graph.
    status_owner_timer_ = node->create_wall_timer(
      std::chrono::seconds(5), [this]() { check_status_topic_ownership(); });

  }

  /// Warn when another node publishes the status topic this manager owns. Edge-triggered on the
  /// set of names, so an operator gets one line per change instead of a warning every tick, and
  /// gets a second line when the duplicate goes away.
  void check_status_topic_ownership()
  {
    if (nullptr == status_pub_ || nullptr == node_) {
      return;
    }
    const auto & gid = status_pub_->get_gid();
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> own_gid{};
    std::copy(std::begin(gid.data), std::end(gid.data), own_gid.begin());

    auto others =
      packml_ros::foreign_publishers_on(*node_, status_pub_->get_topic_name(), own_gid);
    std::sort(others.begin(), others.end());
    if (others == reported_status_publishers_) {
      return;
    }
    reported_status_publishers_ = others;

    if (others.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
        "[Status] '%s' has no other publishers again — this manager is its only writer",
        status_pub_->get_topic_name());
      return;
    }

    std::string list;
    for (const auto & name : others) {
      if (!list.empty()) { list += ", "; }
      list += name;
    }
    RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
      "[Status] '%s' is also published by: %s. Only this manager should write it — an equipment "
      "module adopts whatever arrives as the machine's state, and may skip coordinated work it "
      "believes is already done. Check for a duplicated launch entry or a second manager.",
      status_pub_->get_topic_name(), list.c_str());
  }

};
