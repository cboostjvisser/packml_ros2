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
#include <atomic>
#include <chrono>
#include <future>
#include <map>
#include <memory>

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

#include <packml_sm/common.hpp>
#include <packml_sm/state_machine.hpp>
#include <packml_ros/transition_guard.hpp>

#include <packml_msgs/srv/mode_transition.hpp>
#include <packml_msgs/srv/state_transition.hpp>
#include <packml_msgs/msg/status.hpp>

#include <packml_msgs/msg/state.hpp>
#include <packml_msgs/msg/node_health.hpp>
#include <packml_msgs/msg/node_heartbeat.hpp>
#include <packml_msgs/msg/alarm.hpp>
#include <packml_msgs/srv/all_status.hpp>
#include <packml_msgs/srv/mode_change.hpp>
#include <packml_msgs/srv/state_change.hpp>
#include "packml_ros/health_monitor.hpp"

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

  /**
  * @brief Pointer for transition service server
  */
  rclcpp::Service<packml_msgs::srv::StateTransition>::SharedPtr trans_server_;

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
    if (health.action == packml_msgs::msg::NodeHealth::NONE) {
      protocol_.heartbeat.clear_latch();
    } else {
      protocol_.heartbeat.set_latch(health.status, health.action, health.error_code, health.message);
    }
    heartbeat_publisher_->publish(make_heartbeat(health));
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

    auto onStateTranseReq =
      [this](const std::shared_ptr<packml_msgs::srv::StateTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::StateTransition::Response> res) -> void {
            auto state = static_cast<packml_sm::State>(req->state.val);
            auto result = protocol_.transitions.request_state(state);

            if (result.already_there) {
              RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "Node already in state: " << to_string(state));
              res->success = true;
              return;
            }

            RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "Node State changing to: " << to_string(state));

            if (!result.error.empty()) {
              RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), result.error);
            }

            if (result.accepted && on_state_trans_req(state)) {
              RCLCPP_INFO(rclcpp::get_logger("packml_ros"), "Node approved state switch");
              res->success = true;
            } else {
              std::string error_string = "Node did not approve state switch";
              res->message = error_string;
              res->success = false;
              RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), error_string);
            }
        };

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

    trans_server_ = node->template create_service<packml_msgs::srv::StateTransition>("~/" + std::string(packml_ros::kStateTransitionService), onStateTranseReq);
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
        // One atomic snapshot of the latch (no torn read vs a concurrent post_event).
        const auto latch = protocol_.heartbeat.latch_snapshot();
        if (latch.active) {
          // A post_event() fault is latched — repeat it instead of polling the getter.
          health.status     = latch.status;
          health.action     = latch.action;
          health.error_code = latch.error_code;
          health.message    = latch.message;
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
  rclcpp::Client<packml_msgs::srv::StateTransition>::SharedPtr state_tr_client;
  rclcpp::Client<packml_msgs::srv::ModeTransition>::SharedPtr mode_tr_client;
  rclcpp::Subscription<packml_msgs::msg::Status>::SharedPtr status_sub;

  rclcpp::executors::SingleThreadedExecutor callbck_grp_exec;
  rclcpp::CallbackGroup::SharedPtr callback_grp;

  PackmlClientInterface(std::string name, rclcpp::Node::SharedPtr parent_node) {
    auto state_tr_service_name = name + "/" + packml_ros::kStateTransitionService;
    auto mode_tr_service_name = name + "/" + packml_ros::kModeTransitionService;
    // auto status_sub_name = "packml_status";

    callback_grp = parent_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callbck_grp_exec.add_callback_group(callback_grp, parent_node->get_node_base_interface());

    state_tr_client = parent_node->create_client<packml_msgs::srv::StateTransition>(state_tr_service_name, rclcpp::ServicesQoS(), callback_grp);
    mode_tr_client = parent_node->create_client<packml_msgs::srv::ModeTransition>(mode_tr_service_name, rclcpp::ServicesQoS(), callback_grp);
    // status_sub = parent_node->create_subscription<packml_msgs::msg::Status>(status_sub_name, rclcpp::SensorDataQoS(), [](const packml_msgs::msg::Status& status){});
  }
};

class PackmlManagerInterface
{
  // Client name and client interface object
  std::map<std::string, std::shared_ptr<PackmlClientInterface>> client_map_;

  rclcpp::Service<packml_msgs::srv::ModeChange>::SharedPtr mode_server_;
  rclcpp::Service<packml_msgs::srv::StateChange>::SharedPtr state_server_;
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

  /// Publisher for alarm events — one message per alarm raise/update/clear.
  rclcpp::Publisher<packml_msgs::msg::Alarm>::SharedPtr alarm_pub_;



  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<packml_sm::StateMachine> sm_;

protected:
  // Health-gate bypass config (see init()). A RESET from STOPPED with a required
  // EM in ERROR is bypassed ONLY when bypass is explicitly enabled AND the machine
  // is in the configured manual/maintenance mode. Default: no bypass — the gate
  // enforces health in every mode. (Heartbeat TIMEOUT is never bypassable.)
  bool manual_mode_allows_health_bypass_{false};
  int64_t health_bypass_mode_{-1};   // mode value that permits bypass; -1 = none
  bool all_required_seen_{false};    // latches true once every required node reported

  // TODO: This should be private!
  // Also this should be in state machine class?
  // Read by publish_status() from the Qt SM thread (via on_mode_changed/on_state_changed)
  // and written/read on the ROS executor thread → atomic to avoid a data race.
  std::atomic<packml_sm::ModeType> current_mode{0};

  // TODO: This should be private!
  // Written by on_state_changed on the Qt SM thread and read by the health gate +
  // publish_status on the ROS executor thread → atomic so the gate never reads a
  // torn/stale current_state (a stale read could skip the RESET health check).
  std::atomic<packml_sm::State> current_state{packml_sm::State::UNDEFINED};
  packml_sm::State switching_state;

  static rclcpp::Client<packml_msgs::srv::ModeTransition>::SharedPtr get_mode_client(std::shared_ptr<PackmlClientInterface> client) {
    return client->mode_tr_client;
  }

  static rclcpp::Client<packml_msgs::srv::StateTransition>::SharedPtr get_state_client(std::shared_ptr<PackmlClientInterface> client) {
    return client->state_tr_client;
  }

  template <typename T = packml_msgs::srv::StateTransition>
  std::map<std::string, typename rclcpp::Client<T>::FutureAndRequestId> call_all_clients(std::function<typename rclcpp::Client<T>::SharedPtr(std::shared_ptr<PackmlClientInterface>)> func, T::Request::SharedPtr request) {
    std::map<std::string, typename rclcpp::Client<T>::FutureAndRequestId> futures;
  // std::map<std::string, typename rclcpp::Client<T>::SharedFuture> call_all_clients(std::function<typename rclcpp::Client<T>::SharedPtr(std::shared_ptr<PackmlClientInterface>)> func, T::Request::SharedPtr request) {
  //   std::map<std::string, typename rclcpp::Client<T>::SharedFuture> futures;

    // for all clients, check if service is available and send request
    for (const auto & [client, val] : client_map_) {
      // std::cout << "Requesting node " << key << " to change mode to: " << packml_sm::to_string(switching_mode) << std::endl;

      if (!func(val)->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), "Client :" << client << " Service: " << func(val)->get_service_name() << " is unavailable!");
        // TODO: If one of the clients is not online, we problably should error out?
        //  Or; maybe that node is not needed in the current mode and we should just report back information about which client succeeded and which did not
      }
      else {
        futures.emplace(client, func(val)->async_send_request(request));
      }

      // else {
      //   // std::future
      //   futures.emplace(key,  func(val)->async_send_request(request, [&futures, key](rclcpp::Client<T>::SharedFuture future){
      //     // futures.emplace(key, future);
      //     std::cout << "future called!" << std::endl;
      //   }));
      // }

      // auto request = std::make_shared<packml_msgs::srv::ModeTransition::Request>();
      // request->mode = request->mode;

    }

    return futures;
  }

  template <typename T = packml_msgs::srv::StateTransition>
  bool wait_all_futures(std::map<std::string, typename rclcpp::Client<T>::FutureAndRequestId>& futures, std::function<bool(std::string, typename T::Response::SharedPtr)> on_value) {
  // bool wait_all_futures(std::map<std::string, typename rclcpp::Client<T>::SharedFuture>& futures, std::function<bool(std::string, typename T::Response::SharedPtr)> on_value) {

      if (futures.size() <= 0) {
        RCLCPP_WARN(rclcpp::get_logger("packml_ros"), "No futures to wait on!");
        return true;
      } else if (futures.size() != client_map_.size()) {
        // TODO: see line 243
        RCLCPP_WARN(rclcpp::get_logger("packml_ros"), "Not all clients responded with a future, maybe some are offline?");
        return false;
      }

      // We return success if all on_value() callbacks returned true
      bool success = true;
      bool done = false;

      // Bound the wait so a client that stops responding cannot hang the caller
      // forever — on the main executor (mode changes) an unbounded wait would also
      // starve the periodic health-timeout checks. On timeout, fail the request.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!done && success) {
        if (std::chrono::steady_clock::now() >= deadline) {
          RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
            "Timed out (5s) waiting for client service responses; failing the request");
          return false;
        }
        bool any_waiting = false;

        // Spin each client executor with a bounded timeout so the deadline is
        // actually checked even when no response has arrived (an untimed spin_once()
        // would block indefinitely, defeating the deadline).
        for (auto const& [client_name, client] : client_map_) {
          client->callbck_grp_exec.spin_once(std::chrono::milliseconds(20));
           RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"), client_name << ": Spinned once");
        }

        // For all returned future service responses, check if data ready
        for (auto & [client_name, future_and_request_id] : futures) {
          if (future_and_request_id.valid()) {
            if (auto response = future_and_request_id.wait_for(std::chrono::seconds(0)); response == std::future_status::ready) {
              auto service_response = future_and_request_id.get();

              success = on_value(client_name, service_response);
            }
            else {
              // We are still waiting on one of the future values
              any_waiting = true;
            }
          }
        }
        done = !any_waiting;
      }

      return success;
  }


  void publish_status()
  {
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

      // TODO: make mapping between packml_msgs::msg::Mode constant declarations and packml_sm::Mode
      switching_mode = static_cast<packml_sm::ModeType>(req->mode.val);

      std::string error_message;
      bool success = true;
      auto change_result = sm_->changeMode(switching_mode);

      // If state machine successfully changed mode, then change clients
      if (!change_result.has_value()) {
        error_message = change_result.error();
        success = false;
      }
      else
      {
        auto handle_value = [res](std::string client, packml_msgs::srv::ModeTransition::Response::SharedPtr value){
            if (value->success) {
              RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), client << " switched to new mode");
              return true;
            }

            RCLCPP_WARN_STREAM(rclcpp::get_logger("packml_ros"), client << " did not switch mode! Error: " << value->message);
            return false;
          };

        auto request = std::make_shared<packml_msgs::srv::ModeTransition::Request>();
        request->mode = req->mode;

        auto futures = call_all_clients<packml_msgs::srv::ModeTransition>(PackmlManagerInterface::get_mode_client, request);

        success = wait_all_futures<packml_msgs::srv::ModeTransition>(futures, handle_value);
        if (!success) {
          error_message = "Error in one of the packml clients";
        }
      }

      if (!success) {
        res->success = false;
        res->error_code = res->INVALID_MODE_REQUEST;
        res->message = error_message;
      }
      else
      {
        // Set current mode
        current_mode = switching_mode;

        // Publish new state
        publish_status();

        // Send service response
        res->success = true;
        res->error_code = res->SUCCESS;
      }

  };

  void on_change_state(packml_msgs::srv::StateChange::Request::SharedPtr req, packml_msgs::srv::StateChange::Response::SharedPtr res) {
    std::string error_message;

    auto command = packml_ros::to_transition_cmd(req->command);

    if (command == packml_sm::TransitionCmd::NO_COMMAND) {
      error_message =  "Unrecognized transition request command: " + to_string(command);
      res->success = false;
      res->error_code = res->UNRECOGNIZED_REQUEST;
      res->message = error_message;
      return;
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
        return;
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
      // packml_status topic. (Previously this blocked on a reused single-shot
      // std::promise — racy, and it starved the single-threaded executor's heartbeat
      // and timeout callbacks for the duration of every transition.)
      res->success = true;
      res->error_code = res->SUCCESS;
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

  /// Publish an alarm event from a HealthMonitor AlarmEvent.
  void on_alarm_event(const AlarmEvent & ev)
  {
    if (!alarm_pub_) {
      return;
    }
    if (ev.trigger) {
      RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
        "[Alarm] RAISED  node='%s' id=%d severity=%d%s: %s",
        ev.node_name.c_str(), ev.error_code, ev.severity,
        ev.is_timeout ? " (timeout)" : "", ev.message.c_str());
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
    msg.message    = ev.message;
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

    // The state machine is the single source of truth for transition legality:
    // attempt the command and let the SM reject it if it isn't valid from the
    // current state (e.g. HOLD/SUSPEND while STOPPED, or ABORT while already
    // ABORTED). HealthMonitor only calls this on NEW/escalating events (repeats
    // are suppressed upstream), so there is no per-tick spam, and a rejection is
    // expected — logged at DEBUG rather than re-encoding the SM's rules here.
    auto result = sm_->changeState(cmd);
    if (result.has_value()) {
      RCLCPP_WARN(rclcpp::get_logger("packml_ros"),
        "[HealthMonitor] Fired PackML action %d from Equipment Module health event",
        health_action);
    } else {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"),
        "[HealthMonitor] action " << health_action
          << " not applicable from current state: " << result.error());
    }
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
    state_server_ = node->create_service<packml_msgs::srv::StateChange>("~/" + std::string(packml_ros::kChangeStateService), [this](const std::shared_ptr<packml_msgs::srv::StateChange::Request>& req, const std::shared_ptr<packml_msgs::srv::StateChange::Response>& res){on_change_state(req, res); });
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
    // Health-gate bypass: disabled by default (gate enforces ERROR in every mode).
    // To allow an operator to RESET past an EM ERROR for diagnostics, set
    // manual_mode_allows_health_bypass=true AND manual_mode=<the manual mode value>.
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
            }
          }));

      RCLCPP_INFO(rclcpp::get_logger("packml_ros"),
        "[HealthMonitor] Monitoring required node: %s (topic: %s)",
        req_node.c_str(), topic.c_str());
    }

    // Periodic timeout checker (every 200 ms by default).
    health_timeout_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        health_monitor_->check_timeouts();
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

  }

};