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
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

  // Clients live in the node's default callback group: fan-out responses arrive as
  // asynchronous callbacks delivered by whatever executor spins the manager node.
  // (A previous design gave each client its own callback group plus a manually
  // spun executor so a blocking wait inside a service callback could pump the
  // responses without re-entering the main executor; the fan-out is asynchronous
  // now, so none of that machinery is needed.)
  PackmlClientInterface(std::string name, rclcpp::Node::SharedPtr parent_node) {
    auto state_tr_service_name = name + "/" + packml_ros::kStateTransitionService;
    auto mode_tr_service_name = name + "/" + packml_ros::kModeTransitionService;

    state_tr_client = parent_node->create_client<packml_msgs::srv::StateTransition>(state_tr_service_name);
    mode_tr_client = parent_node->create_client<packml_msgs::srv::ModeTransition>(mode_tr_service_name);
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

  /// Loaded once at init() if `error_catalog_file` is set (see init()); left
  /// default-constructed/unused otherwise. Fail-open: an absent or unloadable
  /// catalog just means on_alarm_event() doesn't enrich alarm messages.
  packml_ros::MachineCatalog machine_catalog_;
  bool has_error_catalog_{false};
  std::string catalog_language_{"en"};

  rclcpp::Node::SharedPtr node_;

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
    std::chrono::steady_clock::time_point deadline;
    std::map<std::string, ClientStatus> clients;
    std::map<std::string, int64_t> request_ids;     ///< to prune unanswered SENT requests
    /// Clients whose service was not yet discovered at fan-out time; retried every
    /// deadline-check tick until it appears or the deadline expires. The closures
    /// capture this tracker (shared_ptr) — finalize_fanout() MUST clear this map to
    /// break the resulting ownership cycle.
    std::map<std::string, std::function<bool()>> unsent_;
    std::function<void(const std::string &, int64_t)> prune_request;
    bool finalized{false};
  };

  std::mutex fanouts_mutex_;   // guards active_fanouts_ and every ClientFanout's fields
  std::vector<std::shared_ptr<ClientFanout>> active_fanouts_;
  std::mutex status_publish_mutex_;  // see publish_status()

  // DESTRUCTION ORDER IS LOAD-BEARING: sm_ must stay the LAST declared data member
  // of this class. Members are destroyed in reverse declaration order, and
  // ~StateMachine synchronously stops the Qt state-machine thread and drains its
  // callbacks — the on_state_changed callback (Qt thread) touches the fan-out,
  // status, and health members of this class, so the state machine must be torn
  // down FIRST, while everything the callback uses is still alive. Declare any
  // new data member ABOVE this line.
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
    }
    for (const auto & name : undiscovered) {
      report_fanout_failure(fanout->kind, name, "service unavailable for the entire fan-out deadline");
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
  template <typename T>
  void fanout_transition_to_clients(
    const std::string & kind,
    std::function<typename rclcpp::Client<T>::SharedPtr(std::shared_ptr<PackmlClientInterface>)> get_client,
    typename T::Request::SharedPtr request)
  {
    if (client_map_.empty()) {
      return;
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

      // TODO: make mapping between packml_msgs::msg::Mode constant declarations and packml_sm::Mode
      switching_mode = static_cast<packml_sm::ModeType>(req->mode.val);

      auto change_result = sm_->changeMode(switching_mode);

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

      // Fan out BEFORE publishing status, for the same reason as the state path
      // (see on_state_changed in packml_ros-new.hpp): an EM that sees the new mode
      // on the latched status topic first would "already there"-shortcut the
      // transition request and skip its on_mode_trans_req() hook.
      auto request = std::make_shared<packml_msgs::srv::ModeTransition::Request>();
      request->mode = req->mode;
      fanout_transition_to_clients<packml_msgs::srv::ModeTransition>(
        kModeFanoutKind, &PackmlManagerInterface::get_mode_client, request);

      publish_status();

      res->success = true;
      res->error_code = res->SUCCESS;
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

  /// Publish an alarm event from a HealthMonitor AlarmEvent. When an error
  /// catalog is loaded (see init()), a raise's message is enriched with the
  /// catalog's description ahead of the node's own free-text message.
  void on_alarm_event(const AlarmEvent & ev)
  {
    if (!alarm_pub_) {
      return;
    }

    std::string message = ev.message;
    if (has_error_catalog_ && ev.trigger) {
      const packml_ros::MachineEntry * entry = ev.is_timeout
        ? machine_catalog_.find_global(machine_catalog_.reserved("heartbeat_timeout"))
        : machine_catalog_.find(ev.node_name, ev.error_code);
      if (entry) {
        const std::string resolved =
          machine_catalog_.resolve_message(entry->entry, catalog_language_, "");
        message = ev.message.empty() ? resolved : resolved + ": " + ev.message;
      } else if (!ev.is_timeout && ev.error_code != 0) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("packml_ros"), *node_->get_clock(), 5000,
          "[ErrorCatalog] node '%s' error_code=%d has no catalog entry — "
          "alarm message not enriched",
          ev.node_name.c_str(), ev.error_code);
      }
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