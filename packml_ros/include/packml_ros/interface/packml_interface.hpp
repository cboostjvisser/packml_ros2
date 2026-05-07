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
#include <packml_msgs/srv/all_status.hpp>
#include <packml_msgs/srv/mode_change.hpp>
#include <packml_msgs/srv/state_change.hpp>

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
  packml_ros::TransitionGuard guard_;

  protected:

  inline auto get_current_packml_mode() const -> packml_sm::ModeType { return guard_.current_mode(); }

  inline auto get_current_packml_state() const -> packml_sm::State { return guard_.current_state(); }

  inline bool is_switching_mode() const { return guard_.is_switching_mode(); }

  inline bool is_switching_state() const { return guard_.is_switching_state(); }

  template <typename NodeT>
  inline void init(std::shared_ptr<NodeT> node) {

    auto onStateTranseReq =
      [this](const std::shared_ptr<packml_msgs::srv::StateTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::StateTransition::Response> res) -> void {
            auto state = static_cast<packml_sm::State>(req->state.val);
            auto result = guard_.request_state(state);

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
            auto result = guard_.request_mode(mode);

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

        if (guard_.on_status_update(state, mode)) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"),
            "Status changed - State: " << to_string(state) << ", Mode: " << packml_sm::to_string(mode));
          on_status_changed();
        }
      };

    trans_server_ = node->template create_service<packml_msgs::srv::StateTransition>("~/" + std::string(packml_ros::kStateTransitionService), onStateTranseReq);
    mode_server_ = node->template create_service<packml_msgs::srv::ModeTransition>("~/" + std::string(packml_ros::kModeTransitionService), onModeTransReq);
    status_sub_ = node->template create_subscription<packml_msgs::msg::Status>(packml_ros::kStatusTopic, rclcpp::SensorDataQoS(), onStatusChanged);

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



  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<packml_sm::StateMachine> sm_;

protected:
  // TODO: This should be private!
  // Also this should be in state machine class?
  packml_sm::ModeType current_mode;

  // TODO: This should be private!
  packml_sm::State current_state;
  packml_sm::State switching_state;

  std::promise<bool> changed_prom_;


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

      // TODO: this while should timout after some time.
      while (!done && success) {
        bool any_waiting = false;

        // Spin the executor of all clients, to receive service responses
        for (auto const& [client_name, client] : client_map_) {
          client->callbck_grp_exec.spin_once();
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
    RCLCPP_DEBUG(rclcpp::get_logger("packml_ros"), "Borrowing message");
    auto msg = status_pub_->borrow_loaned_message();

    auto state = packml_msgs::msg::State();
    // TODO: make mapping between packml_msgs::msg::State constant declarations and packml_sm::State
    state.val = static_cast<signed char>(current_state);
    // state.set__val(current_state);
    msg.get().state = state;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"), "Current state: " << current_state);

    auto mode = packml_msgs::msg::Mode();
    // TODO: make mapping between packml_msgs::msg::Mode constant declarations and packml_sm::Mode
    mode.val = static_cast<signed char>(current_mode);
    msg.get().mode = mode;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("packml_ros"), "Current mode: " << packml_sm::to_string(current_mode));

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
    // TODO: make mapping between packml_msgs::msg::State constant declarations and packml_sm::State
    // auto command = static_cast<packml_sm::TransitionCmd>(req->command);

    std::string error_message;

    auto command = packml_ros::to_transition_cmd(req->command);

    if (command == packml_sm::TransitionCmd::NO_COMMAND) {
      // invalid command!
      error_message =  "Unrecognized transition request command: " + to_string(command);
      res->success = false;
      res->error_code = res->UNRECOGNIZED_REQUEST;
      res->message = error_message;
    }
    else {
      auto change_result = sm_->changeState(command);

      if (!change_result.has_value()) {
        res->success = false;
        res->error_code = res->INVALID_TRANSITION_REQUEST;
        res->message = change_result.error();
      }
      else {
        RCLCPP_INFO(rclcpp::get_logger("packml_ros"), "Starting wait on promise");
        // TODO: wait on state_machine->on_state_changed and then return final response
        changed_prom_.get_future().get();

        // Send service response
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


  }

  void on_all_status(std::shared_ptr<packml_msgs::srv::AllStatus::Request> req, std::shared_ptr<packml_msgs::srv::AllStatus::Response> res) {
    (void)req;
    (void)res;
    // TODO: change the packml_msgs::srv::AllStatus to just contain packml_msgs::msg::Status.
  };

protected:

  void init(rclcpp::Node::SharedPtr node, std::shared_ptr<packml_sm::StateMachine> sm) {

    node_ = node;
    sm_ = sm;

    node->declare_parameter("node_names", std::vector<std::string>{});
    std::vector<std::string> node_names_;

    node_names_= node->get_parameter("node_names").as_string_array();

    current_mode = 0;
    current_state = packml_sm::State::UNDEFINED;
    switching_mode = 0;
    switching_state = packml_sm::State::UNDEFINED;

    // Create clients for all nodes
    for (auto & node_name : node_names_) {
      client_map_[node_name] = std::make_shared<PackmlClientInterface>(node_name, node);
    }

    // Perfect forwarding didn't work here
    // mode_server_ = node->create_service<packml_msgs::srv::ModeTransition>("changeMode", [this](auto&& req, auto&& res){/*on_change_mode(std::forward<decltype(hdr)>(hdr), std::forward<decltype(req)>(req), std::forward<decltype(res)>(res));*/});

    mode_server_ = node->create_service<packml_msgs::srv::ModeChange>("~/changeMode", [this](const std::shared_ptr<packml_msgs::srv::ModeChange::Request>& req, const std::shared_ptr<packml_msgs::srv::ModeChange::Response>& res){on_change_mode(req, res); });
    state_server_ = node->create_service<packml_msgs::srv::StateChange>("~/changeState", [this](const std::shared_ptr<packml_msgs::srv::StateChange::Request>& req, const std::shared_ptr<packml_msgs::srv::StateChange::Response>& res){on_change_state(req, res); });
    status_server_ = node->create_service<packml_msgs::srv::AllStatus>("~/allStatus", [this](const std::shared_ptr<packml_msgs::srv::AllStatus::Request>& req, const std::shared_ptr<packml_msgs::srv::AllStatus::Response>& res){on_all_status(req, res); });
    status_pub_ = node->create_publisher<packml_msgs::msg::Status>(packml_ros::kStatusTopic, rclcpp::SensorDataQoS());

  }

};