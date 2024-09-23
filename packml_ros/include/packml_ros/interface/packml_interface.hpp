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

  packml_sm::ModeType current_mode;
  packml_sm::ModeType switching_mode;

  packml_sm::State current_state;
  packml_sm::State switching_state;

  bool waiting_for_new_mode;
  bool waiting_for_new_state;

  protected:

  inline void init(rclcpp::Node::SharedPtr node) {
    /**
    * @brief Callback function upon transition request by a client
    * @param req - data coming from the client
    * @param res - response to the client
    */
    auto onStateTranseReq =
      [this](const std::shared_ptr<packml_msgs::srv::StateTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::StateTransition::Response> res) -> void {
            auto state = static_cast<packml_sm::State>(req->state.val);
            std::cout << "Node State changing to: " << to_string(state) << std::endl;

            bool success = false;
            std::string error_string;

            if (waiting_for_new_state) {
              error_string = "Changing state but already waiting on new state";
            } else if (waiting_for_new_mode) {
              error_string = "Changing state but mode change is still active";
            }
            // TODO: else disabled, because currently a node cannot catch-up if it missed a state change
            // else {
              if (on_state_trans_req()) {
                std::cout << "Node approved state switch" << std::endl;
                waiting_for_new_state = true;
                switching_state = state;
                success = true;
              }
              else {
                error_string = "Node did not approve state switch";
              }
              // current_state = state;
            // }

            if (!success) {
              res->message = error_string;
              std::cout << error_string << std::endl;
            }
            res->success = success;
        };

    auto onModeTransReq =
      [this](const std::shared_ptr<packml_msgs::srv::ModeTransition::Request> req,
        std::shared_ptr<packml_msgs::srv::ModeTransition::Response> res)-> void {
            auto mode = static_cast<packml_sm::ModeType>(req->mode.val);
            std::cout << "Node Mode changing to: " << to_string(mode) << std::endl;

            bool success = false;
            std::string error_string;

            if (waiting_for_new_state) {
              error_string = "Changing state but already waiting on new state";
            } else if (waiting_for_new_mode) {
              error_string = "Changing state but mode change is still active";
            }
            // TODO: else disabled, because currently a node cannot catch-up if it missed a mode change
            // else {
              if (on_mode_trans_req()){
                std::cout << "Node approved mode switch" << std::endl;
                waiting_for_new_mode = true;
                switching_mode = mode;
                success = true;
              } else {
                error_string = "Node did not approve mode switch";
              }
              // current_mode = mode;

            // }

            if (!success) {
              res->message = error_string;
              std::cout << error_string << std::endl;
            }
            res->success = success;

        };

    auto onStatusChanged =
      [this](const packml_msgs::msg::Status& status) -> void {
        auto state = static_cast<packml_sm::State>(status.state.val);
        auto mode = static_cast<packml_sm::ModeType>(status.mode.val);

        bool already_switched = false;

        if (current_state != state) {
          already_switched = true;
          std::cout << "State change" << std::endl;
          if (switching_state != state && switching_state != packml_sm::State::UNDEFINED) {
            std::cout << "State published(" << state <<  ") is not the state expected (" << switching_state << ") switching to" << std::endl;
          }
          // TODO: else disabled, because currently a node cannot catch-up if it missed a state change
          // else {
            current_state = state;
            waiting_for_new_state = false;
            std::cout << "Status changed to: State: " << state << std::endl;
            on_status_changed();
          // }
        }

        if (current_mode != mode) {
          if (already_switched) {
            std::cout << "State and Mode switch detected!" << std::endl;
          }
          std::cout << "Mode change" << std::endl;
          if (switching_mode != mode && switching_mode != packml_sm::ModeType::UNDEFINED) {
            std::cout << "Mode published(" << mode << ") is not the Mode expected (" << switching_mode << ") switching to" << std::endl;
          }
          // TODO: else disabled, because currently a node cannot catch-up if it missed a mode change
          // else {
            current_mode = mode;
            waiting_for_new_mode = false;
            std::cout << "Mode changed to:" << mode << std::endl;
            on_status_changed();
          // }
        }
      };

    trans_server_ = node->create_service<packml_msgs::srv::StateTransition>("~/packml_state_transition", onStateTranseReq);
    mode_server_ = node->create_service<packml_msgs::srv::ModeTransition>("~/packml_mode_transition", onModeTransReq);
    status_sub_ = node->create_subscription<packml_msgs::msg::Status>("packml_status", rclcpp::SensorDataQoS(), onStatusChanged);

    std::cout << "Services created!" << std::endl;

    current_mode = packml_sm::ModeType::UNDEFINED;
    current_state = packml_sm::State::UNDEFINED;
    switching_mode = packml_sm::ModeType::UNDEFINED;
    switching_state = packml_sm::State::UNDEFINED;
  }

  virtual bool on_state_trans_req() = 0;

  virtual bool on_mode_trans_req() = 0;

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
    auto state_tr_service_name = name + "/packml_state_transition";
    auto mode_tr_service_name = name + "/packml_mode_transition";
    // auto status_sub_name = "packml_status";

    callback_grp = parent_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callbck_grp_exec.add_callback_group(callback_grp, parent_node->get_node_base_interface());

    state_tr_client = parent_node->create_client<packml_msgs::srv::StateTransition>(state_tr_service_name, rmw_qos_profile_services_default, callback_grp);
    mode_tr_client = parent_node->create_client<packml_msgs::srv::ModeTransition>(mode_tr_service_name, rmw_qos_profile_services_default, callback_grp);
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
      // std::cout << "Requesting node " << key << " to change mode to: " << switching_mode << std::endl;

      if (!func(val)->wait_for_service(std::chrono::seconds(1))){
        std::cout << "Client :" << client << " Service: "<< func(val)->get_service_name() << " is unavailable!" << std::endl;
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
        std::cout << "No futures to wait on!" << std::endl;
        return false;
      } else if (futures.size() != client_map_.size()) {
        // TODO: see line 243
        std::cout << "Not all clients responded with a future, maybe some are offline?" << std::endl;
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
           std::cout << client_name << ": Spinnend once" << std::endl;
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
    std::cout << "Borrowing message" << std::endl;
    auto msg = status_pub_->borrow_loaned_message();

    auto state = packml_msgs::msg::State();
    // TODO: make mapping between packml_msgs::msg::State constant declarations and packml_sm::State
    state.val = static_cast<signed char>(current_state);
    // state.set__val(current_state);
    msg.get().state = state;

    std::cout << "Current state: " << current_state << std::endl;

    auto mode = packml_msgs::msg::Mode();
    // TODO: make mapping between packml_msgs::msg::Mode constant declarations and packml_sm::Mode
    mode.val = static_cast<signed char>(current_mode);
    msg.get().mode = mode;

    std::cout << "Current mode: " << current_mode << std::endl;

    std::cout << "publising message" << std::endl;
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
              std::cout << client << " switched to new mode" << std::endl;
              return true;
            }

            std::cout << client << " did not switch mode! Error: " << value->message << std::endl;
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
        res->error_code = 1;
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
    bool success = true;

    auto command = packml_ros::to_transition_cmd(req->command);

    if (command == packml_sm::TransitionCmd::NO_COMMAND) {
      // invalid command!
      error_message =  "Unrecognized transition request command: " + to_string(command);
      res->success = false;
      res->error_code = res->UNRECGONIZED_REQUEST;
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
        std::cout <<  "Starting wait on promise" << std::endl;
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
    //       res->error_code = res->UNRECGONIZED_REQUEST;
    //       res->message = ss.str();
    //     }


  }

  void on_all_status(std::shared_ptr<packml_msgs::srv::AllStatus::Request> req, std::shared_ptr<packml_msgs::srv::AllStatus::Response> res) {
    // TODO: change the packml_msgs::srv::AllStatus to just contain packml_msgs::msg::Status.
  };

protected:

  void init(rclcpp::Node::SharedPtr node, std::shared_ptr<packml_sm::StateMachine> sm) {

    node_ = node;
    sm_ = sm;

    node->declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> node_names_;

    node_names_= node->get_parameter("node_names").as_string_array();

    current_mode = packml_sm::ModeType::UNDEFINED;
    current_state = packml_sm::State::UNDEFINED;
    switching_mode = packml_sm::ModeType::UNDEFINED;
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
    status_pub_ = node->create_publisher<packml_msgs::msg::Status>("packml_status", rclcpp::SensorDataQoS());

  }

};