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

#ifndef PACKML_ROS__PACKML_ROS_HPP_
#define PACKML_ROS__PACKML_ROS_HPP_

#include <QtCore>

#include <memory>
#include <chrono>
#include <thread>
#include <sstream>
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_sm/common.hpp"
#include "packml_sm/state_machine.hpp"
#include "rclcpp/rclcpp.hpp"

#include <packml_msgs/srv/state_change.hpp>
#include <packml_msgs/srv/all_status.hpp>
#include <packml_msgs/srv//mode_change.hpp>

// Global variables for the node topics and services
/**
* @brief Variable that defines the number of state to transition to
*/
class SMNode : PackmlManagerInterface
{
  // int command_int = 0;

  /**
  * @brief State machine object
  */
  std::shared_ptr<packml_sm::StateMachine> sm;

  /**
  * @brief Pointer for transition service server
  */
  rclcpp::Service<packml_msgs::srv::StateChange>::SharedPtr trans_server_;

  /**
  * @brief Pointer for state and elapsed time status update service server
  */
  rclcpp::Service<packml_msgs::srv::AllStatus>::SharedPtr status_server_;

  rclcpp::Service<packml_msgs::srv::ModeChange>::SharedPtr mode_server_;

  // Global variables to keep track of time the SM has been in a state
  /**
  * @brief Counter for elapsed time in the stopped state
  */
  float stopped_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the idle state
  */
  float idle_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the starting state
  */
  float starting_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the execute state
  */
  float execute_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the completing state
  */
  float completing_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the complete state
  */
  float complete_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the clearing state
  */
  float clearing_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the suspended state
  */
  float suspended_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the aborting state
  */
  float aborting_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the aborted state
  */
  float aborted_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the holding state
  */
  float holding_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the held state
  */
  float held_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the unholding state
  */
  float unholding_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the suspending state
  */
  float suspending_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the unsuspending state
  */
  float unsuspending_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the resetting state
  */
  float resetting_state_t = 0.0;

  /**
  * @brief Counter for elapsed time in the stopping state
  */
  float stopping_state_t = 0.0;

public:
  /**
  * @brief The class constructor
  */
  explicit SMNode(rclcpp::Node::SharedPtr node)
  {

    // Create SM and connect to Qt components
    // Execute method runs forever until stopped
    // sm = packml_sm::StateMachine::continuousCycleSM();
    sm = packml_sm::StateMachine::singleCycleSM();  // Execute method runs once

    init(node, sm);

  // try
  // {
  //   auto set_param_result = node->set_parameter(rclcpp::Parameter("node_names", std::vector<std::string>{"hvr_node"}));
  //   if (!set_param_result.successful)
  //   {
  //     throw std::runtime_error(set_param_result.reason);
  //   }
  // }
  // catch (const std::runtime_error& e)
  // {
  //   std::cout << "Error setting parameter!";
  // }

    // Needs to be calibrated with the time of the PLC
    sm->setExecute(std::bind(myExecuteMethod));
    sm->activate();

    printf("SM created\n");


    auto modeRequest = [this](const std::shared_ptr<packml_msgs::srv::ModeChange::Request> req,
                                          std::shared_ptr<packml_msgs::srv::ModeChange::Response> res)-> void {

      packml_sm::ModeType mode;

      switch (req->mode.val) {
        case packml_msgs::msg::Mode::MAINTENANCE:
          mode = packml_sm::ModeType::MAINTENANCE;
          break;
        case packml_msgs::msg::Mode::MANUAL:
          mode = packml_sm::ModeType::MANUAL;
          break;
        case packml_msgs::msg::Mode::PRODUCTION:
          mode = packml_sm::ModeType::PRODUCTION;
          break;
        case packml_msgs::msg::Mode::UNDEFINED:
        default:
          mode = packml_sm::ModeType::UNDEFINED;
          break;
      }

      auto succes = sm->changeMode(mode);

      if (succes.has_value())
      {
        res->success = true;
        res->error_code = res->SUCCESS;
        res->message = "Succes!";
      }
      else
      {
        res->success = false;
        res->error_code = res->INVALID_MODE_REQUEST;
        res->message = succes.error();
      }

    };

    /**
    * @brief Callback function upon transition request by a client
    * @param req - data coming from the client
    * @param res - response to the client
    */
    auto transRequest =
      [this](const std::shared_ptr<packml_msgs::srv::StateChange::Request> req,
        std::shared_ptr<packml_msgs::srv::StateChange::Response> res) -> void {
        bool command_rtn = false;
        bool command_valid = true;
        auto command_int = static_cast<int>(req->command);
        std::stringstream ss;
        std::cout << "Evaluating transition request command: " << command_int << std::endl;
        switch (command_int) {
          case packml_msgs::srv::StateChange::Request::ABORT:
            command_rtn = sm->abort();
            break;
          case packml_msgs::srv::StateChange::Request::STOP:
            command_rtn = sm->stop();
            break;
          case packml_msgs::srv::StateChange::Request::CLEAR:
            command_rtn = sm->clear();
            break;
          case packml_msgs::srv::StateChange::Request::HOLD:
            command_rtn = sm->hold();
            break;
          case packml_msgs::srv::StateChange::Request::RESET:
            command_rtn = sm->reset();
            break;
          case packml_msgs::srv::StateChange::Request::START:
            command_rtn = sm->start();
            break;
          // case packml_msgs::srv::StateChange::Request::STOP:
          //   command_rtn = sm->stop();
          //   break;
          case packml_msgs::srv::StateChange::Request::SUSPEND:
            command_rtn = sm->suspend();
            break;
          case packml_msgs::srv::StateChange::Request::UNHOLD:
            command_rtn = sm->unhold();
            break;
          case packml_msgs::srv::StateChange::Request::UNSUSPEND:
            command_rtn = sm->unsuspend();
            break;
          default:
            command_valid = false;
            break;
        }
        if (command_valid) {
          if (command_rtn) {
            ss << "Successful transition request command: " << command_int;
            res->success = true;
            res->error_code = res->SUCCESS;
            res->message = ss.str();
          } else {
            ss << "Invalid transition request command: " << command_int;
            res->success = false;
            res->error_code = res->INVALID_TRANSITION_REQUEST;
            res->message = ss.str();
          }
        } else {
          ss << "Unrecognized transition request command: " << command_int;
          res->success = false;
          res->error_code = res->UNRECGONIZED_REQUEST;
          res->message = ss.str();
        }
      };

    /**
    * @brief Callback function upon status update request by a client
    * @param req - data coming from the client
    * @param res - response to the client
    */
    auto statusRequest =
      [this](const std::shared_ptr<packml_msgs::srv::AllStatus::Request> req,
        std::shared_ptr<packml_msgs::srv::AllStatus::Response> res) -> void {
        (void)req;
        packml_sm::State curr_state_ = getCurrentState();
        res->stopped_state = false;
        res->idle_state = false;
        res->starting_state = false;
        res->execute_state = false;
        res->completing_state = false;
        res->complete_state = false;
        res->clearing_state = false;
        res->suspended_state = false;
        res->aborting_state = false;
        res->aborted_state = false;
        res->holding_state = false;
        res->held_state = false;
        res->unholding_state = false;
        res->suspending_state = false;
        res->unsuspending_state = false;
        res->resetting_state = false;
        res->stopping_state = false;
        switch (curr_state_) {
          case packml_sm::State::STOPPED:
            res->stopped_state = true;
            stopped_state_t = stopped_state_t + 0.2;
            break;
          case packml_sm::State::STARTING:
            res->starting_state = true;
            starting_state_t = starting_state_t + 0.2;
            break;
          case packml_sm::State::IDLE:
            res->idle_state = true;
            idle_state_t = idle_state_t + 0.2;
            break;
          case packml_sm::State::SUSPENDED:
            res->suspended_state = true;
            suspended_state_t = suspended_state_t + 0.2;
            break;
          case packml_sm::State::EXECUTE:
            res->execute_state = true;
            execute_state_t = execute_state_t + 0.2;
            break;
          case packml_sm::State::STOPPING:
            res->stopping_state = true;
            stopping_state_t = stopping_state_t + 0.2;
            break;
          case packml_sm::State::ABORTING:
            res->aborting_state = true;
            aborting_state_t = aborting_state_t + 0.2;
            break;
          case packml_sm::State::ABORTED:
            res->aborted_state = true;
            aborted_state_t = aborted_state_t + 0.2;
            break;
          case packml_sm::State::HOLDING:
            res->holding_state = true;
            holding_state_t = holding_state_t + 0.2;
            break;
          case packml_sm::State::HELD:
            res->held_state = true;
            held_state_t = held_state_t + 0.2;
            break;
          case packml_sm::State::RESETTING:
            res->resetting_state = true;
            resetting_state_t = resetting_state_t + 0.2;
            break;
          case packml_sm::State::SUSPENDING:
            res->suspending_state = true;
            suspending_state_t = suspending_state_t + 0.2;
            break;
          case packml_sm::State::UNSUSPENDING:
            res->unsuspending_state = true;
            unsuspending_state_t = unsuspending_state_t + 0.2;
            break;
          case packml_sm::State::CLEARING:
            res->clearing_state = true;
            clearing_state_t = clearing_state_t + 0.2;
            break;
          case packml_sm::State::UNHOLDING:
            res->unholding_state = true;
            unholding_state_t = unholding_state_t + 0.2;
            break;
          case packml_sm::State::COMPLETING:
            res->completing_state = true;
            completing_state_t = completing_state_t + 0.2;
            break;
          case packml_sm::State::COMPLETE:
            res->complete_state = true;
            complete_state_t = complete_state_t + 0.2;
            break;
          default:
            break;
        }
        res->t_stopped_state = stopped_state_t;
        res->t_starting_state = starting_state_t;
        res->t_idle_state = idle_state_t;
        res->t_suspended_state = suspended_state_t;
        res->t_execute_state = execute_state_t;
        res->t_stopping_state = stopping_state_t;
        res->t_aborting_state = aborting_state_t;
        res->t_aborted_state = aborted_state_t;
        res->t_holding_state = holding_state_t;
        res->t_held_state = held_state_t;
        res->t_resetting_state = resetting_state_t;
        res->t_suspending_state = suspending_state_t;
        res->t_unsuspending_state = unsuspending_state_t;
        res->t_clearing_state = clearing_state_t;
        res->t_unholding_state = unholding_state_t;
        res->t_completing_state = completing_state_t;
        res->t_complete_state = complete_state_t;
      };
    // Create service to control the execution of the SM from RViz GUI
    trans_server_ = node->create_service<packml_msgs::srv::StateChange>("~/transition", transRequest);
    status_server_ = node->create_service<packml_msgs::srv::AllStatus>("~/allStatus", statusRequest);
    mode_server_ = node->create_service<packml_msgs::srv::ModeChange>("~/modeChange", modeRequest);
  }

  /**
  * @brief The class constructor
  */
  virtual ~SMNode() {}

  /**
  * @brief Function to bind to the Execute state for the state machine, waiting for a set amount
  * of time
  * @return 0 - when completed
  */
  static int myExecuteMethod()
  {
    printf("This is my execute method (begin)\n");
    // while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    printf("This is my execute method (end)\n");
    return 0;  // returning zero indicates non-failure
  }

  /**
  * @brief Function to query for the current state machine state.
  * @return state int
  */
  virtual packml_sm::State getCurrentState()
  {
    return sm->getCurrentState();
  }
};

/**
* @brief Function to be run in a thread to execute a QT object for a state machine
*/
void qtWorker(int argc, char * argv[])
{
  QCoreApplication a(argc, argv);
  a.exec();
  printf("Thread ready\n");
}

#endif  // PACKML_ROS__PACKML_ROS_HPP_
