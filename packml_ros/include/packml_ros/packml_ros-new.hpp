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
#include "packml_sm/modes_config.hpp"
#include "packml_sm/common.hpp"
#include "packml_sm/state_machine.hpp"
#include "rclcpp/rclcpp.hpp"

#include <packml_msgs/srv/state_change.hpp>
#include <packml_msgs/srv/all_status.hpp>
// #include <packml_msgs/srv  //mode_change.hpp>

// Global variables for the node topics and services
/**
 * @brief Variable that defines the number of state to transition to
 */
class SMNode_new : PackmlManagerInterface
{
  // int command_int = 0;

  /**
   * @brief State machine object
   */
  std::shared_ptr<packml_sm::StateMachine> sm;

  // /**
  // * @brief Pointer for transition service server
  // */
  // rclcpp::Service<packml_msgs::srv::StateChange>::SharedPtr trans_server_;

  // /**
  // * @brief Pointer for state and elapsed time status update service server
  // */
  // rclcpp::Service<packml_msgs::srv::AllStatus>::SharedPtr status_server_;

  // rclcpp::Service<packml_msgs::srv::ModeChange>::SharedPtr mode_server_;

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
  explicit SMNode_new(rclcpp::Node::SharedPtr node)
  {


    // try
    // {
    //   auto set_param_result = node->set_parameter(rclcpp::Parameter("node_names",
    //   std::vector<std::string>{"test_packml_node"})); if (!set_param_result.successful)
    //   {
    //     throw std::runtime_error(set_param_result.reason);
    //   }
    // }
    // catch (const std::runtime_error& e)
    // {
    //   std::cout << "Error setting parameter!";
    // }

    // Create continuous-cycle SM (runs EXECUTE in a loop until STOP).
    sm = packml_sm::StateMachine::continuousCycleSM();

    sm->on_state_changed = [this](packml_sm::State value, QString name) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("packml_ros"), "State changed to: " << name.toStdString() << "(" << value << ")");

      // The state machine already changed state and IS the source of truth, so the
      // manager's view and the latched status update immediately and unconditionally.
      // Blocking the Qt thread here to wait for every Equipment Module to acknowledge
      // would, on any failure, skip the update entirely — leaving current_state stale
      // and the status topic silent about a transition that has already happened.
      current_state.store(value);

      // When the machine returns to STOPPED (e.g. operator CLEAR), re-arm the health
      // timeout check so still-silent required Equipment Modules immediately trigger
      // ABORT again instead of leaving the machine stuck.
      if (value == packml_sm::State::STOPPED) {
        rearm_health_timeouts();
      }

      // Notify the Equipment Modules without blocking this (Qt) thread; missing,
      // rejected, or failed-to-complete acknowledgements are surfaced as WARN logs/alarms
      // by the fan-out, and feed completion_tracker_ so the setStateOperation-bound
      // function waiting on this state's real completion (see init()) can resolve. Fan out
      // BEFORE publishing status: an EM that saw the new state on the latched status topic
      // first would answer the transition request with the "already there" shortcut and
      // skip its on_state_trans_req() hook. Initiating the requests first preserves the
      // request-then-status order in the common case (delivery order across an action and
      // a topic is not strictly guaranteed, so EM hooks should not depend on it — react to
      // on_status_changed for authoritative state).
      fanout_state_transition(value);

      publish_status();
    };

    init(node, sm);

    // Initial mode is configurable via the 'initial_mode' parameter (int).
    // Default is 0 (Invalid/undefined). Users should set this to the desired
    // starting mode value defined in their modes YAML file.
    node->declare_parameter(packml_ros::kParamInitialMode, 0);
    auto initial_mode = static_cast<packml_sm::ModeType>(node->get_parameter(packml_ros::kParamInitialMode).as_int());

    // 'modes_config_file' is declared and parsed by PackmlManagerInterface::init() (called
    // above), which owns the resulting table. Declaring it a second time here would throw
    // "already been declared".
    //
    // Uses that same kept table (mode_masks_) rather than parsing a second copy here: a copy
    // parsed and then discarded leaves ~/changeMode with nothing to apply, so the first runtime
    // mode change wipes the configured mask. One table, one owner, consulted identically by this
    // boot call and by ~/changeMode.
    {
      auto it = mode_masks_.find(initial_mode);
      if (it != mode_masks_.end()) {
        sm->changeMode(initial_mode, it->second);
      } else {
        sm->changeMode(initial_mode);
      }
    }

    // EXECUTE holds until something commands the machine out of it. Bound explicitly, rather than
    // left to the library's hold-when-unbound behaviour, because the two are indistinguishable at
    // runtime and only one of them is a decision: this manager coordinates equipment modules, it
    // does not run the machine's own production logic, so ending a batch is an external event
    // (the orchestrator's Stop today; the standard's Complete command once that exists) and never
    // a timer expiring here. The commented-out binding this replaces was
    // `setExecute(std::bind(myExecuteMethod))`, calibrated "with the time of the PLC" -- a
    // deployment that genuinely has per-cycle production logic to run should bind it here and get
    // a real EXECUTE --SC--> boundary, not resurrect that stub, which returned immediately.
    sm->setInterruptibleStateOperation(packml_sm::State::EXECUTE,
      [](std::stop_token stop_token) -> int {
        while (!stop_token.stop_requested()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return 0;
      });

    // A manager whose state machine did not start is worse than one that is absent: init() above
    // has already advertised every service and created the status publisher, so the node passes
    // every liveness check an operator or a launch file can make, while never entering a state,
    // never publishing one, and refusing every command. Refuse to exist instead, and let the
    // supervisor restart or report it.
    if (!sm->activate()) {
      const auto faults = sm->errorEscapeReport();
      for (const auto & fault : faults) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("packml_ros"),
          "State machine graph is unsafe to run: " << fault);
      }
      throw std::runtime_error(
        faults.empty()
          ? "packml state machine failed to activate (no QCoreApplication on this process?)"
          : "packml state machine failed to activate: its graph has acting states whose failure "
            "has nowhere to go");
    }

    printf("SM created\n");

    // auto modeRequest = [this](const std::shared_ptr<packml_msgs::srv::ModeChange::Request> req,
    //                                       std::shared_ptr<packml_msgs::srv::ModeChange::Response> res)-> void {

    //   packml_sm::ModeType mode;

    //   switch (req->mode.val) {
    //     case packml_msgs::msg::Mode::MAINTENANCE:
    //       mode = packml_sm::ModeType::MAINTENANCE;
    //       break;
    //     case packml_msgs::msg::Mode::MANUAL:
    //       mode = packml_sm::ModeType::MANUAL;
    //       break;
    //     case packml_msgs::msg::Mode::PRODUCTION:
    //       mode = packml_sm::ModeType::PRODUCTION;
    //       break;
    //     case packml_msgs::msg::Mode::UNDEFINED:
    //     default:
    //       mode = packml_sm::ModeType::UNDEFINED;
    //       break;
    //   }

    //   auto succes = sm->changeMode(mode);

    //   if (succes.has_value())
    //   {
    //     res->success = true;
    //     res->error_code = res->SUCCESS;
    //     res->message = "Succes!";
    //   }
    //   else
    //   {
    //     res->success = false;
    //     res->error_code = res->INVALID_MODE_REQUEST;
    //     res->message = succes.error();
    //   }

    // };

    // /**
    // * @brief Callback function upon transition request by a client
    // * @param req - data coming from the client
    // * @param res - response to the client
    // */
    // auto transRequest =
    //   [this](const std::shared_ptr<packml_msgs::srv::StateChange::Request> req,
    //     std::shared_ptr<packml_msgs::srv::StateChange::Response> res) -> void {
    //     bool command_rtn = false;
    //     bool command_valid = true;
    //     auto command_int = static_cast<int>(req->command);
    //     std::stringstream ss;
    //     std::cout << "Evaluating transition request command: " << command_int << std::endl;
    //     switch (command_int) {
    //       case packml_msgs::srv::StateChange::Request::ABORT:
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
    //   };

    // /**
    // * @brief Callback function upon status update request by a client
    // * @param req - data coming from the client
    // * @param res - response to the client
    // */
    // auto statusRequest =
    //   [this](const std::shared_ptr<packml_msgs::srv::AllStatus::Request> req,
    //     std::shared_ptr<packml_msgs::srv::AllStatus::Response> res) -> void {
    //     (void)req;
    //     packml_sm::State curr_state_ = getCurrentState();
    //     res->stopped_state = false;
    //     res->idle_state = false;
    //     res->starting_state = false;
    //     res->execute_state = false;
    //     res->completing_state = false;
    //     res->complete_state = false;
    //     res->clearing_state = false;
    //     res->suspended_state = false;
    //     res->aborting_state = false;
    //     res->aborted_state = false;
    //     res->holding_state = false;
    //     res->held_state = false;
    //     res->unholding_state = false;
    //     res->suspending_state = false;
    //     res->unsuspending_state = false;
    //     res->resetting_state = false;
    //     res->stopping_state = false;
    //     switch (curr_state_) {
    //       case packml_sm::State::STOPPED:
    //         res->stopped_state = true;
    //         stopped_state_t = stopped_state_t + 0.2;
    //         break;
    //       case packml_sm::State::STARTING:
    //         res->starting_state = true;
    //         starting_state_t = starting_state_t + 0.2;
    //         break;
    //       case packml_sm::State::IDLE:
    //         res->idle_state = true;
    //         idle_state_t = idle_state_t + 0.2;
    //         break;
    //       case packml_sm::State::SUSPENDED:
    //         res->suspended_state = true;
    //         suspended_state_t = suspended_state_t + 0.2;
    //         break;
    //       case packml_sm::State::EXECUTE:
    //         res->execute_state = true;
    //         execute_state_t = execute_state_t + 0.2;
    //         break;
    //       case packml_sm::State::STOPPING:
    //         res->stopping_state = true;
    //         stopping_state_t = stopping_state_t + 0.2;
    //         break;
    //       case packml_sm::State::ABORTING:
    //         res->aborting_state = true;
    //         aborting_state_t = aborting_state_t + 0.2;
    //         break;
    //       case packml_sm::State::ABORTED:
    //         res->aborted_state = true;
    //         aborted_state_t = aborted_state_t + 0.2;
    //         break;
    //       case packml_sm::State::HOLDING:
    //         res->holding_state = true;
    //         holding_state_t = holding_state_t + 0.2;
    //         break;
    //       case packml_sm::State::HELD:
    //         res->held_state = true;
    //         held_state_t = held_state_t + 0.2;
    //         break;
    //       case packml_sm::State::RESETTING:
    //         res->resetting_state = true;
    //         resetting_state_t = resetting_state_t + 0.2;
    //         break;
    //       case packml_sm::State::SUSPENDING:
    //         res->suspending_state = true;
    //         suspending_state_t = suspending_state_t + 0.2;
    //         break;
    //       case packml_sm::State::UNSUSPENDING:
    //         res->unsuspending_state = true;
    //         unsuspending_state_t = unsuspending_state_t + 0.2;
    //         break;
    //       case packml_sm::State::CLEARING:
    //         res->clearing_state = true;
    //         clearing_state_t = clearing_state_t + 0.2;
    //         break;
    //       case packml_sm::State::UNHOLDING:
    //         res->unholding_state = true;
    //         unholding_state_t = unholding_state_t + 0.2;
    //         break;
    //       case packml_sm::State::COMPLETING:
    //         res->completing_state = true;
    //         completing_state_t = completing_state_t + 0.2;
    //         break;
    //       case packml_sm::State::COMPLETE:
    //         res->complete_state = true;
    //         complete_state_t = complete_state_t + 0.2;
    //         break;
    //       default:
    //         break;
    //     }
    //     res->t_stopped_state = stopped_state_t;
    //     res->t_starting_state = starting_state_t;
    //     res->t_idle_state = idle_state_t;
    //     res->t_suspended_state = suspended_state_t;
    //     res->t_execute_state = execute_state_t;
    //     res->t_stopping_state = stopping_state_t;
    //     res->t_aborting_state = aborting_state_t;
    //     res->t_aborted_state = aborted_state_t;
    //     res->t_holding_state = holding_state_t;
    //     res->t_held_state = held_state_t;
    //     res->t_resetting_state = resetting_state_t;
    //     res->t_suspending_state = suspending_state_t;
    //     res->t_unsuspending_state = unsuspending_state_t;
    //     res->t_clearing_state = clearing_state_t;
    //     res->t_unholding_state = unholding_state_t;
    //     res->t_completing_state = completing_state_t;
    //     res->t_complete_state = complete_state_t;
    //   };

    //   // Create service to control the execution of the SM from RViz GUI
    //   trans_server_ = node->create_service<packml_msgs::srv::StateChange>("~/transition", transRequest);
    //   status_server_ = node->create_service<packml_msgs::srv::AllStatus>("~/allStatus", statusRequest);
    //   mode_server_ = node->create_service<packml_msgs::srv::ModeChange>("~/modeChange", modeRequest);
  }

  /**
   * @brief The class destructor
   */
  virtual ~SMNode_new()
  {
    // MUST run first, before any member (in particular `sm`) starts unwinding: this
    // wakes any in-flight CompletionTracker wait so StateMachine::drainActingStates()
    // (invoked from ~StateMachine(), which has no internal timeout) doesn't stall
    // teardown for up to state_complete_timeout_ms. See PackmlManagerInterface::shutdown().
    shutdown();
  }

  // /**
  // * @brief Function to bind to the Execute state for the state machine, waiting for a set amount
  // * of time
  // * @return 0 - when completed
  // */
  // static int myExecuteMethod()
  // {
  //   printf("This is my execute method (begin)\n");
  //   // while (rclcpp::ok()) {
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  //   // }
  //   printf("This is my execute method (end)\n");
  //   return 0;  // returning zero indicates non-failure
  // }

  /**
   * @brief Function to query for the current state machine state.
   * @return state int
   */
  virtual packml_sm::State getCurrentState() { return sm->getCurrentState(); }
};

/**
 * @brief Function to be run in a thread to execute a QT object for a state machine
 */
inline void qtWorker(int argc, char* argv[])
{
  QCoreApplication a(argc, argv);
  a.exec();
  printf("Thread ready\n");
}

#endif  // PACKML_ROS__PACKML_ROS_HPP_
