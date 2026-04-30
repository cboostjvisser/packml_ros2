// Software License Agreement (Apache License)
//
// Copyright (c) 2017 Austin Deric
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

#ifndef PACKML_PLUGIN__PACKML_WIDGET_HPP_
#define PACKML_PLUGIN__PACKML_WIDGET_HPP_

#include <QtGui>
#include <QtCore>
#include <QtWidgets>

#include <memory>
#include <mutex>

#include "./ui_packml.h"  // UI layout and components
#include "rclcpp/rclcpp.hpp"  // ROS 2 node
#include "packml_msgs/msg/status.hpp"
#include "packml_msgs/srv/state_change.hpp"  // Datatypes for packml topics and services
#include "packml_msgs/srv/all_status.hpp"


/**
 * @brief This class creates a Widget (GUI) for an RViz plugin, to control and monitor a
 * PackML state machine
 */
class PackmlWidget : public QWidget
{
  Q_OBJECT

public:
  /**
  * @brief Constructor of the class that creates the GUI plugin in Rviz, inherits a Widget
  */
  explicit PackmlWidget(QWidget * parent = 0);

  void setServiceNames(const std::string & transition_service, const std::string & status_service);
  void setStatusSource(const std::string & status_source, const std::string & status_topic);

  /**
  * @brief Destructor for the widget object
  */
  ~PackmlWidget() override {
    if (ros_spin_thread_.joinable()) ros_spin_thread_.join();
  }

  /**
  * @brief Node for the Widget or RViz plugin
  */
  std::shared_ptr<rclcpp::Node> nh_;


  /**
  * @brief Creates the GUI layout with an UI object as reference
  */
  std::unique_ptr<Ui::PackmlPanel> ui_;


  /**
  * @brief Function called indirectly from a QTimer to update the status of the state machine and enable
  * or disable the buttons that can be pressed from that state according to the standard
  * PackML state machine description
  * @param msg - Response message from an AllStatus service
  */
  void updateButtonState(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg);


  /**
  * @brief Function that disables all buttons, used by the button update state function
  */
  void disableAllButtons();


  /**
  * @brief Client to request transitions on the state machine from the real PLC
  */
  rclcpp::Client<packml_msgs::srv::StateChange>::SharedPtr transition_client_;


  /**
  * @brief Client to request an update on the real PLC's state
  */
  rclcpp::Client<packml_msgs::srv::AllStatus>::SharedPtr status_client_;
  rclcpp::Subscription<packml_msgs::msg::Status>::SharedPtr status_sub_;

  // Cached latest status snapshot from topic mode for UI/state updates.
  std::shared_ptr<packml_msgs::srv::AllStatus::Response> topic_status_cache_;
  int8_t topic_current_state_ = 0;
  std::mutex topic_cache_mutex_;


  /**
  * @brief Function called indirectly from a QTimer to update the elapsed time of the state machine
  * @param msg - Response message from an AllStatus service
  */
  void callbackTime(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg);

public Q_SLOTS:
  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onStartButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onAbortButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onClearButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onHoldButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onResetButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onUnsuspendButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onUnholdButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onSuspendButton();

  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed
  */
  void onStopButton();

private:
  // Internal polling infrastructure for periodic status refresh.
  QTimer * poll_timer_ = nullptr;
  std::thread ros_spin_thread_;
  void pollStatusAsync();

  /**
  * @brief Helper to send a PackML state transition command to the backend.
  * @param command The PackML command (from packml_msgs::srv::StateChange::Request)
  * This function waits for the transition service, sends the command, and disables all buttons briefly.
  */
  void sendTransitionCommand(int command);
};

#endif  // PACKML_PLUGIN__PACKML_WIDGET_HPP_
