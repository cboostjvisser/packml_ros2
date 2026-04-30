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


#include <algorithm>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <packml_msgs/msg/state.hpp>
#include <packml_msgs/srv/state_change.hpp>
#include <sstream>
#include <memory>
#include <string>
#include "packml_plugin/packml_widget.hpp"
#include <QTimer>
#include <QMetaObject>
#include <QPushButton>
#include <memory>
#include <thread>
#include <utility>

namespace
{
void clearStatusFlags(packml_msgs::srv::AllStatus::Response & msg)
{
  msg.stopped_state = false;
  msg.idle_state = false;
  msg.starting_state = false;
  msg.execute_state = false;
  msg.completing_state = false;
  msg.complete_state = false;
  msg.clearing_state = false;
  msg.suspended_state = false;
  msg.aborting_state = false;
  msg.aborted_state = false;
  msg.holding_state = false;
  msg.held_state = false;
  msg.unholding_state = false;
  msg.suspending_state = false;
  msg.unsuspending_state = false;
  msg.resetting_state = false;
  msg.stopping_state = false;
}

void setStateFlag(packml_msgs::srv::AllStatus::Response & msg, int8_t state)
{
  clearStatusFlags(msg);
  switch (state) {
    case packml_msgs::msg::State::STOPPED:
      msg.stopped_state = true;
      break;
    case packml_msgs::msg::State::IDLE:
      msg.idle_state = true;
      break;
    case packml_msgs::msg::State::STARTING:
      msg.starting_state = true;
      break;
    case packml_msgs::msg::State::EXECUTE:
      msg.execute_state = true;
      break;
    case packml_msgs::msg::State::COMPLETING:
      msg.completing_state = true;
      break;
    case packml_msgs::msg::State::COMPLETE:
      msg.complete_state = true;
      break;
    case packml_msgs::msg::State::CLEARING:
      msg.clearing_state = true;
      break;
    case packml_msgs::msg::State::SUSPENDED:
      msg.suspended_state = true;
      break;
    case packml_msgs::msg::State::ABORTING:
      msg.aborting_state = true;
      break;
    case packml_msgs::msg::State::ABORTED:
      msg.aborted_state = true;
      break;
    case packml_msgs::msg::State::HOLDING:
      msg.holding_state = true;
      break;
    case packml_msgs::msg::State::HELD:
      msg.held_state = true;
      break;
    case packml_msgs::msg::State::UNHOLDING:
      msg.unholding_state = true;
      break;
    case packml_msgs::msg::State::SUSPENDING:
      msg.suspending_state = true;
      break;
    case packml_msgs::msg::State::UNSUSPENDING:
      msg.unsuspending_state = true;
      break;
    case packml_msgs::msg::State::RESETTING:
      msg.resetting_state = true;
      break;
    case packml_msgs::msg::State::STOPPING:
      msg.stopping_state = true;
      break;
    default:
      break;
  }
}

void incrementStateDwell(packml_msgs::srv::AllStatus::Response & msg, int8_t state, double dt)
{
  switch (state) {
    case packml_msgs::msg::State::STOPPED:
      msg.t_stopped_state += dt;
      break;
    case packml_msgs::msg::State::IDLE:
      msg.t_idle_state += dt;
      break;
    case packml_msgs::msg::State::STARTING:
      msg.t_starting_state += dt;
      break;
    case packml_msgs::msg::State::EXECUTE:
      msg.t_execute_state += dt;
      break;
    case packml_msgs::msg::State::COMPLETING:
      msg.t_completing_state += dt;
      break;
    case packml_msgs::msg::State::COMPLETE:
      msg.t_complete_state += dt;
      break;
    case packml_msgs::msg::State::CLEARING:
      msg.t_clearing_state += dt;
      break;
    case packml_msgs::msg::State::SUSPENDED:
      msg.t_suspended_state += dt;
      break;
    case packml_msgs::msg::State::ABORTING:
      msg.t_aborting_state += dt;
      break;
    case packml_msgs::msg::State::ABORTED:
      msg.t_aborted_state += dt;
      break;
    case packml_msgs::msg::State::HOLDING:
      msg.t_holding_state += dt;
      break;
    case packml_msgs::msg::State::HELD:
      msg.t_held_state += dt;
      break;
    case packml_msgs::msg::State::UNHOLDING:
      msg.t_unholding_state += dt;
      break;
    case packml_msgs::msg::State::SUSPENDING:
      msg.t_suspending_state += dt;
      break;
    case packml_msgs::msg::State::UNSUSPENDING:
      msg.t_unsuspending_state += dt;
      break;
    case packml_msgs::msg::State::RESETTING:
      msg.t_resetting_state += dt;
      break;
    case packml_msgs::msg::State::STOPPING:
      msg.t_stopping_state += dt;
      break;
    default:
      break;
  }
}

std::string normalizeStatusSource(std::string source)
{
  std::transform(source.begin(), source.end(), source.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return source;
}
}  // namespace

PackmlWidget::PackmlWidget(QWidget * parent)
: QWidget(parent)
{
  nh_ = rclcpp::Node::make_shared("uiPlugin");  // Creates a node
  // UI setup
  ui_ = std::make_unique<Ui::PackmlPanel>();
  ui_->setupUi(this);
  connect(ui_->start_button, &QPushButton::clicked, this, &PackmlWidget::onStartButton);
  connect(ui_->abort_button, &QPushButton::clicked, this, &PackmlWidget::onAbortButton);
  connect(ui_->clear_button, &QPushButton::clicked, this, &PackmlWidget::onClearButton);
  connect(ui_->hold_button, &QPushButton::clicked, this, &PackmlWidget::onHoldButton);
  connect(ui_->reset_button, &QPushButton::clicked, this, &PackmlWidget::onResetButton);
  connect(ui_->unsuspend_button, &QPushButton::clicked, this, &PackmlWidget::onUnsuspendButton);
  connect(ui_->unhold_button, &QPushButton::clicked, this, &PackmlWidget::onUnholdButton);
  connect(ui_->suspend_button, &QPushButton::clicked, this, &PackmlWidget::onSuspendButton);
  connect(ui_->stop_button, &QPushButton::clicked, this, &PackmlWidget::onStopButton);

  // Start ROS2 spinning in a background thread
  ros_spin_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(nh_);
    exec.spin();
  });

  // Start polling timer (Qt timer)
  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, &PackmlWidget::pollStatusAsync);
  poll_timer_->start(200);
}

void PackmlWidget::setServiceNames(const std::string & transition_service, const std::string & status_service)
{
  transition_client_ = nh_->create_client<packml_msgs::srv::StateChange>(transition_service);
  status_client_ = nh_->create_client<packml_msgs::srv::AllStatus>(status_service);
}

void PackmlWidget::setStatusSource(const std::string & status_source, const std::string & status_topic)
{
  const std::string normalized_source = normalizeStatusSource(status_source.empty() ? "service" : status_source);
  const std::string resolved_topic = status_topic.empty() ? "/packml_status" : status_topic;

  {
    std::lock_guard<std::mutex> lock(topic_cache_mutex_);
    topic_status_cache_.reset();
    topic_current_state_ = packml_msgs::msg::State::UNDEFINED;
  }

  if (normalized_source == "topic") {
    status_sub_ = nh_->create_subscription<packml_msgs::msg::Status>(
      resolved_topic,
      rclcpp::SensorDataQoS(),
      [this](const packml_msgs::msg::Status::SharedPtr msg) {
        std::shared_ptr<packml_msgs::srv::AllStatus::Response> local_status =
          std::make_shared<packml_msgs::srv::AllStatus::Response>();

        {
          std::lock_guard<std::mutex> lock(topic_cache_mutex_);
          if (topic_status_cache_) {
            *local_status = *topic_status_cache_;
          }
          setStateFlag(*local_status, msg->state.val);
          topic_current_state_ = msg->state.val;
          topic_status_cache_ = local_status;
        }

        QMetaObject::invokeMethod(this, [this]() {
          std::shared_ptr<packml_msgs::srv::AllStatus::Response> snapshot;
          {
            std::lock_guard<std::mutex> lock(topic_cache_mutex_);
            if (topic_status_cache_) {
              snapshot = std::make_shared<packml_msgs::srv::AllStatus::Response>(*topic_status_cache_);
            }
          }
          if (snapshot) {
            updateButtonState(snapshot);
          }
        }, Qt::QueuedConnection);
      });
  } else {
    status_sub_.reset();
  }
}

// Async polling using QTimer and ROS2 async service call
void PackmlWidget::pollStatusAsync()
{
  if (status_sub_) {
    std::shared_ptr<packml_msgs::srv::AllStatus::Response> snapshot;
    {
      std::lock_guard<std::mutex> lock(topic_cache_mutex_);
      if (topic_status_cache_) {
        incrementStateDwell(*topic_status_cache_, topic_current_state_, 0.2);
        snapshot = std::make_shared<packml_msgs::srv::AllStatus::Response>(*topic_status_cache_);
      }
    }
    if (snapshot) {
      updateButtonState(snapshot);
    }

    return;
  }

  if (!status_client_) {
    return;
  }

  if (!status_client_->wait_for_service(std::chrono::milliseconds(10))) {
    // Service not available yet, skip this timer tick
    return;
  }
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto future = status_client_->async_send_request(update,
    [this](rclcpp::Client<packml_msgs::srv::AllStatus>::SharedFuture result_future) {
      auto result = result_future.get();
      // Post GUI update to Qt event loop for thread safety
      QMetaObject::invokeMethod(this, [this, result]() { updateButtonState(result); }, Qt::QueuedConnection);
    });
}



void PackmlWidget::sendTransitionCommand(int command)
{
  if (!transition_client_) {
    return;
  }

  if (!transition_client_->wait_for_service(std::chrono::milliseconds(500))) {
    return;
  }
  auto trans = std::make_shared<packml_msgs::srv::StateChange::Request>();
  trans->command = command;
  transition_client_->async_send_request(trans);
  QTimer::singleShot(200, this, [this]() { disableAllButtons(); });
}

void PackmlWidget::onStartButton()      { sendTransitionCommand(packml_msgs::srv::StateChange::Request::START); }
void PackmlWidget::onAbortButton()      { sendTransitionCommand(packml_msgs::srv::StateChange::Request::ABORT); }
void PackmlWidget::onClearButton()      { sendTransitionCommand(packml_msgs::srv::StateChange::Request::CLEAR); }
void PackmlWidget::onHoldButton()       { sendTransitionCommand(packml_msgs::srv::StateChange::Request::HOLD); }
void PackmlWidget::onResetButton()      { sendTransitionCommand(packml_msgs::srv::StateChange::Request::RESET); }
void PackmlWidget::onUnsuspendButton()  { sendTransitionCommand(packml_msgs::srv::StateChange::Request::UNSUSPEND); }
void PackmlWidget::onUnholdButton()     { sendTransitionCommand(packml_msgs::srv::StateChange::Request::UNHOLD); }
void PackmlWidget::onSuspendButton()    { sendTransitionCommand(packml_msgs::srv::StateChange::Request::SUSPEND); }
void PackmlWidget::onStopButton()       { sendTransitionCommand(packml_msgs::srv::StateChange::Request::STOP); }

void PackmlWidget::updateButtonState(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg)
{
  // Only update button enabled state if it actually needs to change
  static packml_msgs::srv::AllStatus::Response last_state;
  static bool first = true;
  bool state_changed = first ||
    (last_state.stopped_state != msg->stopped_state ||
     last_state.idle_state != msg->idle_state ||
     last_state.execute_state != msg->execute_state ||
     last_state.held_state != msg->held_state ||
     last_state.suspended_state != msg->suspended_state);
  first = false;
  if (state_changed) {
    disableAllButtons();
    last_state = *msg;
  }
  // Always update dwell time display
  callbackTime(msg);
  // Only update button enabled state if it actually needs to change
  auto setEnabledIfNeeded = [](QPushButton* btn, bool enable) {
    if (btn->isEnabled() != enable) btn->setEnabled(enable);
  };

  if (msg->stopped_state == true) {
    ui_->stopped_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->abort_button, true);
    setEnabledIfNeeded(ui_->reset_button, true);
  } else {
    ui_->stopped_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->idle_state == true) {
    ui_->idle_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
    setEnabledIfNeeded(ui_->start_button, true);
  } else {
    ui_->idle_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->starting_state == true) {
    ui_->starting_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->starting_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->execute_state == true) {
    ui_->execute_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
    setEnabledIfNeeded(ui_->hold_button, true);
    setEnabledIfNeeded(ui_->suspend_button, true);
  } else {
    ui_->execute_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->completing_state == true) {
    ui_->completing_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->completing_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->complete_state == true) {
    ui_->complete_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
    setEnabledIfNeeded(ui_->reset_button, true);
  } else {
    ui_->complete_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->suspended_state == true) {
    ui_->suspended_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
    setEnabledIfNeeded(ui_->unsuspend_button, true);
  } else {
    ui_->suspended_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->aborting_state == true) {
    ui_->aborting_state->setStyleSheet("QLabel { color : red; }");
  } else {
    ui_->aborting_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->aborted_state == true) {
    ui_->aborted_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->clear_button, true);
  } else {
    ui_->aborted_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->holding_state == true) {
    ui_->holding_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->holding_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->held_state == true) {
    ui_->held_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
    setEnabledIfNeeded(ui_->unhold_button, true);
  } else {
    ui_->held_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->unholding_state == true) {
    ui_->unholding_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->unholding_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->suspending_state == true) {
    ui_->suspending_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->suspending_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->unsuspending_state == true) {
    ui_->unsuspending_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->unsuspending_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->resetting_state == true) {
    ui_->resetting_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->stop_button, true);
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->resetting_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->clearing_state == true) {
    ui_->clearing_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->clearing_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->stopping_state == true) {
    ui_->stopping_state->setStyleSheet("QLabel { color : red; }");
    setEnabledIfNeeded(ui_->abort_button, true);
  } else {
    ui_->stopping_state->setStyleSheet("QLabel { color : black; }");
  }
}

void PackmlWidget::disableAllButtons()
{
  ui_->abort_button->setEnabled(false);
  ui_->clear_button->setEnabled(false);
  ui_->hold_button->setEnabled(false);
  ui_->reset_button->setEnabled(false);
  ui_->start_button->setEnabled(false);
  ui_->stop_button->setEnabled(false);
  ui_->suspend_button->setEnabled(false);
  ui_->unhold_button->setEnabled(false);
  ui_->unsuspend_button->setEnabled(false);
}

void PackmlWidget::callbackTime(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg)
{
  std::stringstream stream0;
  stream0 << std::fixed << std::setprecision(2) << msg->t_stopped_state;
  std::string str_stopped = stream0.str();
  ui_->stopped_state->setText(QString::fromStdString("Stopped: " + str_stopped + "s"));
  std::stringstream stream1;
  stream1 << std::fixed << std::setprecision(2) << msg->t_idle_state;
  std::string str_idle = stream1.str();
  ui_->idle_state->setText(QString::fromStdString("Idle: " + str_idle + "s"));
  std::stringstream stream2;
  stream2 << std::fixed << std::setprecision(2) << msg->t_starting_state;
  std::string str_starting = stream2.str();
  ui_->starting_state->setText(QString::fromStdString("Starting: " + str_starting + "s"));
  std::stringstream stream3;
  stream3 << std::fixed << std::setprecision(2) << msg->t_execute_state;
  std::string str_execute = stream3.str();
  ui_->execute_state->setText(QString::fromStdString("Executing: " + str_execute + "s"));
  std::stringstream stream4;
  stream4 << std::fixed << std::setprecision(2) << msg->t_completing_state;
  std::string str_completing = stream4.str();
  ui_->completing_state->setText(QString::fromStdString("Completing: " + str_completing + "s"));
  std::stringstream stream5;
  stream5 << std::fixed << std::setprecision(2) << msg->t_complete_state;
  std::string str_complete = stream5.str();
  ui_->complete_state->setText(QString::fromStdString("Complete: " + str_complete + "s"));
  std::stringstream stream6;
  stream6 << std::fixed << std::setprecision(2) << msg->t_suspended_state;
  std::string str_suspended = stream6.str();
  ui_->suspended_state->setText(QString::fromStdString("Suspended: " + str_suspended + "s"));
  std::stringstream stream7;
  stream7 << std::fixed << std::setprecision(2) << msg->t_aborting_state;
  std::string str_aborting = stream7.str();
  ui_->aborting_state->setText(QString::fromStdString("Aborting: " + str_aborting + "s"));
  std::stringstream stream8;
  stream8 << std::fixed << std::setprecision(2) << msg->t_aborted_state;
  std::string str_aborted = stream8.str();
  ui_->aborted_state->setText(QString::fromStdString("Aborted: " + str_aborted + "s"));
  std::stringstream stream9;
  stream9 << std::fixed << std::setprecision(2) << msg->t_holding_state;
  std::string str_holding = stream9.str();
  ui_->holding_state->setText(QString::fromStdString("Holding: " + str_holding + "s"));
  std::stringstream stream10;
  stream10 << std::fixed << std::setprecision(2) << msg->t_held_state;
  std::string str_held = stream10.str();
  ui_->held_state->setText(QString::fromStdString("Held: " + str_held + "s"));
  std::stringstream stream11;
  stream11 << std::fixed << std::setprecision(2) << msg->t_unholding_state;
  std::string str_unholding = stream11.str();
  ui_->unholding_state->setText(QString::fromStdString("Unholding: " + str_unholding + "s"));
  std::stringstream stream12;
  stream12 << std::fixed << std::setprecision(2) << msg->t_suspending_state;
  std::string str_suspending = stream12.str();
  ui_->suspending_state->setText(QString::fromStdString("Suspending: " + str_suspending + "s"));
  std::stringstream stream13;
  stream13 << std::fixed << std::setprecision(2) << msg->t_unsuspending_state;
  std::string str_unsuspending = stream13.str();
  ui_->unsuspending_state->setText(
    QString::fromStdString("Unsuspending: " + str_unsuspending + "s"));
  std::stringstream stream14;
  stream14 << std::fixed << std::setprecision(2) << msg->t_resetting_state;
  std::string str_resetting = stream14.str();
  ui_->resetting_state->setText(QString::fromStdString("Resetting: " + str_resetting + "s"));
  std::stringstream stream15;
  stream15 << std::fixed << std::setprecision(2) << msg->t_clearing_state;
  std::string str_clearing = stream15.str();
  ui_->clearing_state->setText(QString::fromStdString("Clearing: " + str_clearing + "s"));
  std::stringstream stream16;
  stream16 << std::fixed << std::setprecision(2) << msg->t_stopping_state;
  std::string str_stopping = stream16.str();
  ui_->stopping_state->setText(QString::fromStdString("Stopping: " + str_stopping + "s"));
}
