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
// Demo: PackML Health System — Multiple Equipment Modules
//
// This file contains three Equipment Module nodes used in the health demo:
//   - DemoMotorDriver   : publishes periodic simulated motor health
//   - DemoSensorHub     : publishes sensor readings + optional fault injection
//   - DemoConveyor      : upstream-jam simulation (SUSPEND)
//
// Fault injection is controlled via ROS 2 parameters set at runtime:
//
//   ros2 param set /demo_motor_driver inject_fault "abort"
//   ros2 param set /demo_motor_driver inject_fault "hold"
//   ros2 param set /demo_motor_driver inject_fault "none"
//   ros2 param set /demo_motor_driver inject_fault "silent"   # stops heartbeat
//
// Launch with: packml_health_demo.launch.py

#include <rclcpp/rclcpp.hpp>
#include "packml_ros/interface/packml_interface.hpp"
#include "packml_msgs/msg/node_health.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

using NodeHealth = packml_msgs::msg::NodeHealth;
using namespace std::chrono_literals;

// ============================================================================
// Shared helpers
// ============================================================================

/// Returns a NodeHealth with action=NONE (healthy state)
static NodeHealth make_healthy(const std::string & note = "")
{
  NodeHealth h;
  h.status = NodeHealth::HEALTHY;
  h.action = NodeHealth::NONE;
  h.message = note.empty() ? "OK" : note;
  return h;
}

// ============================================================================
// Demo Motor Driver
// ============================================================================
//
// Simulates a motor with temperature buildup and overcurrent injection.
// Fault injection via `inject_fault` parameter.
//
//   "none"   → HEALTHY
//   "warn"   → DEGRADED + WARN  (temperature warning)
//   "hold"   → ERROR  + HOLD    (temperature critical)
//   "abort"  → ERROR  + ABORT   (overcurrent — immediate via post_event)
//   "silent" → heartbeat timer suspended (simulates crashed node)

class DemoMotorDriver : public PackmlNodeInterface
{
public:
  explicit DemoMotorDriver(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    node_->declare_parameter<std::string>("inject_fault", "none");
    init(node_);

    // React to inject_fault changes here, in the parameter callback, rather than
    // from get_health_status() — which must stay a side-effect-free pure getter
    // (the heartbeat timer checks is_active() BEFORE calling get_health_status(),
    // so flipping the active flag from inside that getter is unreachable once
    // heartbeats are paused — there is no tick left to call it again).
    param_cb_handle_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() != "inject_fault") {
            continue;
          }
          const std::string & fault = p.as_string();

          // Fire post_event ONCE as soon as the fault changes to "abort", for
          // immediate notification instead of waiting for the next periodic tick.
          if (fault == "abort") {
            NodeHealth h;
            h.status = NodeHealth::ERROR;
            h.action = NodeHealth::ABORT;
            h.message = "Motor overcurrent: 14A (limit: 8A) — hardware at risk";
            h.error_code = 200;

            post_event(h);
          }

          // "silent" simulates a crashed node (heartbeat publishing paused);
          // any other value resumes it.
          set_heartbeat_active(fault != "silent");
        }
        return rcl_interfaces::msg::SetParametersResult{}.set__successful(true);
      });

    RCLCPP_INFO(node_->get_logger(), "DemoMotorDriver ready");
  }

  NodeHealth get_health_status() override
  {
    const std::string fault = node_->get_parameter("inject_fault").as_string();

    if (fault == "warn") {
      NodeHealth h;
      h.status = NodeHealth::DEGRADED;
      h.action = NodeHealth::WARN;
      h.message = "Motor temperature elevated: 65°C (warn: 60°C)";
      h.error_code = 100;   // user-defined: TEMP_WARNING
      return h;
    }

    if (fault == "hold") {
      NodeHealth h;
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::HOLD;
      h.message = "Motor over-temperature: 85°C (limit: 80°C)";
      h.error_code = 101;   // user-defined: TEMP_CRITICAL
      return h;
    }

    if (fault == "abort") {
      // post_event is triggered from the parameter change callback (see constructor).
      // get_health_status() is a pure getter — no side effects here.
      NodeHealth h;
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::ABORT;
      h.message = "Motor overcurrent: 14A (limit: 8A) — hardware at risk";
      h.error_code = 200;
      return h;
    }

    // "silent" mode: heartbeat publishing is paused via set_heartbeat_active() in
    // the parameter callback above (this getter stays side-effect-free). If this
    // is still called on the one in-flight tick before the pause takes effect,
    // report healthy — the value is about to stop being published anyway.
    if (fault == "silent") {
      return make_healthy();
    }

    return make_healthy("Motor operating normally");
  }

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    RCLCPP_INFO(node_->get_logger(), "State transition requested: %s",
                packml_sm::to_string(state).c_str());
    return true;
  }

  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    RCLCPP_INFO(node_->get_logger(), "Mode transition requested: %s",
                packml_sm::to_string(mode).c_str());
    return true;
  }

  void on_status_changed() override
  {
    RCLCPP_INFO(node_->get_logger(), "PackML status changed → state=%s mode=%s",
                packml_sm::to_string(get_current_packml_state()).c_str(),
                packml_sm::to_string(get_current_packml_mode()).c_str());
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

// ============================================================================
// Demo Sensor Hub
// ============================================================================
//
// Simulates a multi-sensor hub.
//
//   inject_fault = "none"    → HEALTHY
//   inject_fault = "sensor"  → ERROR + HOLD  (sensor disconnected)
//   inject_fault = "range"   → ERROR + SUSPEND (out-of-range: external cause)

class DemoSensorHub : public PackmlNodeInterface
{
public:
  explicit DemoSensorHub(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    node_->declare_parameter<std::string>("inject_fault", "none");
    init(node_);
    RCLCPP_INFO(node_->get_logger(), "DemoSensorHub ready");
  }

  NodeHealth get_health_status() override
  {
    const std::string fault = node_->get_parameter("inject_fault").as_string();

    if (fault == "sensor") {
      NodeHealth h;
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::HOLD;
      h.message = "Sensor 3 disconnected — internal wiring issue";
      h.error_code = 300;   // user-defined: SENSOR_DISCONNECTED
      return h;
    }

    if (fault == "range") {
      // Out-of-range likely caused by product/external condition → SUSPEND
      NodeHealth h;
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::SUSPEND;
      h.message = "Pressure sensor out of range — possible upstream blockage";
      h.error_code = 301;   // user-defined: SENSOR_OUT_OF_RANGE
      return h;
    }

    return make_healthy("All sensors nominal");
  }

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    RCLCPP_INFO(node_->get_logger(), "State → %s", packml_sm::to_string(state).c_str());
    return true;
  }
  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    RCLCPP_INFO(node_->get_logger(), "Mode → %s", packml_sm::to_string(mode).c_str());
    return true;
  }
  void on_status_changed() override {}

private:
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// Demo Conveyor
// ============================================================================
//
// Simulates a conveyor that can jam (internal: HOLD) or detect
// upstream starvation (external: SUSPEND).
//
//   inject_fault = "jam"      → ERROR + HOLD
//   inject_fault = "starved"  → ERROR + SUSPEND
//   inject_fault = "none"     → HEALTHY

class DemoConveyor : public PackmlNodeInterface
{
public:
  explicit DemoConveyor(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    node_->declare_parameter<std::string>("inject_fault", "none");
    init(node_);
    RCLCPP_INFO(node_->get_logger(), "DemoConveyor ready");
  }

  NodeHealth get_health_status() override
  {
    const std::string fault = node_->get_parameter("inject_fault").as_string();

    if (fault == "jam") {
      NodeHealth h;
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::HOLD;
      h.message = "Conveyor mechanical jam at position 3";
      h.error_code = 400;   // user-defined: CONVEYOR_JAM
      return h;
    }

    if (fault == "starved") {
      NodeHealth h;
      h.status = NodeHealth::ERROR;
      h.action = NodeHealth::SUSPEND;
      h.message = "Upstream feed empty — waiting for material";
      h.error_code = 401;   // user-defined: CONVEYOR_STARVED
      return h;
    }

    return make_healthy("Conveyor running");
  }

protected:
  bool on_state_trans_req(packml_sm::State state) override
  {
    RCLCPP_INFO(node_->get_logger(), "State → %s", packml_sm::to_string(state).c_str());
    return true;
  }
  bool on_mode_trans_req(packml_sm::ModeType mode) override
  {
    RCLCPP_INFO(node_->get_logger(), "Mode → %s", packml_sm::to_string(mode).c_str());
    return true;
  }
  void on_status_changed() override {}

private:
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// Main — run one of the three nodes based on first argument
// ============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string which = (argc > 1) ? std::string(argv[1]) : "motor";

  if (which == "motor") {
    auto node = std::make_shared<rclcpp::Node>("demo_motor_driver");
    DemoMotorDriver em(node);
    rclcpp::spin(node);
  } else if (which == "sensor") {
    auto node = std::make_shared<rclcpp::Node>("demo_sensor_hub");
    DemoSensorHub em(node);
    rclcpp::spin(node);
  } else if (which == "conveyor") {
    auto node = std::make_shared<rclcpp::Node>("demo_conveyor");
    DemoConveyor em(node);
    rclcpp::spin(node);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
      "Unknown node '%s'. Use: motor | sensor | conveyor", which.c_str());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
