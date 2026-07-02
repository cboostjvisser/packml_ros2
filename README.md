# packml_ros2

[![GH_build](https://img.shields.io/github/workflow/status/ros-industrial/packml_ros2/GH-Actions-CI/master?label=Build&logo=Github&style=flat-square)](https://github.com/ros-industrial/packml_ros2/actions/workflows/gh-act.yml)
[![codecov Coverage Status](https://codecov.io/gh/ros-industrial/packml_ros2/branch/master/graph/badge.svg)](https://app.codecov.io/gh/ros-industrial/packml_ros2/branch/master)
[![License](https://img.shields.io/github/license/ros-industrial/packml_ros2.svg?style=flat-square)](https://github.com/ros-industrial/packml_ros2)

This package implements a state machine as prescribed in the Packaging Machine Language (PackML) standard in simulation. 

This package also contains an RViz 2 plugin to be able to visualize the state of the state machine, the elapsed time in that state, and to control the triggering of state transitions of the machine through buttons. 

Finally, the package contains an example of the interface of the RViz2 plugin with a real PackML state machine running in a PLC, communicating its state and triggering transitions via OPCUA public tags. 

## List of packages
* `packml_plugin`: RViz2 plugin for PackML state machine standard template visualization and control
* `packml_sm`: Simulator library in C++ ported to ROS 2 from the original PackML repository in ROS 1. Two types of state machines, with continuous Execute state and with timed Execute state.
* `packml_msgs`: Service/message type definitions for states, transitions, GUI control, and node **health** (`NodeHealth`, `NodeHeartbeat`, `Alarm`).
* `packml_ros`: ROS 2 node in C++ to run the simulator library and communicate with the RViz2 plugin. Also provides the Equipment-Module interface, heartbeat publishing, and the manager-side `HealthMonitor` + health gate.
* `packml_ros_py`: Python bindings (pybind11) so you can write PackML Nodes (Equipment Modules) in Python with the same shared protocol/heartbeat logic as C++.

Extras:
* `packml_plc`: Example of a driver in Python to interface with a PackML state machine implemented in a Siemens PLC (with pre-configured OPCUA variable tags according to the PLCs configuration). Direct communication with the RViz2 plugin (receive states and send events to trigger transitions). 

## Pre-requisites
* Ubuntu 24.04
* ROS 2 [Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
* `ros_industrial_cmake_boilerplate` (build dependency of the C++ packages)
* `pybind11-stubgen` (optional — generates Python type stubs for `packml_ros_py`):

      pip install pybind11-stubgen


## Build from source
* Source your ROS environment

      . /opt/ros/jazzy/setup.bash

* Setup workspace and install dependencies

      mkdir -p ~/colcon_ws/src
      cd ~/colcon_ws/src
      git clone https://github.com/1487quantum/packml_ros2.git
      cd ../
      rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

* Build the workspace

      cd ~/colcon_ws
      colcon build
      . ~/colcon_ws/install/setup.bash


## Run the code

* For the simulator only, in a terminal run 
        
      ros2 run packml_ros packml_ros_node

  Then in another terminal run 
    
      rviz2
    
  Selet `Panels>Add New Panel` and load the plugin called `packml_plugin` from the list of plugins. The state machine diagram should appear in RViz, along the buttons for the control of the machine.

* For the real PLC, in a terminal run 
  
      ros2 run packml_plc packml_plc_listener.py
  
  Then in another terminal run 
  
      ros2 run packml_plc_sender.py
  
  Finally RViz2 
  
      ros2 run rviz2 rviz2
        
  Load the plugin called `packml_plugin` from the list of plugins. As with the simulator, the state machine diagram should appear in RViz, along the buttons for the control of the machine.

## Define your own modes

By default, no mode names are defined and there is no state configuration. So a requested mode will just be an integer and each state is allowed. To define your own modes:

1. Create a YAML-file similar to `packml_sm/modes/default_modes(_minimal).yaml` in your project. The initial mode is always 0, so it is advised to make this *Invalid/Undefined* or something similar. For example:

        modes:
            Invalid: 0
            Production: 1
            Service: 2
            DryRun: 3
            Commissioning: 4

2. Add the following line to the `CMakeList.txt` of your project

        // CMakeList.txt
        packml_sm_generate_modes(${PROJECT_NAME} path/to/your/modes.yaml)

3. Include the modes in your code

        // your_code.hpp/cpp
        #include "packml_modes.hpp"

        // Usage
        auto mode = get_current_packml_mode();
        auto mode_as_string = packml_sm::to_string(mode);
        switch (mode)
        {
            case packml_modes::Invalid:
            case packml_modes::Production:
            case packml_modes::Service:
            // etc.
        }

## Health & heartbeat monitoring

A PackML **Manager** (Machine) can supervise the health of its **Equipment Modules**
(PackML Nodes) over ROS 2. Each Equipment Module publishes a periodic `NodeHeartbeat`
carrying a `NodeHealth` (status + recommended PackML action). The manager embeds a
`HealthMonitor` that:

* **routes faults to the state machine** — a node reporting `HOLD`/`SUSPEND`/`ABORT`
  drives the corresponding PackML transition (only new/escalating events fire; `WARN`
  is observed but never transitions);
* **detects dead nodes** — a required node that stops sending heartbeats times out
  (`interval × heartbeat_timeout_factor`) and triggers `ABORT`;
* **enforces a health gate** — `RESET` from `STOPPED` is blocked until every *required*
  node is healthy and present. A timed-out / never-seen node is **non-bypassable**; an
  actionable error can be bypassed in MANUAL mode for diagnostics;
* **publishes alarms** — every raise/update/clear is published on `packml_alarms`
  (`Alarm.msg`, mirroring the PackML `Admin.Alarm` tag).

Configure the manager with `required_nodes`, `heartbeat_timeout_factor` (default `3.0`),
`heartbeat_startup_grace_ms` (default `30000`), and — for the MANUAL-mode bypass —
`manual_mode_allows_health_bypass` (default `false`) plus `manual_mode` (default `-1`).
Try it end-to-end:

      ros2 launch packml_ros2 packml_health_demo.launch.py
      # then inject faults at runtime:
      ros2 param set /demo_motor_driver inject_fault abort     # or: warn | hold | silent | none
      ros2 param set /demo_sensor_hub   inject_fault range     # SUSPEND (external cause)
      ros2 param set /demo_conveyor     inject_fault starved   # SUSPEND (external cause)

## Write your own Equipment Module

A third party can write a PackML Node in **C++ or Python** without editing this repo —
derive from the node interface and override `get_health_status()`:

**C++** (`#include "packml_ros/interface/packml_interface.hpp"`):

      class MyMotor : public PackmlNodeInterface {
      public:
        explicit MyMotor(rclcpp::Node::SharedPtr node) { init(node); }
        packml_msgs::msg::NodeHealth get_health_status() override {
          packml_msgs::msg::NodeHealth h;
          if (over_temp_) { h.status = h.ERROR; h.action = h.HOLD; h.message = "over-temp"; }
          else            { h.status = h.HEALTHY; h.action = h.NONE; }
          return h;
        }
        void on_estop() {  // urgent: publish immediately, don't wait for the timer
          packml_msgs::msg::NodeHealth h; h.status = h.ERROR; h.action = h.ABORT;
          post_event(h);
        }
      };

**Python** (`from packml_ros_py.packml_node import PackmlNode`):

      class MyMotor(PackmlNode):
          def get_health_status(self):
              msg = NodeHealth()
              if self.over_temp:
                  msg.status, msg.action, msg.message = msg.ERROR, msg.HOLD, "over-temp"
              else:
                  msg.status, msg.action = msg.HEALTHY, msg.NONE
              return msg

The base class publishes the heartbeat for you (`heartbeat_interval_ms`, default
`1000`). Use `post_event(health)` for safety-critical events that can't wait for the
next tick. **Keep `get_health_status()` a pure getter** — no side effects at all: don't
call `post_event()`, don't call `set_heartbeat_active()`, don't mutate node state from
inside it. The periodic timer checks whether heartbeats are paused *before* calling
`get_health_status()`, so a state-mutating getter can silently make its own follow-up
logic unreachable.

## Contributors
* Dejanira Araiza Illan
* Chen Bainian
* Derrick Ang Ming Yan
