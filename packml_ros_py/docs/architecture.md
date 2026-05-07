# PackML ROS2 Python Bindings — Architecture

## Overview

`packml_ros_py` provides Python nodes that can be managed by the C++ PackML
manager (packml_ros), with **shared C++ transition logic**(TransitionGuard) via pybind11.


## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│             TransitionGuard (C++, no ROS/Qt deps)           │
│                                                             │
│  packml_ros/include/packml_ros/transition_guard.hpp         │
│  packml_ros/src/transition_guard.cpp                        │
│                                                             │
│  Responsibilities:                                          │
│  - Validate state/mode transition requests                  │
│  - Track current state, mode, and in-flight switches        │
│  - Provide "already there" / "warn: double switch" logic    │
│                                                             │
│  Dependencies: packml_sm/common.hpp only (enums + typedefs) │
│  No ROS. No Qt. No threads.                                 │
└──────────────┬──────────────────────────────┬───────────────┘
               │                              │
               │                              │
    ┌──────────▼──────────────┐    ┌──────────▼──────────────┐
    │  C++ PackmlNodeInterface│    │  pybind11 module         │
    │  (packml_ros)           │    │  (_packml_bindings.so)   │
    │                         │    │                          │
    │  Uses TransitionGuard   │    │  Wraps:                  │
    │  internally (or can be  │    │  - TransitionGuard       │
    │  refactored to do so)   │    │  - State enum            │
    │                         │    │  - TransitionCmd enum    │
    │  + rclcpp services      │    │  - TransitionResult      │
    └─────────────────────────┘    └──────────┬───────────────┘
                                              │
                                   ┌──────────▼───────────────┐
                                   │  Python PackmlNode       │
                                   │  (packml_ros_py)         │
                                   │                          │
                                   │  Owns a TransitionGuard  │
                                   │  + rclpy services/subs   │
                                   │  + user override hooks   │
                                   └──────────────────────────┘
```


## Package Layout

```
packml_ros_py/
├── CMakeLists.txt              # ament_cmake + pybind11 build
├── package.xml
├── src/
│   └── bindings.cpp            # pybind11 module definition
├── packml_ros_py/
│   ├── __init__.py             # Exports: PackmlNode, State, TransitionCmd, ModeType
│   ├── enums.py                # Re-exports C++ enums from _packml_bindings
│   ├── packml_node.py          # PackmlNode base class (delegates to TransitionGuard)
│   └── example_node.py         # Example equipment module
└── test/
    ├── test_bindings.py        # Direct pybind11 binding tests
    ├── test_enums.py           # Enum value parity tests
    ├── test_packml_node.py     # ROS2 service/subscription tests
    └── test_integration.py     # Launch-based integration test
```


## Key Design Decisions

### Why pybind11 wrapping instead of pure Python?

The transition validation logic (state/mode tracking, "already switching" guards,
conflict detection) is the **protocol contract** between manager and nodes. If it
drifts between C++ and Python, nodes misbehave. Single source of truth in C++
eliminates that risk.

### How project-specific modes work

Modes are **user-defined per project** via a YAML file. The `packml_sm_generate_modes()`
CMake function generates named constants at build time — a C++ header and a Python
module, both under the `packml_modes` namespace:

```yaml
# config/my_modes.yaml
modes:
  Invalid: 0
  Production: 1
  Maintenance: 2
```

```cmake
packml_sm_generate_modes(${PROJECT_NAME} config/my_modes.yaml
  INCLUDE_PREFIX my_package)
```

C++ — `#include "my_package/my_modes.hpp"`:
```cpp
sm->changeMode(packml_modes::Production);
```

Python — `from packml_modes import PRODUCTION`:
```python
def on_mode_transition_request(self, target_mode: int) -> bool:
    return target_mode in (packml_modes.PRODUCTION, packml_modes.MAINTENANCE)
```

### Why TransitionGuard lives in packml_ros (not packml_sm)

`packml_sm` is purely about the state machine itself — states, transitions, commands.
`TransitionGuard` is about the coordination protocol between a *manager* and a *managed
node*, which is a ROS integration concern owned by `packml_ros`.


## Usage

### Python Equipment Module

```python
from packml_ros_py import PackmlNode, State

class ConveyorModule(PackmlNode):
    def __init__(self):
        super().__init__('conveyor_module')

    def on_state_transition_request(self, target_state: State) -> bool:
        self.get_logger().info(f'Transitioning to {target_state.name}')
        return True

    def on_mode_transition_request(self, target_mode: int) -> bool:
        return True

    def on_packml_status_changed(self, status) -> None:
        pass
```

### Launch (mixed C++ and Python nodes under one manager)

```bash
ros2 launch packml_ros2 packml_mixed_demo.launch.py
```

### Interacting with the manager

```bash
# Reset the state machine
ros2 service call /packml_manager/changeState packml_msgs/srv/StateChange "{command: 1}"

# Start
ros2 service call /packml_manager/changeState packml_msgs/srv/StateChange "{command: 2}"

# Stop
ros2 service call /packml_manager/changeState packml_msgs/srv/StateChange "{command: 3}"
```