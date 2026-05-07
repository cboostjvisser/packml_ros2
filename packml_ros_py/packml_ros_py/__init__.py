# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PackML ROS2 Python node library.

Provides a Python base class (PackmlNode) that can be managed by the
PackML ROS2 manager, mirroring the C++ PackmlNodeInterface behavior.
The transition validation logic is backed by C++ TransitionGuard via pybind11.
"""

from packml_ros_py.packml_node import PackmlNode
from packml_ros_py.enums import State, TransitionCmd, ModeType
from packml_ros_py._packml_bindings import (
    STATE_TRANSITION_SERVICE,
    MODE_TRANSITION_SERVICE,
    STATUS_TOPIC,
)

__all__ = [
    'PackmlNode', 'State', 'TransitionCmd', 'ModeType',
    'STATE_TRANSITION_SERVICE', 'MODE_TRANSITION_SERVICE', 'STATUS_TOPIC',
]
