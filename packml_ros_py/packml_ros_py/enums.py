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

"""PackML enumerations re-exported from C++ via pybind11.

These are the enum values from packml_sm::State and packml_sm::TransitionCmd.
Single source of truth is the C++ code;
this module simply re-exports the pybind11-bound enums for convenience.

ModeType is intentionally an int (not an enum) because mode values are
user-defined per project via YAML configuration.  There is no fixed set.
"""

from packml_ros_py._packml_bindings import State, TransitionCmd

# ModeType is a plain int — modes are user-defined per project.
ModeType = int

__all__ = ['State', 'TransitionCmd', 'ModeType']
