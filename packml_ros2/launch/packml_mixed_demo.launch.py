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

"""Demo: C++ and Python PackML nodes managed by one manager.

This launch file demonstrates the PackML manager coordinating
both a C++ "equipment module" and a Python "equipment module" simultaneously.

Usage:
    ros2 launch packml_ros2 packml_mixed_demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch a PackML manager with one C++ and one Python managed node."""

    # C++ equipment module node
    cpp_equipment_node = Node(
        package='packml_ros',
        executable='example_packml_node',
        name='cpp_equipment_module',
        output='screen',
    )

    # Python equipment module node
    py_equipment_node = Node(
        package='packml_ros_py',
        executable='example_packml_node',
        name='py_equipment_module',
        output='screen',
    )

    # PackML manager (C++ state machine) managing both nodes
    manager_node = Node(
        package='packml_ros',
        executable='packml_ros_node',
        name='packml_manager',
        output='screen',
        parameters=[{
            'node_names': ['cpp_equipment_module', 'py_equipment_module'],
            # Acting-state completion is coordinated automatically for every node in
            # node_names -- there is no separate opt-in list: each node's own
            # defers_completion() override decides whether it participates for a given
            # state (both example nodes above defer only RESETTING, reporting it from
            # on_deferred_work() ~500ms after accepting the transition). Raise
            # this if a real Equipment Module's own commanded work can legitimately
            # take longer than the default.
            #
            # 'state_complete_timeout_ms': 30000,
        }],
    )

    return LaunchDescription([
        cpp_equipment_node,
        py_equipment_node,
        manager_node,
    ])
