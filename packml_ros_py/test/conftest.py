#  Copyright (c) 2026 PackML ROS2 Contributors
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
Pytest configuration for the packml_ros_py tests.

Pins this suite to its own ROS domain, matching what packml_ros/test/test_main.cpp does for
the C++ binaries (domains 71 and 72 there; see its comment for the reasoning).  These tests
publish and subscribe the same bare global topic names the C++ suites do -- packml_status
above all -- so on a shared domain a concurrently running C++ suite's manager is
indistinguishable from this suite's own.

Set at import time, before any rclpy.init() and before launch_ros starts a node as a
subprocess, so both the in-process nodes and the launched ones land here.  Unconditional for
the same reason as the C++ side: an inherited ROS_DOMAIN_ID from the developer's shell is
precisely the case worth overriding.
"""

import os

os.environ['ROS_DOMAIN_ID'] = '73'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
