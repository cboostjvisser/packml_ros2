# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(CMakeFindDependencyMacro)
find_dependency(Qt5 COMPONENTS Core)

# Make the packml_ros_generate_error_codes() function available to downstream packages.
include("${CMAKE_CURRENT_LIST_DIR}/packml_ros_generate_error_codes.cmake")

# Make the packml_ros_aggregate_error_catalog() function available to downstream
# (bringup) packages.
include("${CMAKE_CURRENT_LIST_DIR}/packml_ros_aggregate_error_catalog.cmake")
