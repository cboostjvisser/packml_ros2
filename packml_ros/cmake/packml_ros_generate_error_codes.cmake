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

# Store the directory of this file at include-time so the function can locate
# the Python generation script regardless of where it is called from.
set(_packml_ros_cmake_dir "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL
  "Directory containing packml_ros cmake scripts")

#
# packml_ros_generate_error_codes(target yaml_file
#                                 NAMESPACE <ns>
#                                 [INCLUDE_PREFIX <prefix>]
#                                 [PYTHON_OUTPUT <path>])
#
# Generate a C++ header with strictly-typed error code constants from an error
# catalog YAML file and add the generated include directory to <target>.
#
# Only the integer codes (error_codes: section) are generated. The descriptions
# are loaded at runtime by PackmlNodeInterface, so translations can be updated
# without recompiling.
#
# When INCLUDE_PREFIX is given, the header is placed under a subdirectory with
# that name, so consumers include it as:
#   #include "<prefix>/<yaml_name>.hpp"
# When omitted, the header is at the include root:
#   #include "<yaml_name>.hpp"
#
# :param target:    CMake target that will include the generated header.
# :param yaml_file: Path to the error catalog YAML (absolute or relative to
#                   CMAKE_CURRENT_SOURCE_DIR).
# :param NAMESPACE: C++ namespace for the generated constants (required).
#
# Example:
#   packml_ros_generate_error_codes(motor_driver_lib
#     config/error_catalog.yaml
#     NAMESPACE      MotorErrors
#     INCLUDE_PREFIX motor_driver
#     PYTHON_OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/python)
#
# The header is NOT installed by this function — that is the caller's
# responsibility, since each package has its own include namespace.
#
function(packml_ros_generate_error_codes target yaml_file)
  cmake_parse_arguments(ARG "" "NAMESPACE;PYTHON_OUTPUT;INCLUDE_PREFIX" "" ${ARGN})
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  if(NOT ARG_NAMESPACE)
    message(FATAL_ERROR "packml_ros_generate_error_codes: NAMESPACE is required")
  endif()

  get_filename_component(yaml_abs "${yaml_file}" ABSOLUTE
    BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  get_filename_component(yaml_name "${yaml_abs}" NAME_WE)

  set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/packml_error_codes/include")
  if(ARG_INCLUDE_PREFIX)
    set(output_header "${output_dir}/${ARG_INCLUDE_PREFIX}/${yaml_name}.hpp")
  else()
    set(output_header "${output_dir}/${yaml_name}.hpp")
  endif()

  set(script "${_packml_ros_cmake_dir}/generate_error_codes_header.py")

  set(_gen_outputs "${output_header}")
  set(_gen_command "${Python3_EXECUTABLE}" "${script}" "${yaml_abs}"
    "${output_header}" "${ARG_NAMESPACE}")

  if(ARG_PYTHON_OUTPUT)
    get_filename_component(_py_out_abs "${ARG_PYTHON_OUTPUT}" ABSOLUTE
      BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    string(TOLOWER "${ARG_NAMESPACE}" _py_module)
    set(_py_init "${_py_out_abs}/${_py_module}/__init__.py")
    list(APPEND _gen_outputs "${_py_init}")
    list(APPEND _gen_command "${_py_out_abs}")
  endif()

  add_custom_command(
    OUTPUT ${_gen_outputs}
    COMMAND ${_gen_command}
    DEPENDS "${yaml_abs}" "${script}"
    COMMENT "Generating PackML error codes from ${yaml_abs}"
  )

  string(REPLACE "::" "__" _target_safe "${target}")
  add_custom_target(${_target_safe}_packml_error_codes_gen DEPENDS ${_gen_outputs})

  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${output_dir}>)

  add_dependencies(${target} ${_target_safe}_packml_error_codes_gen)
endfunction()
