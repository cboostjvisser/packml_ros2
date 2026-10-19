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

# Store the directory of this file at include-time (same cached var the
# codegen macro sets — both files live side by side and are always included
# together via packml_ros-extras.cmake, so re-setting it here to the same
# value is idempotent).
set(_packml_ros_cmake_dir "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL
  "Directory containing packml_ros cmake scripts")

#
# packml_ros_aggregate_error_catalog(<target> <error_map.yaml>
#                                     [OUTPUT_DIR <dir>]
#                                     [FORMATS <fmt>...]
#                                     [STRICT]
#                                     [CHECK_INSTANCES <id>...])
#
# Run the offline error-catalog aggregation tool (aggregate_error_catalog.py)
# against a bringup package's error_map.yaml at build time, producing
# machine_error_catalog.yaml (the manager's runtime artifact) plus the
# machine_errors.{json,md} sheets (default FORMATS: yaml,json,md — no csv;
# one human-facing sheet format is enough and avoids keeping two in sync).
#
# :param target:      Name used to derive the generated target
#                      (<target>_packml_error_catalog_gen). Does not need to
#                      be a real compile target — see the ALL note below.
# :param error_map.yaml: Path to the integration map (absolute or relative to
#                      CMAKE_CURRENT_SOURCE_DIR).
# :param OUTPUT_DIR:   Where generated artifacts are written. Default:
#                      ${CMAKE_CURRENT_BINARY_DIR}/error_catalog.
# :param FORMATS:      Output formats to generate (default: yaml;json;md).
# :param STRICT:       Promote every lint warning to a build-failing error.
# :param CHECK_INSTANCES: Instance ids that must have a configured label (W4).
#
# The caller is responsible for installing the generated files to its own
# share/ directory (same stance as packml_ros_generate_error_codes).
#
# HARD REQUIREMENT this function satisfies: a plain add_custom_command does
# not make the lint run on `colcon build`
# unless some target depends on its output, and a pure-bringup package (like
# a demo/integration package) may have NO compile targets to hang that
# dependency off of. This function therefore creates
# add_custom_target(..._gen ALL DEPENDS ...) — WITH ALL — so aggregation (and
# its safety-relevant duplicate-global lint) runs on every default build with
# no consumer required. It also requests a DEPFILE so a catalog file edited
# after the last configure still retriggers regeneration on the next
# incremental build. Remaining limitation: the DEPFILE only sees
# install-space files, so editing a node's catalog in its own source tree
# doesn't retrigger this until that node package is rebuilt/reinstalled too
# (`colcon build --packages-above <node_pkg>` covers both).
#
function(packml_ros_aggregate_error_catalog target error_map)
  cmake_parse_arguments(ARG "STRICT" "OUTPUT_DIR" "FORMATS;CHECK_INSTANCES" ${ARGN})
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  get_filename_component(map_abs "${error_map}" ABSOLUTE
    BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  if(ARG_OUTPUT_DIR)
    get_filename_component(output_dir "${ARG_OUTPUT_DIR}" ABSOLUTE
      BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  else()
    set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/error_catalog")
  endif()

  if(NOT ARG_FORMATS)
    set(ARG_FORMATS yaml json md)
  endif()
  string(REPLACE ";" "," _formats_csv "${ARG_FORMATS}")

  # Script resolution: the cmake dir first (installed layout — this .cmake
  # file and a copy of the script are installed side by side under
  # share/packml_ros/cmake/), then ../scripts (source tree, e.g. when this
  # function is exercised from within packml_ros's own build before install).
  set(script "${_packml_ros_cmake_dir}/aggregate_error_catalog.py")
  if(NOT EXISTS "${script}")
    set(script "${_packml_ros_cmake_dir}/../scripts/aggregate_error_catalog.py")
  endif()

  # --list-inputs at configure time: this requires actually parsing
  # error_map.yaml and resolving every package:// URI, which CMake itself
  # cannot do — delegate to the tool. A failure here (e.g. a referenced
  # package not yet built) is deferred to the real build-time run below,
  # which reports it as a proper lint error instead of a configure failure.
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${script}" --map "${map_abs}" --list-inputs
    OUTPUT_VARIABLE _list_inputs_output
    ERROR_VARIABLE _list_inputs_error
    RESULT_VARIABLE _list_inputs_result
  )
  set(_depends "${map_abs}" "${script}")
  if(_list_inputs_result EQUAL 0 AND _list_inputs_output)
    string(STRIP "${_list_inputs_output}" _list_inputs_output)
    string(REPLACE "\n" ";" _list_inputs_list "${_list_inputs_output}")
    list(APPEND _depends ${_list_inputs_list})
  endif()

  set(primary_output "${output_dir}/machine_error_catalog.yaml")
  set(depfile "${CMAKE_CURRENT_BINARY_DIR}/${target}_error_catalog.d")

  set(_command "${Python3_EXECUTABLE}" "${script}"
    --map "${map_abs}"
    --output-dir "${output_dir}"
    --formats "${_formats_csv}"
    --depfile "${depfile}")
  if(ARG_STRICT)
    list(APPEND _command --strict)
  endif()
  if(ARG_CHECK_INSTANCES)
    string(REPLACE ";" "," _check_instances_csv "${ARG_CHECK_INSTANCES}")
    list(APPEND _command --check-instances "${_check_instances_csv}")
  endif()

  set(_outputs "${primary_output}")
  foreach(fmt IN LISTS ARG_FORMATS)
    if(NOT fmt STREQUAL "yaml")
      list(APPEND _outputs "${output_dir}/machine_errors.${fmt}")
    endif()
  endforeach()

  add_custom_command(
    OUTPUT ${_outputs}
    COMMAND ${_command}
    DEPENDS ${_depends}
    DEPFILE "${depfile}"
    COMMENT "Aggregating error catalog from ${map_abs}"
  )

  string(REPLACE "::" "__" _target_safe "${target}")
  # ALL is load-bearing here — see the function-level comment above.
  add_custom_target(${_target_safe}_packml_error_catalog_gen ALL DEPENDS ${_outputs})
endfunction()
