# Copyright (c) 2024 PackML Contributors
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
set(_packml_sm_cmake_dir "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL
  "Directory containing packml_sm cmake scripts")

#
# packml_sm_generate_modes(target yaml_file
#                          [INCLUDE_PREFIX <prefix>]
#                          [PYTHON_OUTPUT <path>])
#
# Generate a C++ header with PackML mode constants from a YAML file and add
# the generated include directory to <target>.
#
# When INCLUDE_PREFIX is given, the header is placed under a subdirectory
# with that name, so consumers include it as:
#   #include "<prefix>/<yaml_name>.hpp"
# When omitted, the header is at the include root:
#   #include "<yaml_name>.hpp"
#
# When PYTHON_OUTPUT is given, a Python module with the same constants is
# also generated at <path>.
#
# :param target:    CMake target that will include the generated header.
# :param yaml_file: Path to the YAML modes definition file (absolute or
#                   relative to CMAKE_CURRENT_SOURCE_DIR).
#
# The YAML file format:
#   modes:
#     Invalid: 0
#     Production: 1
#     Maintenance: 2
#
# The generated header is placed in:
#   ${CMAKE_CURRENT_BINARY_DIR}/packml_modes/include/[<prefix>/]<yaml_name>.hpp
#
# and exposes constants in the packml_modes namespace:
#   namespace packml_modes {
#     constexpr packml_sm::ModeType Invalid = 0;
#     ...
#   }
#
# The header is NOT installed by this function — that is the caller's
# responsibility, since each package has its own include namespace.
#
function(packml_sm_generate_modes target yaml_file)
  cmake_parse_arguments(ARG "" "PYTHON_OUTPUT;INCLUDE_PREFIX" "" ${ARGN})
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  get_filename_component(yaml_abs "${yaml_file}" ABSOLUTE
    BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  get_filename_component(yaml_name "${yaml_abs}" NAME_WE)

  set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/packml_modes/include")
  if(ARG_INCLUDE_PREFIX)
    set(output_header "${output_dir}/${ARG_INCLUDE_PREFIX}/${yaml_name}.hpp")
  else()
    set(output_header "${output_dir}/${yaml_name}.hpp")
  endif()

  set(script "${_packml_sm_cmake_dir}/generate_modes_header.py")

  # Build the generation command — optionally including the Python output directory.
  set(_gen_outputs "${output_header}")
  set(_gen_command "${Python3_EXECUTABLE}" "${script}" "${yaml_abs}" "${output_header}")

  if(ARG_PYTHON_OUTPUT)
    get_filename_component(_py_out_abs "${ARG_PYTHON_OUTPUT}" ABSOLUTE
      BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    # The generator creates <dir>/packml_modes/__init__.py
    set(_py_init "${_py_out_abs}/packml_modes/__init__.py")
    list(APPEND _gen_outputs "${_py_init}")
    list(APPEND _gen_command "${_py_out_abs}")
  endif()

  add_custom_command(
    OUTPUT ${_gen_outputs}
    COMMAND ${_gen_command}
    DEPENDS "${yaml_abs}" "${script}"
    COMMENT "Generating PackML modes from ${yaml_abs}"
  )

  # Use a safe name for the helper target (replace :: with __)
  string(REPLACE "::" "__" _target_safe "${target}")
  add_custom_target(${_target_safe}_packml_modes_gen DEPENDS ${_gen_outputs})

  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${output_dir}>)

  add_dependencies(${target} ${_target_safe}_packml_modes_gen)
endfunction()
