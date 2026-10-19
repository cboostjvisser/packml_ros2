// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---

#pragma once

#include <iostream>
#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

#include "packml_sm/common.hpp"

namespace packml_sm {

namespace detail {

/// Read the "modes" mapping (name -> value) out of an already-loaded document.
inline std::map<std::string, ModeType>
declared_modes_of(const YAML::Node & root)
{
  std::map<std::string, ModeType> modes;
  if (!root["modes"] || !root["modes"].IsMap()) {
    return modes;
  }
  for (const auto & entry : root["modes"]) {
    try {
      modes[entry.first.as<std::string>()] = entry.second.as<ModeType>();
    } catch (const YAML::Exception & e) {
      std::cerr << "[modes_config] Failed to parse mode value for '"
                << entry.first.as<std::string>() << "': " << e.what() << std::endl;
    }
  }
  return modes;
}

}  // namespace detail

/// Parse the modes a YAML configuration file declares, as name -> value.
///
/// Deliberately independent of "state_masks": that section says which states a mode allows, and a
/// declared mode with no mask is legitimate -- it means fully open. So a file may declare modes and
/// mask none of them.
///
/// @param yaml_file  Path to the YAML configuration file.
/// @return           The declared modes. Empty if the file cannot be opened or declares none.
inline std::map<std::string, ModeType>
parse_declared_modes(const std::string & yaml_file)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_file);
  } catch (const YAML::Exception & e) {
    std::cerr << "[modes_config] Failed to load YAML file '" << yaml_file
              << "': " << e.what() << std::endl;
    return {};
  }
  return detail::declared_modes_of(root);
}

/// Parse a modes YAML configuration file and return per-mode state masks.
///
/// The YAML file must contain a top-level "modes" mapping (name -> int value)
/// and an optional "state_masks" mapping (name -> state_name -> bool).
/// States not listed under a mode default to true (available).
///
/// @param yaml_file  Path to the YAML configuration file.
/// @return           Map from ModeType value to AvailableStates.  Returns an
///                   empty map if the file cannot be opened or has no masks.
inline std::map<ModeType, AvailableStates>
parse_modes_config(const std::string & yaml_file)
{
  std::map<ModeType, AvailableStates> result;

  // Build the complete map of PackML state names -> State enum values.
  const std::map<std::string, State> state_name_map = {
    {"ABORTING",     State::ABORTING},
    {"ABORTED",      State::ABORTED},
    {"CLEARING",     State::CLEARING},
    {"STOPPING",     State::STOPPING},
    {"STOPPED",      State::STOPPED},
    {"RESETTING",    State::RESETTING},
    {"IDLE",         State::IDLE},
    {"STARTING",     State::STARTING},
    {"EXECUTE",      State::EXECUTE},
    {"HOLDING",      State::HOLDING},
    {"HELD",         State::HELD},
    {"UNHOLDING",    State::UNHOLDING},
    {"SUSPENDING",   State::SUSPENDING},
    {"SUSPENDED",    State::SUSPENDED},
    {"UNSUSPENDING", State::UNSUSPENDING},
    {"COMPLETING",   State::COMPLETING},
    {"COMPLETE",     State::COMPLETE},
  };

  // All-true defaults used when a state is not listed under a mode mask.
  AvailableStates all_true;
  for (const auto & [name, state] : state_name_map) {
    all_true[state] = true;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_file);
  } catch (const YAML::Exception & e) {
    std::cerr << "[modes_config] Failed to load YAML file '" << yaml_file
              << "': " << e.what() << std::endl;
    return result;
  }

  const auto mode_values = detail::declared_modes_of(root);

  // Parse per-mode state masks.
  if (!root["state_masks"] || !root["state_masks"].IsMap()) {
    std::cerr << "[modes_config] No 'state_masks' section found in '" << yaml_file
              << "', all states will default to available" << std::endl;
    return result;
  }

  for (const auto & mode_entry : root["state_masks"]) {
    const std::string mode_name = mode_entry.first.as<std::string>();
    auto mode_it = mode_values.find(mode_name);
    if (mode_it == mode_values.end()) {
      std::cerr << "[modes_config] state_masks entry '" << mode_name
                << "' has no matching entry in 'modes', skipping" << std::endl;
      continue;
    }

    // Start from all-true defaults so unlisted states remain available.
    AvailableStates avail = all_true;

    if (mode_entry.second && mode_entry.second.IsMap()) {
      for (const auto & state_entry : mode_entry.second) {
        const std::string state_name = state_entry.first.as<std::string>();
        auto state_it = state_name_map.find(state_name);
        if (state_it == state_name_map.end()) {
          std::cerr << "[modes_config] Unknown state name '" << state_name
                    << "' in mask for mode '" << mode_name << "', skipping" << std::endl;
          continue;
        }
        try {
          avail[state_it->second] = state_entry.second.as<bool>();
        } catch (const YAML::Exception & e) {
          std::cerr << "[modes_config] Invalid boolean value for state '"
                    << state_name << "' in mode '" << mode_name
                    << "': " << e.what() << ", defaulting to true" << std::endl;
        }
      }
    }

    for (const auto & state_name : enforce_mandatory_states(avail)) {
      std::cerr << "[modes_config] mode '" << mode_name << "' disables " << state_name
                << ", which is mandatory -- restoring it. The applied mask differs from the file."
                << std::endl;
    }

    result[mode_it->second] = avail;
  }

  return result;
}

}  // namespace packml_sm
