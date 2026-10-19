// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---

#ifndef PACKML_SM__MODES_REGISTRY_HPP_
#define PACKML_SM__MODES_REGISTRY_HPP_

#include <initializer_list>
#include <iostream>
#include <map>
#include <mutex>
#include <string>

namespace packml_sm
{

// ModeType is an alias for int, allowing user-defined mode values via
// the packml_sm_generate_modes CMake function.
using ModeType = int;

/// The mode vocabulary this process knows about, as value -> name.
///
/// A program acquires its vocabulary from whichever of these apply:
///   - every generated modes header linked into it, each of which registers its own table from a
///     namespace-scope initialiser (see cmake/generate_modes_header.py);
///   - a deployment's `modes_config_file`, registered by PackmlManagerInterface at startup.
///
/// The registry exists because the alternative -- a generated `is_known_mode()` compiled into a
/// public header -- forces every consumer of that header to adopt the generating package's mode
/// values. Two generated headers in one translation unit then collide outright, and packml_ros
/// cannot validate a mode against a table it does not know at compile time.
///
/// Registration order across generated headers is unspecified (they initialise as unordered
/// namespace-scope variables), so a value registered twice under DIFFERENT names keeps whichever
/// arrived last and reports the disagreement. That is a diagnostic for a program carrying two mode
/// vocabularies at once, and it affects only the name -- is_known_mode() answers true either way.
/// A `modes_config_file` registers well after static initialisation, so a deployment's own names
/// deterministically win.
namespace detail
{

struct ModeRegistry
{
  std::mutex mutex;
  std::map<ModeType, std::string> names;
};

inline ModeRegistry & mode_registry()
{
  static ModeRegistry registry;
  return registry;
}

inline void register_one(ModeRegistry & registry, ModeType value, const std::string & name)
{
  const auto existing = registry.names.find(value);
  if (existing != registry.names.end() && existing->second != name) {
    std::cerr << "[modes_registry] mode " << value << " was registered as '" << existing->second
              << "' and is now '" << name << "'; this program carries two mode vocabularies"
              << std::endl;
  }
  registry.names[value] = name;
}

}  // namespace detail

/// One declared mode, as the generated headers spell it.
struct ModeDeclaration
{
  const char * name;
  ModeType value;
};

/// Add `modes` to the process mode vocabulary. Returns true so a generated header can call this
/// from a namespace-scope initialiser.
inline bool register_modes(std::initializer_list<ModeDeclaration> modes)
{
  auto & registry = detail::mode_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  for (const auto & mode : modes) {
    detail::register_one(registry, mode.value, mode.name);
  }
  return true;
}

/// Overload for a table parsed at runtime, keyed by name as the YAML spells it.
inline bool register_modes(const std::map<std::string, ModeType> & modes)
{
  auto & registry = detail::mode_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  for (const auto & [name, value] : modes) {
    detail::register_one(registry, value, name);
  }
  return true;
}

/// Is `mode` one of the modes this process declared? Note a declared sentinel such as Invalid
/// answers true -- this asks whether the value is KNOWN, not whether it is a sensible thing to
/// switch to.
inline bool is_known_mode(ModeType mode)
{
  auto & registry = detail::mode_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  return registry.names.find(mode) != registry.names.end();
}

/// The whole vocabulary, value -> name. Empty means no modes header was linked in and no
/// modes_config_file was loaded, which leaves is_known_mode() rejecting everything.
inline std::map<ModeType, std::string> known_modes()
{
  auto & registry = detail::mode_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  return registry.names;
}

/// The declared name of `mode`, or its number when nothing declared it.
///
/// Declared as a non-template overload next to ModeType so it is visible wherever ModeType is,
/// rather than depending on a generated header having been included first: the generic
/// to_string<T> in common.hpp would otherwise win at every call site that cannot see one.
inline std::string to_string(ModeType mode)
{
  auto & registry = detail::mode_registry();
  const std::lock_guard<std::mutex> lock(registry.mutex);
  const auto it = registry.names.find(mode);
  return it != registry.names.end() ? it->second : std::to_string(mode);
}

}  // namespace packml_sm

#endif  // PACKML_SM__MODES_REGISTRY_HPP_
