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
// ErrorCatalog — implementor-defined error codes with multi-language descriptions.
//
// The error code integers are strictly typed in C++ (generated from each node's
// per-node YAML `error_codes:` section at build time — see
// cmake/generate_error_codes_header.py). Per-node YAML is otherwise parsed only by
// the (Python) offline aggregation tool, which folds every node's codes and
// descriptions into one machine_error_catalog.yaml; MachineCatalog below is what
// actually loads that at runtime, so translations can be updated by re-aggregating
// without recompiling.
//
// This header has no ROS/packml_msgs dependency; action_from_string below maps
// action names to NodeHealth constants and is implemented in error_catalog.cpp,
// which is the only file in this library that includes packml_msgs.

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace packml_ros {

/// One error code entry: the integer, its symbolic name, an action/severity hint,
/// and one description string per locale.
struct ErrorEntry
{
  int32_t code{0};
  std::string name;        ///< symbolic name from error_codes: (e.g. "OVERCURRENT")
  int32_t action{0};       ///< NodeHealth action constant (NONE..ABORT)
  std::string severity;    ///< CRITICAL | ERROR | WARNING | INFO (documentation only)
  std::map<std::string, std::string> descriptions;  ///< locale tag → text
  // Appended after descriptions so existing positional ErrorEntry{…} initializers
  // (which end at descriptions) keep compiling.
  std::string category;    ///< controlled vocabulary (electrical, mechanical, …);
                           ///< documentation only, doesn't drive machine behavior
  bool instanced{false};   ///< true → descriptions carry a {instance} placeholder

  /// Resolve the description for a locale, with fallback:
  ///   requested locale → "en" → first available entry → "" (empty).
  std::string description(const std::string & locale) const;
};

// ---------------------------------------------------------------------------
// Action name mapping
// ---------------------------------------------------------------------------

/// Map a PackML action name ("NONE","WARN","HOLD","SUSPEND","ABORT") to its
/// NodeHealth action integer.  Unknown names map to 0 (NONE).
int32_t action_from_string(const std::string & action_name);

// ---------------------------------------------------------------------------
// MachineCatalog — the integrated, global-numbered catalog
// ---------------------------------------------------------------------------
//
// Produced offline by the aggregation tool and loaded by the manager at
// startup, exactly like the modes config. It maps each
// (node, node-local code) to a globally-unique fault number, resolves localized
// operator messages (substituting instance labels into {instance} templates),
// and carries the manager-synthesized "reserved" faults (e.g. heartbeat_timeout).

/// Sentinel `MachineEntry::node_name` for manager-synthesized reserved faults
/// (e.g. heartbeat_timeout) — no Equipment Module owns these.
constexpr const char * kManagerNodeName = "manager";

/// One entry in the machine catalog: a node-local ErrorEntry plus its assigned
/// global fault number and the owning node (kManagerNodeName for reserved entries).
struct MachineEntry
{
  int32_t global{0};
  std::string node_name;   ///< owning node; kManagerNodeName for reserved entries
  ErrorEntry entry;        ///< entry.code is the node-local code (0 for reserved)
};

/// The integrated global catalog loaded from machine_error_catalog.yaml.
class MachineCatalog
{
public:
  /// Look up by owning node + node-local code.  Returns nullptr if absent.
  const MachineEntry * find(const std::string & node_name, int32_t local_code) const;

  /// Look up by global fault number (covers node entries and reserved).
  /// Returns nullptr if absent.
  const MachineEntry * find_global(int32_t global) const;

  /// Global number for a manager-reserved fault name (e.g. "heartbeat_timeout"),
  /// or 0 if the name is not reserved.
  int32_t reserved(const std::string & name) const;

  /// Localized label for an instance id, with fallback:
  ///   requested locale → "en" → first available → the raw id itself.
  std::string instance_label(
    const std::string & instance_id, const std::string & locale) const;

  /// Resolve an operator message: entry.description(locale) with a {instance}
  /// placeholder (if any) replaced by the localized instance label.  A blank
  /// instance_id substitutes a neutral "(unspecified)" label instead of
  /// leaking the raw template placeholder.
  std::string resolve_message(
    const ErrorEntry & entry, const std::string & locale,
    const std::string & instance_id) const;

  /// Configured display languages (BCP 47 tags).
  const std::vector<std::string> & languages() const { return languages_; }

  /// Names of all nodes present in the catalog (for the manager's startup
  /// drift check — a catalog node with no matching required_nodes entry).
  std::vector<std::string> node_names() const;

  /// True if the catalog carries any entries for the given node.
  bool has_node(const std::string & node_name) const;

  /// Total number of node code entries (excludes reserved).
  std::size_t size() const;

  // --- population (used by load_machine_catalog_from_yaml) ---
  void add_node_entry(const MachineEntry & e);
  void add_reserved_entry(const std::string & name, const MachineEntry & e);
  void set_instance_labels(std::map<std::string, std::map<std::string, std::string>> labels);
  void set_languages(std::vector<std::string> langs);

private:
  /// If `winning_global` was already claimed by a different (node, local) or
  /// reserved entry, zero that entry's stored global so find()/find_global()
  /// agree on one owner (the loader has already logged a "duplicate global —
  /// last wins" warning; this makes the indices actually round-trip on it).
  void invalidate_stale_global(int32_t winning_global, const MachineEntry & winner);

  std::map<std::string, std::map<int32_t, MachineEntry>> by_node_local_;  ///< node → local → entry
  std::map<int32_t, MachineEntry> by_global_;                             ///< global → entry (nodes + reserved)
  std::map<std::string, int32_t> reserved_;                               ///< reserved name → global
  std::map<std::string, std::map<std::string, std::string>> instances_;   ///< id → locale → label
  std::vector<std::string> languages_;
};

/// Result of loading the machine catalog YAML, including non-fatal warnings.
struct MachineCatalogLoadResult
{
  bool ok{false};                       ///< false → file missing / parse error /
                                         ///< recognized zero 'reserved' or 'nodes' entries
  MachineCatalog catalog;
  std::vector<std::string> warnings;
  std::string error;                    ///< populated when ok == false
};

/// Load a MachineCatalog from the generated machine_error_catalog.yaml.
///
/// Expected structure:
///   schema_version: 1
///   languages: [en, nl]
///   reserved:
///     heartbeat_timeout: { global: 1, severity: CRITICAL, action: ABORT, en: "..." }
///   instances:
///     cell_north: { en: "North Cell", nl: "Noord Cel" }
///   nodes:
///     motor_driver_node:
///       base: 1000
///       codes:
///         301: { global: 1301, name: OVERCURRENT, severity: CRITICAL,
///                action: ABORT, category: electrical, instanced: false,
///                descriptions: { en: "...", nl: "..." } }
MachineCatalogLoadResult load_machine_catalog_from_yaml(const std::string & path);

}  // namespace packml_ros
