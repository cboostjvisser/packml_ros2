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

#include "packml_ros/error_catalog.hpp"

#include <algorithm>
#include <filesystem>
#include <system_error>

#include <yaml-cpp/yaml.h>

#include <packml_msgs/msg/node_health.hpp>

#include "packml_ros/error_catalog_schema_keys.hpp"

namespace packml_ros {

namespace {
constexpr const char * kInstancePlaceholder = "{instance}";

/// Substituted for {instance} when resolve_message() has no instance_id to work
/// with, so an instanced fault raised without one still reads as prose instead
/// of leaking the raw template placeholder.
constexpr const char * kUnspecifiedInstanceLabel = "(unspecified)";

/// Defense-in-depth cap on machine_error_catalog.yaml's file size, checked
/// before handing it to yaml-cpp. This file is a build-generated artifact
/// (the aggregation tool is where a vendor's raw, untrusted per-node catalog
/// is actually parsed — see aggregate_error_catalog.py's own, stricter
/// per-file bound and node-count-limited loader), not third-party input
/// itself, so this is a narrower defense: it catches an oversized or
/// corrupted/tampered file at the manager's one remaining YAML entry point,
/// not a full mitigation against a maliciously crafted document (yaml-cpp's
/// recursive-descent parser has no exposed hook to bound node/anchor count
/// the way the Python loader's compose_node override does).
constexpr std::uintmax_t kMaxMachineCatalogBytes = 10 * 1024 * 1024;  // 10 MB

bool has_instance_placeholder(const ErrorEntry & entry)
{
  for (const auto & [locale, text] : entry.descriptions) {
    (void)locale;
    if (text.find(kInstancePlaceholder) != std::string::npos) {
      return true;
    }
  }
  return false;
}
}  // namespace

// ---------------------------------------------------------------------------
// ErrorEntry
// ---------------------------------------------------------------------------

std::string ErrorEntry::description(const std::string & locale) const
{
  if (const auto it = descriptions.find(locale); it != descriptions.end()) {
    return it->second;
  }
  if (const auto it = descriptions.find("en"); it != descriptions.end()) {
    return it->second;
  }
  if (!descriptions.empty()) {
    return descriptions.begin()->second;
  }
  return "";
}

// ---------------------------------------------------------------------------
// Action name mapping
// ---------------------------------------------------------------------------

int32_t action_from_string(const std::string & action_name)
{
  using NodeHealth = packml_msgs::msg::NodeHealth;
  if (action_name == "NONE") {return NodeHealth::NONE;}
  if (action_name == "WARN") {return NodeHealth::WARN;}
  if (action_name == "HOLD") {return NodeHealth::HOLD;}
  if (action_name == "SUSPEND") {return NodeHealth::SUSPEND;}
  if (action_name == "ABORT") {return NodeHealth::ABORT;}
  return NodeHealth::NONE;
}

namespace {
/// True if `name` is one of action_from_string's recognized inputs — used by the
/// machine loader to warn on a typo/unrecognized action instead of silently
/// treating it as NONE (a real, safety-relevant classification in its own right).
bool is_known_action_name(const std::string & name)
{
  return name == "NONE" || name == "WARN" || name == "HOLD" ||
         name == "SUSPEND" || name == "ABORT";
}
}  // namespace

// ---------------------------------------------------------------------------
// MachineCatalog
// ---------------------------------------------------------------------------

void MachineCatalog::invalidate_stale_global(int32_t winning_global, const MachineEntry & winner)
{
  const auto prev = by_global_.find(winning_global);
  if (prev == by_global_.end()) {
    return;
  }
  const MachineEntry & stale = prev->second;
  // Reserved entries share the meaningless default entry.code == 0, so their
  // identity is their name; node entries are identified by (node_name, code).
  const bool same_entity = (stale.node_name == kManagerNodeName)
    ? (stale.entry.name == winner.entry.name)
    : (stale.node_name == winner.node_name && stale.entry.code == winner.entry.code);
  if (same_entity) {
    return;  // same entry re-added with a new global — not a collision
  }
  if (stale.node_name == kManagerNodeName) {
    const auto it = reserved_.find(stale.entry.name);
    if (it != reserved_.end()) {
      it->second = 0;
    }
  } else {
    by_node_local_[stale.node_name][stale.entry.code].global = 0;
  }
}

void MachineCatalog::add_node_entry(const MachineEntry & e)
{
  if (e.global != 0) {  // 0 means "no mapping" — keep it out of the global index
    invalidate_stale_global(e.global, e);
    by_global_[e.global] = e;
  }
  by_node_local_[e.node_name][e.entry.code] = e;
}

void MachineCatalog::add_reserved_entry(const std::string & name, const MachineEntry & e)
{
  if (e.global != 0) {
    invalidate_stale_global(e.global, e);
    by_global_[e.global] = e;
  }
  reserved_[name] = e.global;
}

void MachineCatalog::set_instance_labels(
  std::map<std::string, std::map<std::string, std::string>> labels)
{
  instances_ = std::move(labels);
}

void MachineCatalog::set_languages(std::vector<std::string> langs)
{
  languages_ = std::move(langs);
}

const MachineEntry * MachineCatalog::find(
  const std::string & node_name, int32_t local_code) const
{
  const auto node_it = by_node_local_.find(node_name);
  if (node_it == by_node_local_.end()) {
    return nullptr;
  }
  const auto code_it = node_it->second.find(local_code);
  return code_it == node_it->second.end() ? nullptr : &code_it->second;
}

const MachineEntry * MachineCatalog::find_global(int32_t global) const
{
  const auto it = by_global_.find(global);
  return it == by_global_.end() ? nullptr : &it->second;
}

int32_t MachineCatalog::reserved(const std::string & name) const
{
  const auto it = reserved_.find(name);
  return it == reserved_.end() ? 0 : it->second;
}

std::string MachineCatalog::instance_label(
  const std::string & instance_id, const std::string & locale) const
{
  const auto it = instances_.find(instance_id);
  if (it == instances_.end()) {
    return instance_id;  // unknown id → the raw token
  }
  const auto & by_locale = it->second;
  if (const auto j = by_locale.find(locale); j != by_locale.end()) {
    return j->second;
  }
  if (const auto j = by_locale.find("en"); j != by_locale.end()) {
    return j->second;
  }
  if (!by_locale.empty()) {
    return by_locale.begin()->second;
  }
  return instance_id;  // present but empty → the raw token
}

std::string MachineCatalog::resolve_message(
  const ErrorEntry & entry, const std::string & locale,
  const std::string & instance_id) const
{
  std::string text = entry.description(locale);
  const std::string placeholder = kInstancePlaceholder;
  if (text.find(placeholder) == std::string::npos) {
    return text;
  }

  // A blank instance_id (e.g. an instanced fault raised without one) has no
  // per-instance context to substitute — fall back to a neutral label rather
  // than shipping the raw "{instance}" template syntax to an operator.
  const std::string label =
    instance_id.empty() ? kUnspecifiedInstanceLabel : instance_label(instance_id, locale);
  for (std::size_t pos = text.find(placeholder); pos != std::string::npos;
    pos = text.find(placeholder, pos + label.size()))
  {
    text.replace(pos, placeholder.size(), label);
  }
  return text;
}

std::vector<std::string> MachineCatalog::node_names() const
{
  std::vector<std::string> names;
  names.reserve(by_node_local_.size());
  for (const auto & [name, codes] : by_node_local_) {
    (void)codes;
    names.push_back(name);
  }
  return names;
}

bool MachineCatalog::has_node(const std::string & node_name) const
{
  return by_node_local_.find(node_name) != by_node_local_.end();
}

std::size_t MachineCatalog::size() const
{
  std::size_t total = 0;
  for (const auto & [node_name, codes] : by_node_local_) {
    (void)node_name;
    total += codes.size();
  }
  return total;
}

MachineCatalogLoadResult load_machine_catalog_from_yaml(const std::string & path)
{
  MachineCatalogLoadResult result;

  std::error_code fs_error;
  const auto file_size = std::filesystem::file_size(path, fs_error);
  if (!fs_error && file_size > kMaxMachineCatalogBytes) {
    result.ok = false;
    result.error = "Machine error catalog YAML '" + path + "' is " +
      std::to_string(file_size) + " bytes, over the " +
      std::to_string(kMaxMachineCatalogBytes) + "-byte safety limit";
    return result;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    result.ok = false;
    result.error = "Failed to load machine error catalog YAML '" + path + "': " + e.what();
    return result;
  }

  // The generated file is trusted, but a truncated or hand-edited one can be
  // valid YAML of the wrong shape (an overflowing global, a scalar where a map
  // is expected, a non-bool flag). Parse defensively so such a file fails open
  // — ok == false — rather than throwing a yaml-cpp exception out of the
  // manager's init() — a bad catalog must degrade text, never crash the manager.
  // Counts entries actually added below; a file that parses but recognizes
  // neither a 'reserved' nor a 'nodes' entry has silently degraded to nothing
  // (e.g. a wrong top-level key from a format drift) and must fail, not load an
  // empty catalog that then enriches nothing with no visible cause.
  std::size_t entries_added = 0;

  try {
    // --- languages ---
    std::vector<std::string> languages;
    if (root[schema::kLanguagesKey]) {
      for (const auto & lang : root[schema::kLanguagesKey]) {
        languages.push_back(lang.as<std::string>());
      }
    }
    result.catalog.set_languages(languages);

    // --- instances: id → { locale: label } ---
    std::map<std::string, std::map<std::string, std::string>> instances;
    if (root[schema::kInstancesKey]) {
      for (const auto & kv : root[schema::kInstancesKey]) {
        const auto id = kv.first.as<std::string>();
        for (const auto & label : kv.second) {
          instances[id][label.first.as<std::string>()] = label.second.as<std::string>();
        }
      }
    }
    result.catalog.set_instance_labels(instances);

    // --- reserved: manager-synthesized faults (name → { global, ...description }) ---
    if (root[schema::kReservedKey]) {
      for (const auto & kv : root[schema::kReservedKey]) {
        const auto name = kv.first.as<std::string>();
        const YAML::Node & block = kv.second;
        if (!block[schema::kGlobalKey]) {
          result.warnings.push_back(
            "reserved '" + name + "' has no '" + schema::kGlobalKey + "' number — skipped");
          continue;
        }

        MachineEntry me;
        me.global = block[schema::kGlobalKey].as<int32_t>();
        me.node_name = kManagerNodeName;
        me.entry.name = name;

        for (const auto & field : block) {
          const auto key = field.first.as<std::string>();
          if (key == schema::kGlobalKey) {
            continue;
          } else if (key == schema::kActionKey) {
            const auto action_str = field.second.as<std::string>();
            if (!is_known_action_name(action_str)) {
              result.warnings.push_back(
                "reserved '" + name + "' has unrecognized action '" + action_str +
                "' — treated as NONE");
            }
            me.entry.action = action_from_string(action_str);
          } else if (key == schema::kSeverityKey) {
            me.entry.severity = field.second.as<std::string>();
          } else if (key == schema::kCategoryKey) {
            me.entry.category = field.second.as<std::string>();
          } else {
            // Anything else is treated as a locale → description text. If the
            // file declares a 'languages:' list, a key outside it is very
            // likely a misspelled field name (e.g. "sevrity") rather than a
            // real, unconfigured locale — warn instead of silently absorbing
            // it as a locale with a blank severity/action/category left behind.
            if (!languages.empty() &&
              std::find(languages.begin(), languages.end(), key) == languages.end())
            {
              result.warnings.push_back(
                "reserved '" + name + "' has key '" + key +
                "' which is not in the configured 'languages' list — treated as a "
                "locale description, but check for a typo of a field name "
                "(global/action/severity/category)");
            }
            me.entry.descriptions[key] = field.second.as<std::string>();
          }
        }
        me.entry.instanced = has_instance_placeholder(me.entry);

        if (me.global != 0 && result.catalog.find_global(me.global) != nullptr) {
          result.warnings.push_back(
            "duplicate global fault number " + std::to_string(me.global) +
            " (reserved '" + name + "') — last wins");
        }
        result.catalog.add_reserved_entry(name, me);
        ++entries_added;
      }
    }

    // --- nodes: node → { base, codes: { local: { global, ... } } } ---
    if (root[schema::kNodesKey]) {
      for (const auto & node_kv : root[schema::kNodesKey]) {
        const auto node_name = node_kv.first.as<std::string>();
        const YAML::Node & node_block = node_kv.second;
        if (!node_block[schema::kCodesKey]) {
          result.warnings.push_back(
            "node '" + node_name + "' has no '" + schema::kCodesKey + "' section");
          continue;
        }
        for (const auto & code_kv : node_block[schema::kCodesKey]) {
          const YAML::Node & cb = code_kv.second;

          MachineEntry me;
          me.node_name = node_name;
          me.entry.code = code_kv.first.as<int32_t>();
          me.global = cb[schema::kGlobalKey] ? cb[schema::kGlobalKey].as<int32_t>() : 0;
          if (cb[schema::kNameKey]) {me.entry.name = cb[schema::kNameKey].as<std::string>();}
          if (cb[schema::kSeverityKey]) {me.entry.severity = cb[schema::kSeverityKey].as<std::string>();}
          if (cb[schema::kCategoryKey]) {me.entry.category = cb[schema::kCategoryKey].as<std::string>();}
          if (cb[schema::kActionKey]) {
            const auto action_str = cb[schema::kActionKey].as<std::string>();
            if (!is_known_action_name(action_str)) {
              result.warnings.push_back(
                "node '" + node_name + "' code " + std::to_string(me.entry.code) +
                " has unrecognized action '" + action_str + "' — treated as NONE");
            }
            me.entry.action = action_from_string(action_str);
          }
          if (cb[schema::kInstancedKey]) {me.entry.instanced = cb[schema::kInstancedKey].as<bool>();}
          if (cb[schema::kDescriptionsKey]) {
            for (const auto & desc : cb[schema::kDescriptionsKey]) {
              const auto locale = desc.first.as<std::string>();
              if (!languages.empty() &&
                std::find(languages.begin(), languages.end(), locale) == languages.end())
              {
                result.warnings.push_back(
                  "node '" + node_name + "' code " + std::to_string(me.entry.code) +
                  " has a description locale '" + locale +
                  "' which is not in the configured 'languages' list — check for a typo");
              }
              me.entry.descriptions[locale] = desc.second.as<std::string>();
            }
          }

          if (me.global != 0 && result.catalog.find_global(me.global) != nullptr) {
            result.warnings.push_back(
              "duplicate global fault number " + std::to_string(me.global) +
              " (node '" + node_name + "' code " + std::to_string(me.entry.code) +
              ") — last wins");
          }
          result.catalog.add_node_entry(me);
          ++entries_added;
        }
      }
    }
  } catch (const std::exception & e) {
    result.ok = false;
    result.error = "Failed to parse machine error catalog YAML '" + path + "': " + e.what();
    return result;
  }

  if (entries_added == 0) {
    result.ok = false;
    result.error = "Machine error catalog '" + path +
      "' recognized no 'reserved' or 'nodes' entries";
    return result;
  }

  result.ok = true;
  return result;
}

}  // namespace packml_ros
