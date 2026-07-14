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
#include <set>

#include <yaml-cpp/yaml.h>

#include <packml_msgs/msg/node_health.hpp>

#include "packml_ros/error_catalog_schema_keys.hpp"

namespace packml_ros {

namespace {
constexpr const char * kInstancePlaceholder = "{instance}";

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
// NodeCatalog
// ---------------------------------------------------------------------------

const ErrorEntry * NodeCatalog::find(int32_t code) const
{
  for (const auto & entry : entries) {
    if (entry.code == code) {
      return &entry;
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// ErrorCatalog
// ---------------------------------------------------------------------------

void ErrorCatalog::load_node_catalog(const NodeCatalog & catalog)
{
  catalogs_[catalog.node_name] = catalog;
}

const ErrorEntry * ErrorCatalog::find(const std::string & node_name, int32_t code) const
{
  const auto it = catalogs_.find(node_name);
  if (it == catalogs_.end()) {
    return nullptr;
  }
  return it->second.find(code);
}

std::vector<NodeCatalog> ErrorCatalog::all() const
{
  std::vector<NodeCatalog> out;
  out.reserve(catalogs_.size());
  for (const auto & [name, cat] : catalogs_) {
    out.push_back(cat);
  }
  return out;
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

std::string action_to_string(int32_t action)
{
  using NodeHealth = packml_msgs::msg::NodeHealth;
  switch (action) {
    case NodeHealth::NONE: return "NONE";
    case NodeHealth::WARN: return "WARN";
    case NodeHealth::HOLD: return "HOLD";
    case NodeHealth::SUSPEND: return "SUSPEND";
    case NodeHealth::ABORT: return "ABORT";
    default: return "NONE";
  }
}

// ---------------------------------------------------------------------------
// YAML loading
// ---------------------------------------------------------------------------

CatalogLoadResult load_node_catalog_from_yaml(const std::string & path)
{
  CatalogLoadResult result;

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    result.ok = false;
    result.error = "Failed to load error catalog YAML '" + path + "': " + e.what();
    return result;
  }

  if (!root[schema::kNodeNameKey]) {
    result.ok = false;
    result.error = "Error catalog YAML '" + path + "' has no '" + schema::kNodeNameKey + "' key";
    return result;
  }
  result.catalog.node_name = root[schema::kNodeNameKey].as<std::string>();

  // --- error_codes: name → int (authoritative set of codes) ---
  std::map<std::string, int32_t> code_by_name;
  std::set<int32_t> seen_codes;
  if (root[schema::kErrorCodesKey]) {
    for (const auto & kv : root[schema::kErrorCodesKey]) {
      const auto name = kv.first.as<std::string>();
      const auto code = kv.second.as<int32_t>();

      if (code == 0) {
        result.warnings.push_back(
          "error code '" + name + "' uses reserved value 0 — entry rejected");
        continue;
      }
      if (!seen_codes.insert(code).second) {
        result.warnings.push_back(
          "duplicate error code value " + std::to_string(code) +
          " ('" + name + "') — last wins");
      }
      code_by_name[name] = code;
    }
  } else {
    result.warnings.push_back(std::string("no '") + schema::kErrorCodesKey + "' section found");
  }

  // --- descriptions: name → { action, severity, <locale>: text, ... } ---
  std::set<std::string> described_names;
  if (root[schema::kDescriptionsKey]) {
    for (const auto & kv : root[schema::kDescriptionsKey]) {
      const auto name = kv.first.as<std::string>();
      const YAML::Node & block = kv.second;
      described_names.insert(name);

      const auto code_it = code_by_name.find(name);
      if (code_it == code_by_name.end()) {
        result.warnings.push_back(
          "description '" + name + "' has no matching entry in error_codes — ignored");
        continue;
      }

      ErrorEntry entry;
      entry.code = code_it->second;
      entry.name = name;

      for (const auto & field : block) {
        const auto key = field.first.as<std::string>();
        if (key == schema::kActionKey) {
          entry.action = action_from_string(field.second.as<std::string>());
        } else if (key == schema::kSeverityKey) {
          entry.severity = field.second.as<std::string>();
        } else if (key == schema::kCategoryKey) {
          entry.category = field.second.as<std::string>();
        } else if (key == schema::kInstancedKey) {
          entry.instanced = field.second.as<bool>();
        } else {
          // Anything else is treated as a locale → description text.
          entry.descriptions[key] = field.second.as<std::string>();
        }
      }

      if (entry.descriptions.empty()) {
        result.warnings.push_back(
          "error code '" + name + "' has no locale descriptions");
      }

      // Instanced entries must carry the {instance} placeholder in every locale;
      // plain entries must not carry it anywhere.
      for (const auto & [locale, text] : entry.descriptions) {
        const bool has_placeholder = text.find(kInstancePlaceholder) != std::string::npos;
        if (entry.instanced && !has_placeholder) {
          result.warnings.push_back(
            "instanced error code '" + name + "' description [" + locale +
            "] has no \"{instance}\" placeholder");
        } else if (!entry.instanced && has_placeholder) {
          result.warnings.push_back(
            "non-instanced error code '" + name + "' description [" + locale +
            "] contains a \"{instance}\" placeholder");
        }
      }

      result.catalog.entries.push_back(std::move(entry));
    }
  } else {
    result.warnings.push_back(std::string("no '") + schema::kDescriptionsKey + "' section found");
  }

  // --- Validate: every code must have a description ---
  for (const auto & [name, code] : code_by_name) {
    if (described_names.find(name) == described_names.end()) {
      result.warnings.push_back(
        "error code '" + name + "' (" + std::to_string(code) +
        ") has no description block");
    }
  }

  // Keep entries sorted by code for stable export ordering.
  std::sort(
    result.catalog.entries.begin(), result.catalog.entries.end(),
    [](const ErrorEntry & a, const ErrorEntry & b) { return a.code < b.code; });

  result.ok = true;
  return result;
}

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

  // A blank instance_id leaves the template unfilled (e.g. a service lookup with
  // no instance); a populated one substitutes the localized label everywhere.
  if (!instance_id.empty()) {
    const std::string placeholder = kInstancePlaceholder;
    const std::string label = instance_label(instance_id, locale);
    for (std::size_t pos = text.find(placeholder); pos != std::string::npos;
      pos = text.find(placeholder, pos + label.size()))
    {
      text.replace(pos, placeholder.size(), label);
    }
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
  // manager's init(), which the fail-open contract (design §6) forbids.
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
            me.entry.action = action_from_string(field.second.as<std::string>());
          } else if (key == schema::kSeverityKey) {
            me.entry.severity = field.second.as<std::string>();
          } else if (key == schema::kCategoryKey) {
            me.entry.category = field.second.as<std::string>();
          } else {
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
          if (cb[schema::kActionKey]) {me.entry.action = action_from_string(cb[schema::kActionKey].as<std::string>());}
          if (cb[schema::kInstancedKey]) {me.entry.instanced = cb[schema::kInstancedKey].as<bool>();}
          if (cb[schema::kDescriptionsKey]) {
            for (const auto & desc : cb[schema::kDescriptionsKey]) {
              me.entry.descriptions[desc.first.as<std::string>()] =
                desc.second.as<std::string>();
            }
          }

          if (me.global != 0 && result.catalog.find_global(me.global) != nullptr) {
            result.warnings.push_back(
              "duplicate global fault number " + std::to_string(me.global) +
              " (node '" + node_name + "' code " + std::to_string(me.entry.code) +
              ") — last wins");
          }
          result.catalog.add_node_entry(me);
        }
      }
    }
  } catch (const std::exception & e) {
    result.ok = false;
    result.error = "Failed to parse machine error catalog YAML '" + path + "': " + e.what();
    return result;
  }

  result.ok = true;
  return result;
}

}  // namespace packml_ros
