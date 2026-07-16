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
// Tests for the MachineCatalog core (pure C++, no ROS node required).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "packml_ros/error_catalog.hpp"
#include "packml_msgs/msg/node_health.hpp"

using packml_ros::ErrorEntry;
using packml_ros::MachineCatalog;
using packml_ros::MachineEntry;
using packml_ros::MachineCatalogLoadResult;
using packml_ros::load_machine_catalog_from_yaml;
using NodeHealth = packml_msgs::msg::NodeHealth;

namespace {

/// Write content to a unique temp YAML file; returns its path.
/// The fixture removes it on teardown.
class TempYaml
{
public:
  explicit TempYaml(const std::string & content)
  {
    static int counter = 0;
    path_ = std::filesystem::temp_directory_path() /
      ("error_catalog_test_" + std::to_string(::getpid()) + "_" +
       std::to_string(counter++) + ".yaml");
    std::ofstream f(path_);
    f << content;
    f.close();
  }

  ~TempYaml()
  {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
  }

  std::string path() const { return path_.string(); }

private:
  std::filesystem::path path_;
};

bool has_warning_containing(
  const std::vector<std::string> & warnings, const std::string & needle)
{
  return std::any_of(
    warnings.begin(), warnings.end(),
    [&](const std::string & w) { return w.find(needle) != std::string::npos; });
}

// A generated machine_error_catalog.yaml, as produced by the offline aggregation
// tool: two nodes (motor at base 1000, gripper at base 5000), a reserved
// manager fault, and localized instance labels.
const char * kMachineYaml = R"(
languages: [en, nl]
reserved:
  heartbeat_timeout:
    global: 1
    severity: CRITICAL
    action: ABORT
    en: "Heartbeat lost from {instance}"
    nl: "Heartbeat verloren van {instance}"
instances:
  cell_north:
    en: "North Cell"
    nl: "Noord Cel"
  loading_bay:
    en: "Loading Bay"
nodes:
  motor_driver_node:
    base: 1000
    codes:
      301:
        global: 1301
        name: OVERCURRENT
        severity: CRITICAL
        action: ABORT
        category: electrical
        instanced: false
        descriptions:
          en: "Motor overcurrent detected"
          nl: "Motor overstroom gedetecteerd"
      10:
        global: 1010
        name: E_STOP_TRIGGERED
        severity: CRITICAL
        action: ABORT
        instanced: true
        descriptions:
          en: "Emergency stop triggered at {instance}"
          nl: "Noodstop geactiveerd bij {instance}"
  acme_gripper_node:
    base: 5000
    codes:
      90:
        global: 5090
        name: GRIP_FAULT
        severity: ERROR
        action: HOLD
        descriptions:
          en: "Gripper fault"
)";

}  // namespace

// ---------------------------------------------------------------------------
// Action name mapping
// ---------------------------------------------------------------------------

TEST(ErrorCatalogAction, StringToActionMapping)
{
  EXPECT_EQ(packml_ros::action_from_string("NONE"), NodeHealth::NONE);
  EXPECT_EQ(packml_ros::action_from_string("WARN"), NodeHealth::WARN);
  EXPECT_EQ(packml_ros::action_from_string("HOLD"), NodeHealth::HOLD);
  EXPECT_EQ(packml_ros::action_from_string("SUSPEND"), NodeHealth::SUSPEND);
  EXPECT_EQ(packml_ros::action_from_string("ABORT"), NodeHealth::ABORT);
  // Unknown → NONE
  EXPECT_EQ(packml_ros::action_from_string("BOGUS"), NodeHealth::NONE);
}

// ---------------------------------------------------------------------------
// MachineCatalog — the integrated, global-numbered catalog
// ---------------------------------------------------------------------------

TEST(MachineCatalog, LoadsGeneratedFile)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());

  ASSERT_TRUE(result.ok) << result.error;
  // 3 node code entries (motor 301 + 10, gripper 90); reserved is excluded.
  EXPECT_EQ(result.catalog.size(), 3u);
  ASSERT_EQ(result.catalog.languages().size(), 2u);
  EXPECT_EQ(result.catalog.languages()[0], "en");
  EXPECT_EQ(result.catalog.languages()[1], "nl");
}

TEST(MachineCatalog, FindByNodeAndLocalReturnsGlobal)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * e = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->global, 1301);
  EXPECT_EQ(e->entry.name, "OVERCURRENT");
  EXPECT_EQ(e->entry.category, "electrical");
  EXPECT_EQ(e->entry.action, NodeHealth::ABORT);
  EXPECT_EQ(e->entry.severity, "CRITICAL");
}

TEST(MachineCatalog, FindByGlobal)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * e = result.catalog.find_global(1301);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->node_name, "motor_driver_node");
  EXPECT_EQ(e->entry.code, 301);
  EXPECT_EQ(e->entry.name, "OVERCURRENT");
}

TEST(MachineCatalog, UnknownNodeOrLocalReturnsNullptr)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.catalog.find("ghost_node", 1), nullptr);
  EXPECT_EQ(result.catalog.find("motor_driver_node", 999), nullptr);
  EXPECT_EQ(result.catalog.find_global(9999), nullptr);
}

TEST(MachineCatalog, RemappedCodeResolves)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  // The gripper sits at base 5000; local 90 maps to global 5090.
  const auto * e = result.catalog.find("acme_gripper_node", 90);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->global, 5090);
}

TEST(MachineCatalog, ReservedHeartbeatTimeoutCode)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.catalog.reserved("heartbeat_timeout"), 1);
  EXPECT_EQ(result.catalog.reserved("not_reserved"), 0);

  const auto * e = result.catalog.find_global(1);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->node_name, packml_ros::kManagerNodeName);
  EXPECT_TRUE(e->entry.instanced);  // its text carries {instance}
}

TEST(MachineCatalog, InstanceLabelLocalized)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.catalog.instance_label("cell_north", "nl"), "Noord Cel");
  EXPECT_EQ(result.catalog.instance_label("cell_north", "en"), "North Cell");
}

TEST(MachineCatalog, InstanceLabelFallsBackToRawId)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  // Unknown id → the raw token.
  EXPECT_EQ(result.catalog.instance_label("unknown_id", "en"), "unknown_id");
  // Known id, missing locale → falls back to "en".
  EXPECT_EQ(result.catalog.instance_label("loading_bay", "nl"), "Loading Bay");
}

TEST(MachineCatalog, ResolveMessageSubstitutesInstanceAndFallsBackLocales)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * e = result.catalog.find("motor_driver_node", 10);  // E_STOP_TRIGGERED
  ASSERT_NE(e, nullptr);

  // Localized description + localized instance label.
  EXPECT_EQ(
    result.catalog.resolve_message(e->entry, "nl", "cell_north"),
    "Noodstop geactiveerd bij Noord Cel");

  // Missing description locale → en; missing instance-label locale → en.
  EXPECT_EQ(
    result.catalog.resolve_message(e->entry, "de", "cell_north"),
    "Emergency stop triggered at North Cell");

  // Blank instance_id has nothing to substitute — falls back to a neutral label
  // rather than leaking the raw "{instance}" template syntax.
  EXPECT_EQ(
    result.catalog.resolve_message(e->entry, "en", ""),
    "Emergency stop triggered at (unspecified)");
}

TEST(MachineCatalog, MalformedMachineFileFailsGracefully)
{
  TempYaml yaml("nodes: { unterminated flow mapping");
  auto result = load_machine_catalog_from_yaml(yaml.path());

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

// Defense-in-depth: an oversized machine catalog file (corrupted, tampered,
// or simply misconfigured to point at the wrong file) fails open rather than
// being handed to yaml-cpp — see kMaxMachineCatalogBytes.
TEST(MachineCatalog, OversizedFileFailsOpen)
{
  std::string huge = "languages: [en]\n# padding\n";
  huge += std::string(11 * 1024 * 1024, '#');  // 11 MiB, over the 10 MiB cap
  TempYaml yaml(huge);
  auto result = load_machine_catalog_from_yaml(yaml.path());

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(has_warning_containing({result.error}, "safety limit"));
}

// Valid YAML of the wrong shape must fail open (ok == false), not throw out of
// the loader — a corrupt/hand-edited generated file the manager may still load.
TEST(MachineCatalog, WrongShapeFileFailsGracefully)
{
  // 'global' overflows int32; the parse body (not just LoadFile) must catch it.
  TempYaml yaml(R"(
nodes:
  motor_driver_node:
    codes:
      301:
        global: 99999999999
        name: OVERCURRENT
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST(MachineCatalog, NodeNamesAndHasNode)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto names = result.catalog.node_names();
  EXPECT_EQ(names.size(), 2u);  // "manager" (reserved) is not a node
  EXPECT_TRUE(result.catalog.has_node("motor_driver_node"));
  EXPECT_TRUE(result.catalog.has_node("acme_gripper_node"));
  EXPECT_FALSE(result.catalog.has_node(packml_ros::kManagerNodeName));
  EXPECT_FALSE(result.catalog.has_node("ghost_node"));
}

// A remapped local (key ≠ base + local) round-trips to its assigned global.
TEST(MachineCatalog, RemappedLocalKeyResolves)
{
  TempYaml yaml(R"(
nodes:
  acme_gripper_node:
    base: 5000
    codes:
      30100:
        global: 5099
        name: WILD_VENDOR_CODE
        descriptions:
          en: "Vendor fault"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * e = result.catalog.find("acme_gripper_node", 30100);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->global, 5099);
  ASSERT_NE(result.catalog.find_global(5099), nullptr);
  EXPECT_EQ(result.catalog.find_global(5099)->entry.code, 30100);
}

TEST(MachineCatalog, LanguagesDefaultEmptyWhenAbsent)
{
  TempYaml yaml(R"(
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
        descriptions:
          en: "Motor overcurrent"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(result.catalog.languages().empty());
  ASSERT_NE(result.catalog.find("motor_driver_node", 301), nullptr);
}

// A code with no 'global' key is not indexed under global 0 ("no mapping"), but
// is still reachable by (node, local).
TEST(MachineCatalog, GlobalZeroNotIndexed)
{
  TempYaml yaml(R"(
nodes:
  motor_driver_node:
    codes:
      301:
        name: OVERCURRENT
        descriptions:
          en: "Motor overcurrent"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.catalog.find_global(0), nullptr);
  ASSERT_NE(result.catalog.find("motor_driver_node", 301), nullptr);
}

TEST(MachineCatalog, DuplicateGlobalWarns)
{
  TempYaml yaml(R"(
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
      302:
        global: 1301
        name: OVER_TEMPERATURE
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(has_warning_containing(result.warnings, "duplicate global fault number 1301"));
}

// After a duplicate-global load, find() and find_global() must agree on a
// single owner: the loser's node-local entry keeps existing but its global is
// zeroed (no mapping), rather than both entries claiming the same global.
TEST(MachineCatalog, DuplicateGlobalIndicesStayConsistent)
{
  TempYaml yaml(R"(
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
      302:
        global: 1301
        name: OVER_TEMPERATURE
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * winner = result.catalog.find_global(1301);
  ASSERT_NE(winner, nullptr);
  EXPECT_EQ(winner->entry.name, "OVER_TEMPERATURE");  // 302 loaded last — wins

  const auto * loser = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(loser, nullptr);
  EXPECT_EQ(loser->global, 0) << "loser must not still claim the winner's global";

  // The round trip: looking up the winner's own (node, local) key must return
  // the same global find_global() reported for it.
  const auto * winner_by_local = result.catalog.find(winner->node_name, winner->entry.code);
  ASSERT_NE(winner_by_local, nullptr);
  EXPECT_EQ(winner_by_local->global, 1301);
}

// The same round-trip guarantee applies when two *reserved* faults collide:
// reserved() for the loser's name must no longer report the winner's global.
TEST(MachineCatalog, DuplicateGlobalAcrossReservedStaysConsistent)
{
  TempYaml yaml(R"(
reserved:
  heartbeat_timeout:
    global: 1
    en: "Heartbeat lost"
  fanout_failure:
    global: 1
    en: "Fan-out failed"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.catalog.reserved("fanout_failure"), 1);  // loaded last — wins
  EXPECT_EQ(result.catalog.reserved("heartbeat_timeout"), 0)
    << "loser must not still claim the winner's global";

  const auto * winner = result.catalog.find_global(1);
  ASSERT_NE(winner, nullptr);
  EXPECT_EQ(winner->entry.name, "fanout_failure");
}

// resolve_message on a plain entry leaves any incidental text untouched; an
// unknown instance id substitutes the raw token into an instanced template.
TEST(MachineCatalog, ResolveMessagePlainAndUnknownInstance)
{
  TempYaml yaml(kMachineYaml);
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * plain = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(plain, nullptr);
  EXPECT_EQ(
    result.catalog.resolve_message(plain->entry, "en", "cell_north"),
    "Motor overcurrent detected");

  const auto * estop = result.catalog.find("motor_driver_node", 10);
  ASSERT_NE(estop, nullptr);
  EXPECT_EQ(
    result.catalog.resolve_message(estop->entry, "en", "unmapped_bay"),
    "Emergency stop triggered at unmapped_bay");
}

// A file that parses but recognizes neither a 'reserved' nor a 'nodes' entry has
// silently degraded to nothing (e.g. a top-level key typo/format drift) and must
// fail — an empty-but-"ok" catalog would enrich nothing with no visible cause.
TEST(MachineCatalog, ZeroRecognizedEntriesFailsOpen)
{
  TempYaml yaml("languages: [en]\n");
  auto result = load_machine_catalog_from_yaml(yaml.path());

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

// A typo'd/unrecognized action name silently becoming NONE would drop a
// safety-relevant classification with no trace — the loader must at least warn.
TEST(MachineCatalog, UnrecognizedActionWarns)
{
  TempYaml yaml(R"(
reserved:
  door_fault:
    global: 1
    action: ABROT
    en: "Door fault"
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
        action: BOGUS
        descriptions:
          en: "Motor overcurrent"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(has_warning_containing(result.warnings, "reserved 'door_fault'"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "unrecognized action 'ABROT'"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "node 'motor_driver_node' code 301"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "unrecognized action 'BOGUS'"));

  // Both still load, degraded to NONE rather than rejected outright (fail-open).
  const auto * reserved = result.catalog.find_global(result.catalog.reserved("door_fault"));
  ASSERT_NE(reserved, nullptr);
  EXPECT_EQ(reserved->entry.action, NodeHealth::NONE);
  const auto * node_entry = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(node_entry, nullptr);
  EXPECT_EQ(node_entry->entry.action, NodeHealth::NONE);
}

// A misspelled field name in a 'reserved' block (e.g. "sevrity" for "severity")
// would otherwise be silently absorbed as a locale description with the real
// severity left blank. Cross-checking against the file's own 'languages:' list
// catches it without inventing a hardcoded locale vocabulary.
TEST(MachineCatalog, ReservedUnconfiguredLocaleWarns)
{
  TempYaml yaml(R"(
languages: [en, nl]
reserved:
  door_fault:
    global: 1
    sevrity: CRITICAL
    en: "Door fault"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(has_warning_containing(result.warnings, "reserved 'door_fault'"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "key 'sevrity'"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "not in the configured 'languages'"));

  // Real severity stayed blank — exactly the silent-corruption risk being flagged.
  const auto * reserved = result.catalog.find_global(result.catalog.reserved("door_fault"));
  ASSERT_NE(reserved, nullptr);
  EXPECT_TRUE(reserved->entry.severity.empty());
}

// Same cross-check for a per-node code's description locale keys.
TEST(MachineCatalog, NodeDescriptionUnconfiguredLocaleWarns)
{
  TempYaml yaml(R"(
languages: [en, nl]
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
        descriptions:
          en: "Motor overcurrent"
          eng: "Duplicate-ish typo locale"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(has_warning_containing(result.warnings, "node 'motor_driver_node' code 301"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "locale 'eng'"));
}

// No 'languages:' declared → nothing to cross-check against, so no false positives.
TEST(MachineCatalog, NoLanguagesDeclaredSkipsLocaleCrossCheck)
{
  TempYaml yaml(R"(
reserved:
  door_fault:
    global: 1
    xx: "Some locale not in any declared list"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(has_warning_containing(result.warnings, "not in the configured"));
}

// ---------------------------------------------------------------------------
// Category taxonomy, schema version, locale-tag normalization
// ---------------------------------------------------------------------------

// A typo'd/unrecognized category (including a case variant of a real one, the
// exact fragmentation this vocabulary exists to catch) must at least warn —
// mirrors UnrecognizedActionWarns.
TEST(MachineCatalog, UnrecognizedCategoryWarns)
{
  TempYaml yaml(R"(
reserved:
  door_fault:
    global: 1
    category: Electrical
    en: "Door fault"
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
        category: elec
        descriptions:
          en: "Motor overcurrent"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(has_warning_containing(result.warnings, "reserved 'door_fault'"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "category 'Electrical'"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "node 'motor_driver_node' code 301"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "category 'elec'"));

  // Still loads, category kept as-authored (fail-open — this is documentation
  // only, it never drives machine behavior).
  const auto * node_entry = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(node_entry, nullptr);
  EXPECT_EQ(node_entry->entry.category, "elec");
}

TEST(MachineCatalog, KnownCategoryDoesNotWarn)
{
  TempYaml yaml(kMachineYaml);  // uses category: electrical throughout
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(has_warning_containing(result.warnings, "outside the known category"));
}

// A schema_version matching this build's compiled-in value is silent.
TEST(MachineCatalog, MatchingSchemaVersionDoesNotWarn)
{
  TempYaml yaml("schema_version: 1\n" + std::string(kMachineYaml).substr(1));
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(has_warning_containing(result.warnings, "schema_version"));
}

// A file with no schema_version at all (e.g. hand-written before the field
// existed) has nothing to compare — must not manufacture a false warning.
TEST(MachineCatalog, MissingSchemaVersionDoesNotWarn)
{
  TempYaml yaml(kMachineYaml);  // kMachineYaml carries no schema_version key
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(has_warning_containing(result.warnings, "schema_version"));
}

// A mismatched schema_version warns but still loads fail-open — a version
// skew doesn't necessarily mean the fields that matter are actually unreadable.
TEST(MachineCatalog, MismatchedSchemaVersionWarnsButStillLoads)
{
  TempYaml yaml("schema_version: 999\n" + std::string(kMachineYaml).substr(1));
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(has_warning_containing(result.warnings, "schema_version 999"));
  EXPECT_NE(result.catalog.find("motor_driver_node", 301), nullptr);
}

// description()/instance_label() match a requested locale that differs from a
// stored one only by case or BCP-47 region subtag, instead of silently
// falling through to the next fallback tier.
TEST(MachineCatalog, DescriptionLocaleMatchIsCaseAndRegionInsensitive)
{
  TempYaml yaml(kMachineYaml);  // descriptions keyed "en"/"nl"
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  const auto * e = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->entry.description("EN"), "Motor overcurrent detected");
  EXPECT_EQ(e->entry.description("en-US"), "Motor overcurrent detected");
  EXPECT_EQ(e->entry.description("NL"), "Motor overstroom gedetecteerd");
}

TEST(MachineCatalog, InstanceLabelMatchIsCaseAndRegionInsensitive)
{
  TempYaml yaml(kMachineYaml);  // instance labels keyed "en"/"nl"
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_EQ(result.catalog.instance_label("cell_north", "EN"), "North Cell");
  EXPECT_EQ(result.catalog.instance_label("cell_north", "nl-BE"), "Noord Cel");
}

// A stored description locale that itself carries a region subtag ("en-US")
// still satisfies a plain "languages: [en]" cross-check and a plain "en"
// lookup — the normalization applies symmetrically to authored keys too.
TEST(MachineCatalog, StoredRegionTaggedLocaleSatisfiesPlainLanguageLookup)
{
  TempYaml yaml(R"(
languages: [en]
nodes:
  motor_driver_node:
    codes:
      301:
        global: 1301
        name: OVERCURRENT
        descriptions:
          en-US: "Motor overcurrent detected"
)");
  auto result = load_machine_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(has_warning_containing(result.warnings, "not in the configured"));

  const auto * e = result.catalog.find("motor_driver_node", 301);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->entry.description("en"), "Motor overcurrent detected");
}
