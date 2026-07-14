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
// Tests for the ErrorCatalog core (pure C++, no ROS node required).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "packml_ros/error_catalog.hpp"
#include "packml_msgs/msg/node_health.hpp"

using packml_ros::ErrorCatalog;
using packml_ros::ErrorEntry;
using packml_ros::NodeCatalog;
using packml_ros::CatalogLoadResult;
using packml_ros::load_node_catalog_from_yaml;
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

const char * kMotorYaml = R"(
node_name: motor_driver_node
error_codes:
  OVERCURRENT: 301
  OVER_TEMPERATURE: 302
  ENCODER_DEGRADED: 200
descriptions:
  OVERCURRENT:
    action: ABORT
    severity: CRITICAL
    en: "Motor overcurrent detected"
    nl: "Motor overstroom gedetecteerd"
    de: "Motor Ueberstrom erkannt"
  OVER_TEMPERATURE:
    action: HOLD
    severity: ERROR
    en: "Motor over-temperature"
    nl: "Motor te hoog temperatuur"
  ENCODER_DEGRADED:
    action: WARN
    severity: WARNING
    en: "Encoder signal degraded"
    nl: "Encoder signaal verminderd"
)";

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

// A node catalog exercising the instanced/category fields and the placeholder
// lint: E_STOP is correctly instanced, DOOR is instanced but its text omits
// {instance}, BADPLAIN is plain but its text carries {instance}.
const char * kSafetyYaml = R"(
node_name: safety_node
error_codes:
  E_STOP: 10
  DOOR: 20
  BADPLAIN: 30
descriptions:
  E_STOP:
    action: ABORT
    severity: CRITICAL
    category: safety
    instanced: true
    en: "Emergency stop triggered at {instance}"
  DOOR:
    action: ABORT
    severity: CRITICAL
    instanced: true
    en: "Guard door open"
  BADPLAIN:
    action: WARN
    severity: WARNING
    en: "Something happened at {instance}"
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

TEST(ErrorCatalogAction, ActionToStringRoundTrip)
{
  EXPECT_EQ(packml_ros::action_to_string(NodeHealth::ABORT), "ABORT");
  EXPECT_EQ(packml_ros::action_to_string(NodeHealth::HOLD), "HOLD");
  EXPECT_EQ(packml_ros::action_to_string(NodeHealth::WARN), "WARN");
}

// ---------------------------------------------------------------------------
// YAML loaded on init — all entries parsed
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, LoadsAllEntries)
{
  TempYaml yaml(kMotorYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.catalog.node_name, "motor_driver_node");
  EXPECT_EQ(result.catalog.entries.size(), 3u);
}

TEST(ErrorCatalogLoad, MissingFileFailsGracefully)
{
  auto result = load_node_catalog_from_yaml("/nonexistent/path/does_not_exist.yaml");
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST(ErrorCatalogLoad, MissingNodeNameFails)
{
  TempYaml yaml("error_codes:\n  FOO: 1\n");
  auto result = load_node_catalog_from_yaml(yaml.path());
  EXPECT_FALSE(result.ok);
}

// ---------------------------------------------------------------------------
// find(node, code) returns correct entry
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, FindReturnsCorrectEntry)
{
  TempYaml yaml(kMotorYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);

  const ErrorEntry * e = result.catalog.find(301);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->name, "OVERCURRENT");
  EXPECT_EQ(e->action, NodeHealth::ABORT);
  EXPECT_EQ(e->severity, "CRITICAL");
  EXPECT_EQ(e->description("en"), "Motor overcurrent detected");
  EXPECT_EQ(e->description("nl"), "Motor overstroom gedetecteerd");
}

// ---------------------------------------------------------------------------
// missing locale falls back to "en"
// ---------------------------------------------------------------------------

TEST(ErrorCatalogDescription, FallsBackToEnglish)
{
  TempYaml yaml(kMotorYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);

  const ErrorEntry * e = result.catalog.find(302);  // has en, nl but no fr
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->description("fr"), "Motor over-temperature");  // fr → en
}

// ---------------------------------------------------------------------------
// missing "en" falls back to first entry
// ---------------------------------------------------------------------------

TEST(ErrorCatalogDescription, FallsBackToFirstWhenNoEnglish)
{
  const char * yaml_str = R"(
node_name: test_node
error_codes:
  ONLY_NL: 42
descriptions:
  ONLY_NL:
    action: HOLD
    nl: "Alleen Nederlands"
)";
  TempYaml yaml(yaml_str);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);

  const ErrorEntry * e = result.catalog.find(42);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->description("fr"), "Alleen Nederlands");  // no en → first available
}

TEST(ErrorCatalogDescription, EmptyDescriptionsReturnsEmptyString)
{
  ErrorEntry e;
  e.code = 1;
  EXPECT_EQ(e.description("en"), "");
}

// ---------------------------------------------------------------------------
// unknown code returns nullptr
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, UnknownCodeReturnsNullptr)
{
  TempYaml yaml(kMotorYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.catalog.find(999), nullptr);
}

// ---------------------------------------------------------------------------
// code 0 rejected at load time (reserved)
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, CodeZeroRejected)
{
  const char * yaml_str = R"(
node_name: test_node
error_codes:
  RESERVED: 0
  VALID: 5
descriptions:
  RESERVED:
    action: ABORT
    en: "should be rejected"
  VALID:
    action: HOLD
    en: "ok"
)";
  TempYaml yaml(yaml_str);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);

  EXPECT_EQ(result.catalog.find(0), nullptr);
  EXPECT_NE(result.catalog.find(5), nullptr);
  EXPECT_TRUE(has_warning_containing(result.warnings, "reserved value 0"));
}

// ---------------------------------------------------------------------------
// duplicate code value → warning
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, DuplicateCodeValueWarns)
{
  const char * yaml_str = R"(
node_name: test_node
error_codes:
  FIRST: 10
  SECOND: 10
descriptions:
  FIRST:
    action: HOLD
    en: "first"
  SECOND:
    action: HOLD
    en: "second"
)";
  TempYaml yaml(yaml_str);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(has_warning_containing(result.warnings, "duplicate error code"));
}

// ---------------------------------------------------------------------------
// two nodes with same code value — independent (no collision)
// ---------------------------------------------------------------------------

TEST(ErrorCatalogAggregate, TwoNodesSameCodeIndependent)
{
  const char * node_a = R"(
node_name: node_a
error_codes:
  FAULT: 100
descriptions:
  FAULT:
    action: HOLD
    en: "Node A fault"
)";
  const char * node_b = R"(
node_name: node_b
error_codes:
  FAULT: 100
descriptions:
  FAULT:
    action: ABORT
    en: "Node B fault"
)";
  TempYaml ya(node_a);
  TempYaml yb(node_b);
  auto ra = load_node_catalog_from_yaml(ya.path());
  auto rb = load_node_catalog_from_yaml(yb.path());
  ASSERT_TRUE(ra.ok);
  ASSERT_TRUE(rb.ok);

  ErrorCatalog cat;
  cat.load_node_catalog(ra.catalog);
  cat.load_node_catalog(rb.catalog);

  EXPECT_EQ(cat.size(), 2u);
  ASSERT_NE(cat.find("node_a", 100), nullptr);
  ASSERT_NE(cat.find("node_b", 100), nullptr);
  EXPECT_EQ(cat.find("node_a", 100)->description("en"), "Node A fault");
  EXPECT_EQ(cat.find("node_b", 100)->description("en"), "Node B fault");
  EXPECT_EQ(cat.find("node_a", 100)->action, NodeHealth::HOLD);
  EXPECT_EQ(cat.find("node_b", 100)->action, NodeHealth::ABORT);
}

TEST(ErrorCatalogAggregate, UnknownNodeReturnsNullptr)
{
  ErrorCatalog cat;
  EXPECT_EQ(cat.find("ghost_node", 1), nullptr);
}

// ---------------------------------------------------------------------------
// code in error_codes missing from descriptions → warning
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, MissingDescriptionWarns)
{
  const char * yaml_str = R"(
node_name: test_node
error_codes:
  DOCUMENTED: 1
  UNDOCUMENTED: 2
descriptions:
  DOCUMENTED:
    action: HOLD
    en: "has a description"
)";
  TempYaml yaml(yaml_str);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(has_warning_containing(result.warnings, "UNDOCUMENTED"));

  // The documented code is still queryable.
  EXPECT_NE(result.catalog.find(1), nullptr);
  // The undocumented code has no description block, so it is not in entries.
  EXPECT_EQ(result.catalog.find(2), nullptr);
}

TEST(ErrorCatalogLoad, OrphanDescriptionWarns)
{
  const char * yaml_str = R"(
node_name: test_node
error_codes:
  REAL: 1
descriptions:
  REAL:
    action: HOLD
    en: "real"
  ORPHAN:
    action: ABORT
    en: "no matching code"
)";
  TempYaml yaml(yaml_str);
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(has_warning_containing(result.warnings, "ORPHAN"));
  EXPECT_EQ(result.catalog.entries.size(), 1u);
}

// ---------------------------------------------------------------------------
// Entries are sorted by code (stable export ordering)
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, EntriesSortedByCode)
{
  TempYaml yaml(kMotorYaml);  // codes 301, 302, 200
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok);
  ASSERT_EQ(result.catalog.entries.size(), 3u);
  EXPECT_EQ(result.catalog.entries[0].code, 200);
  EXPECT_EQ(result.catalog.entries[1].code, 301);
  EXPECT_EQ(result.catalog.entries[2].code, 302);
}

// ---------------------------------------------------------------------------
// load_node_catalog replaces an existing node's catalog (last wins)
// ---------------------------------------------------------------------------

TEST(ErrorCatalogAggregate, ReloadReplacesNode)
{
  ErrorCatalog cat;

  NodeCatalog v1;
  v1.node_name = "node";
  v1.entries.push_back(ErrorEntry{1, "OLD", NodeHealth::HOLD, "ERROR", {{"en", "old"}}, "", false});
  cat.load_node_catalog(v1);

  NodeCatalog v2;
  v2.node_name = "node";
  v2.entries.push_back(ErrorEntry{2, "NEW", NodeHealth::ABORT, "CRITICAL", {{"en", "new"}}, "", false});
  cat.load_node_catalog(v2);

  EXPECT_EQ(cat.size(), 1u);
  EXPECT_EQ(cat.find("node", 1), nullptr);     // old entry gone
  ASSERT_NE(cat.find("node", 2), nullptr);     // new entry present
  EXPECT_EQ(cat.find("node", 2)->description("en"), "new");
}

// ---------------------------------------------------------------------------
// Node loader — instanced / category fields and the placeholder lint
// ---------------------------------------------------------------------------

TEST(ErrorCatalogLoad, CategoryParsed)
{
  TempYaml yaml(kSafetyYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());

  ASSERT_TRUE(result.ok) << result.error;
  const auto * e = result.catalog.find(10);
  ASSERT_NE(e, nullptr);
  EXPECT_EQ(e->category, "safety");
}

TEST(ErrorCatalogLoad, InstancedFlagParsed)
{
  TempYaml yaml(kSafetyYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());

  ASSERT_TRUE(result.ok) << result.error;
  const auto * e = result.catalog.find(10);
  ASSERT_NE(e, nullptr);
  EXPECT_TRUE(e->instanced);
}

TEST(ErrorCatalogLoad, InstancedTemplateMissingPlaceholderWarns)
{
  TempYaml yaml(kSafetyYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());

  ASSERT_TRUE(result.ok) << result.error;
  // DOOR is instanced but its description has no {instance}.
  EXPECT_TRUE(has_warning_containing(result.warnings, "DOOR"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "no \"{instance}\" placeholder"));
}

TEST(ErrorCatalogLoad, PlainEntryWithPlaceholderWarns)
{
  TempYaml yaml(kSafetyYaml);
  auto result = load_node_catalog_from_yaml(yaml.path());

  ASSERT_TRUE(result.ok) << result.error;
  // BADPLAIN is not instanced but its description contains {instance}.
  EXPECT_TRUE(has_warning_containing(result.warnings, "BADPLAIN"));
  EXPECT_TRUE(has_warning_containing(result.warnings, "non-instanced"));
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

  // Blank instance_id leaves the template unfilled.
  EXPECT_EQ(
    result.catalog.resolve_message(e->entry, "en", ""),
    "Emergency stop triggered at {instance}");
}

TEST(MachineCatalog, MalformedMachineFileFailsGracefully)
{
  TempYaml yaml("nodes: { unterminated flow mapping");
  auto result = load_machine_catalog_from_yaml(yaml.path());

  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
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

// The instanced placeholder lint runs per-locale: only the offending locale warns.
TEST(ErrorCatalogLoad, InstancedMixedLocaleWarnsOnlyMissing)
{
  TempYaml yaml(R"(
node_name: safety_node
error_codes:
  E_STOP: 10
descriptions:
  E_STOP:
    severity: CRITICAL
    instanced: true
    en: "Emergency stop at {instance}"
    nl: "Noodstop"
)");
  auto result = load_node_catalog_from_yaml(yaml.path());
  ASSERT_TRUE(result.ok) << result.error;

  EXPECT_TRUE(has_warning_containing(result.warnings, "[nl] has no \"{instance}\""));
  EXPECT_FALSE(has_warning_containing(result.warnings, "[en] has no \"{instance}\""));
}
