// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/node_map.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <string>

namespace ros2_medkit_gateway {

class NodeMapTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Write a test YAML file
    yaml_path_ = "/tmp/test_node_map.yaml";
    std::ofstream f(yaml_path_);
    f << R"(
area_id: plc_systems
area_name: PLC Systems
component_id: openplc_runtime
component_name: OpenPLC Runtime
auto_browse: false

nodes:
  - node_id: "ns=1;s=TankLevel"
    entity_id: tank_process
    data_name: tank_level
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: false
    alarm:
      fault_code: PLC_LOW_LEVEL
      severity: WARNING
      message: Tank level below minimum
      threshold: 100.0
      above_threshold: false

  - node_id: "ns=1;s=TankTemperature"
    entity_id: tank_process
    data_name: tank_temperature
    display_name: Tank Temperature
    unit: C
    data_type: float
    writable: false
    alarm:
      fault_code: PLC_HIGH_TEMP
      severity: ERROR
      message: Tank temperature above maximum
      threshold: 80.0
      above_threshold: true

  - node_id: "ns=1;s=PumpSpeed"
    entity_id: fill_pump
    data_name: pump_speed
    display_name: Pump Speed
    unit: "%"
    data_type: float
    writable: true

  - node_id: "ns=1;s=PumpRunning"
    entity_id: fill_pump
    data_name: pump_running
    display_name: Pump Running
    data_type: bool
    writable: false
)";
    f.close();
  }

  std::string yaml_path_;
};

TEST_F(NodeMapTest, LoadsYaml) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));
  EXPECT_EQ(map.entries().size(), 4u);
  EXPECT_EQ(map.area_id(), "plc_systems");
  EXPECT_EQ(map.component_id(), "openplc_runtime");
  EXPECT_FALSE(map.auto_browse());
}

TEST_F(NodeMapTest, EntriesForEntity) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto tank = map.entries_for_entity("tank_process");
  EXPECT_EQ(tank.size(), 2u);

  auto pump = map.entries_for_entity("fill_pump");
  EXPECT_EQ(pump.size(), 2u);

  auto empty = map.entries_for_entity("nonexistent");
  EXPECT_TRUE(empty.empty());
}

TEST_F(NodeMapTest, WritableEntries) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto writable = map.writable_entries_for_entity("fill_pump");
  EXPECT_EQ(writable.size(), 1u);
  EXPECT_EQ(writable[0]->data_name, "pump_speed");

  auto tank_writable = map.writable_entries_for_entity("tank_process");
  EXPECT_TRUE(tank_writable.empty());
}

TEST_F(NodeMapTest, FindByDataName) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto * entry = map.find_by_data_name("tank_process", "tank_level");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->unit, "mm");
  EXPECT_FALSE(entry->writable);

  auto * missing = map.find_by_data_name("tank_process", "nonexistent");
  EXPECT_EQ(missing, nullptr);
}

TEST_F(NodeMapTest, FindByNodeId) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto * entry = map.find_by_node_id("ns=1;s=PumpSpeed");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->entity_id, "fill_pump");
  EXPECT_TRUE(entry->writable);
}

TEST_F(NodeMapTest, AlarmEntries) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto alarms = map.alarm_entries();
  EXPECT_EQ(alarms.size(), 2u);

  // Check alarm config
  bool found_low_level = false;
  bool found_high_temp = false;
  for (const auto * e : alarms) {
    if (e->alarm->fault_code == "PLC_LOW_LEVEL") {
      EXPECT_EQ(e->alarm->severity, "WARNING");
      EXPECT_DOUBLE_EQ(e->alarm->threshold, 100.0);
      EXPECT_FALSE(e->alarm->above_threshold);
      found_low_level = true;
    }
    if (e->alarm->fault_code == "PLC_HIGH_TEMP") {
      EXPECT_EQ(e->alarm->severity, "ERROR");
      EXPECT_DOUBLE_EQ(e->alarm->threshold, 80.0);
      EXPECT_TRUE(e->alarm->above_threshold);
      found_high_temp = true;
    }
  }
  EXPECT_TRUE(found_low_level);
  EXPECT_TRUE(found_high_temp);
}

TEST_F(NodeMapTest, EntityDefs) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto & defs = map.entity_defs();
  EXPECT_EQ(defs.size(), 2u);

  // Find tank_process def
  const PlcEntityDef * tank_def = nullptr;
  const PlcEntityDef * pump_def = nullptr;
  for (const auto & d : defs) {
    if (d.id == "tank_process") {
      tank_def = &d;
    }
    if (d.id == "fill_pump") {
      pump_def = &d;
    }
  }

  ASSERT_NE(tank_def, nullptr);
  EXPECT_EQ(tank_def->data_names.size(), 2u);
  EXPECT_TRUE(tank_def->writable_names.empty());
  EXPECT_TRUE(tank_def->has_faults);

  ASSERT_NE(pump_def, nullptr);
  EXPECT_EQ(pump_def->data_names.size(), 2u);
  EXPECT_EQ(pump_def->writable_names.size(), 1u);
  EXPECT_FALSE(pump_def->has_faults);
}

TEST_F(NodeMapTest, LoadNonexistentFile) {
  NodeMap map;
  EXPECT_FALSE(map.load("/nonexistent/path.yaml"));
}

// -- ros2_topic tests --

TEST_F(NodeMapTest, Ros2TopicAutoGenerated) {
  NodeMap map;
  ASSERT_TRUE(map.load(yaml_path_));

  auto * entry = map.find_by_data_name("tank_process", "tank_level");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->ros2_topic, "/plc/tank_process/tank_level");

  auto * pump = map.find_by_data_name("fill_pump", "pump_speed");
  ASSERT_NE(pump, nullptr);
  EXPECT_EQ(pump->ros2_topic, "/plc/fill_pump/pump_speed");
}

TEST_F(NodeMapTest, Ros2TopicExplicitOverride) {
  std::string path = "/tmp/test_node_map_override.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Val"
    entity_id: ent
    data_name: val
    ros2_topic: /custom/my_topic
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  EXPECT_EQ(map.entries()[0].ros2_topic, "/custom/my_topic");
}

TEST_F(NodeMapTest, Ros2TopicSanitizesSpecialChars) {
  std::string path = "/tmp/test_node_map_sanitize.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=V1"
    entity_id: tank-process
    data_name: tank.level
  - node_id: "ns=1;s=V2"
    entity_id: "my entity"
    data_name: "data name"
  - node_id: "ns=1;s=V3"
    entity_id: ok_name
    data_name: 123sensor
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));

  // Hyphens and dots replaced with underscores
  EXPECT_EQ(map.entries()[0].ros2_topic, "/plc/tank_process/tank_level");

  // Spaces replaced with underscores
  EXPECT_EQ(map.entries()[1].ros2_topic, "/plc/my_entity/data_name");

  // Leading digit gets underscore prefix
  EXPECT_EQ(map.entries()[2].ros2_topic, "/plc/ok_name/_123sensor");
}

TEST_F(NodeMapTest, Ros2TopicInvalidCustomFallsBackToAuto) {
  std::string path = "/tmp/test_node_map_invalid_topic.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=V1"
    entity_id: ent
    data_name: val
    ros2_topic: "not a valid topic!"
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Invalid custom topic should fall back to auto-generated
  EXPECT_EQ(map.entries()[0].ros2_topic, "/plc/ent/val");
}

TEST_F(NodeMapTest, Ros2TopicEmptyEntityId) {
  std::string path = "/tmp/test_node_map_empty.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=V1"
    entity_id: ""
    data_name: val
)";
  f.close();

  NodeMap map;
  ASSERT_TRUE(map.load(path));
  // Empty entity_id should become _unnamed
  EXPECT_EQ(map.entries()[0].ros2_topic, "/plc/_unnamed/val");
}

// -- parse_node_id tests --
//
// OPC-UA Node IDs per OPC 10000-6 section 5.3.1.10. The parser supports all
// four identifier types (i, s, g, b) plus the optional namespace prefix.

TEST(ParseNodeIdTest, NumericWithNamespace) {
  const auto nid = NodeMap::parse_node_id("ns=2;i=42");
  EXPECT_EQ(nid.getNamespaceIndex(), 2);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Numeric);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 42u);
}

TEST(ParseNodeIdTest, NumericDefaultNamespace) {
  const auto nid = NodeMap::parse_node_id("i=2253");  // Server_ServerStatus
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Numeric);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 2253u);
}

TEST(ParseNodeIdTest, StringSiemensStyle) {
  // Siemens TIA Portal uses quoted DB addresses with embedded dots.
  const auto nid = NodeMap::parse_node_id("ns=3;s=\"Tank_DB\".\"level_mm\"");
  EXPECT_EQ(nid.getNamespaceIndex(), 3);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::String);
  EXPECT_EQ(nid.getIdentifierAs<opcua::String>(), std::string_view{"\"Tank_DB\".\"level_mm\""});
}

TEST(ParseNodeIdTest, StringBeckhoffStyle) {
  const auto nid = NodeMap::parse_node_id("ns=4;s=MAIN.Tank.level");
  EXPECT_EQ(nid.getNamespaceIndex(), 4);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::String);
  EXPECT_EQ(nid.getIdentifierAs<opcua::String>(), std::string_view{"MAIN.Tank.level"});
}

TEST(ParseNodeIdTest, StringWithEmbeddedSemicolon) {
  // Pathological but legal: string identifier can contain semicolons.
  // The parser takes the rest of the string after `;s=` verbatim.
  const auto nid = NodeMap::parse_node_id("ns=2;s=value;with;semicolons");
  EXPECT_EQ(nid.getNamespaceIndex(), 2);
  EXPECT_EQ(nid.getIdentifierAs<opcua::String>(), std::string_view{"value;with;semicolons"});
}

TEST(ParseNodeIdTest, GuidWithNamespace) {
  const auto nid = NodeMap::parse_node_id("ns=3;g=09087e75-8e5e-499e-954f-f2a9603db28a");
  EXPECT_EQ(nid.getNamespaceIndex(), 3);
  ASSERT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Guid);
}

TEST(ParseNodeIdTest, GuidWithBracesMicrosoftForm) {
  // Microsoft registry GUID format with curly braces is accepted.
  const auto nid = NodeMap::parse_node_id("ns=3;g={09087e75-8e5e-499e-954f-f2a9603db28a}");
  EXPECT_EQ(nid.getNamespaceIndex(), 3);
  EXPECT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Guid);
}

TEST(ParseNodeIdTest, OpaqueBase64) {
  // Base64 of "helloworld12345!" (16 bytes).
  const auto nid = NodeMap::parse_node_id("ns=2;b=aGVsbG93b3JsZDEyMzQ1IQ==");
  EXPECT_EQ(nid.getNamespaceIndex(), 2);
  EXPECT_EQ(nid.getIdentifierType(), opcua::NodeIdType::ByteString);
}

TEST(ParseNodeIdTest, MalformedMissingType) {
  const auto nid = NodeMap::parse_node_id("ns=2");
  // Should return default-constructed (empty) NodeId on parse failure.
  EXPECT_EQ(nid.getIdentifierType(), opcua::NodeIdType::Numeric);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

TEST(ParseNodeIdTest, MalformedBadNamespace) {
  const auto nid = NodeMap::parse_node_id("ns=abc;i=1");
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

TEST(ParseNodeIdTest, MalformedBadGuid) {
  const auto nid = NodeMap::parse_node_id("ns=3;g=not-a-valid-guid");
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

TEST(ParseNodeIdTest, MalformedUnknownTypeChar) {
  const auto nid = NodeMap::parse_node_id("ns=2;x=42");
  // Unknown identifier type falls through to default.
  EXPECT_EQ(nid.getNamespaceIndex(), 0);
  EXPECT_EQ(nid.getIdentifierAs<uint32_t>(), 0u);
}

}  // namespace ros2_medkit_gateway
