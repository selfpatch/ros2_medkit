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

// Zero-config native A&C (auto_alarms): pure-function coverage of the
// subscription-source precedence rule and the system-message filter, both
// exercised without a live OPC-UA server (injected fake AlarmEventConfig /
// AutoAlarmsConfig / NodeId values).

#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace ros2_medkit_gateway {

TEST(EffectiveAlarmSourcesTest, AutoDisabledReturnsExplicitSourcesUnchanged) {
  std::vector<AlarmEventConfig> explicit_sources(1);
  explicit_sources[0].source_node_id_str = "ns=2;s=Alarms.A";
  AutoAlarmsConfig auto_cfg;  // enabled = false by default
  auto result = OpcuaPoller::effective_alarm_sources(explicit_sources, auto_cfg);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].source_node_id_str, "ns=2;s=Alarms.A");
}

TEST(EffectiveAlarmSourcesTest, AutoEnabledAppendsSyntheticSourceWhenNotCovered) {
  std::vector<AlarmEventConfig> explicit_sources(1);
  explicit_sources[0].source_node_id_str = "ns=2;s=Alarms.A";
  explicit_sources[0].entity_id = "line";
  explicit_sources[0].fault_code = "PLC_LINE_JAM";

  AutoAlarmsConfig auto_cfg;
  auto_cfg.enabled = true;
  auto_cfg.source_node_id_str = "i=2253";
  auto_cfg.entity_id = "plc_runtime";

  auto result = OpcuaPoller::effective_alarm_sources(explicit_sources, auto_cfg);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].source_node_id_str, "ns=2;s=Alarms.A");
  EXPECT_EQ(result[1].source_node_id_str, "i=2253");
  // The synthetic entry carries no fault_code/mappings, so
  // NodeMap::resolve_alarm() always reports it unmatched - that is exactly
  // what routes every event on this source through auto-derivation.
  EXPECT_TRUE(result[1].fault_code.empty());
  EXPECT_TRUE(result[1].mappings.empty());
  EXPECT_EQ(result[1].entity_id, "plc_runtime");
}

TEST(EffectiveAlarmSourcesTest, NoAutoSourceAddedWhenAutoDisabledEvenIfSourceWouldCollide) {
  std::vector<AlarmEventConfig> explicit_sources(1);
  explicit_sources[0].source_node_id_str = "i=2253";
  AutoAlarmsConfig auto_cfg;
  auto_cfg.enabled = false;
  auto_cfg.source_node_id_str = "i=2253";
  auto result = OpcuaPoller::effective_alarm_sources(explicit_sources, auto_cfg);
  EXPECT_EQ(result.size(), 1u);
}

TEST(EffectiveAlarmSourcesTest, PrecedenceNoDuplicateWhenExplicitSourceAlreadyCoversAutoSource) {
  // An explicit event_alarms entry on the SAME source auto_alarms would
  // subscribe to must not get a second, redundant monitored item - on_event
  // falls through to auto-derivation for whatever that entry's own
  // mappings/fault_code do not match instead (explicit still wins for
  // matched alarms - the precedence rule).
  std::vector<AlarmEventConfig> explicit_sources(1);
  explicit_sources[0].source_node_id_str = "i=2253";
  explicit_sources[0].entity_id = "line";
  explicit_sources[0].fault_code = "PLC_GENERIC_ALARM";

  AutoAlarmsConfig auto_cfg;
  auto_cfg.enabled = true;
  auto_cfg.source_node_id_str = "i=2253";
  auto_cfg.entity_id = "plc_runtime";

  auto result = OpcuaPoller::effective_alarm_sources(explicit_sources, auto_cfg);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].entity_id, "line");
  EXPECT_EQ(result[0].fault_code, "PLC_GENERIC_ALARM");
}

TEST(EffectiveAlarmSourcesTest, MultipleExplicitSourcesPreservedAlongsideSynthetic) {
  std::vector<AlarmEventConfig> explicit_sources(2);
  explicit_sources[0].source_node_id_str = "ns=2;s=Alarms.A";
  explicit_sources[1].source_node_id_str = "ns=2;s=Alarms.B";

  AutoAlarmsConfig auto_cfg;
  auto_cfg.enabled = true;
  auto_cfg.source_node_id_str = "i=2253";

  auto result = OpcuaPoller::effective_alarm_sources(explicit_sources, auto_cfg);
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[2].source_node_id_str, "i=2253");
}

TEST(EffectiveAlarmSourcesTest, EquivalentSpellingOfAutoSourceIsRecognizedAsCovered) {
  // An explicit event_alarms source spelled ``ns=0;i=2253`` targets the same
  // physical node as the auto default ``i=2253``. A raw string compare would
  // miss the overlap and add a second monitored item on one node, so every
  // event would double-fire (one mapped fault + one auto fault). Canonical
  // comparison must recognize them as the same source (no synthetic added).
  std::vector<AlarmEventConfig> explicit_sources(1);
  explicit_sources[0].source_node_id_str = "ns=0;i=2253";
  explicit_sources[0].entity_id = "line";
  explicit_sources[0].fault_code = "PLC_GENERIC_ALARM";

  AutoAlarmsConfig auto_cfg;
  auto_cfg.enabled = true;
  auto_cfg.source_node_id_str = "i=2253";
  auto_cfg.entity_id = "plc_runtime";

  auto result = OpcuaPoller::effective_alarm_sources(explicit_sources, auto_cfg);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].source_node_id_str, "ns=0;i=2253");
}

TEST(NodeIdsEquivalentTest, IdenticalStringsAreEquivalent) {
  EXPECT_TRUE(OpcuaPoller::node_ids_equivalent("ns=2;s=Tank.Pressure", "ns=2;s=Tank.Pressure"));
}

TEST(NodeIdsEquivalentTest, DefaultAndExplicitNamespaceZeroNumericAreEquivalent) {
  // The default numeric Server object and its explicit ns=0 spelling denote
  // one physical node.
  EXPECT_TRUE(OpcuaPoller::node_ids_equivalent("i=2253", "ns=0;i=2253"));
  EXPECT_TRUE(OpcuaPoller::node_ids_equivalent("ns=0;i=2253", "i=2253"));
}

TEST(NodeIdsEquivalentTest, DifferentNumericIdsAreNotEquivalent) {
  EXPECT_FALSE(OpcuaPoller::node_ids_equivalent("i=2253", "i=2254"));
}

TEST(NodeIdsEquivalentTest, DifferentNamespacesAreNotEquivalent) {
  EXPECT_FALSE(OpcuaPoller::node_ids_equivalent("ns=1;s=Pump", "ns=2;s=Pump"));
}

TEST(NodeIdsEquivalentTest, UnparseableSpellingsFallBackToRawEquality) {
  // Two distinct unparseable strings have no canonical form and must stay
  // distinct; an identical unparseable string still matches itself.
  EXPECT_FALSE(OpcuaPoller::node_ids_equivalent("not-a-node-id", "also-bad"));
  EXPECT_TRUE(OpcuaPoller::node_ids_equivalent("not-a-node-id", "not-a-node-id"));
}

TEST(IsConditionEventTest, NullConditionIdIsRejected) {
  // Part 9 §5.5.2.13: a non-condition event (e.g. a Siemens Server-object
  // system message such as "CPU not in RUN") resolves the ConditionId SAO
  // to NodeId.Null - the system-message filter for auto_alarms.
  EXPECT_FALSE(OpcuaPoller::is_condition_event(opcua::NodeId()));
}

TEST(IsConditionEventTest, RealConditionIdIsAccepted) {
  EXPECT_TRUE(OpcuaPoller::is_condition_event(opcua::NodeId(3, static_cast<uint32_t>(1845))));
}

}  // namespace ros2_medkit_gateway
