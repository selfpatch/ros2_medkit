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

#include <fstream>
#include <mutex>
#include <string>
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

TEST(AlarmRoutingTest, TheEventPathsCopyDoesNotFollowARenameItNeverAskedFor) {
  // on_event runs on the event pump thread; the config-less rename runs on the
  // poll thread and clears then reassigns auto_alarms.entity_id under a lock the
  // event path neither holds nor can take. Reading the node map from on_event
  // is therefore a data race on a std::string, and a ConditionRefresh burst on
  // the first adopted session lands on exactly that window. The poller keeps its
  // own copy, and only the thread that renames replaces it.
  OpcuaClient client;  // never connected: the routing copy is pure bookkeeping
  NodeMap node_map;
  node_map.set_component_identity("opcua-127_0_0_1", "opcua-127_0_0_1");
  node_map.mutable_auto_alarms().enabled = true;
  ASSERT_TRUE(node_map.finalize_auto_alarms_overlay());
  const std::string placeholder_entity = node_map.auto_alarms().entity_id;
  ASSERT_EQ(placeholder_entity, "opcua-127_0_0_1_alarms");

  OpcuaPoller poller(client, node_map);
  poller.refresh_alarm_routing();
  const auto subscribed_with = poller.alarm_routing();
  ASSERT_TRUE(subscribed_with);
  EXPECT_EQ(subscribed_with->auto_alarms.entity_id, placeholder_entity);

  // The rename the poll thread performs once the adopted device names itself.
  node_map.mutable_auto_alarms().entity_id.clear();
  node_map.set_component_identity("siemens_ag_cpu_1505sp_f", "Siemens AG CPU 1505SP F");
  ASSERT_TRUE(node_map.finalize_auto_alarms_overlay());
  ASSERT_EQ(node_map.auto_alarms().entity_id, "siemens_ag_cpu_1505sp_f_alarms");

  // The event path still reads what it was handed. Both the snapshot it already
  // holds and a fresh read of the accessor: the copy is what the poller owns,
  // not a view onto the map.
  EXPECT_EQ(subscribed_with->auto_alarms.entity_id, placeholder_entity);
  EXPECT_EQ(poller.alarm_routing()->auto_alarms.entity_id, placeholder_entity)
      << "the event path's copy tracked a rename it never asked for - it is reading the node map";

  // ... until the thread that renamed replaces it, which is what
  // setup_event_subscriptions does at subscribe time.
  poller.refresh_alarm_routing();
  EXPECT_EQ(poller.alarm_routing()->auto_alarms.entity_id, "siemens_ag_cpu_1505sp_f_alarms");
  EXPECT_EQ(subscribed_with->auto_alarms.entity_id, placeholder_entity)
      << "a refresh rewrote the snapshot a callback was already holding";
}

TEST(AlarmRoutingTest, ARepinMovesConditionsAlreadyPinnedToTheNewEntity) {
  // apply_condition_state pins a fault's entity at the first sighting of its
  // ConditionId, so a config-less rename has to reach the conditions the poller
  // already holds as well as the routing new ones are derived with. Without the
  // re-pin, every later report and clear for those ConditionIds is filed under
  // an entity the rename dropped.
  OpcuaClient client;
  NodeMap node_map;
  OpcuaPoller poller(client, node_map);

  std::mutex deliveries_mutex;
  std::vector<AlarmEventDelivery> deliveries;
  poller.set_event_alarm_callback([&deliveries_mutex, &deliveries](const AlarmEventDelivery & delivery) {
    std::lock_guard<std::mutex> lock(deliveries_mutex);
    deliveries.push_back(delivery);
  });

  AlarmEventConfig cfg;
  cfg.source_node_id_str = "i=2253";
  cfg.entity_id = "opcua-127_0_0_1_alarms";
  cfg.fault_code = "PLC_OVERPRESSURE";
  const opcua::NodeId condition(3, static_cast<uint32_t>(1845));

  AlarmEventInput raise;
  raise.enabled_state = true;
  raise.active_state = true;
  raise.active_state_present = true;
  poller.apply_condition_state_for_test(cfg, condition, raise, /*severity=*/750, "Overpressure",
                                        /*event_id=*/nullptr, /*require_confirm_for_clear=*/false);
  {
    std::lock_guard<std::mutex> lock(deliveries_mutex);
    ASSERT_EQ(deliveries.size(), 1u) << "the raise was not delivered, so the pin cannot be observed";
    EXPECT_EQ(deliveries.front().entity_id, "opcua-127_0_0_1_alarms");
    deliveries.clear();
  }

  poller.repin_auto_alarms_entity("opcua-127_0_0_1_alarms", "siemens_ag_cpu_1505sp_f_alarms");

  // The same condition going inactive. Its entity is the renamed one, so the
  // clear reaches the entity the raise will have been moved to.
  AlarmEventInput heal = raise;
  heal.active_state = false;
  heal.acked_state = true;
  poller.apply_condition_state_for_test(cfg, condition, heal, /*severity=*/750, "Overpressure",
                                        /*event_id=*/nullptr, /*require_confirm_for_clear=*/false);

  std::lock_guard<std::mutex> lock(deliveries_mutex);
  ASSERT_EQ(deliveries.size(), 1u);
  EXPECT_EQ(deliveries.front().entity_id, "siemens_ag_cpu_1505sp_f_alarms")
      << "a condition pinned before the rename kept the entity the rename dropped";
  EXPECT_EQ(deliveries.front().fault_code, "PLC_OVERPRESSURE");
}

// Part 9 §5.5.7 brackets the ConditionRefresh replay on every event
// MonitoredItem of the subscription, and each item carries only its own
// notifier's conditions. With one item per machine object, the second item's
// RefreshEnd used to reconcile every tracked condition against its own burst
// and clear the first machine's live alarm.
TEST(ConditionRefreshTest, EachSourceReconcilesOnlyItsOwnConditions) {
  const std::string path = "/tmp/test_opcua_poller_refresh_per_source.yaml";
  {
    std::ofstream f(path);
    f << R"(
area_id: line
component_id: machines
event_alarms:
  - alarm_source: "ns=1;s=MachineA"
    entity_id: machine_a
    fault_code: A_ALARM
  - alarm_source: "ns=1;s=MachineB"
    entity_id: machine_b
    fault_code: B_ALARM
)";
  }
  OpcuaClient client;
  NodeMap node_map;
  ASSERT_TRUE(node_map.load(path));
  ASSERT_EQ(node_map.event_alarms().size(), 2u);
  OpcuaPoller poller(client, node_map);
  poller.refresh_alarm_routing();
  std::mutex deliveries_mutex;
  std::vector<AlarmEventDelivery> deliveries;
  poller.set_event_alarm_callback([&](const AlarmEventDelivery & delivery) {
    std::lock_guard<std::mutex> lock(deliveries_mutex);
    deliveries.push_back(delivery);
  });
  const AlarmEventConfig a = node_map.event_alarms()[0];
  const AlarmEventConfig b = node_map.event_alarms()[1];
  const opcua::NodeId cond_a(1, std::string("MachineA.Heating"));
  const opcua::NodeId cond_b(1, std::string("MachineB.Mould"));

  // EventId, Severity, Message, Enabled, Active, Acked, Confirmed, Shelving,
  // BranchId, ConditionName, SourceName.
  auto active = [](const char * name) {
    std::vector<opcua::Variant> v(11);
    v[1] = opcua::Variant::fromScalar(static_cast<uint16_t>(700));
    v[2] = opcua::Variant::fromScalar(opcua::LocalizedText("en", name));
    v[3] = opcua::Variant::fromScalar(true);
    v[4] = opcua::Variant::fromScalar(true);
    v[5] = opcua::Variant::fromScalar(false);
    v[6] = opcua::Variant::fromScalar(false);
    v[9] = opcua::Variant::fromScalar(opcua::String(name));
    return v;
  };
  const std::vector<opcua::Variant> bracket(11);
  const opcua::NodeId alarm_type(0, static_cast<uint32_t>(2915));
  const opcua::NodeId start(0, static_cast<uint32_t>(2787));
  const opcua::NodeId end(0, static_cast<uint32_t>(2788));
  auto codes = [&](AlarmAction action) {
    std::lock_guard<std::mutex> lock(deliveries_mutex);
    std::vector<std::string> out;
    for (const auto & d : deliveries) {
      if (d.action == action) {
        out.push_back(d.fault_code);
      }
    }
    return out;
  };

  poller.on_event_for_test(a, active("Heating"), alarm_type, cond_a);
  poller.on_event_for_test(b, active("Mould"), alarm_type, cond_b);
  ASSERT_EQ(codes(AlarmAction::ReportConfirmed).size(), 2u);

  // A refresh that replays both, bracketed on both items.
  poller.on_event_for_test(a, bracket, start, opcua::NodeId());
  poller.on_event_for_test(b, bracket, start, opcua::NodeId());
  poller.on_event_for_test(a, active("Heating"), alarm_type, cond_a);
  poller.on_event_for_test(b, active("Mould"), alarm_type, cond_b);
  poller.on_event_for_test(a, bracket, end, opcua::NodeId());
  poller.on_event_for_test(b, bracket, end, opcua::NodeId());
  EXPECT_TRUE(codes(AlarmAction::ClearFault).empty()) << "a refresh that replayed both alarms cleared one";

  // A refresh of A alone, without A's alarm: only A's alarm cleared offline.
  poller.on_event_for_test(a, bracket, start, opcua::NodeId());
  poller.on_event_for_test(a, bracket, end, opcua::NodeId());
  EXPECT_EQ(codes(AlarmAction::ClearFault), std::vector<std::string>{"A_ALARM"});

  // An End with no Start of its own reconciles nothing.
  poller.on_event_for_test(b, bracket, end, opcua::NodeId());
  EXPECT_EQ(codes(AlarmAction::ClearFault), std::vector<std::string>{"A_ALARM"});
}

// An event field of the right type but an array (a vendor associated value,
// a server filling Message or Severity with an array) used to throw
// BadVariantAccess out of on_event, and the notification was dropped. The
// event is handled with that field left empty.
TEST(EventFieldsTest, AnArrayFieldDoesNotDropTheEvent) {
  const std::string path = "/tmp/test_opcua_poller_array_fields.yaml";
  {
    std::ofstream f(path);
    f << R"(
area_id: line
component_id: machines
event_alarms:
  - alarm_source: "ns=1;s=Machine"
    entity_id: machine
    fault_code: MACHINE_ALARM
    associated_values: ["PartIds"]
)";
  }
  OpcuaClient client;
  NodeMap node_map;
  ASSERT_TRUE(node_map.load(path));
  OpcuaPoller poller(client, node_map);
  poller.refresh_alarm_routing();
  std::vector<AlarmEventDelivery> deliveries;
  poller.set_event_alarm_callback([&](const AlarmEventDelivery & delivery) {
    deliveries.push_back(delivery);
  });

  std::vector<opcua::Variant> v(12);
  const std::vector<uint16_t> severities{700, 800};
  const std::vector<opcua::LocalizedText> messages{opcua::LocalizedText("en", "a"), opcua::LocalizedText("en", "b")};
  const std::vector<opcua::String> parts{opcua::String("P1"), opcua::String("P2")};
  v[1] = opcua::Variant::fromArray(severities);
  v[2] = opcua::Variant::fromArray(messages);
  v[3] = opcua::Variant::fromScalar(true);
  v[4] = opcua::Variant::fromScalar(true);
  v[5] = opcua::Variant::fromScalar(false);
  v[6] = opcua::Variant::fromScalar(false);
  v[9] = opcua::Variant::fromArray(parts);   // ConditionName as an array
  v[11] = opcua::Variant::fromArray(parts);  // the associated value
  ASSERT_NO_THROW(poller.on_event_for_test(node_map.event_alarms()[0], v, opcua::NodeId(0, static_cast<uint32_t>(2915)),
                                           opcua::NodeId(1, std::string("Machine.Alarm"))));
  ASSERT_EQ(deliveries.size(), 1u);
  EXPECT_EQ(deliveries[0].fault_code, "MACHINE_ALARM");
  EXPECT_EQ(deliveries[0].action, AlarmAction::ReportConfirmed);
}

TEST(IsConditionEventTest, NullConditionIdIsRejected) {
  // Part 9 §5.5.2.13: a non-condition event (e.g. a Siemens Server-object
  // system message such as "CPU not in RUN") resolves the ConditionId SAO
  // to NodeId.Null. on_event's guard drops such events for every alarm
  // source - explicit event_alarms and auto_alarms alike.
  EXPECT_FALSE(OpcuaPoller::is_condition_event(opcua::NodeId()));
}

TEST(IsConditionEventTest, RealConditionIdIsAccepted) {
  EXPECT_TRUE(OpcuaPoller::is_condition_event(opcua::NodeId(3, static_cast<uint32_t>(1845))));
}

}  // namespace ros2_medkit_gateway
