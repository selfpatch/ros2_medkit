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

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
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

TEST(DetectionOrderTest, ACallbackAndARecheckHandOverEdgesInOrder) {
  // Under a subscription with native alarms on, a data-change callback runs on
  // the event pump thread while the poll thread rechecks waiting faults. Here
  // the callback starts a 1 ms wait and later clears it, and the recheck ends
  // the wait with a raise, so both threads work out edges for one fault. The
  // tracker hands them out strictly alternating; if a thread could hand its
  // edge over after the other thread worked out a later one, the sink would
  // see two raises or two clears in a row.
  const auto path = std::filesystem::temp_directory_path() / "test_opcua_poller_detection_order.yaml";
  std::ofstream(path) << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Temp"
    entity_id: tank
    data_name: temp
    data_type: float
    raise_after_ms: 1
    alarm:
      fault_code: HIGH_TEMP
      threshold: 80.0
)";
  NodeMap node_map;
  const bool loaded = node_map.load(path.string());
  std::remove(path.string().c_str());
  ASSERT_TRUE(loaded);

  OpcuaClient client;  // never connected: values are delivered by hand
  OpcuaPoller poller(client, node_map);
  std::vector<bool> sent;  // appended under the poller's alarm lock
  poller.set_alarm_callback([&sent](const std::string &, const ros2_medkit::fault_detection::FaultSignal & s) {
    sent.push_back(s.active);
  });
  // Hold the window between working out the edges and handing them over open
  // for 400 us on every pass. The recheck raises 1 ms after the bad value and
  // the clear comes 1.1 ms after it, inside that window, so without the lock
  // the clear is often handed over before the raise.
  poller.set_handover_delay_for_test(std::chrono::microseconds(400));
  poller.evaluate_alarms_for_test(DetectionPipeline::Pass::kRecheck);  // subscription mode from here

  constexpr int kRounds = 1500;
  std::atomic<bool> done{false};
  std::thread pump([&] {
    for (int i = 0; i < kRounds; ++i) {
      poller.on_data_change_for_test("ns=1;s=Temp", OpcuaValue{90.0});
      std::this_thread::sleep_for(std::chrono::microseconds(1100));
      poller.on_data_change_for_test("ns=1;s=Temp", OpcuaValue{70.0});
    }
    done = true;
  });
  std::thread poll([&] {
    while (!done.load()) {
      poller.evaluate_alarms_for_test(DetectionPipeline::Pass::kRecheck);
    }
  });
  pump.join();
  poll.join();

  ASSERT_GT(sent.size(), 100u) << "the recheck should have raised the fault many times";
  EXPECT_TRUE(sent.front()) << "the first edge must be a raise";
  int out_of_order = 0;
  for (size_t i = 1; i < sent.size(); ++i) {
    if (sent[i] == sent[i - 1]) {
      ++out_of_order;
    }
  }
  EXPECT_EQ(out_of_order, 0) << "of " << sent.size() << " edges handed over";
  EXPECT_FALSE(sent.back()) << "the last value was healthy, so the last edge must be a clear";
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
