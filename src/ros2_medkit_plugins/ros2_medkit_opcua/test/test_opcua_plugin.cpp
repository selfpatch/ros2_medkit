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

// Unit tests for OpcuaPlugin's DataProvider/OperationProvider/FaultProvider
// methods. These test the provider layer directly (no HTTP, no OPC-UA server)
// by constructing the plugin with a pre-loaded NodeMap and a fake poller
// snapshot. The gateway's PluginContext is stubbed with a minimal
// FakePluginContext that returns entities and faults on demand.

#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

// -- Stub PluginRequest/PluginResponse (same pattern as graph_provider test) --

namespace ros2_medkit_gateway {

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t) const {
  return {};
}
std::string PluginRequest::header(const std::string &) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
std::string PluginRequest::query_param(const std::string &) const {
  return {};
}

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json &) {
}
void PluginResponse::send_error(int, const std::string &, const std::string &, const nlohmann::json &) {
}

// -- FakePluginContext --

class FakePluginContext : public RosPluginContext {
 public:
  std::unordered_map<std::string, PluginEntityInfo> entities;
  nlohmann::json all_faults = nlohmann::json::object();

  rclcpp::Node * node() const override {
    return nullptr;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities.find(id);
    if (it != entities.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*entity_id*/) const override {
    return {};
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    // Contract: a bare JSON array of fault objects scoped to the entity.
    nlohmann::json result = nlohmann::json::array();
    if (all_faults.contains("faults")) {
      for (const auto & f : all_faults["faults"]) {
        if (f.value("source_id", "") == entity_id) {
          result.push_back(f);
        }
      }
    }
    return result;
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & /*res*/,
                                                            const std::string & entity_id) const override {
    return get_entity(entity_id);
  }

  void register_capability(SovdEntityType /*type*/, const std::string & /*capability*/) override {
  }
  void register_entity_capability(const std::string & /*entity_id*/, const std::string & /*capability*/) override {
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType /*type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*collection*/,
                              const std::string & /*client_id*/) const override {
    return {true, "", ""};
  }
  tl::expected<LockInfo, LockError> acquire_lock(const std::string & /*entity_id*/, const std::string & /*collection*/,
                                                 const std::vector<std::string> & /*scopes*/,
                                                 int /*expiration_s*/) override {
    return tl::make_unexpected(LockError{"not supported", ""});
  }
  tl::expected<void, LockError> release_lock(const std::string & /*entity_id*/,
                                             const std::string & /*lock_id*/) override {
    return tl::make_unexpected(LockError{"not supported", ""});
  }
  IntrospectionInput get_entity_snapshot() const override {
    return {};
  }
  nlohmann::json list_all_faults() const override {
    return all_faults;
  }
  void register_sampler(
      const std::string & /*topic*/,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &
      /*sampler*/) override {
  }
  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
};

// -- Test fixture --

class OpcuaPluginTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Write a test node map YAML
    yaml_path_ = "/tmp/test_opcua_plugin_nodemap.yaml";
    std::ofstream f(yaml_path_);
    f << R"(
area_id: test_plc
component_id: test_runtime
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank
    data_name: level
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: true
    min_value: 0.0
    max_value: 1000.0
  - node_id: "ns=2;i=2"
    entity_id: tank
    data_name: pressure
    display_name: Tank Pressure
    unit: bar
    data_type: float
    writable: false
)";
    f.close();

    // Configure plugin with the test node map
    nlohmann::json config;
    config["node_map_path"] = yaml_path_;
    config["endpoint_url"] = "opc.tcp://nonexistent:4840";
    plugin_.configure(config);

    // Set up fake context with PLC entities
    ctx_.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
    ctx_.entities["test_runtime"] = {SovdEntityType::COMPONENT, "test_runtime", "/test_plc", "/test_plc/test_runtime"};
    plugin_.set_context(ctx_);
  }

  std::string yaml_path_;
  OpcuaPlugin plugin_;
  FakePluginContext ctx_;
};

// -- DataProvider tests --

TEST_F(OpcuaPluginTest, ListDataReturnsItems) {
  auto result = plugin_.list_data("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->content.contains("items"));
  EXPECT_EQ(result->content["items"].size(), 2u);
  EXPECT_EQ(result->content["items"][0]["id"], "level");
  EXPECT_EQ(result->content["items"][1]["id"], "pressure");
}

TEST_F(OpcuaPluginTest, ListDataEntityNotFound) {
  auto result = plugin_.list_data("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::EntityNotFound);
  EXPECT_EQ(result.error().http_status, 404);
}

TEST_F(OpcuaPluginTest, ReadDataReturnsValue) {
  auto result = plugin_.read_data("tank", "level");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content["id"], "level");
  EXPECT_EQ(result->content["unit"], "mm");
  EXPECT_EQ(result->content["data_type"], "float");
  EXPECT_EQ(result->content["writable"], true);
}

TEST_F(OpcuaPluginTest, ReadDataNotFound) {
  auto result = plugin_.read_data("tank", "nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::ResourceNotFound);
}

TEST_F(OpcuaPluginTest, WriteDataReadOnly) {
  nlohmann::json body = {{"value", 5.0}};
  auto result = plugin_.write_data("tank", "pressure", body);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::ReadOnly);
  EXPECT_EQ(result.error().http_status, 400);
}

TEST_F(OpcuaPluginTest, WriteDataMissingValue) {
  nlohmann::json body = {{"not_value", 42}};
  auto result = plugin_.write_data("tank", "level", body);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::InvalidValue);
}

// -- OperationProvider tests --

TEST_F(OpcuaPluginTest, ListOperationsOnlyWritable) {
  auto result = plugin_.list_operations("tank");
  ASSERT_TRUE(result.has_value());
  // Only 'level' is writable, 'pressure' is read-only
  EXPECT_EQ(result->items.size(), 1u);
  EXPECT_EQ(result->items[0].id, "set_level");
}

TEST_F(OpcuaPluginTest, ListOperationsEntityNotFound) {
  auto result = plugin_.list_operations("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::EntityNotFound);
}

TEST_F(OpcuaPluginTest, ExecuteOperationMissingValue) {
  nlohmann::json params = {{"not_value", 42}};
  auto result = plugin_.execute_operation("tank", "set_level", params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::InvalidParameters);
}

TEST_F(OpcuaPluginTest, ExecuteOperationReadOnly) {
  nlohmann::json params = {{"value", 5.0}};
  auto result = plugin_.execute_operation("tank", "set_pressure", params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::Rejected);
}

// -- FaultProvider tests --

TEST_F(OpcuaPluginTest, ListFaultsEmpty) {
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->content.contains("items"));
  EXPECT_TRUE(result->content["items"].empty());
}

TEST_F(OpcuaPluginTest, ListFaultsWithData) {
  ctx_.all_faults = {{"faults", {{{"fault_code", "PLC_LOW_LEVEL"}, {"source_id", "tank"}, {"severity", 2}}}}};
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content["items"].size(), 1u);
  EXPECT_EQ(result->content["items"][0]["code"], "PLC_LOW_LEVEL");
}

TEST_F(OpcuaPluginTest, GetFaultNotFound) {
  auto result = plugin_.get_fault("tank", "NONEXISTENT");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, FaultProviderError::FaultNotFound);
}

TEST_F(OpcuaPluginTest, GetFaultFound) {
  // Regression pin for the shape contract: list_entity_faults yields a bare
  // array of fault objects scoped to the entity, not a {"faults": [...]}
  // object. get_fault must locate the matching fault in that bare array.
  ctx_.all_faults = {{"faults", {{{"fault_code", "PLC_LOW_LEVEL"}, {"source_id", "tank"}, {"severity", 2}}}}};
  auto result = plugin_.get_fault("tank", "PLC_LOW_LEVEL");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->content.value("fault_code", ""), "PLC_LOW_LEVEL");
}

// -- configure() validation (issue #481) --

TEST(OpcuaPluginConfigureTest, ThrowsOnInvalidNodeMap) {
  // A node map that fails validation (here a duplicate fault_code) must make
  // configure() throw so PluginManager disables the plugin instead of starting
  // the poller against a map whose global fault-code uniqueness was rejected.
  std::string path = "/tmp/test_opcua_plugin_invalid_nodemap.yaml";
  std::ofstream f(path);
  f << R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;i=1"
    entity_id: ent_a
    data_name: a
    alarm:
      fault_code: DUP_CODE
      threshold: 1.0
  - node_id: "ns=1;i=2"
    entity_id: ent_b
    data_name: b
    alarm:
      fault_code: DUP_CODE
      threshold: 2.0
)";
  f.close();

  OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = path;
  config["endpoint_url"] = "opc.tcp://nonexistent:4840";
  EXPECT_THROW(plugin.configure(config), std::runtime_error);
}

// Issue #389: condition-replay strategy parsing (used by the poller's
// reconnect / restart active-condition replay).
TEST(ConditionReplayStrategyParse, KnownValues) {
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("method"), ConditionReplayStrategy::Method);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("read"), ConditionReplayStrategy::Read);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("read_fallback"), ConditionReplayStrategy::Read);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("off"), ConditionReplayStrategy::Off);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("auto"), ConditionReplayStrategy::Auto);
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("AUTO"), ConditionReplayStrategy::Auto);
  // Unknown -> safe default (Auto: method with read fallback).
  EXPECT_EQ(OpcuaPoller::parse_replay_strategy("bogus"), ConditionReplayStrategy::Auto);
}

// -- Issue #479/#478: read-replay reconcile must not falsely clear faults -----

TEST(ReconcileShouldClearCondition, ClearsActiveConditionAbsentFromSuccessfulScan) {
  // An active condition that the (successful) scan did not observe is gone -
  // but only when its source is positively known to model condition nodes.
  std::set<std::string> seen;                      // condition not seen this scan
  std::set<std::string> failed_sources;            // its source scanned fine
  std::set<std::string> modeled_sources{"src-1"};  // source exposes condition nodes
  EXPECT_TRUE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                  modeled_sources));
  EXPECT_TRUE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Healed, "cond-1", "src-1", seen, failed_sources,
                                                  modeled_sources));
}

TEST(ReconcileShouldClearCondition, KeepsConditionStillSeen) {
  std::set<std::string> seen{"cond-1"};
  std::set<std::string> failed_sources;
  std::set<std::string> modeled_sources{"src-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

TEST(ReconcileShouldClearCondition, SkipsClearWhenSourceScanFailed) {
  // The false-clear guard: source scan failed (browse error / disconnect),
  // so the condition's absence from ``seen`` must NOT clear it.
  std::set<std::string> seen;                     // not observed...
  std::set<std::string> failed_sources{"src-1"};  // ...because its source failed
  std::set<std::string> modeled_sources{"src-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Healed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

// Issue #478 safety-gate: an empty scan from a source that has NEVER yielded a
// condition instance node (EventNotifier-only server, e.g. S7-1500) must NOT
// clear the still-active tracked fault. This is the single most important
// outcome of the hardening.
TEST(ReconcileShouldClearCondition, NeverClearsWhenSourceNotPositivelyModeled) {
  std::set<std::string> seen;             // condition not seen (server has no nodes)
  std::set<std::string> failed_sources;   // the browse itself succeeded (empty)
  std::set<std::string> modeled_sources;  // src-1 has never yielded a condition node
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Confirmed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Healed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

TEST(ReconcileShouldClearCondition, NeverClearsInactiveCondition) {
  std::set<std::string> seen;
  std::set<std::string> failed_sources;
  std::set<std::string> modeled_sources{"src-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Cleared, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
  EXPECT_FALSE(OpcuaPoller::should_clear_condition(SovdAlarmStatus::Suppressed, "cond-1", "src-1", seen, failed_sources,
                                                   modeled_sources));
}

// -- Issue #480: ConditionRefresh burst reconcile must clear a tracked fault
// that the server did NOT replay (it cleared while we were offline). Unlike the
// read fallback, a delivered RefreshEnd is an authoritative subscription-wide
// replay, so there is no per-source modeling gate. --------------------------

TEST(ReconcileAfterRefresh, ClearsActiveConditionNotReplayed) {
  // Active before the reconnect, absent from the RefreshStart..RefreshEnd burst
  // => the alarm cleared offline and its latched fault must reconcile away.
  std::set<std::string> seen;  // server replayed nothing (no active conditions)
  EXPECT_TRUE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-1", seen));
  EXPECT_TRUE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Healed, "cond-1", seen));
}

TEST(ReconcileAfterRefresh, KeepsConditionReplayedInBurst) {
  // The server replayed this ConditionId during the burst => still active, keep.
  std::set<std::string> seen{"cond-1"};
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-1", seen));
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Healed, "cond-1", seen));
}

TEST(ReconcileAfterRefresh, NeverClearsInactiveCondition) {
  // A condition that was never active (Suppressed/Cleared) is not a latched
  // fault and must not be touched even when absent from the burst.
  std::set<std::string> seen;
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Cleared, "cond-1", seen));
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Suppressed, "cond-1", seen));
}

TEST(ReconcileAfterRefresh, KeepsOtherActiveConditionsWhenOneReplayed) {
  // Peer condition (same fault_code family, different ConditionId) replayed;
  // the un-replayed one still clears. Guards the distinct-ConditionId keying.
  std::set<std::string> seen{"cond-keep"};
  EXPECT_FALSE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-keep", seen));
  EXPECT_TRUE(OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus::Confirmed, "cond-gone", seen));
}

// -- Issue #478: read-scan snapshot classification (Retain / EnabledState /
// transient-keep filter that mirrors ConditionRefresh semantics). --------------

namespace {
OpcuaClient::ConditionStateSnapshot make_snap(bool enabled, bool active, bool retain, bool read_failed = false) {
  OpcuaClient::ConditionStateSnapshot s;
  s.enabled_state = enabled;
  s.active_state = active;
  s.retain = retain;
  s.state_read_failed = read_failed;
  return s;
}
}  // namespace

TEST(ReadSnapshotClassify, RetainTrueActiveIsFed) {
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/true, /*retain=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Feed);
}

TEST(ReadSnapshotClassify, RetainTrueInactiveStillInterestingIsFed) {
  // ActiveState=false but Retain=true => unacked/unconfirmed, still of interest
  // (ConditionRefresh would replay it). Fed so the HEALED latch is preserved.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/false, /*retain=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Feed);
}

TEST(ReadSnapshotClassify, RetainFalseIsSkipped) {
  // Retain=false => no longer interesting; not seen so a stale fault reconciles.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/false, /*retain=*/false));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Skip);
}

TEST(ReadSnapshotClassify, DisabledIsSkippedEvenIfActiveAndRetained) {
  // EnabledState=false must never be treated as active in the read path, even
  // when ActiveState/Retain would otherwise mark it interesting.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/false, /*active=*/true, /*retain=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Skip);
}

TEST(ReadSnapshotClassify, TransientReadFailureIsKeptNotFed) {
  // A transient read/browse failure (flagged) keeps the condition (so reconcile
  // does not clear it) but does not feed an unreliable state into the machine.
  // KeepOnly takes precedence over every other field.
  auto d = OpcuaPoller::classify_read_snapshot(
      make_snap(/*enabled=*/false, /*active=*/false, /*retain=*/false, /*read_failed=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::KeepOnly);
}

TEST(ReadSnapshotClassify, ActiveWithUnreadableRetainIsFedNotSkipped) {
  // Regression (issue #478): Retain defaults false when its optional node is
  // unreadable this scan. A reliably-active condition must be Fed (so it lands
  // in ``seen``), never Skipped on the Retain check - otherwise reconcile would
  // clear a still-active fault on a modeled source.
  auto d = OpcuaPoller::classify_read_snapshot(make_snap(/*enabled=*/true, /*active=*/true, /*retain=*/false));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::Feed);
}

TEST(ReadSnapshotClassify, ActiveButFlaggedUnreliableIsKeptNotFed) {
  // KeepOnly still wins over the active fast-path: an active condition whose
  // snapshot is flagged unreliable (e.g. a transient Retain read failure) is
  // preserved via ``seen`` but not fed an untrustworthy state.
  auto d = OpcuaPoller::classify_read_snapshot(
      make_snap(/*enabled=*/true, /*active=*/true, /*retain=*/false, /*read_failed=*/true));
  EXPECT_EQ(d, OpcuaPoller::ReadReplayDisposition::KeepOnly);
}

}  // namespace ros2_medkit_gateway
