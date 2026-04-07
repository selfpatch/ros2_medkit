// Copyright 2026 bburda
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

#include <gtest/gtest.h>

#include "ros2_medkit_gateway/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/providers/data_provider.hpp"
#include "ros2_medkit_gateway/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/providers/operation_provider.hpp"

using namespace ros2_medkit_gateway;
// json alias already available via ros2_medkit_gateway namespace headers

// --- Mock plugin implementing DataProvider + OperationProvider ---

class MockDataOpPlugin : public GatewayPlugin, public DataProvider, public OperationProvider {
 public:
  std::string name() const override {
    return name_;
  }
  void configure(const json &) override {
  }
  void shutdown() override {
  }

  // DataProvider
  tl::expected<json, DataProviderErrorInfo> list_data(const std::string & entity_id) override {
    return json{{"items", json::array({{{"id", "test_data"}, {"entity", entity_id}}})}};
  }
  tl::expected<json, DataProviderErrorInfo> read_data(const std::string &, const std::string & resource) override {
    return json{{"value", resource}};
  }
  tl::expected<json, DataProviderErrorInfo> write_data(const std::string &, const std::string &,
                                                       const json &) override {
    return json{{"status", "ok"}};
  }

  // OperationProvider
  tl::expected<json, OperationProviderErrorInfo> list_operations(const std::string & entity_id) override {
    return json{{"items", json::array({{{"id", "test_op"}, {"entity", entity_id}}})}};
  }
  tl::expected<json, OperationProviderErrorInfo> execute_operation(const std::string &, const std::string & op,
                                                                   const json &) override {
    return json{{"executed", op}};
  }

  std::string name_ = "test_plugin";
};

// --- Mock plugin without providers ---

class MockBarePlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "bare_plugin";
  }
  void configure(const json &) override {
  }
  void shutdown() override {
  }
};

// =============================================================================
// Entity Ownership Tests
// =============================================================================

TEST(PluginEntityRouting, UnknownEntityReturnsNullopt) {
  PluginManager mgr;
  EXPECT_FALSE(mgr.get_entity_owner("nonexistent").has_value());
  EXPECT_EQ(mgr.get_data_provider_for_entity("nonexistent"), nullptr);
  EXPECT_EQ(mgr.get_operation_provider_for_entity("nonexistent"), nullptr);
}

TEST(PluginEntityRouting, RegisteredEntityReturnsOwner) {
  PluginManager mgr;
  mgr.register_entity_ownership("uds_gateway", {"ecu1", "ecu2"});

  auto owner1 = mgr.get_entity_owner("ecu1");
  ASSERT_TRUE(owner1.has_value());
  EXPECT_EQ(*owner1, "uds_gateway");

  auto owner2 = mgr.get_entity_owner("ecu2");
  ASSERT_TRUE(owner2.has_value());
  EXPECT_EQ(*owner2, "uds_gateway");

  EXPECT_FALSE(mgr.get_entity_owner("ecu3").has_value());
}

TEST(PluginEntityRouting, MultiplePluginsOwnDifferentEntities) {
  PluginManager mgr;
  mgr.register_entity_ownership("uds_gateway", {"ecu1"});
  mgr.register_entity_ownership("opcua_gateway", {"plc1"});

  EXPECT_EQ(*mgr.get_entity_owner("ecu1"), "uds_gateway");
  EXPECT_EQ(*mgr.get_entity_owner("plc1"), "opcua_gateway");
}

// =============================================================================
// Provider Routing Tests (with in-process plugins via add_plugin)
// =============================================================================

TEST(PluginEntityRouting, DataProviderResolvedForOwnedEntity) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockDataOpPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  // Register ownership
  mgr.register_entity_ownership("test_plugin", {"my_ecu"});

  // Should resolve to the plugin's DataProvider
  auto * dp = mgr.get_data_provider_for_entity("my_ecu");
  ASSERT_NE(dp, nullptr);
  EXPECT_EQ(dp, static_cast<DataProvider *>(raw));

  // Verify it actually works
  auto result = dp->list_data("my_ecu");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["items"][0]["entity"], "my_ecu");
}

TEST(PluginEntityRouting, OperationProviderResolvedForOwnedEntity) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockDataOpPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.register_entity_ownership("test_plugin", {"my_ecu"});

  auto * op = mgr.get_operation_provider_for_entity("my_ecu");
  ASSERT_NE(op, nullptr);
  EXPECT_EQ(op, static_cast<OperationProvider *>(raw));

  auto result = op->execute_operation("my_ecu", "reset", json::object());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["executed"], "reset");
}

TEST(PluginEntityRouting, BarePluginReturnsNullProviders) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockBarePlugin>();
  mgr.add_plugin(std::move(plugin));

  mgr.register_entity_ownership("bare_plugin", {"entity1"});

  // Entity is owned, but plugin doesn't implement providers
  auto owner = mgr.get_entity_owner("entity1");
  ASSERT_TRUE(owner.has_value());
  EXPECT_EQ(*owner, "bare_plugin");

  EXPECT_EQ(mgr.get_data_provider_for_entity("entity1"), nullptr);
  EXPECT_EQ(mgr.get_operation_provider_for_entity("entity1"), nullptr);
}

TEST(PluginEntityRouting, UnownedEntityReturnsNullProviders) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockDataOpPlugin>();
  mgr.add_plugin(std::move(plugin));

  // Don't register ownership - entity is not owned by any plugin
  EXPECT_EQ(mgr.get_data_provider_for_entity("unowned"), nullptr);
  EXPECT_EQ(mgr.get_operation_provider_for_entity("unowned"), nullptr);
}

// =============================================================================
// FaultProvider Routing Tests
// =============================================================================

class MockFaultPlugin : public GatewayPlugin, public FaultProvider {
 public:
  std::string name() const override {
    return "fault_plugin";
  }
  void configure(const json &) override {
  }
  void shutdown() override {
  }

  tl::expected<json, FaultProviderErrorInfo> list_faults(const std::string & entity_id) override {
    return json{{"items", json::array({{{"code", "DTC_001"}, {"entity", entity_id}}})}};
  }
  tl::expected<json, FaultProviderErrorInfo> get_fault(const std::string &, const std::string & code) override {
    return json{{"code", code}, {"status", "pending"}};
  }
  tl::expected<json, FaultProviderErrorInfo> clear_fault(const std::string &, const std::string & code) override {
    return json{{"code", code}, {"cleared", true}};
  }
};

TEST(PluginEntityRouting, FaultProviderResolvedForOwnedEntity) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockFaultPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.register_entity_ownership("fault_plugin", {"my_ecu"});

  auto * fp = mgr.get_fault_provider_for_entity("my_ecu");
  ASSERT_NE(fp, nullptr);
  EXPECT_EQ(fp, static_cast<FaultProvider *>(raw));

  auto result = fp->list_faults("my_ecu");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["items"][0]["code"], "DTC_001");
}

TEST(PluginEntityRouting, BarePluginReturnsNullFaultProvider) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockBarePlugin>();
  mgr.add_plugin(std::move(plugin));

  mgr.register_entity_ownership("bare_plugin", {"entity1"});
  EXPECT_EQ(mgr.get_fault_provider_for_entity("entity1"), nullptr);
}

TEST(PluginEntityRouting, UnownedEntityReturnsNullFaultProvider) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockFaultPlugin>();
  mgr.add_plugin(std::move(plugin));

  EXPECT_EQ(mgr.get_fault_provider_for_entity("unowned"), nullptr);
}

// =============================================================================
// Error Propagation Tests
// =============================================================================

class MockErrorPlugin : public GatewayPlugin, public DataProvider, public FaultProvider {
 public:
  std::string name() const override {
    return "error_plugin";
  }
  void configure(const json &) override {
  }
  void shutdown() override {
  }

  tl::expected<json, DataProviderErrorInfo> list_data(const std::string &) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::TransportError, "backend unavailable", 503});
  }
  tl::expected<json, DataProviderErrorInfo> read_data(const std::string &, const std::string &) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ResourceNotFound, "no such resource", 404});
  }
  tl::expected<json, DataProviderErrorInfo> write_data(const std::string &, const std::string &,
                                                       const json &) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only entity", 403});
  }

  tl::expected<json, FaultProviderErrorInfo> list_faults(const std::string &) override {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::TransportError, "not reachable", 503});
  }
  tl::expected<json, FaultProviderErrorInfo> get_fault(const std::string &, const std::string &) override {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::FaultNotFound, "unknown fault", 404});
  }
  tl::expected<json, FaultProviderErrorInfo> clear_fault(const std::string &, const std::string &) override {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "cannot clear", 409});
  }
};

TEST(PluginEntityRouting, DataProviderErrorPropagation) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockErrorPlugin>();
  mgr.add_plugin(std::move(plugin));
  mgr.register_entity_ownership("error_plugin", {"bad_ecu"});

  auto * dp = mgr.get_data_provider_for_entity("bad_ecu");
  ASSERT_NE(dp, nullptr);

  auto result = dp->list_data("bad_ecu");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 503);
  EXPECT_EQ(result.error().message, "backend unavailable");
  EXPECT_EQ(result.error().code, DataProviderError::TransportError);
}

TEST(PluginEntityRouting, FaultProviderErrorPropagation) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockErrorPlugin>();
  mgr.add_plugin(std::move(plugin));
  mgr.register_entity_ownership("error_plugin", {"bad_ecu"});

  auto * fp = mgr.get_fault_provider_for_entity("bad_ecu");
  ASSERT_NE(fp, nullptr);

  auto result = fp->get_fault("bad_ecu", "DTC_999");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().message, "unknown fault");
}

// =============================================================================
// Entity Ownership Conflict Tests
// =============================================================================

TEST(PluginEntityRouting, OwnershipConflictLastWins) {
  PluginManager mgr;
  mgr.register_entity_ownership("plugin_a", {"shared_entity"});
  mgr.register_entity_ownership("plugin_b", {"shared_entity"});

  auto owner = mgr.get_entity_owner("shared_entity");
  ASSERT_TRUE(owner.has_value());
  EXPECT_EQ(*owner, "plugin_b");
}

TEST(PluginEntityRouting, ClearEntityOwnership) {
  PluginManager mgr;
  mgr.register_entity_ownership("plugin_a", {"ent1", "ent2"});
  mgr.register_entity_ownership("plugin_b", {"ent3"});

  mgr.clear_entity_ownership("plugin_a");

  EXPECT_FALSE(mgr.get_entity_owner("ent1").has_value());
  EXPECT_FALSE(mgr.get_entity_owner("ent2").has_value());
  // plugin_b's entity should be unaffected
  EXPECT_EQ(*mgr.get_entity_owner("ent3"), "plugin_b");
}

// =============================================================================
// OperationProvider list+filter contract (used by handle_get_operation)
// =============================================================================

TEST(PluginEntityRouting, OperationListFilterFindsMatchingItem) {
  // Verifies the contract that handle_get_operation depends on:
  // list_operations returns {"items": [...]}, handler scans for matching "id"
  PluginManager mgr;
  auto plugin = std::make_unique<MockDataOpPlugin>();
  mgr.add_plugin(std::move(plugin));
  mgr.register_entity_ownership("test_plugin", {"my_ecu"});

  auto * op = mgr.get_operation_provider_for_entity("my_ecu");
  ASSERT_NE(op, nullptr);
  auto result = op->list_operations("my_ecu");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->contains("items"));
  ASSERT_TRUE((*result)["items"].is_array());

  // Simulate handler's filter logic: find item with matching "id"
  bool found = false;
  for (const auto & item : (*result)["items"]) {
    if (item.value("id", "") == "test_op") {
      found = true;
      EXPECT_EQ(item["entity"], "my_ecu");
    }
  }
  EXPECT_TRUE(found);

  // Non-matching ID should not be found
  bool found_nonexistent = false;
  for (const auto & item : (*result)["items"]) {
    if (item.value("id", "") == "nonexistent_op") {
      found_nonexistent = true;
    }
  }
  EXPECT_FALSE(found_nonexistent);
}

TEST(PluginEntityRouting, OwnershipForNonLoadedPluginReturnsNullProviders) {
  PluginManager mgr;
  mgr.register_entity_ownership("ghost_plugin", {"entity1"});

  auto owner = mgr.get_entity_owner("entity1");
  ASSERT_TRUE(owner.has_value());
  EXPECT_EQ(*owner, "ghost_plugin");

  EXPECT_EQ(mgr.get_data_provider_for_entity("entity1"), nullptr);
  EXPECT_EQ(mgr.get_operation_provider_for_entity("entity1"), nullptr);
  EXPECT_EQ(mgr.get_fault_provider_for_entity("entity1"), nullptr);
}

TEST(PluginEntityRouting, ClearAndReregisterOwnership) {
  PluginManager mgr;
  mgr.register_entity_ownership("plugin_a", {"ent1", "ent2"});

  // Simulate refresh: entity2 disappeared, entity3 appeared
  mgr.clear_entity_ownership("plugin_a");
  mgr.register_entity_ownership("plugin_a", {"ent1", "ent3"});

  EXPECT_TRUE(mgr.get_entity_owner("ent1").has_value());
  EXPECT_FALSE(mgr.get_entity_owner("ent2").has_value());  // removed
  EXPECT_TRUE(mgr.get_entity_owner("ent3").has_value());   // added
}
