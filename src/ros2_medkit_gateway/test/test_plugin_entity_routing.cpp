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
#include "ros2_medkit_gateway/providers/operation_provider.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

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
