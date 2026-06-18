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

#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/lifecycle_provider.hpp"
#include "ros2_medkit_gateway/dto/lifecycle.hpp"

using namespace ros2_medkit_gateway;

// --- Mock plugin implementing LifecycleProvider ---

class MockLifecyclePlugin : public GatewayPlugin, public LifecycleProvider {
 public:
  std::string name() const override {
    return "lifecycle_plugin";
  }
  void configure(const json & /*config*/) override {
  }
  void shutdown() override {
  }

  // LifecycleProvider
  tl::expected<dto::LifecycleStatusResponse, LifecycleProviderErrorInfo>
  get_status(const std::string & /*entity_id*/) override {
    dto::LifecycleStatusResponse resp;
    resp.status = "ready";
    return resp;
  }

  tl::expected<std::monostate, LifecycleProviderErrorInfo>
  request_transition(const std::string & /*entity_id*/, std::string_view /*transition*/) override {
    return std::monostate{};
  }
};

// --- Mock plugin without LifecycleProvider ---

class MockBarePlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "bare_plugin";
  }
  void configure(const json & /*config*/) override {
  }
  void shutdown() override {
  }
};

// =============================================================================
// LifecycleProvider Routing Tests
// =============================================================================

TEST(LifecycleProviderRouting, UnownedEntityReturnsNull) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockLifecyclePlugin>();
  mgr.add_plugin(std::move(plugin));

  // No ownership registered - entity is not owned by any plugin
  EXPECT_EQ(mgr.get_lifecycle_provider_for_entity("unowned"), nullptr);
}

TEST(LifecycleProviderRouting, LifecycleProviderResolvedForOwnedEntity) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockLifecyclePlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.register_entity_ownership("lifecycle_plugin", {"my_node"});

  auto * lp = mgr.get_lifecycle_provider_for_entity("my_node");
  ASSERT_NE(lp, nullptr);
  EXPECT_EQ(lp, static_cast<LifecycleProvider *>(raw));

  auto result = lp->get_status("my_node");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
}

TEST(LifecycleProviderRouting, BarePluginReturnsNullLifecycleProvider) {
  PluginManager mgr;

  auto plugin = std::make_unique<MockBarePlugin>();
  mgr.add_plugin(std::move(plugin));

  mgr.register_entity_ownership("bare_plugin", {"entity1"});

  // Entity is owned, but plugin doesn't implement LifecycleProvider
  EXPECT_EQ(mgr.get_lifecycle_provider_for_entity("entity1"), nullptr);
}

TEST(LifecycleProviderRouting, OwnershipForNonLoadedPluginReturnsNull) {
  PluginManager mgr;
  mgr.register_entity_ownership("ghost_plugin", {"entity1"});

  EXPECT_EQ(mgr.get_lifecycle_provider_for_entity("entity1"), nullptr);
}
