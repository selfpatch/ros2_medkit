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

// @verifies REQ_INTEROP_UPDATE_PROVIDER_NOTIFY

#include <gtest/gtest.h>

#include "ros2_medkit_gateway/plugins/entity_change_scope.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"

using ros2_medkit_gateway::EntityChangeScope;

TEST(EntityChangeScopeTest, DefaultIsFullRefresh) {
  EntityChangeScope scope;
  EXPECT_TRUE(scope.is_full_refresh());
  EXPECT_FALSE(scope.area_id.has_value());
  EXPECT_FALSE(scope.component_id.has_value());
}

TEST(EntityChangeScopeTest, FullRefreshSentinelRoundTrip) {
  auto scope = EntityChangeScope::full_refresh();
  EXPECT_TRUE(scope.is_full_refresh());
}

TEST(EntityChangeScopeTest, ForComponentScopesToComponentOnly) {
  auto scope = EntityChangeScope::for_component("ecu-primary");
  EXPECT_FALSE(scope.is_full_refresh());
  EXPECT_FALSE(scope.area_id.has_value());
  ASSERT_TRUE(scope.component_id.has_value());
  EXPECT_EQ(*scope.component_id, "ecu-primary");
}

TEST(EntityChangeScopeTest, ForAreaScopesToAreaOnly) {
  auto scope = EntityChangeScope::for_area("vehicle");
  EXPECT_FALSE(scope.is_full_refresh());
  EXPECT_FALSE(scope.component_id.has_value());
  ASSERT_TRUE(scope.area_id.has_value());
  EXPECT_EQ(*scope.area_id, "vehicle");
}

TEST(EntityChangeScopeTest, AreaAndComponentCoexist) {
  EntityChangeScope scope;
  scope.area_id = "vehicle";
  scope.component_id = "ecu-primary";
  EXPECT_FALSE(scope.is_full_refresh());
  EXPECT_EQ(*scope.area_id, "vehicle");
  EXPECT_EQ(*scope.component_id, "ecu-primary");
}

// -- PluginContext default no-op behavior (backwards-compat guarantee) --

namespace {

/// Minimal PluginContext that defaults every other method to a trap. Used to
/// prove that `notify_entities_changed` has a usable default even when the
/// plugin implementer touches nothing else, which is the whole point of the
/// v6 -> v7 compatibility story.
class BareContext : public ros2_medkit_gateway::PluginContext {
 public:
  rclcpp::Node * node() const override {
    return nullptr;
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo> get_entity(const std::string &) const override {
    return std::nullopt;
  }
  std::vector<ros2_medkit_gateway::PluginEntityInfo> get_child_apps(const std::string &) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string &) const override {
    return nlohmann::json::array();
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo> validate_entity_for_route(
      const ros2_medkit_gateway::PluginRequest &, ros2_medkit_gateway::PluginResponse &,
      const std::string &) const override {
    return std::nullopt;
  }
  void register_capability(ros2_medkit_gateway::SovdEntityType, const std::string &) override {
  }
  void register_entity_capability(const std::string &, const std::string &) override {
  }
  std::vector<std::string> get_type_capabilities(ros2_medkit_gateway::SovdEntityType) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string &) const override {
    return {};
  }
  ros2_medkit_gateway::LockAccessResult check_lock(const std::string &, const std::string &,
                                                    const std::string &) const override {
    return {};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError> acquire_lock(
      const std::string &, const std::string &, const std::vector<std::string> &, int) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string &,
                                                                   const std::string &) override {
    return {};
  }
  ros2_medkit_gateway::ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ros2_medkit_gateway::ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
};

}  // namespace

TEST(PluginContextTest, NotifyEntitiesChangedDefaultIsNoOp) {
  // A plugin built against v6 inherits the default implementation. Calling it
  // must be safe and must not throw - the gateway interprets silence as "no
  // change happened".
  BareContext ctx;
  EXPECT_NO_THROW(ctx.notify_entities_changed(EntityChangeScope::full_refresh()));
  EXPECT_NO_THROW(ctx.notify_entities_changed(EntityChangeScope::for_component("ecu-primary")));
  EXPECT_NO_THROW(ctx.notify_entities_changed(EntityChangeScope::for_area("vehicle")));
}
