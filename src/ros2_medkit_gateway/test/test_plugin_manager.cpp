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

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

/// Minimal mock plugin for compile-time testing (no .so needed)
class MockPlugin : public GatewayPlugin, public UpdateProvider, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "mock";
  }
  void configure(const json & config) override {
    configured_ = true;
    config_ = config;
  }
  void shutdown() override {
    shutdown_called_ = true;
  }

  // UpdateProvider
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter &) override {
    return std::vector<std::string>{};
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string &) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "mock"});
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string &, UpdateProgressReporter &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string &, UpdateProgressReporter &) override {
    return {};
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string &) override {
    return false;
  }

  // IntrospectionProvider
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }

  bool configured_ = false;
  bool shutdown_called_ = false;
  json config_;
};

/// Plugin that only provides IntrospectionProvider
class MockIntrospectionOnly : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "introspection_only";
  }
  void configure(const json &) override {
  }
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }
};

/// Plugin that throws during configure
class MockThrowingPlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "throwing";
  }
  void configure(const json &) override {
    throw std::runtime_error("configure failed");
  }
};

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, EmptyManagerHasNoPlugins) {
  PluginManager mgr;
  EXPECT_FALSE(mgr.has_plugins());
  EXPECT_TRUE(mgr.plugin_names().empty());
  EXPECT_EQ(mgr.get_update_provider(), nullptr);
  EXPECT_TRUE(mgr.get_introspection_providers().empty());
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, AddPluginAndDispatch) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  EXPECT_TRUE(mgr.has_plugins());
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "mock");

  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(raw));
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, ConfigurePassesConfig) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.configure_plugins();
  EXPECT_TRUE(raw->configured_);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, ShutdownCallsAllPlugins) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.shutdown_all();
  EXPECT_TRUE(raw->shutdown_called_);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, MultiCapabilityPluginDispatchedToBoth) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());

  EXPECT_NE(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, IntrospectionOnlyPluginNotUpdateProvider) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  EXPECT_EQ(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, MultipleIntrospectionProviders) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  EXPECT_EQ(mgr.get_introspection_providers().size(), 2u);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, DuplicateUpdateProviderFirstWins) {
  PluginManager mgr;
  auto first = std::make_unique<MockPlugin>();
  auto * first_raw = first.get();
  mgr.add_plugin(std::move(first));
  mgr.add_plugin(std::make_unique<MockPlugin>());

  // First UpdateProvider wins
  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(first_raw));
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, ThrowingPluginDisabledDuringConfigure) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowingPlugin>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  // Should not throw
  mgr.configure_plugins();

  // Good plugin still works
  EXPECT_TRUE(good_raw->configured_);
  EXPECT_NE(mgr.get_update_provider(), nullptr);
}

// @verifies REQ_INTEROP_012
TEST(PluginManagerTest, LoadNonexistentPluginReturnsZero) {
  PluginManager mgr;
  std::vector<PluginConfig> configs = {{"nonexistent", "/nonexistent/path.so", json::object()}};
  auto loaded = mgr.load_plugins(configs);
  EXPECT_EQ(loaded, 0u);
  EXPECT_FALSE(mgr.has_plugins());
}
