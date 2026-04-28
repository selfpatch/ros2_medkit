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

#include "ros2_medkit_gateway/core/plugins/plugin_loader.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/core/providers/update_provider.hpp"

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

using namespace ros2_medkit_gateway;
namespace {

std::string plugin_lib_dir() {
  return ament_index_cpp::get_package_prefix("ros2_medkit_gateway") + "/lib/ros2_medkit_gateway/";
}

std::string test_plugin_path() {
  return plugin_lib_dir() + "libtest_gateway_plugin.so";
}

}  // namespace

// --- Happy path ---

TEST(TestPluginLoader, LoadsValidPlugin) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NE(result->plugin, nullptr);
  EXPECT_EQ(result->plugin->name(), "test_plugin");
}

TEST(TestPluginLoader, DiscoverUpdateProviderViaExternC) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NE(result->update_provider, nullptr);
}

TEST(TestPluginLoader, DiscoverIntrospectionProviderViaExternC) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NE(result->introspection_provider, nullptr);
}

// --- Path validation ---

TEST(TestPluginLoader, RejectsNonexistentFile) {
  auto result = PluginLoader::load("/nonexistent/path/to/plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("does not exist"), std::string::npos);
}

TEST(TestPluginLoader, RejectsRelativePath) {
  auto result = PluginLoader::load("relative/path/plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("must be absolute"), std::string::npos);
}

TEST(TestPluginLoader, RejectsNonSoExtension) {
  auto result = PluginLoader::load("/tmp/plugin.dll");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find(".so extension"), std::string::npos);
}

// --- Symbol validation ---

TEST(TestPluginLoader, RejectsMissingVersionSymbol) {
  auto result = PluginLoader::load(plugin_lib_dir() + "libtest_no_symbols_plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("plugin_api_version"), std::string::npos);
}

TEST(TestPluginLoader, RejectsVersionMismatch) {
  auto result = PluginLoader::load(plugin_lib_dir() + "libtest_bad_version_plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("version mismatch"), std::string::npos);
  EXPECT_NE(result.error().find("Rebuild"), std::string::npos);
}

TEST(TestPluginLoader, RejectsMissingFactorySymbol) {
  auto result = PluginLoader::load(plugin_lib_dir() + "libtest_version_only_plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("create_plugin"), std::string::npos);
}

TEST(TestPluginLoader, RejectsNullFactory) {
  auto result = PluginLoader::load(plugin_lib_dir() + "libtest_null_factory_plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("returned null"), std::string::npos);
}

// --- Minimal plugin (no provider query functions) ---

TEST(TestPluginLoader, LoadsMinimalPluginWithNoProviders) {
  auto result = PluginLoader::load(plugin_lib_dir() + "libtest_minimal_plugin.so");
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NE(result->plugin, nullptr);
  EXPECT_EQ(result->plugin->name(), "minimal_plugin");
  EXPECT_EQ(result->update_provider, nullptr);
  EXPECT_EQ(result->introspection_provider, nullptr);
}

// --- GatewayPluginLoadResult move semantics ---

TEST(TestPluginLoader, MoveConstructorTransfersOwnership) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto * orig_plugin = result->plugin.get();
  auto * orig_update = result->update_provider;

  GatewayPluginLoadResult moved(std::move(*result));

  // Moved-to object has the resources
  EXPECT_EQ(moved.plugin.get(), orig_plugin);
  EXPECT_EQ(moved.update_provider, orig_update);

  // Moved-from object is empty
  EXPECT_EQ(result->plugin, nullptr);
  EXPECT_EQ(result->update_provider, nullptr);
  EXPECT_EQ(result->introspection_provider, nullptr);
}

TEST(TestPluginLoader, MoveAssignmentTransfersOwnership) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto * orig_plugin = result->plugin.get();

  GatewayPluginLoadResult assigned;
  assigned = std::move(*result);

  EXPECT_EQ(assigned.plugin.get(), orig_plugin);
  EXPECT_NE(assigned.plugin, nullptr);

  // Moved-from is empty
  EXPECT_EQ(result->plugin, nullptr);
  EXPECT_EQ(result->update_provider, nullptr);
}

TEST(TestPluginLoader, LoadPluginsSuccessPath) {
  // Test load_plugins() through PluginManager with real .so file
  PluginManager mgr;
  std::vector<PluginConfig> configs = {{"test", test_plugin_path(), nlohmann::json::object()}};
  auto loaded = mgr.load_plugins(configs);
  EXPECT_EQ(loaded, 1u);
  EXPECT_TRUE(mgr.has_plugins());
  EXPECT_EQ(mgr.plugin_names()[0], "test_plugin");
  EXPECT_NE(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}
