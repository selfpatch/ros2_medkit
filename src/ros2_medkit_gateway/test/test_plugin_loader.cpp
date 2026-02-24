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

#include "ros2_medkit_gateway/plugins/plugin_loader.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/providers/update_provider.hpp"

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <string>

using namespace ros2_medkit_gateway;

namespace {

std::string test_plugin_path() {
  auto prefix = ament_index_cpp::get_package_prefix("ros2_medkit_gateway");
  return prefix + "/lib/ros2_medkit_gateway/libtest_gateway_plugin.so";
}

}  // namespace

// @verifies REQ_INTEROP_012
TEST(TestPluginLoader, LoadsValidPlugin) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NE(result->plugin, nullptr);
  EXPECT_EQ(result->plugin->name(), "test_plugin");
  EXPECT_NE(result->handle, nullptr);
}

// @verifies REQ_INTEROP_012
TEST(TestPluginLoader, LoadedPluginImplementsUpdateProvider) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  auto * update = dynamic_cast<UpdateProvider *>(result->plugin.get());
  EXPECT_NE(update, nullptr);
}

// @verifies REQ_INTEROP_012
TEST(TestPluginLoader, LoadedPluginImplementsIntrospectionProvider) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  auto * introspection = dynamic_cast<IntrospectionProvider *>(result->plugin.get());
  EXPECT_NE(introspection, nullptr);
}

// @verifies REQ_INTEROP_012
TEST(TestPluginLoader, RejectsNonexistentFile) {
  auto result = PluginLoader::load("/nonexistent/path/to/plugin.so");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("Failed to load plugin"), std::string::npos);
}

// @verifies REQ_INTEROP_012
TEST(TestPluginLoader, RejectsMissingFactory) {
  // libc.so.6 is a valid .so but has no plugin_api_version symbol
  auto result = PluginLoader::load("libc.so.6");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("plugin_api_version"), std::string::npos);
}
