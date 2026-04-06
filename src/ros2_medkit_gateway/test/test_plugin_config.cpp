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

/// Tests that plugin config from --params-file YAML reaches the gateway plugin framework.
///
/// The bug: extract_plugin_config() read from get_node_options().parameter_overrides(),
/// which only contains programmatically-set overrides. Parameters from --params-file
/// go into the ROS 2 global rcl context and are NOT copied to NodeOptions::parameter_overrides_.
///
/// The fix: declare_plugin_params_from_yaml() accesses rcl_arguments_get_param_overrides()
/// to discover plugin params and declares them on the node.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/param_utils.hpp"

/// Proves the bug and validates the fix using a lightweight rclcpp::Node.
///
/// 1. Writes a YAML params file with plugin config
/// 2. Inits rclcpp with --params-file (production path)
/// 3. Creates a plain Node (NOT GatewayNode) with the matching name
/// 4. Verifies NodeOptions::parameter_overrides() is empty (the bug)
/// 5. Verifies declare_plugin_params_from_yaml() resolves the YAML values (the fix)
TEST(PluginConfig, YamlPluginParamsReachGateway) {
  // Write YAML with plugin config using the gateway node name
  std::string yaml_path = "/tmp/test_plugin_config_" + std::to_string(getpid()) + ".yaml";
  {
    std::ofstream yaml(yaml_path);
    yaml << "test_plugin_config_node:\n"
         << "  ros__parameters:\n"
         << "    plugins.my_plugin.custom_key: \"custom_value\"\n"
         << "    plugins.my_plugin.mode: \"testing\"\n"
         << "    plugins.my_plugin.timeout: 42\n"
         << "    plugins.my_plugin.verbose: true\n"
         << "    plugins.my_plugin.threshold: 3.14\n";
  }

  // Init rclcpp with --params-file (simulates: ros2 run ... --ros-args --params-file ...)
  const char * args[] = {"test", "--ros-args", "--params-file", yaml_path.c_str()};
  rclcpp::init(4, const_cast<char **>(args));

  // Create a lightweight node (no HTTP server, no DDS subscriptions)
  auto node = std::make_shared<rclcpp::Node>("test_plugin_config_node");

  // BUG: NodeOptions::parameter_overrides() is empty for --params-file params
  const auto & overrides = node->get_node_options().parameter_overrides();
  EXPECT_EQ(overrides.size(), 0u) << "parameter_overrides() should be empty for --params-file YAML params "
                                  << "(this confirms the root cause of the bug)";

  // FIX: declare_plugin_params_from_yaml discovers and declares them
  ros2_medkit_gateway::declare_plugin_params_from_yaml(node.get(), "plugins.my_plugin.");

  // Verify all param types are correctly declared
  ASSERT_TRUE(node->has_parameter("plugins.my_plugin.custom_key"));
  EXPECT_EQ(node->get_parameter("plugins.my_plugin.custom_key").as_string(), "custom_value");

  ASSERT_TRUE(node->has_parameter("plugins.my_plugin.mode"));
  EXPECT_EQ(node->get_parameter("plugins.my_plugin.mode").as_string(), "testing");

  ASSERT_TRUE(node->has_parameter("plugins.my_plugin.timeout"));
  EXPECT_EQ(node->get_parameter("plugins.my_plugin.timeout").as_int(), 42);

  ASSERT_TRUE(node->has_parameter("plugins.my_plugin.verbose"));
  EXPECT_EQ(node->get_parameter("plugins.my_plugin.verbose").as_bool(), true);

  ASSERT_TRUE(node->has_parameter("plugins.my_plugin.threshold"));
  EXPECT_DOUBLE_EQ(node->get_parameter("plugins.my_plugin.threshold").as_double(), 3.14);

  node.reset();
  rclcpp::shutdown();
  std::remove(yaml_path.c_str());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
