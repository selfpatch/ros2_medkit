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

// @verifies REQ_INTEROP_098
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

#include <rcl/arguments.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/rclcpp.hpp>

namespace {

/// Replicate the fix logic from gateway_node.cpp for testability.
/// Declares plugin params from the global rcl context on a given node.
void declare_plugin_params_from_yaml(rclcpp::Node * node, const std::string & prefix) {
  auto rcl_ctx = node->get_node_base_interface()->get_context()->get_rcl_context();
  rcl_params_t * global_params = nullptr;
  if (rcl_arguments_get_param_overrides(&rcl_ctx->global_arguments, &global_params) != RCL_RET_OK ||
      global_params == nullptr) {
    return;
  }

  std::string node_name = node->get_name();
  std::string node_fqn = node->get_fully_qualified_name();
  for (size_t n = 0; n < global_params->num_nodes; ++n) {
    std::string yaml_node = global_params->node_names[n];
    if (yaml_node != node_name && yaml_node != node_fqn && yaml_node != "/**") {
      continue;
    }
    auto * node_p = &global_params->params[n];
    for (size_t p = 0; p < node_p->num_params; ++p) {
      std::string pname = node_p->parameter_names[p];
      if (pname.rfind(prefix, 0) == 0 && !node->has_parameter(pname)) {
        auto & val = node_p->parameter_values[p];
        try {
          if (val.string_value != nullptr) {
            node->declare_parameter(pname, std::string(val.string_value));
          } else if (val.bool_value != nullptr) {
            node->declare_parameter(pname, *val.bool_value);
          } else if (val.integer_value != nullptr) {
            node->declare_parameter(pname, static_cast<int64_t>(*val.integer_value));
          } else if (val.double_value != nullptr) {
            node->declare_parameter(pname, *val.double_value);
          }
        } catch (...) {
        }
      }
    }
  }

  rcl_yaml_node_struct_fini(global_params);
}

}  // namespace

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
  declare_plugin_params_from_yaml(node.get(), "plugins.my_plugin.");

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
