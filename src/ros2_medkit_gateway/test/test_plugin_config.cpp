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
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/param_utils.hpp"

namespace {

/// RAII guard that removes a temp YAML file when it goes out of scope so
/// aborted tests (ASSERT_* failures) do not leak files under /tmp.
class ScopedYamlFile {
 public:
  ScopedYamlFile(std::string path, const std::string & content) : path_(std::move(path)) {
    std::ofstream yaml(path_);
    yaml << content;
  }
  ~ScopedYamlFile() {
    std::remove(path_.c_str());
  }
  ScopedYamlFile(const ScopedYamlFile &) = delete;
  ScopedYamlFile & operator=(const ScopedYamlFile &) = delete;
  ScopedYamlFile(ScopedYamlFile &&) = delete;
  ScopedYamlFile & operator=(ScopedYamlFile &&) = delete;

  const std::string & path() const {
    return path_;
  }

 private:
  std::string path_;
};

}  // namespace

/// Proves the bug and validates the fix using a lightweight rclcpp::Node.
///
/// 1. Writes a YAML params file with plugin config
/// 2. Inits rclcpp with --params-file (production path)
/// 3. Creates a plain Node (NOT GatewayNode) with the matching name
/// 4. Verifies NodeOptions::parameter_overrides() is empty (the bug)
/// 5. Verifies declare_plugin_params_from_yaml() resolves the YAML values (the fix)
TEST(PluginConfig, YamlPluginParamsReachGateway) {
  // Write YAML with plugin config using the gateway node name. ScopedYamlFile
  // cleans up on scope exit so an ASSERT_* abort does not leak /tmp files.
  ScopedYamlFile yaml("/tmp/test_plugin_config_" + std::to_string(getpid()) + ".yaml",
                      "test_plugin_config_node:\n"
                      "  ros__parameters:\n"
                      "    plugins.my_plugin.custom_key: \"custom_value\"\n"
                      "    plugins.my_plugin.mode: \"testing\"\n"
                      "    plugins.my_plugin.timeout: 42\n"
                      "    plugins.my_plugin.verbose: true\n"
                      "    plugins.my_plugin.threshold: 3.14\n");

  // Init rclcpp with --params-file (simulates: ros2 run ... --ros-args --params-file ...)
  const char * args[] = {"test", "--ros-args", "--params-file", yaml.path().c_str()};
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

  // Verify list_parameters finds declared params with the correct prefix.
  // ROS 2 list_parameters uses "." as hierarchy separator.
  // Prefix WITHOUT trailing dot works; prefix WITH trailing dot returns nothing.
  auto result_no_dot = node->list_parameters({"plugins.my_plugin"}, 10);
  EXPECT_GE(result_no_dot.names.size(), 5u) << "list_parameters with prefix (no trailing dot) should find params";

  auto result_with_dot = node->list_parameters({"plugins.my_plugin."}, 10);
  EXPECT_EQ(result_with_dot.names.size(), 0u) << "list_parameters with trailing dot returns nothing - "
                                              << "extract_plugin_config must use prefix without trailing dot";

  node.reset();
  rclcpp::shutdown();
}

/// Verifies that YAML array params (string, bool, int, double) survive the
/// --params-file -> global rcl context -> declare_plugin_params_from_yaml path
/// and are retrievable as typed arrays. Includes an empty-string list element
/// case to lock in the null/empty substitution in the production code.
TEST(PluginConfig, YamlPluginArrayParamsReachGateway) {
  ScopedYamlFile yaml("/tmp/test_plugin_config_arrays_" + std::to_string(getpid()) + ".yaml",
                      "test_plugin_config_arrays_node:\n"
                      "  ros__parameters:\n"
                      "    plugins.arr_plugin.peer_urls: [\"http://host-a:8081\", \"http://host-b:8081\"]\n"
                      "    plugins.arr_plugin.labels: [\"\", \"mid\", \"\"]\n"
                      "    plugins.arr_plugin.enabled_flags: [true, false, true]\n"
                      "    plugins.arr_plugin.timeouts_ms: [100, 200, 300]\n"
                      "    plugins.arr_plugin.thresholds: [0.5, 1.5, 2.5]\n");

  const char * args[] = {"test", "--ros-args", "--params-file", yaml.path().c_str()};
  rclcpp::init(4, const_cast<char **>(args));

  auto node = std::make_shared<rclcpp::Node>("test_plugin_config_arrays_node");
  ros2_medkit_gateway::declare_plugin_params_from_yaml(node.get(), "plugins.arr_plugin.");

  ASSERT_TRUE(node->has_parameter("plugins.arr_plugin.peer_urls"));
  auto peer_urls = node->get_parameter("plugins.arr_plugin.peer_urls").as_string_array();
  ASSERT_EQ(peer_urls.size(), 2u);
  EXPECT_EQ(peer_urls[0], "http://host-a:8081");
  EXPECT_EQ(peer_urls[1], "http://host-b:8081");

  // Empty-string elements must round-trip as empty strings, keeping index
  // alignment. Exercises the `elem != nullptr ? elem : ""` branch in
  // declare_plugin_params_from_yaml (empty YAML strings are the closest
  // we can get to a null entry without building rcl_variant_t by hand).
  ASSERT_TRUE(node->has_parameter("plugins.arr_plugin.labels"));
  auto labels = node->get_parameter("plugins.arr_plugin.labels").as_string_array();
  ASSERT_EQ(labels.size(), 3u);
  EXPECT_EQ(labels[0], "");
  EXPECT_EQ(labels[1], "mid");
  EXPECT_EQ(labels[2], "");

  ASSERT_TRUE(node->has_parameter("plugins.arr_plugin.enabled_flags"));
  auto flags = node->get_parameter("plugins.arr_plugin.enabled_flags").as_bool_array();
  ASSERT_EQ(flags.size(), 3u);
  EXPECT_TRUE(flags[0]);
  EXPECT_FALSE(flags[1]);
  EXPECT_TRUE(flags[2]);

  ASSERT_TRUE(node->has_parameter("plugins.arr_plugin.timeouts_ms"));
  auto timeouts = node->get_parameter("plugins.arr_plugin.timeouts_ms").as_integer_array();
  ASSERT_EQ(timeouts.size(), 3u);
  EXPECT_EQ(timeouts[0], 100);
  EXPECT_EQ(timeouts[1], 200);
  EXPECT_EQ(timeouts[2], 300);

  ASSERT_TRUE(node->has_parameter("plugins.arr_plugin.thresholds"));
  auto thresholds = node->get_parameter("plugins.arr_plugin.thresholds").as_double_array();
  ASSERT_EQ(thresholds.size(), 3u);
  EXPECT_DOUBLE_EQ(thresholds[0], 0.5);
  EXPECT_DOUBLE_EQ(thresholds[1], 1.5);
  EXPECT_DOUBLE_EQ(thresholds[2], 2.5);

  node.reset();
  rclcpp::shutdown();
}

/// Prefix scoping + dotted-key flattening.
///
/// Two things get verified here that the positive-path tests do not:
///
/// 1. Params OUTSIDE the requested prefix must not be declared on the node
///    (guards the `rfind(prefix, 0) == 0` filter).
/// 2. Dotted-key YAML like `plugins.foo.nested.leaf: "value"` gets flattened
///    by rcl into one scalar parameter named verbatim - no intermediate
///    "plugins.foo.nested" parameter is created.
///
/// NOTE on the "unsupported type" else-branch in declare_plugin_params_from_yaml:
/// that branch fires only when rcl_variant_t has all *_value fields nullptr,
/// which rcl_yaml_param_parser never emits for well-formed YAML. Exercising
/// it would require constructing an rcl_variant_t by hand and injecting it,
/// which is out of scope for this helper's black-box tests.
TEST(PluginConfig, YamlPluginPrefixScopingAndDottedKeys) {
  ScopedYamlFile yaml("/tmp/test_plugin_config_scoping_" + std::to_string(getpid()) + ".yaml",
                      "test_plugin_config_scoping_node:\n"
                      "  ros__parameters:\n"
                      "    plugins.foo_plugin.scalar_ok: \"ok\"\n"
                      "    plugins.foo_plugin.nested.leaf: \"value\"\n"
                      "    plugins.bar_plugin.outside_prefix: \"nope\"\n");

  const char * args[] = {"test", "--ros-args", "--params-file", yaml.path().c_str()};
  rclcpp::init(4, const_cast<char **>(args));

  auto node = std::make_shared<rclcpp::Node>("test_plugin_config_scoping_node");
  ros2_medkit_gateway::declare_plugin_params_from_yaml(node.get(), "plugins.foo_plugin.");

  // In-prefix scalars (including dotted-key leaves) are declared.
  EXPECT_TRUE(node->has_parameter("plugins.foo_plugin.scalar_ok"));
  EXPECT_TRUE(node->has_parameter("plugins.foo_plugin.nested.leaf"));

  // No intermediate parameter is synthesized for the dotted path.
  EXPECT_FALSE(node->has_parameter("plugins.foo_plugin.nested"));

  // Out-of-prefix params must be skipped by the prefix filter.
  EXPECT_FALSE(node->has_parameter("plugins.bar_plugin.outside_prefix"));

  node.reset();
  rclcpp::shutdown();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
