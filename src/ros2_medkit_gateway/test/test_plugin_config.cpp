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
#include <vector>

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

/// RAII guard that guarantees rclcpp::shutdown() runs when it leaves scope,
/// including during stack unwinding after an ASSERT_* abort. Without it an
/// aborted test would return early, leave rclcpp initialized, and corrupt
/// later tests in the same process.
class ScopedRclcpp {
 public:
  explicit ScopedRclcpp(int argc = 0, char ** argv = nullptr) {
    rclcpp::init(argc, argv);
  }
  ~ScopedRclcpp() {
    rclcpp::shutdown();
  }
  ScopedRclcpp(const ScopedRclcpp &) = delete;
  ScopedRclcpp & operator=(const ScopedRclcpp &) = delete;
  ScopedRclcpp(ScopedRclcpp &&) = delete;
  ScopedRclcpp & operator=(ScopedRclcpp &&) = delete;
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
  ScopedRclcpp rclcpp_ctx(4, const_cast<char **>(args));

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
  ScopedRclcpp rclcpp_ctx(4, const_cast<char **>(args));

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
  ScopedRclcpp rclcpp_ctx(4, const_cast<char **>(args));

  auto node = std::make_shared<rclcpp::Node>("test_plugin_config_scoping_node");
  ros2_medkit_gateway::declare_plugin_params_from_yaml(node.get(), "plugins.foo_plugin.");

  // In-prefix scalars (including dotted-key leaves) are declared.
  EXPECT_TRUE(node->has_parameter("plugins.foo_plugin.scalar_ok"));
  EXPECT_TRUE(node->has_parameter("plugins.foo_plugin.nested.leaf"));

  // No intermediate parameter is synthesized for the dotted path.
  EXPECT_FALSE(node->has_parameter("plugins.foo_plugin.nested"));

  // Out-of-prefix params must be skipped by the prefix filter.
  EXPECT_FALSE(node->has_parameter("plugins.bar_plugin.outside_prefix"));
}

/// Locks the nested-reconstruction contract of extract_plugin_config().
///
/// Plugins read their config as NESTED JSON objects
/// (config.contains("native_alarms"), config["native_alarms"]["enabled"], ...).
/// The old code built a FLAT object keyed by the dotted string
/// (config["native_alarms.enabled"]), which no plugin ever matched, silently
/// breaking the whole plugins.<name>.* ROS-param surface for nested config.
///
/// This test drives the NodeOptions::parameter_overrides() source (source 1)
/// directly and asserts the reconstructed shape. It FAILS on the old flat code:
/// the flat code never creates the nested objects and instead leaves the dotted
/// keys as top-level strings.
TEST(PluginConfig, ExtractPluginConfigReconstructsNested) {
  ScopedRclcpp rclcpp_ctx;

  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      // 2-level nested
      rclcpp::Parameter("plugins.beckhoff_ads.native_alarms.enabled", true),
      // 3-level nested (severity_bands)
      rclcpp::Parameter("plugins.beckhoff_ads.native_alarms.severity_bands.warning", 200),
      rclcpp::Parameter("plugins.beckhoff_ads.native_alarms.severity_bands.error", 800),
      // sibling nested group
      rclcpp::Parameter("plugins.beckhoff_ads.discovery.enabled", true),
      // single-level scalar stays flat
      rclcpp::Parameter("plugins.beckhoff_ads.ams_net_id", std::string("192.168.56.1.1.1")),
      // array-valued leaf must survive as a JSON array
      rclcpp::Parameter("plugins.beckhoff_ads.trust_list_paths", std::vector<std::string>{"/a.der", "/b.der"}),
      // read-only .path must be excluded from the config object
      rclcpp::Parameter("plugins.beckhoff_ads.path", std::string("/opt/plugin.so")),
  });

  auto node = std::make_shared<rclcpp::Node>("test_extract_nested_node", opts);
  auto config = ros2_medkit_gateway::extract_plugin_config(node.get(), "beckhoff_ads");

  // 2-level nesting: {"native_alarms": {"enabled": true}}
  ASSERT_TRUE(config.contains("native_alarms")) << "native_alarms group missing (old flat code fails here)";
  ASSERT_TRUE(config["native_alarms"].is_object()) << "native_alarms must be a nested object, not a scalar";
  EXPECT_EQ(config["native_alarms"]["enabled"].get<bool>(), true);

  // 3-level nesting: {"native_alarms": {"severity_bands": {"warning": 200, "error": 800}}}
  ASSERT_TRUE(config["native_alarms"].contains("severity_bands"));
  ASSERT_TRUE(config["native_alarms"]["severity_bands"].is_object());
  EXPECT_EQ(config["native_alarms"]["severity_bands"]["warning"].get<int>(), 200);
  EXPECT_EQ(config["native_alarms"]["severity_bands"]["error"].get<int>(), 800);

  // Sibling nested group is independent.
  ASSERT_TRUE(config.contains("discovery"));
  ASSERT_TRUE(config["discovery"].is_object());
  EXPECT_TRUE(config["discovery"]["enabled"].get<bool>());

  // Single-level scalar remains flat (no regression for existing flat keys).
  ASSERT_TRUE(config.contains("ams_net_id"));
  EXPECT_EQ(config["ams_net_id"].get<std::string>(), "192.168.56.1.1.1");

  // Array leaf is preserved verbatim as a JSON array.
  ASSERT_TRUE(config.contains("trust_list_paths"));
  ASSERT_TRUE(config["trust_list_paths"].is_array());
  ASSERT_EQ(config["trust_list_paths"].size(), 2u);
  EXPECT_EQ(config["trust_list_paths"][0].get<std::string>(), "/a.der");
  EXPECT_EQ(config["trust_list_paths"][1].get<std::string>(), "/b.der");

  // .path is excluded, and is NOT smuggled in under a dotted key either.
  EXPECT_FALSE(config.contains("path"));

  // Regression guards: the OLD flat implementation produced these dotted
  // top-level keys. Their absence proves the reconstruction happened.
  EXPECT_FALSE(config.contains("native_alarms.enabled"));
  EXPECT_FALSE(config.contains("native_alarms.severity_bands.warning"));
  EXPECT_FALSE(config.contains("discovery.enabled"));
}

/// A key used as both a leaf and an intermediate must not crash and must
/// resolve to the nested object deterministically, regardless of the order the
/// two overrides are seen in.
TEST(PluginConfig, ExtractPluginConfigLeafIntermediateCollision) {
  ScopedRclcpp rclcpp_ctx;

  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      // Scalar leaf seen first, then a deeper key needing the same name as a group.
      rclcpp::Parameter("plugins.collide.discovery", true),
      rclcpp::Parameter("plugins.collide.discovery.enabled", true),
  });

  auto node = std::make_shared<rclcpp::Node>("test_extract_collision_node", opts);
  auto config = ros2_medkit_gateway::extract_plugin_config(node.get(), "collide");

  // The nested object wins; the scalar is discarded (with a warning).
  ASSERT_TRUE(config.contains("discovery"));
  ASSERT_TRUE(config["discovery"].is_object()) << "nested group must win the leaf/intermediate collision";
  EXPECT_TRUE(config["discovery"]["enabled"].get<bool>());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
