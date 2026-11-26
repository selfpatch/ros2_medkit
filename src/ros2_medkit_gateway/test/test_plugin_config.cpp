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

/// Tests that plugin config reaches the gateway plugin framework from every
/// source (--params-file, NodeOptions::parameter_overrides) and that it is not
/// declared as node parameters, so the parameter services do not serve it.

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

/// Scalars of every type from --params-file reach the config and are not declared.
TEST(PluginConfig, YamlPluginParamsReachGateway) {
  ScopedYamlFile yaml("/tmp/test_plugin_config_" + std::to_string(getpid()) + ".yaml",
                      "test_plugin_config_node:\n"
                      "  ros__parameters:\n"
                      "    plugins.my_plugin.custom_key: \"custom_value\"\n"
                      "    plugins.my_plugin.mode: \"testing\"\n"
                      "    plugins.my_plugin.timeout: 42\n"
                      "    plugins.my_plugin.verbose: true\n"
                      "    plugins.my_plugin.threshold: 3.14\n");

  const char * args[] = {"test", "--ros-args", "--params-file", yaml.path().c_str()};
  ScopedRclcpp rclcpp_ctx(4, const_cast<char **>(args));

  auto node = std::make_shared<rclcpp::Node>("test_plugin_config_node");

  // A params file does not land in NodeOptions::parameter_overrides().
  EXPECT_EQ(node->get_node_options().parameter_overrides().size(), 0u);

  auto config = ros2_medkit_gateway::extract_plugin_config(node.get(), "my_plugin");
  EXPECT_EQ(config.size(), 5u);
  EXPECT_EQ(config.value("custom_key", ""), "custom_value");
  EXPECT_EQ(config.value("mode", ""), "testing");
  ASSERT_TRUE(config.contains("timeout"));
  EXPECT_EQ(config["timeout"].get<int64_t>(), 42);
  ASSERT_TRUE(config.contains("verbose"));
  EXPECT_EQ(config["verbose"].get<bool>(), true);
  ASSERT_TRUE(config.contains("threshold"));
  EXPECT_DOUBLE_EQ(config["threshold"].get<double>(), 3.14);

  // Reading the config declares nothing.
  EXPECT_FALSE(node->has_parameter("plugins.my_plugin.custom_key"));
  EXPECT_FALSE(node->has_parameter("plugins.my_plugin.verbose"));
  EXPECT_TRUE(node->list_parameters({"plugins.my_plugin"}, 10).names.empty());
}

/// YAML array params (string, bool, int, double) reach the config as typed JSON
/// arrays. An empty-string element keeps its index.
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
  auto config = ros2_medkit_gateway::extract_plugin_config(node.get(), "arr_plugin");

  EXPECT_EQ(config["peer_urls"], (nlohmann::json::array({"http://host-a:8081", "http://host-b:8081"})));
  EXPECT_EQ(config["labels"], (nlohmann::json::array({"", "mid", ""})));
  EXPECT_EQ(config["enabled_flags"], (nlohmann::json::array({true, false, true})));
  EXPECT_EQ(config["timeouts_ms"], (nlohmann::json::array({100, 200, 300})));
  EXPECT_EQ(config["thresholds"], (nlohmann::json::array({0.5, 1.5, 2.5})));
  EXPECT_FALSE(node->has_parameter("plugins.arr_plugin.peer_urls"));
}

/// Only the requested plugin's keys are read, and a dotted key is un-nested.
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
  auto config = ros2_medkit_gateway::extract_plugin_config(node.get(), "foo_plugin");

  EXPECT_EQ(config.value("scalar_ok", ""), "ok");
  ASSERT_TRUE(config.contains("nested"));
  EXPECT_EQ(config["nested"].value("leaf", ""), "value");
  EXPECT_FALSE(config.contains("outside_prefix"));
  EXPECT_EQ(config.size(), 2u);
  EXPECT_FALSE(node->has_parameter("plugins.foo_plugin.scalar_ok"));
  EXPECT_FALSE(node->has_parameter("plugins.bar_plugin.outside_prefix"));
}

/// Nested reconstruction from NodeOptions::parameter_overrides(): dotted keys
/// become nested objects, single-level keys stay flat, arrays stay arrays.
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
  ASSERT_TRUE(config.contains("native_alarms")) << "native_alarms group missing ";
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

  // Dotted top-level keys would mean the reconstruction did not happen.
  EXPECT_FALSE(config.contains("native_alarms.enabled"));
  EXPECT_FALSE(config.contains("native_alarms.severity_bands.warning"));
  EXPECT_FALSE(config.contains("discovery.enabled"));
}

/// Nested reconstruction from --params-file, the production source.
TEST(PluginConfig, ExtractPluginConfigReconstructsNestedFromYaml) {
  ScopedYamlFile yaml("/tmp/test_plugin_config_nested_yaml_" + std::to_string(getpid()) + ".yaml",
                      "test_extract_nested_yaml_node:\n"
                      "  ros__parameters:\n"
                      "    plugins.opcua.native_alarms.enabled: true\n"
                      "    plugins.opcua.native_alarms.severity_bands.warning: 200\n"
                      "    plugins.opcua.native_alarms.severity_bands.error: 800\n"
                      "    plugins.opcua.discovery.enabled: true\n"
                      "    plugins.opcua.endpoint: \"opc.tcp://host:4840\"\n"
                      "    plugins.opcua.trust_list_paths: [\"/a.der\", \"/b.der\"]\n"
                      "    plugins.opcua.path: \"/opt/plugin.so\"\n");

  const char * args[] = {"test", "--ros-args", "--params-file", yaml.path().c_str()};
  ScopedRclcpp rclcpp_ctx(4, const_cast<char **>(args));

  auto node = std::make_shared<rclcpp::Node>("test_extract_nested_yaml_node");
  ASSERT_EQ(node->get_node_options().parameter_overrides().size(), 0u)
      << "the keys must come from the params file only";

  auto config = ros2_medkit_gateway::extract_plugin_config(node.get(), "opcua");

  // 2-level nesting: {"native_alarms": {"enabled": true}}
  ASSERT_TRUE(config.contains("native_alarms")) << "native_alarms group missing from the YAML path";
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

  // Single-level scalar stays flat.
  ASSERT_TRUE(config.contains("endpoint"));
  EXPECT_EQ(config["endpoint"].get<std::string>(), "opc.tcp://host:4840");

  // Array leaf survives as a JSON array through the YAML path.
  ASSERT_TRUE(config.contains("trust_list_paths"));
  ASSERT_TRUE(config["trust_list_paths"].is_array());
  ASSERT_EQ(config["trust_list_paths"].size(), 2u);
  EXPECT_EQ(config["trust_list_paths"][0].get<std::string>(), "/a.der");
  EXPECT_EQ(config["trust_list_paths"][1].get<std::string>(), "/b.der");

  // .path is excluded from the config object.
  EXPECT_FALSE(config.contains("path"));

  // No dotted top-level keys leaked.
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

/// Reverse of the collision test above, on insert_nested_param directly: the
/// overrides arrive sorted, so a nested key never precedes its scalar there. A
/// scalar arriving where a nested object already exists is dropped.
TEST(PluginConfig, ExtractPluginConfigLeafIntermediateCollisionReversed) {
  ScopedRclcpp rclcpp_ctx;

  nlohmann::json config = nlohmann::json::object();
  const auto logger = rclcpp::get_logger("test_plugin_config");
  ros2_medkit_gateway::insert_nested_param(config, "discovery.enabled", true, logger);
  ros2_medkit_gateway::insert_nested_param(config, "discovery", true, logger);

  ASSERT_TRUE(config.contains("discovery"));
  ASSERT_TRUE(config["discovery"].is_object()) << "nested group must survive the reversed collision";
  EXPECT_TRUE(config["discovery"]["enabled"].get<bool>());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
