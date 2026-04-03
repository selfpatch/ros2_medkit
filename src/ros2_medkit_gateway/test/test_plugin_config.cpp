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
/// Tests that plugin configuration from --params-file YAML reaches the plugin.
///
/// The bug: extract_plugin_config() reads from get_node_options().parameter_overrides(),
/// which only contains programmatically-set overrides. Parameters from --params-file
/// go into the ROS 2 global rcl context and are NOT copied to NodeOptions::parameter_overrides_.
/// As a result, plugins always receive empty config and use defaults.

#include <gtest/gtest.h>
#include <httplib.h>  // NOLINT(build/include_order)

#include <arpa/inet.h>
#include <cstdio>
#include <fstream>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/gateway_node.hpp"

namespace {

int reserve_free_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(sock);
    return 0;
  }
  socklen_t len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<struct sockaddr *>(&addr), &len) < 0) {
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

std::string test_plugin_path() {
  return ament_index_cpp::get_package_prefix("ros2_medkit_gateway") +
         "/lib/ros2_medkit_gateway/libtest_gateway_plugin.so";
}

}  // namespace

/// Proves that plugin config params from --params-file are accessible to the gateway.
///
/// This test simulates production usage: rclcpp::init with --params-file,
/// then GatewayNode created with only a port override (no plugin config in
/// NodeOptions::parameter_overrides). The YAML file contains plugin config
/// that should reach the plugin's configure() method.
TEST(PluginConfig, YamlPluginParamsReachGateway) {
  int free_port = reserve_free_port();
  ASSERT_NE(free_port, 0) << "Failed to reserve a free port";

  // Write YAML with plugin config
  std::string yaml_path = "/tmp/test_plugin_config_" + std::to_string(getpid()) + ".yaml";
  {
    std::ofstream yaml(yaml_path);
    yaml << "ros2_medkit_gateway:\n"
         << "  ros__parameters:\n"
         << "    server:\n"
         << "      host: \"127.0.0.1\"\n"
         << "      port: " << free_port << "\n"
         << "    plugins: [\"test_plugin\"]\n"
         << "    plugins.test_plugin.path: \"" << test_plugin_path() << "\"\n"
         << "    plugins.test_plugin.custom_key: \"custom_value\"\n"
         << "    plugins.test_plugin.mode: \"testing\"\n"
         << "    plugins.test_plugin.nested.setting: 42\n";
  }

  // Init rclcpp with --params-file (simulates production: ros2 run ... --ros-args --params-file ...)
  const char * args[] = {"test_plugin_config", "--ros-args", "--params-file", yaml_path.c_str()};
  rclcpp::init(4, const_cast<char **>(args));

  // Create node WITHOUT plugin config in parameter_overrides (simulates main.cpp)
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ros2_medkit_gateway::GatewayNode>(options);

  // Verify: parameter_overrides() should contain the plugin params.
  // BUG: --params-file YAML params go to the global rcl context,
  //      NOT to NodeOptions::parameter_overrides().
  const auto & overrides = node->get_node_options().parameter_overrides();
  bool found_plugin_config = false;
  for (const auto & p : overrides) {
    if (p.get_name().rfind("plugins.test_plugin.", 0) == 0 && p.get_name() != "plugins.test_plugin.path") {
      found_plugin_config = true;
      break;
    }
  }

  // This assertion proves the bug: YAML plugin params are NOT in parameter_overrides
  EXPECT_TRUE(found_plugin_config)
      << "Plugin config from --params-file not found in parameter_overrides(). "
      << "extract_plugin_config() reads from this source, so plugins receive empty config. "
      << "Total overrides: " << overrides.size();

  // Sanity check: the params DO exist in the global context (they're just not in overrides).
  // Declaring them proves the YAML was parsed correctly.
  try {
    node->declare_parameter("plugins.test_plugin.custom_key", std::string("default"));
    auto val = node->get_parameter("plugins.test_plugin.custom_key").as_string();
    EXPECT_EQ(val, "custom_value") << "YAML param exists in global context but not in parameter_overrides()";
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    // If already declared (e.g. by a fix), that's fine - get it directly
    auto val = node->get_parameter("plugins.test_plugin.custom_key").as_string();
    EXPECT_EQ(val, "custom_value");
  }

  node.reset();
  rclcpp::shutdown();
  std::remove(yaml_path.c_str());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
