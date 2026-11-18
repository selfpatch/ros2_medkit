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

// In-process because a separate process misses the events the gateway sends
// while its constructor declares: its subscription matches only after DDS
// discovery. One rclcpp::Context shares one participant, which matches local
// endpoints on creation, so a probe built first sees every event.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/gateway_node.hpp"

using ros2_medkit_gateway::GatewayNode;
using std::chrono_literals::operator""ms;

namespace {

constexpr const char * kJwtSecret = "canary_file_jwt_secret_zz_at_least_32_chars";
constexpr const char * kClientSecret = "canary_client_secret_zz";
constexpr const char * kPeerHeader = "Bearer canary_peer_header_zz";
constexpr const char * kPluginSecret = "canary_plugin_password_zz";
constexpr const char * kPluginNestedSecret = "canary_plugin_nested_token_zz";
constexpr const char * kJwtSentinel = "<set at start>";
constexpr const char * kNodeName = "ros2_medkit_gateway";
constexpr const char * kMarker = "zz_events_flushed_marker";

int reserve_local_port() {
  const int sock = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  socklen_t len = sizeof(addr);
  if (sock < 0 || bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
      getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
    if (sock >= 0) {
      close(sock);
    }
    return 0;
  }
  close(sock);
  return ntohs(addr.sin_port);
}

class ScopedFile {
 public:
  ScopedFile(std::string path, const std::string & content) : path_(std::move(path)) {
    std::ofstream out(path_);
    out << content;
  }
  ~ScopedFile() {
    std::remove(path_.c_str());
  }
  ScopedFile(const ScopedFile &) = delete;
  ScopedFile & operator=(const ScopedFile &) = delete;
  ScopedFile(ScopedFile &&) = delete;
  ScopedFile & operator=(ScopedFile &&) = delete;
  const std::string & path() const {
    return path_;
  }

 private:
  std::string path_;
};

}  // namespace

TEST(SecretsNotOnParameterEvents, NoSecretValueIsPublished) {
  const std::string plugin_so =
      ament_index_cpp::get_package_prefix("ros2_medkit_gateway") + "/lib/ros2_medkit_gateway/libtest_gateway_plugin.so";
  const int port = reserve_local_port();
  ASSERT_NE(port, 0);

  ScopedFile params("/tmp/test_secrets_not_on_parameter_events_" + std::to_string(getpid()) + ".yaml",
                    std::string(kNodeName) +
                        ":\n"
                        "  ros__parameters:\n"
                        "    server.host: \"127.0.0.1\"\n"
                        "    server.port: " +
                        std::to_string(port) +
                        "\n"
                        "    refresh_interval_ms: 60000\n"
                        "    auth.enabled: true\n"
                        "    auth.jwt_secret: \"" +
                        kJwtSecret +
                        "\"\n"
                        "    auth.clients: [\"canaryclient:" +
                        kClientSecret +
                        ":admin\"]\n"
                        "    aggregation.peer_auth_header: \"" +
                        kPeerHeader +
                        "\"\n"
                        "    plugins: [\"gwtest\"]\n"
                        "    plugins.gwtest.path: \"" +
                        plugin_so +
                        "\"\n"
                        "    plugins.gwtest.password: \"" +
                        kPluginSecret +
                        "\"\n"
                        "    plugins.gwtest.nested.token: \"" +
                        kPluginNestedSecret + "\"\n");

  auto context = std::make_shared<rclcpp::Context>();
  const char * args[] = {"test", "--ros-args", "--params-file", params.path().c_str()};
  context->init(4, args);
  const auto on_context = rclcpp::NodeOptions().context(context);

  // The probe subscribes before the gateway exists.
  auto probe = std::make_shared<rclcpp::Node>("secrets_events_probe", on_context);
  std::vector<std::string> events;
  bool jwt_declared_with_sentinel = false;
  bool marker_seen = false;
  auto subscription = probe->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", rclcpp::ParameterEventsQoS(), [&](const rcl_interfaces::msg::ParameterEvent & event) {
        if (event.node != std::string("/") + kNodeName) {
          return;
        }
        events.push_back(rcl_interfaces::msg::to_yaml(event));
        for (const auto & p : event.new_parameters) {
          if (p.name == "auth.jwt_secret" && p.value.string_value == kJwtSentinel) {
            jwt_declared_with_sentinel = true;
          }
          if (p.name == kMarker) {
            marker_seen = true;
          }
        }
      });

  rclcpp::ExecutorOptions exec_options;
  exec_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(exec_options);
  executor.add_node(probe);

  auto gateway = std::make_shared<GatewayNode>(on_context);
  // Published by the gateway's own publisher after every constructor event, so
  // seeing it means the earlier events have been delivered.
  gateway->declare_parameter(kMarker, 1);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!marker_seen && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(100ms);
  }
  ASSERT_TRUE(marker_seen) << "the marker event never arrived; the probe saw " << events.size() << " event(s)";

  EXPECT_TRUE(jwt_declared_with_sentinel) << "no event declared auth.jwt_secret with its sentinel";
  ASSERT_FALSE(events.empty());
  for (const char * canary : {kJwtSecret, kClientSecret, kPeerHeader, kPluginSecret, kPluginNestedSecret}) {
    for (const auto & dump : events) {
      EXPECT_EQ(dump.find(canary), std::string::npos)
          << "a secret value was published on /parameter_events: " << canary;
    }
  }

  executor.remove_node(probe);
  gateway.reset();
  context->shutdown("test done");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
