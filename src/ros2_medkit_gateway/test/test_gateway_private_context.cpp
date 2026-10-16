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

// A host process can embed the gateway on a private rclcpp::Context and never
// initialise the default one. Only a host process can do that, so this test is
// in-process: the shipped gateway always initialises the default context.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/ros2/status/ros2_lifecycle_state_reader.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_msgs/srv/list_faults.hpp"

using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::Ros2LifecycleStateReader;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

namespace {

int reserve_local_port() {
  const int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ADD_FAILURE() << "socket: " << std::strerror(errno);
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  socklen_t addr_len = sizeof(addr);
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
      getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    ADD_FAILURE() << "bind/getsockname: " << std::strerror(errno);
    close(sock);
    return 0;
  }
  close(sock);
  return ntohs(addr.sin_port);
}

}  // namespace

TEST(GatewayPrivateContext, RosPathsWorkWithoutTheDefaultContext) {
  ASSERT_FALSE(rclcpp::ok()) << "the default context must stay uninitialised";
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const auto on_context = rclcpp::NodeOptions().context(context);

  // Peer on the same context: a fault manager, a managed node with a parameter, and a talker.
  auto peer = std::make_shared<rclcpp::Node>("private_context_peer", on_context);
  peer->declare_parameter("private_context_answer", 42);
  auto list_faults = peer->create_service<ros2_medkit_msgs::srv::ListFaults>(
      "/fault_manager/list_faults",
      [](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & /*req*/,
         const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & /*res*/) {});
  auto get_state = peer->create_service<lifecycle_msgs::srv::GetState>(
      "/private_context_peer/get_state", [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*req*/,
                                            const std::shared_ptr<lifecycle_msgs::srv::GetState::Response> & res) {
        res->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        res->current_state.label = "active";
      });
  auto talker = peer->create_publisher<std_msgs::msg::String>("/private_context_topic", 10);
  rclcpp::ExecutorOptions peer_exec_options;
  peer_exec_options.context = context;
  auto peer_exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(peer_exec_options);
  peer_exec->add_node(peer);
  std::thread peer_spin([&peer_exec] {
    peer_exec->spin();
  });
  // A cancel() that lands before spin() starts is lost, and join() would block.
  while (!peer_exec->is_spinning()) {
    std::this_thread::yield();
  }

  rclcpp::NodeOptions options = on_context;
  options.append_parameter_override("server.host", std::string("127.0.0.1"));
  options.append_parameter_override("server.port", reserve_local_port());
  options.append_parameter_override("refresh_interval_ms", 60000);
  std::shared_ptr<GatewayNode> gateway;
  try {
    gateway = std::make_shared<GatewayNode>(options);
  } catch (const std::exception & e) {
    ADD_FAILURE() << "GatewayNode construction threw: " << e.what();
  }

  if (gateway) {
    // Fault-service transport, built by the GatewayNode constructor.
    const auto listed = gateway->get_fault_manager()->list_faults();
    EXPECT_TRUE(listed.success) << "list_faults: " << listed.error_message;

    // Parameter transport (_param_client_node), built by the GatewayNode constructor.
    const auto param =
        gateway->get_configuration_manager()->get_parameter("/private_context_peer", "private_context_answer");
    EXPECT_TRUE(param.success) << "get_parameter: " << param.error_message;
    EXPECT_EQ(param.data.value("value", nlohmann::json()), 42) << param.data.dump();

    // Lifecycle-state reader with the gateway as host, as the REST server builds it.
    std::optional<std::string> state;
    try {
      Ros2LifecycleStateReader reader(gateway.get(), 5s);
      state = reader.get_state("/private_context_peer/get_state");
    } catch (const std::exception & e) {
      ADD_FAILURE() << "lifecycle-state reader threw: " << e.what();
    }
    EXPECT_EQ(state, std::optional<std::string>("active"));

    // Subscription executor with the gateway as host, as main() builds it.
    std::atomic<bool> received{false};
    try {
      Ros2SubscriptionExecutor sub_exec(gateway, Ros2SubscriptionExecutor::Config{});
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
      const auto created = sub_exec.run_sync<bool>([&] {
        sub = sub_exec.node()->create_subscription<std_msgs::msg::String>(
            "/private_context_topic", 10, [&received](const std_msgs::msg::String & /*msg*/) {
              received.store(true);
            });
        return true;
      });
      EXPECT_TRUE(created.has_value()) << (created ? "" : created.error());
      const auto deadline = std::chrono::steady_clock::now() + 10s;
      while (!received.load() && std::chrono::steady_clock::now() < deadline) {
        talker->publish(std_msgs::msg::String().set__data("ping"));
        std::this_thread::sleep_for(50ms);
      }
      static_cast<void>(sub_exec.run_sync<bool>([&sub] {
        sub.reset();
        return true;
      }));
    } catch (const std::exception & e) {
      ADD_FAILURE() << "subscription executor threw: " << e.what();
    }
    EXPECT_TRUE(received.load()) << "the subscription node never dispatched a message";
  }

  peer_exec->cancel();
  peer_spin.join();
  peer_exec.reset();
  talker.reset();
  get_state.reset();
  list_faults.reset();
  peer.reset();
  gateway.reset();
  context->shutdown("test done");
  EXPECT_FALSE(rclcpp::ok()) << "something initialised the default context";
}
