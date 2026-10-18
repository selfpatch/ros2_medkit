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

// A host can run the gateway on a private context and never initialise the default one.
// This binary must not call rclcpp::init().

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_graph_watchdog/graph_watchdog_plugin.hpp"

// The plugin's route handler calls these. They live in gateway_core, which this binary does not
// link. No request reaches the handler here.
namespace ros2_medkit_gateway {
PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json & /*data*/) {
}
void PluginResponse::send_error(int /*status*/, const std::string & /*error_code*/, const std::string & /*message*/,
                                const nlohmann::json & /*parameters*/) {
}
}  // namespace ros2_medkit_gateway

namespace {

class FakeContext : public ros2_medkit_gateway::RosPluginContext {
 public:
  explicit FakeContext(rclcpp::Node * node) : node_(node) {
  }
  rclcpp::Node * node() const override {
    return node_;
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo> get_entity(const std::string & /*id*/) const override {
    return std::nullopt;
  }
  std::vector<ros2_medkit_gateway::PluginEntityInfo>
  get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string & /*entity_id*/) const override {
    return nlohmann::json::array();
  }
  std::optional<ros2_medkit_gateway::PluginEntityInfo>
  validate_entity_for_route(const ros2_medkit_gateway::PluginRequest & /*req*/,
                            ros2_medkit_gateway::PluginResponse & /*res*/,
                            const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }
  void register_capability(ros2_medkit_gateway::SovdEntityType /*entity_type*/,
                           const std::string & /*capability_name*/) override {
  }
  void register_entity_capability(const std::string & /*entity_id*/, const std::string & /*capability_name*/) override {
  }
  std::vector<std::string> get_type_capabilities(ros2_medkit_gateway::SovdEntityType /*entity_type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  ros2_medkit_gateway::LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                                                   const std::string & /*collection*/) const override {
    return {};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError>
  acquire_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
               const std::vector<std::string> & /*scopes*/, int /*expiration_seconds*/) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string & /*entity_id*/,
                                                                  const std::string & /*client_id*/) override {
    return {};
  }
  ros2_medkit_gateway::ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ros2_medkit_gateway::ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }

 private:
  rclcpp::Node * node_;
};

}  // namespace

TEST(GraphWatchdogPrivateContext, PluginRunsOnTheGatewayContext) {
  ASSERT_FALSE(rclcpp::ok()) << "the default context must stay uninitialised";
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto gateway_node = std::make_shared<rclcpp::Node>("private_context_gateway", rclcpp::NodeOptions().context(context));

  rclcpp::ExecutorOptions exec_options;
  exec_options.context = context;
  auto exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_options);
  exec->add_node(gateway_node);
  std::thread spin([&exec] {
    exec->spin();
  });
  // A cancel() that lands before spin() starts is lost, and join() would block.
  while (!exec->is_spinning()) {
    std::this_thread::yield();
  }

  {
    ros2_medkit_graph_watchdog::GraphWatchdogPlugin plugin;
    FakeContext ctx(gateway_node.get());
    plugin.configure(nlohmann::json{{"tick_interval_ms", 50}});
    // The gateway's PluginManager disables a plugin whose set_context() throws.
    bool attached = false;
    EXPECT_NO_THROW({
      plugin.set_context(ctx);
      attached = true;
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (attached && plugin.tick_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_GT(plugin.tick_count(), 0u);
    EXPECT_NO_THROW(plugin.shutdown());
  }

  exec->cancel();
  spin.join();
  exec.reset();
  gateway_node.reset();
  context->shutdown("test done");
  EXPECT_FALSE(rclcpp::ok()) << "something initialised the default context";
}
