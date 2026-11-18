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

#include <algorithm>
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
#include "ros2_medkit_param_beacon/param_beacon_plugin.hpp"

using ros2_medkit_gateway::PluginEntityInfo;
using ros2_medkit_gateway::PluginRequest;
using ros2_medkit_gateway::PluginResponse;
using ros2_medkit_gateway::RosPluginContext;
using ros2_medkit_gateway::SovdEntityType;

// Stubs for PluginRequest/PluginResponse (implemented in gateway_core, not linked into tests)
namespace ros2_medkit_gateway {
PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t /*index*/) const {
  return {};
}
std::string PluginRequest::header(const std::string & /*name*/) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
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

class FakeContext : public RosPluginContext {
 public:
  explicit FakeContext(rclcpp::Node * node) : node_(node) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }
  std::optional<PluginEntityInfo> get_entity(const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }
  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string & /*entity_id*/) const override {
    return nlohmann::json::array();
  }
  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & /*res*/,
                                                            const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }
  void register_capability(SovdEntityType type, const std::string & name) override {
    registered_capabilities_.push_back({type, name});
  }
  void register_entity_capability(const std::string & /*entity_id*/, const std::string & /*name*/) override {
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType /*type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  ros2_medkit_gateway::LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                                                   const std::string & /*collection*/) const override {
    return ros2_medkit_gateway::LockAccessResult{true, "", "", ""};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError>
  acquire_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
               const std::vector<std::string> & /*scopes*/, int /*expiration_seconds*/) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{"lock-disabled", "Not available", 503, std::nullopt});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string & /*entity_id*/,
                                                                  const std::string & /*client_id*/) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{"lock-disabled", "Not available", 503, std::nullopt});
  }
  ros2_medkit_gateway::ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ros2_medkit_gateway::ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
  struct CapReg {
    SovdEntityType type;
    std::string name;
  };
  std::vector<CapReg> registered_capabilities_;

 private:
  rclcpp::Node * node_;
};

}  // namespace

TEST(ParamBeaconPrivateContext, ParameterNodeTakesTheGatewayContext) {
  ASSERT_FALSE(rclcpp::ok()) << "the default context must stay uninitialised";
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto gateway_node = std::make_shared<rclcpp::Node>("private_context_gateway", rclcpp::NodeOptions().context(context));

  {
    ParameterBeaconPlugin plugin;
    FakeContext ctx(gateway_node.get());
    plugin.configure(nlohmann::json::object());
    // The gateway's PluginManager disables a plugin whose set_context() throws.
    EXPECT_NO_THROW(plugin.set_context(ctx));
    // The plugin's parameter node joins the gateway's graph.
    bool found = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!found && std::chrono::steady_clock::now() < deadline) {
      const auto names = gateway_node->get_node_names();
      found = std::find(names.begin(), names.end(), "/_param_beacon_node") != names.end();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_TRUE(found) << "_param_beacon_node is not on the gateway's graph";
    EXPECT_NO_THROW(plugin.shutdown());
  }

  gateway_node.reset();
  context->shutdown("test done");
  EXPECT_FALSE(rclcpp::ok()) << "something initialised the default context";
}
