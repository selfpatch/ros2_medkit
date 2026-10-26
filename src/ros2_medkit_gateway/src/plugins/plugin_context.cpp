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

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/plugins/core_plugin_context.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

namespace ros2_medkit_gateway {

namespace {

CorePluginContextWiring node_wiring(GatewayNode * node, FaultManager * fault_manager,
                                    ResourceSamplerRegistry * sampler_registry) {
  CorePluginContextWiring wiring;
  wiring.fault_manager = fault_manager;
  wiring.sampler_registry = sampler_registry;
  wiring.notifier = [node] {
    return node->get_resource_change_notifier();
  };
  wiring.conditions = [node] {
    return node->get_condition_registry();
  };
  wiring.aggregation = [node] {
    return node->get_aggregation_manager();
  };
  // Synchronous: the caller's thread runs the whole discovery refresh. GatewayNode serializes concurrent
  // notifications with its own refresh mutex.
  wiring.on_entities_changed = [node](const EntityChangeScope & scope) {
    node->handle_entity_change_notification(scope);
  };
  wiring.warn = [node](const std::string & message) {
    RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
  };
  return wiring;
}

/// The core context with the node of this process.
class GatewayPluginContext : public CorePluginContext {
 public:
  GatewayPluginContext(GatewayNode * node, FaultManager * fault_manager, ResourceSamplerRegistry * sampler_registry)
    : CorePluginContext(*node, node_wiring(node, fault_manager, sampler_registry)), node_(node) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }

 private:
  GatewayNode * node_;
};

}  // namespace

std::unique_ptr<RosPluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager,
                                                              ResourceSamplerRegistry * sampler_registry) {
  return std::make_unique<GatewayPluginContext>(node, fault_manager, sampler_registry);
}

}  // namespace ros2_medkit_gateway
