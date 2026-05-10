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

#pragma once

#include <memory>

#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"

namespace rclcpp {
class Node;
}

namespace ros2_medkit_gateway {

class FaultManager;
class GatewayNode;
class ResourceSamplerRegistry;

/**
 * @brief ROS-aware plugin context.
 *
 * Extends the middleware-neutral PluginContext with the rclcpp::Node *
 * accessor used by ROS-coupled plugins. Lives in gateway_ros2 so that
 * gateway_core stays free of any rclcpp coupling, even at the level of
 * forward declarations or virtual signatures. Plugins that need ROS
 * primitives (subscriptions, service clients, parameter access) take a
 * RosPluginContext * member; plugins that only consume the neutral
 * surface can keep a PluginContext * and never see rclcpp.
 */
class RosPluginContext : public PluginContext {
 public:
  /// Get the ROS 2 node pointer for subscriptions, service clients, etc.
  virtual rclcpp::Node * node() const = 0;
};

/**
 * @brief Cast helper for plugins.
 *
 * The gateway always passes a RosPluginContext through
 * GatewayPlugin::set_context, so this static_cast is safe for any
 * in-process plugin loaded by PluginManager. Out-of-process or
 * test-only PluginContext implementations that do not derive from
 * RosPluginContext must not be passed through this helper.
 */
inline RosPluginContext * as_ros_plugin_context(PluginContext & ctx) {
  return static_cast<RosPluginContext *>(&ctx);
}

/// Factory for the concrete gateway plugin context. Always returns a
/// RosPluginContext because the gateway is itself a ROS 2 node; callers
/// that only consume the neutral PluginContext API can hold the result
/// as a unique_ptr<PluginContext>.
std::unique_ptr<RosPluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager,
                                                              ResourceSamplerRegistry * sampler_registry = nullptr);

}  // namespace ros2_medkit_gateway
