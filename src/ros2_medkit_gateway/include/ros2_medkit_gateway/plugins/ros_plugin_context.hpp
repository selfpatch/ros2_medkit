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
 * accessor used by ROS-coupled plugins. The header only forward-declares
 * rclcpp::Node, so gateway_core includes it without any ROS include path.
 * Plugins that need ROS primitives (subscriptions, service clients,
 * parameter access) take a RosPluginContext * member; plugins that only
 * consume the neutral surface can keep a PluginContext * and never see rclcpp.
 *
 * Every context the gateway hands out is a RosPluginContext. On a host
 * without a ROS node (see CorePluginContext) node() returns nullptr, so a
 * plugin that needs ROS checks it and declines to start.
 */
class RosPluginContext : public PluginContext {
 public:
  /// Get the ROS 2 node pointer for subscriptions, service clients, etc.
  /// Null on a host without a ROS node.
  virtual rclcpp::Node * node() const = 0;
};

/**
 * @brief Cast helper for plugins.
 *
 * The gateway always passes a RosPluginContext through
 * GatewayPlugin::set_context, so this static_cast is safe for any
 * in-process plugin loaded by PluginManager, on the ROS 2 node and on a
 * host without one. Out-of-process or test-only PluginContext
 * implementations that do not derive from RosPluginContext must not be
 * passed through this helper.
 */
inline RosPluginContext * as_ros_plugin_context(PluginContext & ctx) {
  return static_cast<RosPluginContext *>(&ctx);
}

/// Factory for the plugin context of the ROS 2 gateway node. Its node() is the
/// gateway node. A host without a node uses make_core_plugin_context().
std::unique_ptr<RosPluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager,
                                                              ResourceSamplerRegistry * sampler_registry = nullptr);

}  // namespace ros2_medkit_gateway
