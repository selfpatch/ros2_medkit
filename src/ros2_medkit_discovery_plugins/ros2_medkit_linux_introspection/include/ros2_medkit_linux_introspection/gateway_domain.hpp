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

#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <optional>
#include <string>

namespace ros2_medkit_linux_introspection {

/// Limit the PID lookups of cache to the ROS domain of the gateway node. A process whose
/// environ cannot be read is matched on its name alone, with one warning per PID served.
/// A process of another domain is skipped with a debug line. Without a ROS node there is
/// no domain to match, and the cache is left unchanged.
inline void match_gateway_domain(ros2_medkit_gateway::PluginContext & ctx, PidCache & cache,
                                 const std::string & plugin_name) {
  auto * node = ros2_medkit_gateway::as_ros_plugin_context(ctx)->node();
  if (node == nullptr) {
    return;
  }
  auto logger = node->get_logger().get_child(plugin_name);
  size_t domain_id = node->get_node_base_interface()->get_context()->get_domain_id();
  cache.set_domain_filter(
      domain_id,
      [logger](pid_t pid, const std::string & fqn) {
        RCLCPP_WARN(logger,
                    "Cannot read the environment of PID %d (%s): it is matched by node "
                    "name only, its ROS domain is not checked",
                    static_cast<int>(pid), fqn.c_str());
      },
      [logger, domain_id](pid_t pid, const std::string & fqn, std::optional<size_t> process_domain) {
        auto named = process_domain ? std::to_string(*process_domain) : std::string{"not a domain ID"};
        RCLCPP_DEBUG(logger, "PID %d (%s) is not matched: its ROS_DOMAIN_ID is %s, the gateway's domain is %zu",
                     static_cast<int>(pid), fqn.c_str(), named.c_str(), domain_id);
      });
}

}  // namespace ros2_medkit_linux_introspection
