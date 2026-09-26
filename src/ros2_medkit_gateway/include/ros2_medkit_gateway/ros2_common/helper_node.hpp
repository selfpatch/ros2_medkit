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
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway::ros2_common {

/**
 * @brief Create an in-process helper node of @p host, in the host's namespace and context.
 *
 * The helper is named `_<host name><suffix>`. The leading underscore hides it from
 * `ros2 node list` and from discovery. A node-local `__node` remap keeps a
 * process-wide `-r __node:=<name>` (added by launch_ros `Node(name=...)`) from
 * giving the helper the host's name. All other process arguments still apply.
 * `use_sim_time` is copied from the host, because a parameters file keyed by the
 * host's name does not match the helper.
 *
 * The helper has no parameter services and no parameter event publisher. A helper
 * spun only during its own requests would leave those services unanswered.
 */
std::shared_ptr<rclcpp::Node> make_helper_node(rclcpp::Node & host, const std::string & suffix);

}  // namespace ros2_medkit_gateway::ros2_common
