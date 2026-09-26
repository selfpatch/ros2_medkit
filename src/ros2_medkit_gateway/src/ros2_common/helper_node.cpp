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

#include "ros2_medkit_gateway/ros2_common/helper_node.hpp"

namespace ros2_medkit_gateway::ros2_common {

std::shared_ptr<rclcpp::Node> make_helper_node(rclcpp::Node & host, const std::string & suffix) {
  const std::string name = "_" + std::string{host.get_name()} + suffix;

  bool use_sim_time = false;
  host.get_parameter("use_sim_time", use_sim_time);

  rclcpp::NodeOptions options;
  options.context(host.get_node_base_interface()->get_context());
  // rcl reads node-local remap rules before global ones.
  options.arguments({"--ros-args", "-r", "__node:=" + name});
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", use_sim_time)});
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  return std::make_shared<rclcpp::Node>(name, host.get_namespace(), options);
}

}  // namespace ros2_medkit_gateway::ros2_common
