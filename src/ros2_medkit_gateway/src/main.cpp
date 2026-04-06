// Copyright 2025 bburda, mfaferek93
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

#include "ros2_medkit_gateway/gateway_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ros2_medkit_gateway::GatewayNode>();

  // MultiThreadedExecutor is required because cpp-httplib handler threads call
  // Node::create_generic_subscription() for topic sampling. SingleThreadedExecutor
  // does not synchronize external subscription creation with its internal iteration,
  // causing non-deterministic SIGSEGV on rolling. All gateway callbacks are already
  // protected by mutexes (EntityCache, LogManager, TriggerManager, etc.).
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
