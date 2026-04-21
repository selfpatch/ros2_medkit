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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ros2_medkit_gateway::GatewayNode>();

  // MultiThreadedExecutor is required: the Ros2SubscriptionExecutor's
  // subscription_node lives on the same executor as the gateway node, and
  // parallel dispatch prevents the serial worker and the main executor from
  // starving each other.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Stand up the ROS 2 subscription executor + topic data provider.
  // Issue #375: all subscription create/destroy calls are funneled through the
  // serial worker owned by sub_exec, eliminating the rcl hash-map race that
  // previously killed /data on Rolling when concurrent HTTP handler threads
  // created subscriptions on the same node.
  auto sub_exec = std::make_shared<ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor>(node, executor);
  auto serializer = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
  auto data_provider = std::make_shared<ros2_medkit_gateway::Ros2TopicDataProvider>(sub_exec, serializer);
  node->set_topic_data_provider(data_provider);

  executor.spin();

  // Teardown order (issue #375): stack-unwind destructs executor before node
  // which leaves ~GatewayNode running against a dead executor. Newer rclcpp
  // (rolling; recent jazzy patch releases) asserts 'node needs to be
  // associated with an executor' and aborts with exit -6 when the managers'
  // shutdown paths touch service clients. Explicit teardown avoids that:
  //   1. drop the provider (clears pool entries via the subscription worker)
  //   2. reset sub_exec (joins worker, removes subscription_node from executor)
  //   3. remove the gateway node from the executor and drop our ref so
  //      ~GatewayNode runs with the executor still alive.
  data_provider.reset();
  sub_exec.reset();
  executor.remove_node(node);
  node.reset();

  rclcpp::shutdown();
  return 0;
}
