// Copyright 2025-2026 bburda, mfaferek93
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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace {

// Declare operator-tunable parameters for Ros2SubscriptionExecutor and
// Ros2TopicDataProvider. Every knob is clamped to a conservative range
// because a runaway value here (e.g. queue_depth=1e9) can starve the gateway
// process for memory. Defaults mirror the Config{} constructors so the stock
// deployment behaviour is unchanged.
ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor::Config declare_executor_config(rclcpp::Node & node) {
  ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor::Config cfg;
  const auto queue_depth = node.declare_parameter<int64_t>("subscription_executor.max_queue_depth", 256);
  cfg.max_queue_depth = static_cast<std::size_t>(std::clamp<int64_t>(queue_depth, 16, 4096));
  const auto watchdog_ms = node.declare_parameter<int64_t>("subscription_executor.watchdog_threshold_ms", 5000);
  cfg.watchdog_threshold = std::chrono::milliseconds{std::clamp<int64_t>(watchdog_ms, 100, 60000)};
  const auto watchdog_tick = node.declare_parameter<int64_t>("subscription_executor.watchdog_tick_ms", 1000);
  cfg.watchdog_tick = std::chrono::milliseconds{std::clamp<int64_t>(watchdog_tick, 10, 10000)};
  const auto graph_tick = node.declare_parameter<int64_t>("subscription_executor.graph_poll_tick_ms", 100);
  cfg.graph_poll_tick = std::chrono::milliseconds{std::clamp<int64_t>(graph_tick, 10, 10000)};
  return cfg;
}

ros2_medkit_gateway::Ros2TopicDataProvider::Config declare_data_provider_config(rclcpp::Node & node) {
  ros2_medkit_gateway::Ros2TopicDataProvider::Config cfg;
  const auto pool_size = node.declare_parameter<int64_t>("data_provider.max_pool_size", 256);
  cfg.max_pool_size = static_cast<std::size_t>(std::clamp<int64_t>(pool_size, 1, 4096));
  const auto cold_cap = node.declare_parameter<int64_t>("data_provider.cold_wait_cap", 4);
  cfg.cold_wait_cap = static_cast<std::size_t>(std::clamp<int64_t>(cold_cap, 0, 1024));
  const auto max_parallel = node.declare_parameter<int64_t>("data_provider.max_parallel_samples", 8);
  cfg.max_parallel_samples = static_cast<std::size_t>(std::clamp<int64_t>(max_parallel, 1, 256));
  const auto idle_safety_s = node.declare_parameter<int64_t>("data_provider.idle_safety_net_sec", 15 * 60);
  cfg.idle_safety_net = std::chrono::seconds{std::clamp<int64_t>(idle_safety_s, 0, 24 * 3600)};
  const auto idle_sweep_s = node.declare_parameter<int64_t>("data_provider.idle_sweep_tick_sec", 60);
  cfg.idle_sweep_tick = std::chrono::seconds{std::clamp<int64_t>(idle_sweep_s, 0, 3600)};
  return cfg;
}

}  // namespace

int main(int argc, char ** argv) {
  bool ros_inited = false;
  try {
    rclcpp::init(argc, argv);
    ros_inited = true;

    auto node = std::make_shared<ros2_medkit_gateway::GatewayNode>();

    // MultiThreadedExecutor for the gateway node - HTTP handlers run on several
    // threads, so the main executor must dispatch callbacks in parallel to avoid
    // starving slow handlers. The Ros2SubscriptionExecutor built below owns its
    // own internal single-threaded executor (spun from its worker thread); the
    // subscription node is intentionally not added here.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Stand up the ROS 2 subscription executor + topic data provider.
    // Issue #375: all subscription create/destroy calls are funneled through the
    // serial worker owned by sub_exec, eliminating the rcl hash-map race that
    // previously killed /data on Rolling when concurrent HTTP handler threads
    // created subscriptions on the same node.
    const auto exec_cfg = declare_executor_config(*node);
    const auto dp_cfg = declare_data_provider_config(*node);
    auto sub_exec = std::make_shared<ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor>(node, exec_cfg);
    auto serializer = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
    auto data_provider = std::make_shared<ros2_medkit_gateway::Ros2TopicDataProvider>(sub_exec, serializer, dp_cfg);
    node->set_topic_data_provider(data_provider);

    // Spin in a try/catch so an uncaught handler exception falls through to the
    // explicit teardown block below. Without this, an escaping throw bypasses
    // the teardown ordering and triggers exactly the rclcpp abort described
    // there (~GatewayNode running against a dead executor, exit -6).
    try {
      executor.spin();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node->get_logger(), "Executor.spin() threw an unhandled exception: %s. Falling through to teardown.",
                   ex.what());
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Executor.spin() threw an unknown exception. Falling through to teardown.");
    }

    // Teardown order (issue #375): stack-unwind destructs executor before node
    // which leaves ~GatewayNode running against a dead executor. Newer rclcpp
    // (rolling; recent jazzy patch releases) asserts 'node needs to be
    // associated with an executor' and aborts with exit -6 when the managers'
    // shutdown paths touch service clients. Explicit teardown avoids that:
    //   1. detach the provider from GatewayNode so the managers stop using it
    //      before we drop it (GatewayNode otherwise holds a shared_ptr that
    //      would keep the provider alive past data_provider.reset()).
    //   2. drop the provider (clears pool entries via the subscription worker)
    //   3. reset sub_exec (joins worker, tears down internal subscription executor)
    //   4. remove the gateway node from the executor and drop our ref so
    //      ~GatewayNode runs with the executor still alive.
    node->set_topic_data_provider(nullptr);
    data_provider.reset();
    sub_exec.reset();
    executor.remove_node(node);
    node.reset();

    rclcpp::shutdown();
  } catch (const std::exception & ex) {
    fprintf(stderr, "[ros2_medkit_gateway] Fatal exception in main: %s\n", ex.what());
    if (ros_inited && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  } catch (...) {
    fprintf(stderr, "[ros2_medkit_gateway] Fatal unknown exception in main\n");
    if (ros_inited && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  return 0;
}
