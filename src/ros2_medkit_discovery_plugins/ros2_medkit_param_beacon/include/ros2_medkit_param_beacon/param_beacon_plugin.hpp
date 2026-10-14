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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_beacon_common/beacon_entity_mapper.hpp"
#include "ros2_medkit_beacon_common/beacon_hint_store.hpp"
#include "ros2_medkit_beacon_common/beacon_types.hpp"
#include "ros2_medkit_beacon_common/beacon_validator.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_gateway/ros2_common/graph_node_list.hpp"
#include "ros2_medkit_param_beacon/parameter_client_interface.hpp"

class ParameterBeaconPlugin : public ros2_medkit_gateway::GatewayPlugin,
                              public ros2_medkit_gateway::IntrospectionProvider {
 public:
  ParameterBeaconPlugin() = default;
  ~ParameterBeaconPlugin() noexcept override;
  ParameterBeaconPlugin(const ParameterBeaconPlugin &) = delete;
  ParameterBeaconPlugin & operator=(const ParameterBeaconPlugin &) = delete;
  ParameterBeaconPlugin(ParameterBeaconPlugin &&) = delete;
  ParameterBeaconPlugin & operator=(ParameterBeaconPlugin &&) = delete;

  /// Constructor with injectable client factory (for testing).
  explicit ParameterBeaconPlugin(ros2_medkit_param_beacon::ParameterClientFactory factory)
    : client_factory_(std::move(factory)) {
  }

  // GatewayPlugin
  std::string name() const override;
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  void shutdown() override;
  std::vector<ros2_medkit_gateway::GatewayPlugin::PluginRoute> get_routes() override;

  // IntrospectionProvider
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  // Test exposure
  ros2_medkit_beacon::BeaconHintStore & store() {
    return *store_;
  }
  const rclcpp::Node::SharedPtr & param_node() const {
    return param_node_;
  }

 private:
  // Polling
  void poll_loop();
  void poll_cycle();
  void poll_node(const std::string & fqn);
  ros2_medkit_beacon::BeaconHint parse_parameters(const std::string & fqn,
                                                  const std::vector<rclcpp::Parameter> & params);

  // Client management
  std::shared_ptr<ros2_medkit_param_beacon::ParameterClientInterface> get_or_create_client(const std::string & fqn);
  /// Drop the client, and the backoff, of every node that is not among this cycle's targets.
  void evict_stale_clients(const std::vector<std::string> & targets);
  /// Count one more timeout for the node and set how many cycles skip it.
  void back_off(const std::string & fqn);

  // Config
  /// Longest duration in seconds. Fast DDS keeps a wait's seconds in an int32; a longer wait spins.
  static constexpr double kMaxSeconds = 2147483647.0;
  /// max_hints takes 1 to kMaxHints.
  static constexpr std::int64_t kMaxHints = 2147483647;
  static constexpr std::size_t kDefaultMaxHints = 10000;
  std::string parameter_prefix_{"ros2_medkit.discovery"};
  std::chrono::duration<double> poll_interval_{5.0};
  double poll_budget_sec_{10.0};
  double param_timeout_sec_{2.0};

  // State
  ros2_medkit_gateway::RosPluginContext * ctx_{nullptr};
  /// The gateway node's FQN. Graph reads skip it and its helper nodes.
  std::string gateway_fqn_;
  rclcpp::Node::SharedPtr param_node_;
  std::thread poll_thread_;
  std::atomic<bool> shutdown_requested_{false};
  std::mutex shutdown_mutex_;
  std::condition_variable shutdown_cv_;
  std::unique_ptr<ros2_medkit_beacon::BeaconHintStore> store_;
  ros2_medkit_beacon::BeaconEntityMapper mapper_;
  ros2_medkit_beacon::ValidationLimits limits_;

  // Client cache
  // Lock order: nodes_mutex_ -> clients_mutex_ -> param_ops_mutex_
  std::mutex clients_mutex_;
  std::map<std::string, std::shared_ptr<ros2_medkit_param_beacon::ParameterClientInterface>> clients_;
  ros2_medkit_param_beacon::ParameterClientFactory client_factory_;

  // Serializes parameter client calls (never hold while acquiring clients_mutex_)
  std::mutex param_ops_mutex_;

  // Node list (shared between introspect and poll threads)
  mutable std::shared_mutex nodes_mutex_;
  std::vector<std::string> poll_targets_;
  // Reads the graph when poll_targets_ is empty. Hides leftovers of nodes its own reads saw running.
  ros2_medkit_gateway::ros2_common::GraphNodeListReader graph_node_reader_;

  // Backoff tracking
  std::unordered_map<std::string, int> backoff_counts_;
  std::unordered_map<std::string, int> skip_remaining_;
  size_t start_offset_{0};

  std::atomic<bool> capacity_warned_{false};
  std::mutex skipped_mutex_;
  std::unordered_set<std::string> logged_skipped_entities_;
};
