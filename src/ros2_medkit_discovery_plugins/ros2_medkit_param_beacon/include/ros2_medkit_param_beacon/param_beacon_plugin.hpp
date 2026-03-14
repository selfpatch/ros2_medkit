// Copyright 2026 selfpatch GmbH
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

#include <httplib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
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
#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"
#include "ros2_medkit_param_beacon/parameter_client_interface.hpp"

class ParameterBeaconPlugin : public ros2_medkit_gateway::GatewayPlugin,
                              public ros2_medkit_gateway::IntrospectionProvider {
 public:
  ParameterBeaconPlugin() = default;

  /// Constructor with injectable client factory (for testing).
  explicit ParameterBeaconPlugin(ros2_medkit_param_beacon::ParameterClientFactory factory)
    : client_factory_(std::move(factory)) {
  }

  // GatewayPlugin
  std::string name() const override;
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  void shutdown() override;
  void register_routes(httplib::Server & server, const std::string & api_prefix) override;
  std::vector<ros2_medkit_gateway::GatewayPlugin::RouteDescription> get_route_descriptions() const override;

  // IntrospectionProvider
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  // Test exposure
  ros2_medkit_beacon::BeaconHintStore & store() {
    return *store_;
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
  void evict_stale_clients();

  // Config
  std::string parameter_prefix_{"ros2_medkit.discovery"};
  std::chrono::duration<double> poll_interval_{5.0};
  double poll_budget_sec_{10.0};
  double param_timeout_sec_{2.0};

  // State
  ros2_medkit_gateway::PluginContext * ctx_{nullptr};
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

  // Serialization for SyncParametersClient operations (never hold while acquiring clients_mutex_)
  std::mutex param_ops_mutex_;

  // Node list (shared between introspect and poll threads)
  mutable std::shared_mutex nodes_mutex_;
  std::vector<std::string> poll_targets_;

  // Backoff tracking
  std::unordered_map<std::string, int> backoff_counts_;
  std::unordered_map<std::string, int> skip_remaining_;
  size_t start_offset_{0};

  bool capacity_warned_{false};
  std::unordered_set<std::string> logged_skipped_entities_;
};
