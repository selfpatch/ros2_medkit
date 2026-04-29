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
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing ParameterTransport.
 *
 * Owns the rclcpp::SyncParametersClient cache, the rclcpp::Parameter defaults
 * cache, the negative-cache for unreachable nodes, the spin_mutex serialising
 * parameter-client spins, and the JSON <-> rclcpp::ParameterValue conversion
 * helpers that previously lived inside ConfigurationManager.
 *
 * The adapter exposes only the neutral ParameterTransport surface to the
 * manager; all rclcpp types stay confined to this translation unit.
 */
class Ros2ParameterTransport : public ParameterTransport {
 public:
  /**
   * @param node Non-owning ROS node used to declare parameters and to derive
   *             the gateway's own FQN for the self-node short circuit.
   * @param service_timeout_sec Timeout for SyncParametersClient::wait_for_service
   *                            and underlying parameter-service calls.
   * @param negative_cache_ttl_sec How long an unavailable node remains in the
   *                               negative cache before another IPC attempt.
   *                               Set to 0 to disable.
   */
  Ros2ParameterTransport(rclcpp::Node * node, double service_timeout_sec, double negative_cache_ttl_sec);

  ~Ros2ParameterTransport() override;

  Ros2ParameterTransport(const Ros2ParameterTransport &) = delete;
  Ros2ParameterTransport & operator=(const Ros2ParameterTransport &) = delete;
  Ros2ParameterTransport(Ros2ParameterTransport &&) = delete;
  Ros2ParameterTransport & operator=(Ros2ParameterTransport &&) = delete;

  // ParameterTransport implementation -----------------------------------------

  bool is_self_node(const std::string & node_name) const override;

  ParameterResult list_parameters(const std::string & node_name) override;
  ParameterResult get_parameter(const std::string & node_name, const std::string & param_name) override;
  ParameterResult set_parameter(const std::string & node_name, const std::string & param_name,
                                const json & value) override;

  ParameterResult list_own_parameters() override;
  ParameterResult get_own_parameter(const std::string & param_name) override;

  ParameterResult get_default(const std::string & node_name, const std::string & param_name) override;
  ParameterResult list_defaults(const std::string & node_name) override;

  bool is_node_available(const std::string & node_name) const override;
  void invalidate(const std::string & node_name) override;
  void shutdown() override;

 private:
  /// Get or create a cached SyncParametersClient for the given node.
  std::shared_ptr<rclcpp::SyncParametersClient> get_param_client(const std::string & node_name);

  /// Cache default values for a node (called on first access).
  /// @pre spin_mutex_ must be held by the caller.
  void cache_default_values(const std::string & node_name);

  /// Check if a node is in the negative cache (recently unavailable).
  bool is_node_unavailable(const std::string & node_name) const;

  /// Mark a node as unavailable in the negative cache.
  void mark_node_unavailable(const std::string & node_name);

  /// Convert ROS 2 parameter type to string.
  std::string parameter_type_to_string(rclcpp::ParameterType type) const;

  /// Convert ROS 2 ParameterValue to JSON.
  json parameter_value_to_json(const rclcpp::ParameterValue & value) const;

  /// Convert JSON value to ROS 2 ParameterValue.
  rclcpp::ParameterValue json_to_parameter_value(const json & value, rclcpp::ParameterType hint_type) const;

  /// Get the service timeout as a chrono duration.
  std::chrono::duration<double> get_service_timeout() const;

  /// Try to acquire spin_mutex_ with timeout.
  /// Returns unique_lock on success, nullopt on timeout (result populated with error).
  std::optional<std::unique_lock<std::timed_mutex>> try_acquire_spin_lock(ParameterResult & result);

  rclcpp::Node * node_;

  /// Timeout for waiting for parameter services.
  double service_timeout_sec_;

  /// Negative cache TTL in seconds (0 = disabled).
  double negative_cache_ttl_sec_;

  /// Gateway's own fully qualified node name (for self-query detection).
  std::string own_node_fqn_;

  /// Shutdown flag - prevents use-after-free on param_node_ after shutdown.
  std::atomic<bool> shutdown_requested_{false};

  /// Negative cache: nodes whose parameter service was recently unavailable.
  /// Avoids repeated blocking waits on nodes that don't have parameter services.
  mutable std::shared_mutex negative_cache_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> unavailable_nodes_;

  /// Cache of default parameter values per node.
  /// Key: node_name, Value: map of param_name -> Parameter.
  mutable std::mutex defaults_mutex_;
  std::map<std::string, std::map<std::string, rclcpp::Parameter>> default_values_;

  /// Mutex to serialize spin operations on param_node_.
  /// SyncParametersClient::wait_for_service/get_parameters/etc spin param_node_
  /// internally via spin_node_until_future_complete which is NOT thread-safe.
  /// Declared BEFORE param_node_ so it outlives the node during destruction.
  mutable std::timed_mutex spin_mutex_;

  /// Internal node for parameter client operations.
  /// Created once in constructor - must be in DDS graph early for fast service discovery.
  std::shared_ptr<rclcpp::Node> param_node_;

  /// Cached SyncParametersClient per target node.
  mutable std::mutex clients_mutex_;
  std::map<std::string, std::shared_ptr<rclcpp::SyncParametersClient>> param_clients_;

  /// Maximum entries in negative cache before hard eviction.
  static constexpr size_t kMaxNegativeCacheSize = 500;

  /// Margin added to service_timeout_sec_ for spin_mutex acquisition timeout.
  /// Must exceed service_timeout_sec_ to avoid spurious TIMEOUT errors.
  static constexpr double kSpinLockMarginSec{1.0};

  /// Timeout for shutdown to wait for in-flight IPC.
  static constexpr std::chrono::seconds kShutdownTimeout{10};
};

}  // namespace ros2_medkit_gateway::ros2
