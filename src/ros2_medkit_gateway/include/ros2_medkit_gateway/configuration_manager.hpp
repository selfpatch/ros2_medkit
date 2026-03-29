// Copyright 2025 mfaferek93
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

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Error codes for parameter operations
enum class ParameterErrorCode {
  NONE = 0,             ///< No error (success)
  NOT_FOUND,            ///< Parameter does not exist
  READ_ONLY,            ///< Parameter is read-only and cannot be modified
  SERVICE_UNAVAILABLE,  ///< Parameter service not available (node unreachable)
  TIMEOUT,              ///< Operation timed out
  TYPE_MISMATCH,        ///< Value type doesn't match parameter type
  INVALID_VALUE,        ///< Invalid value for parameter
  NO_DEFAULTS_CACHED,   ///< No default values cached for reset operation
  INTERNAL_ERROR        ///< Internal/unexpected error
};

/// Result of a parameter operation
struct ParameterResult {
  bool success;
  json data;
  std::string error_message;
  ParameterErrorCode error_code{ParameterErrorCode::NONE};  ///< Structured error code for programmatic handling
};

/// Manager for ROS2 node parameters
/// Provides CRUD operations on node parameters via native rclcpp APIs
/// Also caches initial parameter values as "defaults" for reset operations
class ConfigurationManager {
 public:
  explicit ConfigurationManager(rclcpp::Node * node);
  ~ConfigurationManager();

  /// Clean up thread-local param nodes before ROS 2 context shutdown.
  /// Must be called before rclcpp::shutdown() to prevent use-after-free.
  void shutdown();

  /// List all parameters for a node
  /// @param node_name Fully qualified node name (e.g., "/powertrain/engine/engine_temp_sensor")
  /// @return ParameterResult with array of {name, value, type} objects
  ParameterResult list_parameters(const std::string & node_name);

  /// Get a specific parameter value
  /// @param node_name Fully qualified node name
  /// @param param_name Parameter name
  /// @return ParameterResult with {name, value, type, description, read_only}
  ParameterResult get_parameter(const std::string & node_name, const std::string & param_name);

  /// Set a parameter value
  /// @param node_name Fully qualified node name
  /// @param param_name Parameter name
  /// @param value New value (JSON type will be converted to appropriate ROS2 type)
  /// @return ParameterResult with {name, value, type}
  ParameterResult set_parameter(const std::string & node_name, const std::string & param_name, const json & value);

  /// Reset a specific parameter to its default (initial) value
  /// @param node_name Fully qualified node name
  /// @param param_name Parameter name
  /// @return ParameterResult with reset parameter info
  ParameterResult reset_parameter(const std::string & node_name, const std::string & param_name);

  /// Reset all parameters of a node to their default (initial) values
  /// @param node_name Fully qualified node name
  /// @return ParameterResult with count of reset parameters
  ParameterResult reset_all_parameters(const std::string & node_name);

 private:
  /// Get or create a SyncParametersClient for the given node.
  /// Creates a per-thread param node to avoid executor conflicts.
  std::shared_ptr<rclcpp::SyncParametersClient> get_param_client(const std::string & node_name);

  /// Cache default values for a node (called on first access)
  void cache_default_values(const std::string & node_name);

  /// Check if a node is in the negative cache (recently unavailable)
  bool is_node_unavailable(const std::string & node_name) const;

  /// Mark a node as unavailable in the negative cache
  void mark_node_unavailable(const std::string & node_name);

  /// Check if node_name is the gateway's own node (self-query guard)
  bool is_self_node(const std::string & node_name) const;

  /// List parameters for the gateway's own node (direct access, no IPC)
  ParameterResult list_own_parameters();

  /// Get a specific parameter from the gateway's own node
  ParameterResult get_own_parameter(const std::string & param_name);

  /// Convert ROS2 parameter type to string
  static std::string parameter_type_to_string(rclcpp::ParameterType type);

  /// Convert ROS2 ParameterValue to JSON
  static json parameter_value_to_json(const rclcpp::ParameterValue & value);

  /// Convert JSON value to ROS2 ParameterValue
  static rclcpp::ParameterValue json_to_parameter_value(const json & value, rclcpp::ParameterType hint_type);

  /// Get the service timeout as a chrono duration
  std::chrono::duration<double> get_service_timeout() const;

  rclcpp::Node * node_;

  /// Timeout for waiting for parameter services (configurable via parameter_service_timeout_sec parameter)
  double service_timeout_sec_{0.5};

  /// Negative cache TTL (configurable via parameter_service_negative_cache_sec parameter)
  double negative_cache_ttl_sec_{60.0};

  /// Gateway's own fully qualified node name (for self-query detection)
  std::string own_node_fqn_;

  /// Per-node mutexes for parameter operations.
  /// Each node gets its own mutex so one slow node doesn't block queries to other nodes.
  mutable std::shared_mutex node_mutexes_mutex_;
  mutable std::unordered_map<std::string, std::unique_ptr<std::mutex>> node_mutexes_;

  /// Get or create a mutex for a specific node
  std::mutex & get_node_mutex(const std::string & node_name) const;

  /// Negative cache: nodes whose parameter service was recently unavailable.
  /// Avoids repeated blocking waits on nodes that don't have parameter services.
  mutable std::shared_mutex negative_cache_mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> unavailable_nodes_;

  /// Cache of default parameter values per node
  /// Key: node_name, Value: map of param_name -> Parameter
  mutable std::mutex defaults_mutex_;
  std::map<std::string, std::map<std::string, rclcpp::Parameter>> default_values_;

  /// Registry of thread-local param nodes for deterministic cleanup.
  /// Thread-local nodes are registered here on creation and cleared in shutdown()
  /// to prevent use-after-free when ROS 2 context is destroyed before httplib threads exit.
  mutable std::mutex param_nodes_registry_mutex_;
  std::vector<std::weak_ptr<rclcpp::Node>> param_nodes_registry_;

  /// Register a thread-local param node for cleanup tracking
  void register_param_node(std::shared_ptr<rclcpp::Node> node);
};

}  // namespace ros2_medkit_gateway
