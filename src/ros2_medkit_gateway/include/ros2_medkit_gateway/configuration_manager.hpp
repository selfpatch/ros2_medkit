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

#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Result of a parameter operation
struct ParameterResult {
  bool success;
  json data;
  std::string error_message;
};

/// Manager for ROS2 node parameters
/// Provides CRUD operations on node parameters via native rclcpp APIs
class ConfigurationManager {
 public:
  explicit ConfigurationManager(rclcpp::Node * node);

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

  /// Describe a parameter (get metadata)
  /// @param node_name Fully qualified node name
  /// @param param_name Parameter name
  /// @return ParameterResult with parameter descriptor info
  ParameterResult describe_parameter(const std::string & node_name, const std::string & param_name);

  /// Check if a node exists and is reachable for parameter operations
  /// @param node_name Fully qualified node name
  /// @return true if node parameters are accessible
  bool is_node_available(const std::string & node_name);

 private:
  /// Get or create a SyncParametersClient for the given node
  std::shared_ptr<rclcpp::SyncParametersClient> get_param_client(const std::string & node_name);

  /// Convert ROS2 parameter type to string
  static std::string parameter_type_to_string(rclcpp::ParameterType type);

  /// Convert ROS2 ParameterValue to JSON
  static json parameter_value_to_json(const rclcpp::ParameterValue & value);

  /// Convert JSON value to ROS2 ParameterValue
  static rclcpp::ParameterValue json_to_parameter_value(const json & value, rclcpp::ParameterType hint_type);

  rclcpp::Node * node_;

  /// Internal node for parameter client operations
  /// SyncParametersClient requires a node that is NOT in an executor
  /// to perform synchronous operations (spinning internally)
  std::shared_ptr<rclcpp::Node> param_node_;

  /// Cache of parameter clients per node (avoids recreating clients)
  mutable std::mutex clients_mutex_;
  std::map<std::string, std::shared_ptr<rclcpp::SyncParametersClient>> param_clients_;
};

}  // namespace ros2_medkit_gateway
