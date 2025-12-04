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

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_medkit_gateway/discovery_manager.hpp"
#include "ros2_medkit_gateway/models.hpp"
#include "ros2_medkit_gateway/ros2_cli_wrapper.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Result of a synchronous service call
struct ServiceCallResult {
  bool success;
  json response;
  std::string error_message;
};

/// Manager for ROS2 operations (services and actions)
/// Handles service calls synchronously and action calls asynchronously
class OperationManager {
 public:
  explicit OperationManager(rclcpp::Node * node, DiscoveryManager * discovery_manager);

  /// Call a ROS2 service synchronously
  /// @param service_path Full service path (e.g., "/powertrain/engine/calibrate")
  /// @param service_type Service type (e.g., "std_srvs/srv/Trigger")
  /// @param request JSON request body
  /// @return ServiceCallResult with response or error
  ServiceCallResult call_service(const std::string & service_path, const std::string & service_type,
                                 const json & request);

  /// Find and call a service by component and operation name
  /// Uses discovery cache to resolve service path and type if not provided
  /// @param component_ns Component namespace (e.g., "/powertrain/engine")
  /// @param operation_name Operation name (e.g., "calibrate")
  /// @param service_type Optional service type override
  /// @param request JSON request body
  /// @return ServiceCallResult with response or error
  ServiceCallResult call_component_service(const std::string & component_ns, const std::string & operation_name,
                                           const std::optional<std::string> & service_type, const json & request);

  /// Validate message type format (package/srv/Type or package/action/Type)
  static bool is_valid_message_type(const std::string & type);

  /// Check if type is a service type (contains /srv/)
  static bool is_service_type(const std::string & type);

  /// Check if type is an action type (contains /action/)
  static bool is_action_type(const std::string & type);

 private:
  /// Convert JSON to YAML string for ros2 service call
  std::string json_to_yaml(const json & j);

  /// Parse YAML output from ros2 service call to JSON
  json parse_service_response(const std::string & yaml_output);

  rclcpp::Node * node_;
  DiscoveryManager * discovery_manager_;
  std::unique_ptr<ROS2CLIWrapper> cli_wrapper_;
};

}  // namespace ros2_medkit_gateway
