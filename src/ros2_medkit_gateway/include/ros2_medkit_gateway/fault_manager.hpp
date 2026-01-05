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
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_faults.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Result of a fault operation
struct FaultResult {
  bool success;
  json data;
  std::string error_message;
};

/// Manager for fault management operations
/// Provides interface to the ros2_medkit_fault_manager services
class FaultManager {
 public:
  explicit FaultManager(rclcpp::Node * node);

  /// Report a fault from a component
  /// @param fault_code Unique fault identifier
  /// @param severity Fault severity (0=INFO, 1=WARN, 2=ERROR, 3=CRITICAL)
  /// @param description Human-readable description
  /// @param source_id Component identifier (namespace path)
  /// @return FaultResult with success status
  FaultResult report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                           const std::string & source_id);

  /// Get all faults, optionally filtered by component
  /// @param source_id Optional component identifier to filter by (empty = all)
  /// @param include_prefailed Include PREFAILED status faults (debounce not yet confirmed)
  /// @param include_confirmed Include CONFIRMED status faults
  /// @param include_cleared Include CLEARED status faults
  /// @return FaultResult with array of faults
  FaultResult get_faults(const std::string & source_id = "", bool include_prefailed = true,
                         bool include_confirmed = true, bool include_cleared = false);

  /// Get a specific fault by code
  /// @param fault_code Fault identifier
  /// @param source_id Optional component identifier to verify fault belongs to component
  /// @return FaultResult with fault data or error if not found
  FaultResult get_fault(const std::string & fault_code, const std::string & source_id = "");

  /// Clear a fault
  /// @param fault_code Fault identifier to clear
  /// @return FaultResult with success status
  FaultResult clear_fault(const std::string & fault_code);

  /// Check if fault manager service is available
  /// @return true if services are available
  bool is_available() const;

 private:
  /// Convert Fault message to JSON
  static json fault_to_json(const ros2_medkit_msgs::msg::Fault & fault);

  /// Wait for services to become available
  bool wait_for_services(std::chrono::duration<double> timeout);

  rclcpp::Node * node_;

  /// Service clients
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetFaults>::SharedPtr get_faults_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_client_;

  /// Service timeout
  double service_timeout_sec_{5.0};

  /// Mutex for thread-safe service calls
  mutable std::mutex service_mutex_;
};

}  // namespace ros2_medkit_gateway
