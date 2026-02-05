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

#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/get_faults.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/get_rosbags.hpp"
#include "ros2_medkit_msgs/srv/get_snapshots.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Result of a fault operation
struct FaultResult {
  bool success;
  json data;
  std::string error_message;
};

/// Result of get_fault operation with full message types
struct FaultWithEnvResult {
  bool success;
  std::string error_message;
  ros2_medkit_msgs::msg::Fault fault;
  ros2_medkit_msgs::msg::EnvironmentData environment_data;
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
  /// @param include_muted Include muted faults (correlation symptoms) in response
  /// @param include_clusters Include cluster info in response
  /// @return FaultResult with array of faults (and optionally muted_faults and clusters)
  FaultResult get_faults(const std::string & source_id = "", bool include_prefailed = true,
                         bool include_confirmed = true, bool include_cleared = false, bool include_muted = false,
                         bool include_clusters = false);

  /// Get a specific fault by code with environment data
  /// @param fault_code Fault identifier
  /// @param source_id Optional component identifier to verify fault belongs to component
  /// @return FaultWithEnvResult with fault and environment_data, or error if not found
  FaultWithEnvResult get_fault_with_env(const std::string & fault_code, const std::string & source_id = "");

  /// Get a specific fault by code (JSON result - legacy)
  /// @param fault_code Fault identifier
  /// @param source_id Optional component identifier to verify fault belongs to component
  /// @return FaultResult with fault data or error if not found
  /// @note Thread-safe: delegates to get_fault_with_env() which acquires service_mutex_.
  ///       Do NOT call this method while holding service_mutex_.
  FaultResult get_fault(const std::string & fault_code, const std::string & source_id = "");

  /// Clear a fault
  /// @param fault_code Fault identifier to clear
  /// @return FaultResult with success status
  FaultResult clear_fault(const std::string & fault_code);

  /// Get snapshots for a fault
  /// @param fault_code Fault identifier
  /// @param topic Optional topic filter (empty = all topics)
  /// @return FaultResult with snapshot data (JSON in data field)
  FaultResult get_snapshots(const std::string & fault_code, const std::string & topic = "");

  /// Get rosbag file info for a fault
  /// @param fault_code Fault identifier
  /// @return FaultResult with rosbag file path and metadata
  FaultResult get_rosbag(const std::string & fault_code);

  /// Get all rosbag files for an entity (batch operation)
  /// @param entity_fqn Entity fully qualified name for prefix matching
  /// @return FaultResult with arrays of rosbag metadata
  FaultResult get_rosbags(const std::string & entity_fqn);

  /// Check if fault manager service is available
  /// @return true if services are available
  bool is_available() const;

  /// Convert Fault message to JSON (static utility for reuse by SSE handler)
  static json fault_to_json(const ros2_medkit_msgs::msg::Fault & fault);

 private:
  /// Wait for services to become available
  bool wait_for_services(std::chrono::duration<double> timeout);

  rclcpp::Node * node_;

  /// Service clients
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetFault>::SharedPtr get_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetFaults>::SharedPtr get_faults_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetSnapshots>::SharedPtr get_snapshots_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetRosbag>::SharedPtr get_rosbag_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetRosbags>::SharedPtr get_rosbags_client_;

  /// Service timeout
  double service_timeout_sec_{5.0};

  /// Mutex for thread-safe service calls.
  /// Each public method that makes a ROS 2 service call acquires this mutex.
  /// Methods must NOT call other public locking methods while holding this mutex
  /// (e.g., get_fault() delegates to get_fault_with_env() without locking first).
  mutable std::mutex service_mutex_;
};

}  // namespace ros2_medkit_gateway
