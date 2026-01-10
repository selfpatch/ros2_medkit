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
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_faults.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_fault_manager {

/// Central fault manager node
///
/// Provides service interfaces for fault reporting, querying, and clearing.
/// Supports configurable storage backends (memory or SQLite) via ROS parameters.
///
/// Parameters:
/// - storage_type (string): "memory" or "sqlite" (default: "sqlite")
/// - database_path (string): Path to SQLite database file (default: "/var/lib/ros2_medkit/faults.db")
///   Use ":memory:" for in-memory SQLite database (useful for testing)
class FaultManagerNode : public rclcpp::Node {
 public:
  explicit FaultManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Get read-only access to fault storage (for testing)
  const FaultStorage & get_storage() const {
    return *storage_;
  }

  /// Get the storage type being used
  const std::string & get_storage_type() const {
    return storage_type_;
  }

 private:
  /// Create storage backend based on configuration
  std::unique_ptr<FaultStorage> create_storage();

  /// Handle ReportFault service request
  void handle_report_fault(const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
                           const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response);

  /// Handle GetFaults service request
  void handle_get_faults(const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Request> & request,
                         const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Response> & response);

  /// Handle ClearFault service request
  void handle_clear_fault(const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
                          const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response);

  /// Validate severity value
  static bool is_valid_severity(uint8_t severity);

  std::string storage_type_;
  std::string database_path_;
  int32_t confirmation_threshold_{-1};
  bool healing_enabled_{false};
  int32_t healing_threshold_{3};
  double auto_confirm_after_sec_{0.0};
  std::unique_ptr<FaultStorage> storage_;

  rclcpp::Service<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetFaults>::SharedPtr get_faults_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::TimerBase::SharedPtr auto_confirm_timer_;
};

}  // namespace ros2_medkit_fault_manager
