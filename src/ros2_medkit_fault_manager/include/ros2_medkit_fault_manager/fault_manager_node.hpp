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

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_faults.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_fault_manager {

/// Central fault manager node
///
/// Provides service interfaces for fault reporting, querying, and clearing.
/// Stores faults in memory and aggregates reports by fault_code.
class FaultManagerNode : public rclcpp::Node {
 public:
  explicit FaultManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Get read-only access to fault storage (for testing)
  const FaultStorage & get_storage() const {
    return storage_;
  }

 private:
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

  FaultStorage storage_;

  rclcpp::Service<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetFaults>::SharedPtr get_faults_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
};

}  // namespace ros2_medkit_fault_manager
