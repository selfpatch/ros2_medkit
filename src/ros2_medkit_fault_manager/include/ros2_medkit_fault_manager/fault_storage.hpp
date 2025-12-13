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
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_fault_manager {

/// Internal fault state stored in memory
struct FaultState {
  std::string fault_code;
  uint8_t severity{0};
  std::string description;
  rclcpp::Time first_occurred;
  rclcpp::Time last_occurred;
  uint32_t occurrence_count{0};
  std::string status;
  std::set<std::string> reporting_sources;

  /// Convert to ROS 2 message
  ros2_medkit_msgs::msg::Fault to_msg() const;
};

/// Thread-safe in-memory fault storage
class FaultStorage {
 public:
  FaultStorage() = default;

  /// Store or update a fault report
  /// @param fault_code Global fault identifier
  /// @param severity Fault severity level
  /// @param description Human-readable description
  /// @param source_id Reporting source identifier
  /// @param timestamp Current time for tracking
  /// @return true if this is a new fault, false if existing fault was updated
  bool report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                    const std::string & source_id, const rclcpp::Time & timestamp);

  /// Get faults matching filter criteria
  /// @param filter_by_severity Whether to filter by severity
  /// @param severity Severity level to filter (if filter_by_severity is true)
  /// @param statuses List of statuses to include (empty = CONFIRMED only)
  /// @return Vector of matching faults
  std::vector<ros2_medkit_msgs::msg::Fault> get_faults(bool filter_by_severity, uint8_t severity,
                                                       const std::vector<std::string> & statuses) const;

  /// Get a single fault by fault_code
  /// @param fault_code The fault code to look up
  /// @return The fault if found, nullopt otherwise
  std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const std::string & fault_code) const;

  /// Clear a fault by fault_code
  /// @param fault_code The fault code to clear
  /// @return true if fault was found and cleared, false if not found
  bool clear_fault(const std::string & fault_code);

  /// Get total number of stored faults
  size_t size() const;

  /// Check if a fault exists
  bool contains(const std::string & fault_code) const;

 private:
  mutable std::mutex mutex_;
  std::map<std::string, FaultState> faults_;
};

}  // namespace ros2_medkit_fault_manager
