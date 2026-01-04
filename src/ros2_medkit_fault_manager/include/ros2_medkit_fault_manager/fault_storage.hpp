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

/// Abstract interface for fault storage backends
class FaultStorage {
 public:
  virtual ~FaultStorage() = default;

  /// Set the confirmation threshold for automatic PENDING → CONFIRMED transition
  /// @param threshold Number of occurrences required to confirm a fault (0 = disabled)
  virtual void set_confirmation_threshold(uint32_t threshold) = 0;

  /// Get the current confirmation threshold
  virtual uint32_t get_confirmation_threshold() const = 0;

  /// Store or update a fault report
  /// @param fault_code Global fault identifier
  /// @param severity Fault severity level
  /// @param description Human-readable description
  /// @param source_id Reporting source identifier
  /// @param timestamp Current time for tracking
  /// @return true if this is a new fault, false if existing fault was updated
  virtual bool report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                            const std::string & source_id, const rclcpp::Time & timestamp) = 0;

  /// Get faults matching filter criteria
  /// @param filter_by_severity Whether to filter by severity
  /// @param severity Severity level to filter (if filter_by_severity is true)
  /// @param statuses List of statuses to include (empty = CONFIRMED only)
  /// @return Vector of matching faults
  virtual std::vector<ros2_medkit_msgs::msg::Fault> get_faults(bool filter_by_severity, uint8_t severity,
                                                               const std::vector<std::string> & statuses) const = 0;

  /// Get a single fault by fault_code
  /// @param fault_code The fault code to look up
  /// @return The fault if found, nullopt otherwise
  virtual std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const std::string & fault_code) const = 0;

  /// Clear a fault by fault_code
  /// @param fault_code The fault code to clear
  /// @return true if fault was found and cleared, false if not found
  virtual bool clear_fault(const std::string & fault_code) = 0;

  /// Get total number of stored faults
  virtual size_t size() const = 0;

  /// Check if a fault exists
  virtual bool contains(const std::string & fault_code) const = 0;

 protected:
  FaultStorage() = default;
  FaultStorage(const FaultStorage &) = default;
  FaultStorage & operator=(const FaultStorage &) = default;
  FaultStorage(FaultStorage &&) = default;
  FaultStorage & operator=(FaultStorage &&) = default;
};

/// Thread-safe in-memory fault storage implementation
class InMemoryFaultStorage : public FaultStorage {
 public:
  InMemoryFaultStorage() = default;

  void set_confirmation_threshold(uint32_t threshold) override;
  uint32_t get_confirmation_threshold() const override;

  bool report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                    const std::string & source_id, const rclcpp::Time & timestamp) override;

  std::vector<ros2_medkit_msgs::msg::Fault> get_faults(bool filter_by_severity, uint8_t severity,
                                                       const std::vector<std::string> & statuses) const override;

  std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const std::string & fault_code) const override;

  bool clear_fault(const std::string & fault_code) override;

  size_t size() const override;

  bool contains(const std::string & fault_code) const override;

 private:
  mutable std::mutex mutex_;
  std::map<std::string, FaultState> faults_;
  uint32_t confirmation_threshold_{0};  ///< 0 = disabled (no auto-confirmation)
};

}  // namespace ros2_medkit_fault_manager
