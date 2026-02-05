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

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_fault_manager {

/// Debounce configuration for fault filtering
struct DebounceConfig {
  /// Confirmation threshold (typically negative). Fault is CONFIRMED when counter <= this value.
  /// Default: -1 (immediate confirmation - first FAILED event confirms the fault).
  /// Set to lower values (e.g., -3) for debounce filtering.
  int32_t confirmation_threshold{-1};

  /// Whether healing is enabled. When true, faults can transition to HEALED status.
  bool healing_enabled{false};

  /// Healing threshold (positive). Fault is HEALED when counter >= this value.
  /// Default: 3 (3 more PASSED than FAILED events to heal). Only used if healing_enabled.
  int32_t healing_threshold{3};

  /// Whether CRITICAL severity bypasses debounce and confirms immediately.
  bool critical_immediate_confirm{true};

  /// Time-based auto-confirmation. If > 0, PREFAILED faults older than this are auto-confirmed.
  /// 0.0 = disabled.
  double auto_confirm_after_sec{0.0};
};

/// Internal fault state stored in memory
struct FaultState {
  std::string fault_code;
  uint8_t severity{0};
  std::string description;
  rclcpp::Time first_occurred;
  rclcpp::Time last_occurred;
  uint32_t occurrence_count{0};  ///< Count of FAILED events
  std::string status;
  std::set<std::string> reporting_sources;

  // Debounce state (internal, not exposed in Fault.msg)
  int32_t debounce_counter{0};      ///< FAILED decrements (-1), PASSED increments (+1)
  rclcpp::Time last_failed_time{};  ///< Timestamp of last FAILED event
  rclcpp::Time last_passed_time{};  ///< Timestamp of last PASSED event

  /// Convert to ROS 2 message
  ros2_medkit_msgs::msg::Fault to_msg() const;
};

/// Event type alias for convenience
using EventType = ros2_medkit_msgs::srv::ReportFault::Request;

/// Snapshot data captured when a fault is confirmed
struct SnapshotData {
  std::string fault_code;
  std::string topic;
  std::string message_type;
  std::string data;  ///< JSON-encoded message data
  int64_t captured_at_ns{0};
};

/// Rosbag file metadata for time-window recording
struct RosbagFileInfo {
  std::string bulk_data_id;  ///< UUID - globally unique identifier for bulk-data download
  std::string fault_code;
  std::string file_path;
  std::string format;        ///< "sqlite3" or "mcap"
  double duration_sec{0.0};  ///< Total duration of recorded data
  size_t size_bytes{0};      ///< File size in bytes
  int64_t created_at_ns{0};  ///< Timestamp when bag was created
};

/// Abstract interface for fault storage backends
class FaultStorage {
 public:
  virtual ~FaultStorage() = default;

  /// Set debounce configuration
  virtual void set_debounce_config(const DebounceConfig & config) = 0;

  /// Get current debounce configuration
  virtual DebounceConfig get_debounce_config() const = 0;

  /// Report a fault event (FAILED or PASSED)
  /// @param fault_code Global fault identifier
  /// @param event_type EVENT_FAILED (0) or EVENT_PASSED (1)
  /// @param severity Fault severity level (only used for FAILED events)
  /// @param description Human-readable description (only used for FAILED events)
  /// @param source_id Reporting source identifier
  /// @param timestamp Current time for tracking
  /// @return true if this is a new occurrence (new fault or reactivated CLEARED fault),
  ///         false if existing active fault was updated
  virtual bool report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                  const std::string & description, const std::string & source_id,
                                  const rclcpp::Time & timestamp) = 0;

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

  /// Clear a fault by fault_code (manual acknowledgment)
  /// @param fault_code The fault code to clear
  /// @return true if fault was found and cleared, false if not found
  virtual bool clear_fault(const std::string & fault_code) = 0;

  /// Get total number of stored faults
  virtual size_t size() const = 0;

  /// Check if a fault exists
  virtual bool contains(const std::string & fault_code) const = 0;

  /// Check and confirm PREFAILED faults that have been pending too long (time-based confirmation)
  /// @param current_time Current timestamp for age calculation
  /// @return Number of faults that were confirmed
  virtual size_t check_time_based_confirmation(const rclcpp::Time & current_time) = 0;

  /// Store a snapshot captured when a fault was confirmed
  /// @param snapshot The snapshot data to store
  virtual void store_snapshot(const SnapshotData & snapshot) = 0;

  /// Get snapshots for a fault
  /// @param fault_code The fault code to get snapshots for
  /// @param topic_filter Optional topic filter (empty = all topics)
  /// @return Vector of snapshots for the fault
  virtual std::vector<SnapshotData> get_snapshots(const std::string & fault_code,
                                                  const std::string & topic_filter = "") const = 0;

  /// Store rosbag file metadata for a fault
  /// @param info The rosbag file info to store (replaces any existing entry for fault_code)
  virtual void store_rosbag_file(const RosbagFileInfo & info) = 0;

  /// Get rosbag file info for a fault
  /// @param fault_code The fault code to get rosbag for
  /// @return Rosbag file info if exists, nullopt otherwise
  virtual std::optional<RosbagFileInfo> get_rosbag_file(const std::string & fault_code) const = 0;

  /// Delete rosbag file record and the actual file for a fault
  /// @param fault_code The fault code to delete rosbag for
  /// @return true if record was deleted, false if not found
  virtual bool delete_rosbag_file(const std::string & fault_code) = 0;

  /// Get total size of all stored rosbag files in bytes
  /// @return Total size in bytes
  virtual size_t get_total_rosbag_storage_bytes() const = 0;

  /// Get all rosbag files ordered by creation time (oldest first)
  /// @return Vector of rosbag file info
  virtual std::vector<RosbagFileInfo> get_all_rosbag_files() const = 0;

  /// Get rosbag by bulk_data_id (UUID)
  /// @param bulk_data_id The UUID to look up
  /// @return Rosbag file info if exists, nullopt otherwise
  virtual std::optional<RosbagFileInfo> get_rosbag_by_id(const std::string & bulk_data_id) const = 0;

  /// Get file path for a rosbag by bulk_data_id
  /// @param bulk_data_id The UUID to look up
  /// @return File path if exists, empty string otherwise
  virtual std::string get_rosbag_path(const std::string & bulk_data_id) const = 0;

  /// Get rosbags for all faults associated with an entity
  /// @param entity_fqn The entity's fully qualified name to filter by
  /// @return Vector of rosbag file info for faults reported by this entity
  virtual std::vector<RosbagFileInfo> get_rosbags_for_entity(const std::string & entity_fqn) const = 0;

  /// Generate a UUID v4 string
  /// @return A new UUID in format "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
  static std::string generate_uuid();

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

  void set_debounce_config(const DebounceConfig & config) override;
  DebounceConfig get_debounce_config() const override;

  bool report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                          const std::string & description, const std::string & source_id,
                          const rclcpp::Time & timestamp) override;

  std::vector<ros2_medkit_msgs::msg::Fault> get_faults(bool filter_by_severity, uint8_t severity,
                                                       const std::vector<std::string> & statuses) const override;

  std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const std::string & fault_code) const override;

  bool clear_fault(const std::string & fault_code) override;

  size_t size() const override;

  bool contains(const std::string & fault_code) const override;

  size_t check_time_based_confirmation(const rclcpp::Time & current_time) override;

  void store_snapshot(const SnapshotData & snapshot) override;
  std::vector<SnapshotData> get_snapshots(const std::string & fault_code,
                                          const std::string & topic_filter = "") const override;

  void store_rosbag_file(const RosbagFileInfo & info) override;
  std::optional<RosbagFileInfo> get_rosbag_file(const std::string & fault_code) const override;
  bool delete_rosbag_file(const std::string & fault_code) override;
  size_t get_total_rosbag_storage_bytes() const override;
  std::vector<RosbagFileInfo> get_all_rosbag_files() const override;
  std::optional<RosbagFileInfo> get_rosbag_by_id(const std::string & bulk_data_id) const override;
  std::string get_rosbag_path(const std::string & bulk_data_id) const override;
  std::vector<RosbagFileInfo> get_rosbags_for_entity(const std::string & entity_fqn) const override;

 private:
  /// Update fault status based on debounce counter
  void update_status(FaultState & state);

  mutable std::mutex mutex_;
  std::map<std::string, FaultState> faults_;
  std::vector<SnapshotData> snapshots_;
  std::map<std::string, RosbagFileInfo> rosbag_files_;  ///< fault_code -> rosbag info
  std::map<std::string, std::string> rosbag_by_id_;     ///< bulk_data_id -> fault_code (index)
  DebounceConfig config_;
};

}  // namespace ros2_medkit_fault_manager
