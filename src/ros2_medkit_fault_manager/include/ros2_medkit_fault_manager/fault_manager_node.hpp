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
#include "ros2_medkit_fault_manager/correlation/correlation_engine.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/rosbag_capture.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/get_faults.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/get_rosbags.hpp"
#include "ros2_medkit_msgs/srv/get_snapshots.hpp"
#include "ros2_medkit_msgs/srv/list_faults_for_entity.hpp"
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

  /// Check if entity matches any reporting source
  /// @param reporting_sources List of reporting sources from fault
  /// @param entity_id Entity ID to match (exact match or as suffix of FQN)
  /// @return true if entity_id matches any source
  static bool matches_entity(const std::vector<std::string> & reporting_sources, const std::string & entity_id);

 private:
  /// Create storage backend based on configuration
  std::unique_ptr<FaultStorage> create_storage();

  /// Handle ReportFault service request
  void handle_report_fault(const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
                           const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response);

  /// Handle GetFaults service request
  void handle_get_faults(const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Request> & request,
                         const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Response> & response);

  /// Handle GetFault service request (single fault with environment_data)
  void handle_get_fault(const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                        const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response);

  /// Handle ClearFault service request
  void handle_clear_fault(const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
                          const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response);

  /// Handle GetSnapshots service request
  void handle_get_snapshots(const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Request> & request,
                            const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Response> & response);

  /// Handle GetRosbag service request
  void handle_get_rosbag(const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Request> & request,
                         const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Response> & response);

  /// Handle GetRosbags batch service request
  void handle_get_rosbags(const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbags::Request> & request,
                          const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbags::Response> & response);

  /// Handle ListFaultsForEntity service request
  void
  handle_list_faults_for_entity(const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> & request,
                                const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> & response);

  /// Create snapshot configuration from parameters
  SnapshotConfig create_snapshot_config();

  /// Load snapshot configuration from YAML file
  /// @param config_file Path to the YAML configuration file
  /// @param config SnapshotConfig to populate with loaded values
  void load_snapshot_config_from_yaml(const std::string & config_file, SnapshotConfig & config);

  /// Initialize correlation engine from configuration file
  /// @return CorrelationEngine instance if enabled and config is valid, nullptr otherwise
  std::unique_ptr<correlation::CorrelationEngine> create_correlation_engine();

  /// Publish a fault event to the events topic
  /// @param event_type One of FaultEvent::EVENT_CONFIRMED, EVENT_CLEARED, EVENT_UPDATED
  /// @param fault The fault data associated with this event
  /// @param auto_cleared_codes Optional list of auto-cleared symptom fault codes (for EVENT_CLEARED)
  void publish_fault_event(const std::string & event_type, const ros2_medkit_msgs::msg::Fault & fault,
                           const std::vector<std::string> & auto_cleared_codes = {});

  /// Validate severity value
  static bool is_valid_severity(uint8_t severity);

  /// Extract topic name from full topic path (last segment)
  static std::string extract_topic_name(const std::string & topic_path);

  std::string storage_type_;
  std::string database_path_;
  int32_t confirmation_threshold_{-1};
  bool healing_enabled_{false};
  int32_t healing_threshold_{3};
  double auto_confirm_after_sec_{0.0};
  std::unique_ptr<FaultStorage> storage_;

  rclcpp::Service<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetFaults>::SharedPtr get_faults_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetFault>::SharedPtr get_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetSnapshots>::SharedPtr get_snapshots_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetRosbag>::SharedPtr get_rosbag_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetRosbags>::SharedPtr get_rosbags_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ListFaultsForEntity>::SharedPtr list_faults_for_entity_srv_;
  rclcpp::TimerBase::SharedPtr auto_confirm_timer_;

  /// Timer for periodic cleanup of expired correlation data
  rclcpp::TimerBase::SharedPtr correlation_cleanup_timer_;

  /// Publisher for fault events (SSE streaming via gateway)
  rclcpp::Publisher<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr event_publisher_;

  /// Snapshot capture for capturing topic data on fault confirmation
  std::unique_ptr<SnapshotCapture> snapshot_capture_;

  /// Rosbag capture for time-window recording (nullptr if disabled)
  std::unique_ptr<RosbagCapture> rosbag_capture_;

  /// Correlation engine for fault correlation/muting (nullptr if disabled)
  std::unique_ptr<correlation::CorrelationEngine> correlation_engine_;
};

}  // namespace ros2_medkit_fault_manager
