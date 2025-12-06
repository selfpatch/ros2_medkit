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

#include <action_msgs/msg/goal_status_array.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

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

/// Action goal status (matches ROS2 action_msgs/msg/GoalStatus)
enum class ActionGoalStatus : int8_t {
  UNKNOWN = 0,
  ACCEPTED = 1,
  EXECUTING = 2,
  CANCELING = 3,
  SUCCEEDED = 4,
  CANCELED = 5,
  ABORTED = 6
};

/// Convert status enum to string
std::string action_status_to_string(ActionGoalStatus status);

/// Result of sending an action goal
struct ActionSendGoalResult {
  bool success;
  std::string goal_id;  // UUID hex string
  bool goal_accepted;
  std::string error_message;
};

/// Result of canceling an action goal
struct ActionCancelResult {
  bool success;
  int8_t return_code;  // 0=accepted, 1=rejected, 2=unknown_id, 3=terminated
  std::string error_message;
};

/// Result of getting action result
struct ActionGetResultResult {
  bool success;
  ActionGoalStatus status;
  json result;
  std::string error_message;
};

/// Tracked action goal info (stored locally)
struct ActionGoalInfo {
  std::string goal_id;
  std::string action_path;  // e.g., /powertrain/engine/long_calibration
  std::string action_type;  // e.g., example_interfaces/action/Fibonacci
  ActionGoalStatus status;
  json last_feedback;
  std::chrono::system_clock::time_point created_at;
  std::chrono::system_clock::time_point last_update;
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

  /// Validate UUID hex string format (32 hex characters)
  static bool is_valid_uuid_hex(const std::string & uuid_hex);

  /// Check if type is a service type (contains /srv/)
  static bool is_service_type(const std::string & type);

  /// Check if type is an action type (contains /action/)
  static bool is_action_type(const std::string & type);

  // ==================== ACTION OPERATIONS ====================

  /// Send a goal to an action server using ros2 action send_goal
  /// @param action_path Full action path (e.g., "/powertrain/engine/long_calibration")
  /// @param action_type Action type (e.g., "example_interfaces/action/Fibonacci")
  /// @param goal JSON goal data
  /// @return ActionSendGoalResult with goal_id or error
  ActionSendGoalResult send_action_goal(const std::string & action_path, const std::string & action_type,
                                        const json & goal);

  /// Send a goal to an action using component namespace and operation name
  /// Uses discovery cache to resolve action path and type if not provided
  /// @param component_ns Component namespace (e.g., "/powertrain/engine")
  /// @param operation_name Operation name (e.g., "long_calibration")
  /// @param action_type Optional action type override
  /// @param goal JSON goal data
  /// @return ActionSendGoalResult with goal_id or error
  ActionSendGoalResult send_component_action_goal(const std::string & component_ns, const std::string & operation_name,
                                                  const std::optional<std::string> & action_type, const json & goal);

  /// Cancel a running action goal using ros2 action cancel
  /// @param action_path Full action path
  /// @param goal_id Goal UUID hex string
  /// @return ActionCancelResult
  ActionCancelResult cancel_action_goal(const std::string & action_path, const std::string & goal_id);

  /// Get the result of a completed action
  /// This is a blocking call that waits for the action to complete
  /// @param action_path Full action path
  /// @param action_type Action type
  /// @param goal_id Goal UUID hex string
  /// @return ActionGetResultResult with result or error
  ActionGetResultResult get_action_result(const std::string & action_path, const std::string & action_type,
                                          const std::string & goal_id);

  /// Get tracked goal info by goal_id
  /// @param goal_id Goal UUID hex string
  /// @return Optional ActionGoalInfo if found
  std::optional<ActionGoalInfo> get_tracked_goal(const std::string & goal_id) const;

  /// List all tracked goals
  /// @return Vector of all tracked goals
  std::vector<ActionGoalInfo> list_tracked_goals() const;

  /// Get all goals for a specific action path
  /// @param action_path Full action path (e.g., "/powertrain/engine/long_calibration")
  /// @return Vector of goals for that action, sorted by last_update (newest first)
  std::vector<ActionGoalInfo> get_goals_for_action(const std::string & action_path) const;

  /// Get the most recent goal for a specific action path
  /// @param action_path Full action path
  /// @return Optional ActionGoalInfo if any goal exists for this action
  std::optional<ActionGoalInfo> get_latest_goal_for_action(const std::string & action_path) const;

  /// Update goal status in tracking
  /// @param goal_id Goal UUID hex string
  /// @param status New status
  void update_goal_status(const std::string & goal_id, ActionGoalStatus status);

  /// Update goal feedback in tracking
  /// @param goal_id Goal UUID hex string
  /// @param feedback New feedback JSON
  void update_goal_feedback(const std::string & goal_id, const json & feedback);

  /// Remove completed goals older than specified duration
  /// @param max_age Maximum age of completed goals to keep
  void cleanup_old_goals(std::chrono::seconds max_age = std::chrono::seconds(300));

  /// Subscribe to action status topic for real-time updates
  /// Called automatically when a goal is sent
  /// @param action_path Full action path (e.g., "/powertrain/engine/long_calibration")
  void subscribe_to_action_status(const std::string & action_path);

  /// Unsubscribe from action status topic
  /// Called when no more active goals exist for this action
  /// @param action_path Full action path
  void unsubscribe_from_action_status(const std::string & action_path);

 private:
  /// Convert JSON to YAML string for ros2 service call
  std::string json_to_yaml(const json & j);

  /// Parse YAML output from ros2 service call to JSON
  json parse_service_response(const std::string & yaml_output);

  /// Convert UUID hex string to array of byte values for YAML
  std::string uuid_to_yaml_array(const std::string & uuid_hex);

  /// Parse ros2 action send_goal CLI output to extract goal_id and status
  ActionSendGoalResult parse_send_goal_cli_output(const std::string & output);

  /// Parse ros2 action cancel output
  ActionCancelResult parse_cancel_output(const std::string & output);

  /// Track a new goal
  void track_goal(const std::string & goal_id, const std::string & action_path, const std::string & action_type);

  rclcpp::Node * node_;
  DiscoveryManager * discovery_manager_;
  std::unique_ptr<ROS2CLIWrapper> cli_wrapper_;

  /// Map of goal_id -> ActionGoalInfo for tracking active goals
  mutable std::mutex goals_mutex_;
  std::map<std::string, ActionGoalInfo> tracked_goals_;

  /// Callback for action status topic updates
  void on_action_status(const std::string & action_path, const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg);

  /// Convert goal UUID bytes to hex string
  std::string uuid_bytes_to_hex(const std::array<uint8_t, 16> & uuid) const;

  /// Map of action_path -> status subscription
  mutable std::mutex subscriptions_mutex_;
  std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> status_subscriptions_;
};

}  // namespace ros2_medkit_gateway
