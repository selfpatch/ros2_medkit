// Copyright 2026 Selfpatch
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

#ifndef ROS2_MEDKIT_SERIALIZATION__SERVICE_ACTION_TYPES_HPP_
#define ROS2_MEDKIT_SERIALIZATION__SERVICE_ACTION_TYPES_HPP_

#include <optional>
#include <string>
#include <utility>

namespace ros2_medkit_serialization {

/// Helper functions for working with service and action types.
///
/// ROS 2 services have Request and Response types, while actions have
/// Goal, Result, and Feedback types. This class provides utilities to
/// derive these sub-types from the main service/action type.
class ServiceActionTypes {
 public:
  /// Get the request type for a service
  ///
  /// @param service_type Full service type (e.g., "std_srvs/srv/SetBool")
  /// @return Request type string (e.g., "std_srvs/srv/SetBool_Request")
  static std::string get_service_request_type(const std::string & service_type);

  /// Get the response type for a service
  ///
  /// @param service_type Full service type (e.g., "std_srvs/srv/SetBool")
  /// @return Response type string (e.g., "std_srvs/srv/SetBool_Response")
  static std::string get_service_response_type(const std::string & service_type);

  /// Get the goal type for an action
  ///
  /// @param action_type Full action type (e.g., "action_tutorials_interfaces/action/Fibonacci")
  /// @return Goal type string
  static std::string get_action_goal_type(const std::string & action_type);

  /// Get the result type for an action
  ///
  /// @param action_type Full action type
  /// @return Result type string
  static std::string get_action_result_type(const std::string & action_type);

  /// Get the feedback type for an action
  ///
  /// @param action_type Full action type
  /// @return Feedback type string
  static std::string get_action_feedback_type(const std::string & action_type);

  // ==================== Action Internal Service Types ====================
  // Actions are implemented as a set of internal services:
  // - {action}/_action/send_goal (type: {ActionType}_SendGoal)
  // - {action}/_action/cancel_goal (type: action_msgs/srv/CancelGoal)
  // - {action}/_action/get_result (type: {ActionType}_GetResult)

  /// Get the SendGoal service type for an action
  ///
  /// @param action_type Full action type (e.g., "example_interfaces/action/Fibonacci")
  /// @return Service type string (e.g., "example_interfaces/action/Fibonacci_SendGoal")
  static std::string get_action_send_goal_service_type(const std::string & action_type);

  /// Get the GetResult service type for an action
  ///
  /// @param action_type Full action type
  /// @return Service type string (e.g., "example_interfaces/action/Fibonacci_GetResult")
  static std::string get_action_get_result_service_type(const std::string & action_type);

  /// Get the SendGoal request type for an action (used with GenericClient)
  ///
  /// @param action_type Full action type
  /// @return Request message type string (e.g., "example_interfaces/action/Fibonacci_SendGoal_Request")
  static std::string get_action_send_goal_request_type(const std::string & action_type);

  /// Get the SendGoal response type for an action
  ///
  /// @param action_type Full action type
  /// @return Response message type string (e.g., "example_interfaces/action/Fibonacci_SendGoal_Response")
  static std::string get_action_send_goal_response_type(const std::string & action_type);

  /// Get the GetResult request type for an action (used with GenericClient)
  ///
  /// @param action_type Full action type
  /// @return Request message type string (e.g., "example_interfaces/action/Fibonacci_GetResult_Request")
  static std::string get_action_get_result_request_type(const std::string & action_type);

  /// Get the GetResult response type for an action
  ///
  /// @param action_type Full action type
  /// @return Response message type string (e.g., "example_interfaces/action/Fibonacci_GetResult_Response")
  static std::string get_action_get_result_response_type(const std::string & action_type);

  /// Get the FeedbackMessage type for an action (used for feedback subscription)
  ///
  /// @param action_type Full action type
  /// @return Message type string (e.g., "example_interfaces/action/Fibonacci_FeedbackMessage")
  static std::string get_action_feedback_message_type(const std::string & action_type);

  /// Parse a service/action type into components
  ///
  /// @param full_type Full type string (e.g., "std_srvs/srv/SetBool")
  /// @return Tuple of (package, interface_type, name), or nullopt if invalid
  ///         interface_type is "msg", "srv", or "action"
  static std::optional<std::tuple<std::string, std::string, std::string>>
  parse_interface_type(const std::string & full_type);

  /// Check if a type is a service type
  ///
  /// @param full_type Full type string
  /// @return true if the type contains "/srv/"
  static bool is_service_type(const std::string & full_type);

  /// Check if a type is an action type
  ///
  /// @param full_type Full type string
  /// @return true if the type contains "/action/"
  static bool is_action_type(const std::string & full_type);

  /// Check if a type is a message type
  ///
  /// @param full_type Full type string
  /// @return true if the type contains "/msg/"
  static bool is_message_type(const std::string & full_type);
};

}  // namespace ros2_medkit_serialization

#endif  // ROS2_MEDKIT_SERIALIZATION__SERVICE_ACTION_TYPES_HPP_
