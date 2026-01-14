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

#include "ros2_medkit_serialization/service_action_types.hpp"

#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_serialization {

std::string ServiceActionTypes::get_service_request_type(const std::string & service_type) {
  return service_type + "_Request";
}

std::string ServiceActionTypes::get_service_response_type(const std::string & service_type) {
  return service_type + "_Response";
}

std::string ServiceActionTypes::get_action_goal_type(const std::string & action_type) {
  return action_type + "_Goal";
}

std::string ServiceActionTypes::get_action_result_type(const std::string & action_type) {
  return action_type + "_Result";
}

std::string ServiceActionTypes::get_action_feedback_type(const std::string & action_type) {
  return action_type + "_Feedback";
}

std::string ServiceActionTypes::get_action_send_goal_service_type(const std::string & action_type) {
  return action_type + "_SendGoal";
}

std::string ServiceActionTypes::get_action_get_result_service_type(const std::string & action_type) {
  return action_type + "_GetResult";
}

std::string ServiceActionTypes::get_action_send_goal_request_type(const std::string & action_type) {
  return action_type + "_SendGoal_Request";
}

std::string ServiceActionTypes::get_action_send_goal_response_type(const std::string & action_type) {
  return action_type + "_SendGoal_Response";
}

std::string ServiceActionTypes::get_action_get_result_request_type(const std::string & action_type) {
  return action_type + "_GetResult_Request";
}

std::string ServiceActionTypes::get_action_get_result_response_type(const std::string & action_type) {
  return action_type + "_GetResult_Response";
}

std::string ServiceActionTypes::get_action_feedback_message_type(const std::string & action_type) {
  return action_type + "_FeedbackMessage";
}

std::optional<std::tuple<std::string, std::string, std::string>>
ServiceActionTypes::parse_interface_type(const std::string & full_type) {
  // Delegate to TypeCache::parse_type_string to avoid duplicate regex
  return TypeCache::parse_type_string(full_type);
}

bool ServiceActionTypes::is_service_type(const std::string & full_type) {
  return full_type.find("/srv/") != std::string::npos;
}

bool ServiceActionTypes::is_action_type(const std::string & full_type) {
  return full_type.find("/action/") != std::string::npos;
}

bool ServiceActionTypes::is_message_type(const std::string & full_type) {
  return full_type.find("/msg/") != std::string::npos;
}

}  // namespace ros2_medkit_serialization
