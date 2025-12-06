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

#include "ros2_medkit_gateway/operation_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>

#include "ros2_medkit_gateway/output_parser.hpp"

namespace ros2_medkit_gateway {

/// UUID hex string length (16 bytes = 32 hex characters)
constexpr size_t kUuidHexLength = 32;

OperationManager::OperationManager(rclcpp::Node * node, DiscoveryManager * discovery_manager)
  : node_(node), discovery_manager_(discovery_manager), cli_wrapper_(std::make_unique<ROS2CLIWrapper>()) {
  if (!cli_wrapper_->is_command_available("ros2")) {
    RCLCPP_WARN(node_->get_logger(), "ROS 2 CLI not found, service calls may not be available");
  }

  RCLCPP_INFO(node_->get_logger(), "OperationManager initialized");
}

bool OperationManager::is_valid_message_type(const std::string & type) {
  // Valid formats:
  // - package/srv/Type (service)
  // - package/action/Type (action)
  // - package/msg/Type (message - for publishing)
  static const std::regex type_regex(R"(^[a-zA-Z_][a-zA-Z0-9_]*/(srv|action|msg)/[a-zA-Z_][a-zA-Z0-9_]*$)");
  return std::regex_match(type, type_regex);
}

bool OperationManager::is_service_type(const std::string & type) {
  return type.find("/srv/") != std::string::npos;
}

bool OperationManager::is_action_type(const std::string & type) {
  return type.find("/action/") != std::string::npos;
}

bool OperationManager::is_valid_uuid_hex(const std::string & uuid_hex) {
  if (uuid_hex.length() != kUuidHexLength) {
    return false;
  }
  return std::all_of(uuid_hex.begin(), uuid_hex.end(), [](char c) {
    return std::isxdigit(static_cast<unsigned char>(c));
  });
}

std::string OperationManager::json_to_yaml(const json & j) {
  // Convert JSON to YAML format for ros2 CLI commands
  // ros2 action/service commands expect YAML format: {key: value} not {"key": value}
  std::function<std::string(const json &)> convert = [&convert](const json & val) -> std::string {
    if (val.is_object()) {
      std::ostringstream ss;
      ss << "{";
      bool first = true;
      for (auto it = val.begin(); it != val.end(); ++it) {
        if (!first) {
          ss << ", ";
        }
        first = false;
        ss << it.key() << ": " << convert(it.value());
      }
      ss << "}";
      return ss.str();
    } else if (val.is_array()) {
      std::ostringstream ss;
      ss << "[";
      bool first = true;
      for (const auto & item : val) {
        if (!first) {
          ss << ", ";
        }
        first = false;
        ss << convert(item);
      }
      ss << "]";
      return ss.str();
    } else if (val.is_string()) {
      // Quote strings that contain special YAML characters
      std::string s = val.get<std::string>();
      if (s.find_first_of(":{}[],\"'") != std::string::npos) {
        return "'" + s + "'";
      }
      return s;
    } else {
      return val.dump();  // numbers, bools, null
    }
  };

  return convert(j);
}

json OperationManager::parse_service_response(const std::string & yaml_output) {
  try {
    // ros2 service call output format:
    // requester: making request: ...
    // response:
    // std_srvs.srv.Trigger_Response(success=True, message='...')
    //
    // Or for newer ROS2 (YAML format):
    // response:
    // success: true
    // message: '...'

    // Find the response section
    std::string response_section;
    size_t response_pos = yaml_output.find("response:");
    if (response_pos != std::string::npos) {
      // Skip "response:" and any newline
      size_t start = response_pos + 9;  // length of "response:"
      while (start < yaml_output.size() && (yaml_output[start] == '\n' || yaml_output[start] == ' ')) {
        start++;
      }
      response_section = yaml_output.substr(start);
      // Trim trailing whitespace and newlines
      while (!response_section.empty() &&
             (response_section.back() == '\n' || response_section.back() == ' ' || response_section.back() == '\r')) {
        response_section.pop_back();
      }
    } else {
      // Try parsing the whole output
      response_section = yaml_output;
    }

    // Check if it's in Python repr format: TypeName(field=value, ...)
    // Example: std_srvs.srv.Trigger_Response(success=True, message='...')
    static const std::regex repr_regex(R"(^[\w.]+\((.*)\)$)");
    std::smatch match;
    if (std::regex_match(response_section, match, repr_regex)) {
      // Parse Python repr format
      std::string fields_str = match[1].str();
      json result = json::object();

      // Parse field=value pairs
      // Handle nested parentheses and quoted strings
      size_t pos = 0;
      while (pos < fields_str.length()) {
        // Skip whitespace
        while (pos < fields_str.length() && (fields_str[pos] == ' ' || fields_str[pos] == ',')) {
          pos++;
        }
        if (pos >= fields_str.length()) {
          break;
        }

        // Find field name (up to '=')
        size_t eq_pos = fields_str.find('=', pos);
        if (eq_pos == std::string::npos) {
          break;
        }
        std::string field_name = fields_str.substr(pos, eq_pos - pos);
        pos = eq_pos + 1;

        // Parse value
        std::string value_str;
        if (pos < fields_str.length() && fields_str[pos] == '\'') {
          // Quoted string
          pos++;  // Skip opening quote
          size_t end_quote = pos;
          while (end_quote < fields_str.length()) {
            if (fields_str[end_quote] == '\'') {
              // Check if it's escaped
              if (end_quote > 0 && fields_str[end_quote - 1] == '\\') {
                end_quote++;
                continue;
              }
              break;
            }
            end_quote++;
          }
          value_str = fields_str.substr(pos, end_quote - pos);
          pos = end_quote + 1;
          result[field_name] = value_str;
        } else {
          // Non-quoted value (bool, number, etc.)
          size_t comma_pos = fields_str.find(',', pos);
          size_t paren_pos = fields_str.find(')', pos);
          size_t end_pos = std::min(comma_pos, paren_pos);
          if (end_pos == std::string::npos) {
            end_pos = fields_str.length();
          }
          value_str = fields_str.substr(pos, end_pos - pos);
          pos = end_pos;

          // Convert to appropriate JSON type
          if (value_str == "True" || value_str == "true") {
            result[field_name] = true;
          } else if (value_str == "False" || value_str == "false") {
            result[field_name] = false;
          } else if (value_str == "None" || value_str == "null") {
            result[field_name] = nullptr;
          } else {
            // Try to parse as number
            try {
              if (value_str.find('.') != std::string::npos) {
                result[field_name] = std::stod(value_str);
              } else {
                result[field_name] = std::stoll(value_str);
              }
            } catch (...) {
              result[field_name] = value_str;
            }
          }
        }
      }
      return result;
    }

    // Try to parse as YAML (for newer ROS2 format)
    YAML::Node yaml_node = YAML::Load(response_section);

    // Convert YAML to JSON using shared utility
    return OutputParser::yaml_to_json(yaml_node);

  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to parse service response: %s", e.what());
    // Return raw output as string
    return json{{"raw_response", yaml_output}};
  }
}

ServiceCallResult OperationManager::call_service(const std::string & service_path, const std::string & service_type,
                                                 const json & request) {
  ServiceCallResult result;

  try {
    // Build ros2 service call command
    std::ostringstream cmd;
    cmd << "ros2 service call " << ROS2CLIWrapper::escape_shell_arg(service_path) << " "
        << ROS2CLIWrapper::escape_shell_arg(service_type);

    // Add request data if not empty
    if (!request.empty() && !request.is_null()) {
      cmd << " " << ROS2CLIWrapper::escape_shell_arg(json_to_yaml(request));
    }

    RCLCPP_INFO(node_->get_logger(), "Calling service: %s (type: %s)", service_path.c_str(), service_type.c_str());

    std::string output = cli_wrapper_->exec(cmd.str());

    RCLCPP_DEBUG(node_->get_logger(), "Service call output: %s", output.c_str());

    result.success = true;
    result.response = parse_service_response(output);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed for '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

ServiceCallResult OperationManager::call_component_service(const std::string & component_ns,
                                                           const std::string & operation_name,
                                                           const std::optional<std::string> & service_type,
                                                           const json & request) {
  ServiceCallResult result;

  // Determine service type - use provided or look up from discovery
  std::string resolved_type;
  std::string service_path;

  if (service_type.has_value() && !service_type->empty()) {
    resolved_type = *service_type;
    // Construct service path from component namespace and operation name
    service_path = component_ns;
    if (!service_path.empty() && service_path.back() != '/') {
      service_path += "/";
    }
    service_path += operation_name;
  } else {
    // Look up from discovery cache
    auto service_info = discovery_manager_->find_service(component_ns, operation_name);
    if (!service_info.has_value()) {
      result.success = false;
      result.error_message = "Service not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    resolved_type = service_info->type;
    service_path = service_info->full_path;
  }

  // Validate type format
  if (!is_valid_message_type(resolved_type)) {
    result.success = false;
    result.error_message = "Invalid service type format: " + resolved_type;
    return result;
  }

  // Verify it's a service type
  if (!is_service_type(resolved_type)) {
    result.success = false;
    result.error_message = "Type is not a service type: " + resolved_type;
    return result;
  }

  return call_service(service_path, resolved_type, request);
}

// ==================== ACTION OPERATIONS ====================

std::string action_status_to_string(ActionGoalStatus status) {
  switch (status) {
    case ActionGoalStatus::UNKNOWN:
      return "unknown";
    case ActionGoalStatus::ACCEPTED:
      return "accepted";
    case ActionGoalStatus::EXECUTING:
      return "executing";
    case ActionGoalStatus::CANCELING:
      return "canceling";
    case ActionGoalStatus::SUCCEEDED:
      return "succeeded";
    case ActionGoalStatus::CANCELED:
      return "canceled";
    case ActionGoalStatus::ABORTED:
      return "aborted";
    default:
      return "unknown";
  }
}

std::string OperationManager::uuid_to_yaml_array(const std::string & uuid_hex) {
  // Convert hex string to YAML array of byte values [b1, b2, ...]
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < uuid_hex.length() && i < kUuidHexLength; i += 2) {
    if (i > 0) {
      ss << ", ";
    }
    int byte_val = std::stoi(uuid_hex.substr(i, 2), nullptr, 16);
    ss << byte_val;
  }
  ss << "]";
  return ss.str();
}

ActionSendGoalResult OperationManager::parse_send_goal_cli_output(const std::string & output) {
  ActionSendGoalResult result;
  result.success = false;
  result.goal_accepted = false;

  // Look for "Goal accepted with ID: <uuid>"
  static const std::regex goal_id_regex(R"(Goal accepted with ID:\s*([a-f0-9]+))");
  std::smatch match;
  if (std::regex_search(output, match, goal_id_regex)) {
    result.goal_id = match[1].str();
    result.goal_accepted = true;
    result.success = true;
  }

  // Check for rejection
  if (output.find("Goal rejected") != std::string::npos) {
    result.success = true;  // Command worked, but goal was rejected
    result.goal_accepted = false;
    result.error_message = "Goal rejected by action server";
  }

  // Check for server not available
  if (output.find("Action server is not available") != std::string::npos ||
      output.find("not available after waiting") != std::string::npos) {
    result.success = false;
    result.error_message = "Action server not available";
  }

  return result;
}

ActionCancelResult OperationManager::parse_cancel_output(const std::string & output) {
  ActionCancelResult result;
  result.success = true;
  result.return_code = 0;  // ERROR_NONE

  // Check output for success/failure
  if (output.find("Cancel request accepted") != std::string::npos || output.find("canceling") != std::string::npos) {
    result.return_code = 0;
  } else if (output.find("not found") != std::string::npos || output.find("unknown") != std::string::npos) {
    result.return_code = 2;  // ERROR_UNKNOWN_GOAL_ID
    result.error_message = "Unknown goal ID";
  } else if (output.find("rejected") != std::string::npos) {
    result.return_code = 1;  // ERROR_REJECTED
    result.error_message = "Cancel request rejected";
  } else if (output.find("terminated") != std::string::npos || output.find("already finished") != std::string::npos) {
    result.return_code = 3;  // ERROR_GOAL_TERMINATED
    result.error_message = "Goal already terminated";
  }

  return result;
}

void OperationManager::track_goal(const std::string & goal_id, const std::string & action_path,
                                  const std::string & action_type) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  ActionGoalInfo info;
  info.goal_id = goal_id;
  info.action_path = action_path;
  info.action_type = action_type;
  info.status = ActionGoalStatus::ACCEPTED;
  info.created_at = std::chrono::system_clock::now();
  info.last_update = info.created_at;
  tracked_goals_[goal_id] = info;
}

ActionSendGoalResult OperationManager::send_action_goal(const std::string & action_path,
                                                        const std::string & action_type, const json & goal) {
  ActionSendGoalResult result;

  try {
    // Use ros2 action send_goal with short timeout (3s)
    // This is enough for discovery + goal acceptance, but not for long-running actions
    // The action continues running in background after we get the goal_id
    std::ostringstream cmd;
    cmd << "ros2 action send_goal " << ROS2CLIWrapper::escape_shell_arg(action_path) << " "
        << ROS2CLIWrapper::escape_shell_arg(action_type);

    // Add goal data
    if (!goal.empty() && !goal.is_null()) {
      cmd << " " << ROS2CLIWrapper::escape_shell_arg(json_to_yaml(goal));
    } else {
      cmd << " '{}'";
    }

    // Short timeout: enough for discovery + acceptance, action continues in background
    cmd << " -t 3";

    RCLCPP_INFO(node_->get_logger(), "Sending action goal: %s (type: %s)", action_path.c_str(), action_type.c_str());

    std::string output = cli_wrapper_->exec(cmd.str());
    RCLCPP_DEBUG(node_->get_logger(), "Action send_goal output: %s", output.c_str());

    // Parse goal_id from CLI output
    result = parse_send_goal_cli_output(output);

    // Track the goal if accepted
    if (result.goal_accepted && !result.goal_id.empty()) {
      track_goal(result.goal_id, action_path, action_type);
      RCLCPP_INFO(node_->get_logger(), "Action goal accepted with ID: %s", result.goal_id.c_str());

      // Subscribe to status topic for real-time updates
      subscribe_to_action_status(action_path);

      // Check if action already completed (fast actions) or still running
      if (output.find("Goal finished with status: SUCCEEDED") != std::string::npos) {
        update_goal_status(result.goal_id, ActionGoalStatus::SUCCEEDED);
      } else if (output.find("Goal finished with status: CANCELED") != std::string::npos) {
        update_goal_status(result.goal_id, ActionGoalStatus::CANCELED);
      } else if (output.find("Goal finished with status: ABORTED") != std::string::npos) {
        update_goal_status(result.goal_id, ActionGoalStatus::ABORTED);
      } else {
        // Still executing (timed out waiting for result, which is expected)
        update_goal_status(result.goal_id, ActionGoalStatus::EXECUTING);
      }
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Action send_goal failed for '%s': %s", action_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

ActionSendGoalResult OperationManager::send_component_action_goal(const std::string & component_ns,
                                                                  const std::string & operation_name,
                                                                  const std::optional<std::string> & action_type,
                                                                  const json & goal) {
  ActionSendGoalResult result;

  // Determine action type - use provided or look up from discovery
  std::string resolved_type;
  std::string action_path;

  if (action_type.has_value() && !action_type->empty()) {
    resolved_type = *action_type;
    // Construct action path from component namespace and operation name
    action_path = component_ns;
    if (!action_path.empty() && action_path.back() != '/') {
      action_path += "/";
    }
    action_path += operation_name;
  } else {
    // Look up from discovery cache
    auto action_info = discovery_manager_->find_action(component_ns, operation_name);
    if (!action_info.has_value()) {
      result.success = false;
      result.error_message = "Action not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    resolved_type = action_info->type;
    action_path = action_info->full_path;
  }

  // Validate type format
  if (!is_valid_message_type(resolved_type)) {
    result.success = false;
    result.error_message = "Invalid action type format: " + resolved_type;
    return result;
  }

  // Verify it's an action type
  if (!is_action_type(resolved_type)) {
    result.success = false;
    result.error_message = "Type is not an action type: " + resolved_type;
    return result;
  }

  return send_action_goal(action_path, resolved_type, goal);
}

ActionCancelResult OperationManager::cancel_action_goal(const std::string & action_path, const std::string & goal_id) {
  ActionCancelResult result;

  // Validate goal_id format
  if (!is_valid_uuid_hex(goal_id)) {
    result.success = false;
    result.error_message = "Invalid goal_id format: must be 32 hex characters";
    return result;
  }

  try {
    // Build cancel command - ros2 action doesn't have a direct cancel by goal_id
    // We need to use the internal service
    std::ostringstream cmd;
    cmd << "ros2 service call " << ROS2CLIWrapper::escape_shell_arg(action_path + "/_action/cancel_goal")
        << " action_msgs/srv/CancelGoal ";

    // Convert goal_id hex string to UUID bytes array
    // Format: {goal_info: {goal_id: {uuid: [b1, b2, ...]}}}
    cmd << "'{goal_info: {goal_id: {uuid: " << uuid_to_yaml_array(goal_id) << "}}}'";

    RCLCPP_INFO(node_->get_logger(), "Canceling action goal: %s (goal_id: %s)", action_path.c_str(), goal_id.c_str());

    std::string output = cli_wrapper_->exec(cmd.str());
    RCLCPP_DEBUG(node_->get_logger(), "Action cancel output: %s", output.c_str());

    result = parse_cancel_output(output);

    // Update goal status if cancel was accepted
    if (result.success && result.return_code == 0) {
      update_goal_status(goal_id, ActionGoalStatus::CANCELING);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Action cancel failed for '%s': %s", action_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

ActionGetResultResult OperationManager::get_action_result(const std::string & action_path,
                                                          const std::string & action_type,
                                                          const std::string & goal_id) {
  ActionGetResultResult result;
  result.success = false;
  result.status = ActionGoalStatus::UNKNOWN;

  // Validate goal_id format
  if (!is_valid_uuid_hex(goal_id)) {
    result.error_message = "Invalid goal_id format: must be 32 hex characters";
    return result;
  }

  try {
    // Build get_result service call
    // Service type is {action_type}_GetResult
    std::string get_result_type = action_type;
    // Replace /action/ with action:: for service type naming
    // e.g., example_interfaces/action/Fibonacci -> example_interfaces/action/Fibonacci_GetResult
    get_result_type += "_GetResult";

    std::ostringstream cmd;
    cmd << "ros2 service call " << ROS2CLIWrapper::escape_shell_arg(action_path + "/_action/get_result") << " "
        << ROS2CLIWrapper::escape_shell_arg(get_result_type) << " ";

    // Convert goal_id to UUID bytes array
    cmd << "'{goal_id: {uuid: " << uuid_to_yaml_array(goal_id) << "}}'";

    RCLCPP_INFO(node_->get_logger(), "Getting action result: %s (goal_id: %s)", action_path.c_str(), goal_id.c_str());

    std::string output = cli_wrapper_->exec(cmd.str());
    RCLCPP_DEBUG(node_->get_logger(), "Action get_result output: %s", output.c_str());

    // Parse the response
    json response = parse_service_response(output);
    result.success = true;

    // Extract status from response
    if (response.contains("status")) {
      int status_val = response["status"].get<int>();
      result.status = static_cast<ActionGoalStatus>(status_val);
    }

    // Extract result from response
    if (response.contains("result")) {
      result.result = response["result"];
    } else {
      result.result = response;
    }

    // Update local tracking
    update_goal_status(goal_id, result.status);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Get action result failed for '%s': %s", action_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

std::optional<ActionGoalInfo> OperationManager::get_tracked_goal(const std::string & goal_id) const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<ActionGoalInfo> OperationManager::list_tracked_goals() const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  std::vector<ActionGoalInfo> goals;
  goals.reserve(tracked_goals_.size());
  for (const auto & [id, info] : tracked_goals_) {
    goals.push_back(info);
  }
  return goals;
}

std::vector<ActionGoalInfo> OperationManager::get_goals_for_action(const std::string & action_path) const {
  std::vector<ActionGoalInfo> goals;

  // Copy data under lock
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    for (const auto & [id, info] : tracked_goals_) {
      if (info.action_path == action_path) {
        goals.push_back(info);
      }
    }
  }  // Release lock before sorting

  // Sort by last_update, newest first
  std::sort(goals.begin(), goals.end(), [](const ActionGoalInfo & a, const ActionGoalInfo & b) {
    return a.last_update > b.last_update;
  });
  return goals;
}

std::optional<ActionGoalInfo> OperationManager::get_latest_goal_for_action(const std::string & action_path) const {
  auto goals = get_goals_for_action(action_path);
  if (goals.empty()) {
    return std::nullopt;
  }
  return goals.front();  // Already sorted newest first
}

void OperationManager::update_goal_status(const std::string & goal_id, ActionGoalStatus status) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    it->second.status = status;
    it->second.last_update = std::chrono::system_clock::now();
  }
}

void OperationManager::update_goal_feedback(const std::string & goal_id, const json & feedback) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    it->second.last_feedback = feedback;
    it->second.last_update = std::chrono::system_clock::now();
  }
}

void OperationManager::cleanup_old_goals(std::chrono::seconds max_age) {
  std::set<std::string> actions_to_check;

  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto now = std::chrono::system_clock::now();

    for (auto it = tracked_goals_.begin(); it != tracked_goals_.end();) {
      // Only remove completed goals (succeeded, canceled, aborted)
      if (it->second.status == ActionGoalStatus::SUCCEEDED || it->second.status == ActionGoalStatus::CANCELED ||
          it->second.status == ActionGoalStatus::ABORTED) {
        auto age = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update);
        if (age > max_age) {
          actions_to_check.insert(it->second.action_path);
          it = tracked_goals_.erase(it);
          continue;
        }
      }
      ++it;
    }
  }  // Release goals_mutex_ before checking subscriptions

  // Check if any action paths need to be unsubscribed
  for (const auto & action_path : actions_to_check) {
    auto remaining_goals = get_goals_for_action(action_path);
    if (remaining_goals.empty()) {
      unsubscribe_from_action_status(action_path);
    }
  }
}

// ==================== NATIVE STATUS SUBSCRIPTION ====================

std::string OperationManager::uuid_bytes_to_hex(const std::array<uint8_t, 16> & uuid) const {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto & byte : uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

void OperationManager::subscribe_to_action_status(const std::string & action_path) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  // Check if already subscribed
  if (status_subscriptions_.count(action_path) > 0) {
    return;
  }

  // Create subscription to status topic
  std::string status_topic = action_path + "/_action/status";

  auto callback = [this, action_path](const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
    on_action_status(action_path, msg);
  };

  auto subscription = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      status_topic, rclcpp::QoS(10).best_effort(), callback);

  status_subscriptions_[action_path] = subscription;
  RCLCPP_INFO(node_->get_logger(), "Subscribed to action status: %s", status_topic.c_str());
}

void OperationManager::unsubscribe_from_action_status(const std::string & action_path) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  auto it = status_subscriptions_.find(action_path);
  if (it != status_subscriptions_.end()) {
    status_subscriptions_.erase(it);
    RCLCPP_INFO(node_->get_logger(), "Unsubscribed from action status: %s/_action/status", action_path.c_str());
  }
}

void OperationManager::on_action_status(const std::string & action_path,
                                        const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
  std::lock_guard<std::mutex> lock(goals_mutex_);

  for (const auto & status : msg->status_list) {
    // Convert UUID bytes to hex string
    std::string goal_id = uuid_bytes_to_hex(status.goal_info.goal_id.uuid);

    // Find if we're tracking this goal
    auto it = tracked_goals_.find(goal_id);
    if (it != tracked_goals_.end() && it->second.action_path == action_path) {
      // Map status code to our enum
      ActionGoalStatus new_status;
      switch (status.status) {
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
          new_status = ActionGoalStatus::ACCEPTED;
          break;
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
          new_status = ActionGoalStatus::EXECUTING;
          break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
          new_status = ActionGoalStatus::CANCELING;
          break;
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
          new_status = ActionGoalStatus::SUCCEEDED;
          break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
          new_status = ActionGoalStatus::CANCELED;
          break;
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
          new_status = ActionGoalStatus::ABORTED;
          break;
        default:
          new_status = ActionGoalStatus::UNKNOWN;
          break;
      }

      // Only update if status changed
      if (it->second.status != new_status) {
        RCLCPP_INFO(node_->get_logger(), "Goal %s status update: %s -> %s", goal_id.c_str(),
                    action_status_to_string(it->second.status).c_str(), action_status_to_string(new_status).c_str());
        it->second.status = new_status;
        it->second.last_update = std::chrono::system_clock::now();
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
