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

#include <sstream>
#include <regex>

#include <yaml-cpp/yaml.h>

namespace ros2_medkit_gateway {

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

std::string OperationManager::json_to_yaml(const json & j) {
  // For ros2 service call, we can pass JSON directly since JSON is valid YAML 1.2
  // But ros2 CLI sometimes prefers YAML format for complex types
  // For now, use JSON which works for most cases
  return j.dump();
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

    // Convert YAML to JSON
    json result;

    std::function<json(const YAML::Node &)> yaml_to_json = [&yaml_to_json](const YAML::Node & node) -> json {
      if (node.IsNull()) {
        return nullptr;
      }
      if (node.IsScalar()) {
        // Try to parse as different types
        try {
          return node.as<bool>();
        } catch (...) {
        }
        try {
          return node.as<int64_t>();
        } catch (...) {
        }
        try {
          return node.as<double>();
        } catch (...) {
        }
        return node.as<std::string>();
      }
      if (node.IsSequence()) {
        json arr = json::array();
        for (const auto & item : node) {
          arr.push_back(yaml_to_json(item));
        }
        return arr;
      }
      if (node.IsMap()) {
        json obj = json::object();
        for (const auto & kv : node) {
          obj[kv.first.as<std::string>()] = yaml_to_json(kv.second);
        }
        return obj;
      }
      return nullptr;
    };

    result = yaml_to_json(yaml_node);
    return result;

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

}  // namespace ros2_medkit_gateway
