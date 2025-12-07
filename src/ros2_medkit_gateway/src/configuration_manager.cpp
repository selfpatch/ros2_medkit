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

#include "ros2_medkit_gateway/configuration_manager.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace ros2_medkit_gateway {

ConfigurationManager::ConfigurationManager(rclcpp::Node * node) : node_(node) {
  // Create an internal node for parameter client operations
  // SyncParametersClient needs a node that is NOT added to any executor
  // so it can spin internally when waiting for service responses
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  param_node_ = std::make_shared<rclcpp::Node>("_param_client_node", options);
  RCLCPP_INFO(node_->get_logger(), "ConfigurationManager initialized");
}

std::shared_ptr<rclcpp::SyncParametersClient> ConfigurationManager::get_param_client(const std::string & node_name) {
  std::lock_guard<std::mutex> lock(clients_mutex_);

  auto it = param_clients_.find(node_name);
  if (it != param_clients_.end()) {
    return it->second;
  }

  // Create new client for this node using the internal param_node_
  // (not the main gateway node, which is already in an executor)
  auto client = std::make_shared<rclcpp::SyncParametersClient>(param_node_, node_name);
  param_clients_[node_name] = client;
  return client;
}

bool ConfigurationManager::is_node_available(const std::string & node_name) {
  auto client = get_param_client(node_name);
  return client->wait_for_service(1s);
}

ParameterResult ConfigurationManager::list_parameters(const std::string & node_name) {
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      return result;
    }

    // List all parameter names
    auto param_names = client->list_parameters({}, 0);

    // Get all parameter values
    auto parameters = client->get_parameters(param_names.names);

    json params_array = json::array();
    for (const auto & param : parameters) {
      json param_obj;
      param_obj["name"] = param.get_name();
      param_obj["value"] = parameter_value_to_json(param.get_parameter_value());
      param_obj["type"] = parameter_type_to_string(param.get_type());
      params_array.push_back(param_obj);
    }

    result.success = true;
    result.data = params_array;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to list parameters: ") + e.what();
  }

  return result;
}

ParameterResult ConfigurationManager::get_parameter(const std::string & node_name, const std::string & param_name) {
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      return result;
    }

    // Check if parameter exists
    auto param_names = client->list_parameters({param_name}, 1);
    if (param_names.names.empty()) {
      result.success = false;
      result.error_message = "Parameter not found: " + param_name;
      return result;
    }

    // Get parameter value
    auto parameters = client->get_parameters({param_name});
    if (parameters.empty()) {
      result.success = false;
      result.error_message = "Failed to get parameter: " + param_name;
      return result;
    }

    const auto & param = parameters[0];

    // Get parameter descriptor for additional metadata
    auto descriptors = client->describe_parameters({param_name});

    json param_obj;
    param_obj["name"] = param.get_name();
    param_obj["value"] = parameter_value_to_json(param.get_parameter_value());
    param_obj["type"] = parameter_type_to_string(param.get_type());

    if (!descriptors.empty()) {
      param_obj["description"] = descriptors[0].description;
      param_obj["read_only"] = descriptors[0].read_only;
    }

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to get parameter: ") + e.what();
  }

  return result;
}

ParameterResult ConfigurationManager::set_parameter(const std::string & node_name, const std::string & param_name,
                                                    const json & value) {
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      return result;
    }

    // Get current parameter to determine type hint
    auto current_params = client->get_parameters({param_name});
    rclcpp::ParameterType hint_type = rclcpp::ParameterType::PARAMETER_NOT_SET;
    if (!current_params.empty()) {
      hint_type = current_params[0].get_type();
    }

    // Convert JSON value to ROS2 parameter
    rclcpp::ParameterValue param_value = json_to_parameter_value(value, hint_type);
    rclcpp::Parameter param(param_name, param_value);

    // Set parameter
    auto results = client->set_parameters({param});
    if (results.empty() || !results[0].successful) {
      result.success = false;
      result.error_message = results.empty() ? "Failed to set parameter" : results[0].reason;
      return result;
    }

    // Return the new value
    json param_obj;
    param_obj["name"] = param_name;
    param_obj["value"] = parameter_value_to_json(param_value);
    param_obj["type"] = parameter_type_to_string(param_value.get_type());

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to set parameter: ") + e.what();
  }

  return result;
}

ParameterResult ConfigurationManager::describe_parameter(const std::string & node_name,
                                                         const std::string & param_name) {
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      return result;
    }

    auto descriptors = client->describe_parameters({param_name});
    if (descriptors.empty()) {
      result.success = false;
      result.error_message = "Parameter not found: " + param_name;
      return result;
    }

    const auto & desc = descriptors[0];
    json desc_obj;
    desc_obj["name"] = desc.name;
    desc_obj["type"] = parameter_type_to_string(static_cast<rclcpp::ParameterType>(desc.type));
    desc_obj["description"] = desc.description;
    desc_obj["read_only"] = desc.read_only;

    // Add constraints if available
    if (!desc.additional_constraints.empty()) {
      desc_obj["additional_constraints"] = desc.additional_constraints;
    }
    if (!desc.floating_point_range.empty()) {
      json range;
      range["from_value"] = desc.floating_point_range[0].from_value;
      range["to_value"] = desc.floating_point_range[0].to_value;
      range["step"] = desc.floating_point_range[0].step;
      desc_obj["floating_point_range"] = range;
    }
    if (!desc.integer_range.empty()) {
      json range;
      range["from_value"] = desc.integer_range[0].from_value;
      range["to_value"] = desc.integer_range[0].to_value;
      range["step"] = desc.integer_range[0].step;
      desc_obj["integer_range"] = range;
    }

    result.success = true;
    result.data = desc_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to describe parameter: ") + e.what();
  }

  return result;
}

std::string ConfigurationManager::parameter_type_to_string(rclcpp::ParameterType type) {
  switch (type) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return "bool";
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return "int";
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return "double";
    case rclcpp::ParameterType::PARAMETER_STRING:
      return "string";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return "byte_array";
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return "bool_array";
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return "int_array";
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return "double_array";
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return "string_array";
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return "not_set";
  }
}

json ConfigurationManager::parameter_value_to_json(const rclcpp::ParameterValue & value) {
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return value.get<bool>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return value.get<int64_t>();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return value.get<double>();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return value.get<std::string>();
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return json(value.get<std::vector<uint8_t>>());
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return json(value.get<std::vector<bool>>());
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return json(value.get<std::vector<int64_t>>());
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return json(value.get<std::vector<double>>());
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return json(value.get<std::vector<std::string>>());
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return nullptr;
  }
}

rclcpp::ParameterValue ConfigurationManager::json_to_parameter_value(const json & value,
                                                                     rclcpp::ParameterType hint_type) {
  // If we have a type hint, try to match it
  if (hint_type != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    switch (hint_type) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        if (value.is_boolean()) {
          return rclcpp::ParameterValue(value.get<bool>());
        }
        if (value.is_string()) {
          std::string s = value.get<std::string>();
          return rclcpp::ParameterValue(s == "true" || s == "1");
        }
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (value.is_number_integer()) {
          return rclcpp::ParameterValue(value.get<int64_t>());
        }
        if (value.is_number_float()) {
          return rclcpp::ParameterValue(static_cast<int64_t>(value.get<double>()));
        }
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        if (value.is_number()) {
          return rclcpp::ParameterValue(value.get<double>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        if (value.is_string()) {
          return rclcpp::ParameterValue(value.get<std::string>());
        }
        // Convert other types to string
        return rclcpp::ParameterValue(value.dump());
      case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<bool>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<bool>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<int64_t>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<int64_t>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<double>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<double>>());
        }
        break;
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        if (value.is_array()) {
          if (value.empty()) {
            return rclcpp::ParameterValue(std::vector<std::string>{});
          }
          return rclcpp::ParameterValue(value.get<std::vector<std::string>>());
        }
        break;
      default:
        break;
    }
  }

  // Infer type from JSON value
  if (value.is_boolean()) {
    return rclcpp::ParameterValue(value.get<bool>());
  }
  if (value.is_number_integer()) {
    return rclcpp::ParameterValue(value.get<int64_t>());
  }
  if (value.is_number_float()) {
    return rclcpp::ParameterValue(value.get<double>());
  }
  if (value.is_string()) {
    return rclcpp::ParameterValue(value.get<std::string>());
  }
  if (value.is_array()) {
    if (value.empty()) {
      // Empty array with no type hint - default to string array
      return rclcpp::ParameterValue(std::vector<std::string>{});
    }
    // Determine array type from first element
    if (value[0].is_boolean()) {
      return rclcpp::ParameterValue(value.get<std::vector<bool>>());
    }
    if (value[0].is_number_integer()) {
      return rclcpp::ParameterValue(value.get<std::vector<int64_t>>());
    }
    if (value[0].is_number_float()) {
      return rclcpp::ParameterValue(value.get<std::vector<double>>());
    }
    if (value[0].is_string()) {
      return rclcpp::ParameterValue(value.get<std::vector<std::string>>());
    }
  }

  // Default: convert to string
  return rclcpp::ParameterValue(value.dump());
}

}  // namespace ros2_medkit_gateway
