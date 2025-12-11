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
  // Don't inherit node name/namespace from global arguments (--ros-args -r __node:=...)
  // Without this, the internal node would have the same name as the main gateway node
  options.use_global_arguments(false);
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

ParameterResult ConfigurationManager::list_parameters(const std::string & node_name) {
  ParameterResult result;

  RCLCPP_DEBUG(node_->get_logger(), "list_parameters called for node: '%s'", node_name.c_str());

  // Cache default values on first access to this node
  cache_default_values(node_name);

  try {
    auto client = get_param_client(node_name);

    RCLCPP_DEBUG(node_->get_logger(), "Got param client for node: '%s'", node_name.c_str());

    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      RCLCPP_WARN(node_->get_logger(), "Parameter service not available for node: '%s'", node_name.c_str());
      return result;
    }

    RCLCPP_DEBUG(node_->get_logger(), "Service ready for node: '%s'", node_name.c_str());

    // List all parameter names
    auto param_names = client->list_parameters({}, 0);

    RCLCPP_DEBUG(node_->get_logger(), "Got %zu parameter names for node: '%s'", param_names.names.size(),
                 node_name.c_str());

    // Log first few parameter names for debugging
    for (size_t i = 0; i < std::min(param_names.names.size(), static_cast<size_t>(5)); ++i) {
      RCLCPP_DEBUG(node_->get_logger(), "  param[%zu]: '%s'", i, param_names.names[i].c_str());
    }

    // Get all parameter values - try in smaller batches if needed
    std::vector<rclcpp::Parameter> parameters;

    // First try getting all at once
    parameters = client->get_parameters(param_names.names);

    if (parameters.empty() && !param_names.names.empty()) {
      RCLCPP_WARN(node_->get_logger(), "get_parameters returned empty, trying one by one for node: '%s'",
                  node_name.c_str());
      // Try getting parameters one by one
      for (const auto & name : param_names.names) {
        try {
          auto single_params = client->get_parameters({name});
          if (!single_params.empty()) {
            parameters.push_back(single_params[0]);
          }
        } catch (const std::exception & e) {
          RCLCPP_DEBUG(node_->get_logger(), "Failed to get param '%s': %s", name.c_str(), e.what());
        }
      }
    }

    RCLCPP_DEBUG(node_->get_logger(), "Got %zu parameter values for node: '%s'", parameters.size(), node_name.c_str());

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
    RCLCPP_ERROR(node_->get_logger(), "Exception in list_parameters for node '%s': %s", node_name.c_str(), e.what());
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

void ConfigurationManager::cache_default_values(const std::string & node_name) {
  std::lock_guard<std::mutex> lock(defaults_mutex_);

  // Check if already cached
  if (default_values_.find(node_name) != default_values_.end()) {
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Caching default values for node: '%s'", node_name.c_str());

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(2s)) {
      RCLCPP_WARN(node_->get_logger(), "Cannot cache defaults - service not available for node: '%s'",
                  node_name.c_str());
      return;
    }

    // List all parameter names
    auto param_names = client->list_parameters({}, 0);

    // Get all parameter values
    std::vector<rclcpp::Parameter> parameters;
    parameters = client->get_parameters(param_names.names);

    // Fallback to one-by-one if batch fails
    if (parameters.empty() && !param_names.names.empty()) {
      for (const auto & name : param_names.names) {
        try {
          auto single_params = client->get_parameters({name});
          if (!single_params.empty()) {
            parameters.push_back(single_params[0]);
          }
        } catch (const std::exception &) {
          // Skip failed parameters
        }
      }
    }

    // Store as defaults
    std::map<std::string, rclcpp::Parameter> node_defaults;
    for (const auto & param : parameters) {
      node_defaults[param.get_name()] = param;
    }
    default_values_[node_name] = std::move(node_defaults);

    RCLCPP_DEBUG(node_->get_logger(), "Cached %zu default values for node: '%s'", default_values_[node_name].size(),
                 node_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to cache defaults for node '%s': %s", node_name.c_str(), e.what());
  }
}

ParameterResult ConfigurationManager::reset_parameter(const std::string & node_name, const std::string & param_name) {
  ParameterResult result;

  try {
    // Ensure defaults are cached
    cache_default_values(node_name);

    // Look up default value
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    auto node_it = default_values_.find(node_name);
    if (node_it == default_values_.end()) {
      result.success = false;
      result.error_message = "No default values cached for node: " + node_name;
      return result;
    }

    auto param_it = node_it->second.find(param_name);
    if (param_it == node_it->second.end()) {
      result.success = false;
      result.error_message = "No default value for parameter: " + param_name;
      return result;
    }

    const auto & default_param = param_it->second;

    // Set parameter back to default value
    auto client = get_param_client(node_name);
    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      return result;
    }

    auto results = client->set_parameters({default_param});
    if (results.empty() || !results[0].successful) {
      result.success = false;
      result.error_message = results.empty() ? "Failed to reset parameter" : results[0].reason;
      return result;
    }

    // Return the reset value
    json param_obj;
    param_obj["name"] = param_name;
    param_obj["value"] = parameter_value_to_json(default_param.get_parameter_value());
    param_obj["type"] = parameter_type_to_string(default_param.get_type());
    param_obj["reset_to_default"] = true;

    result.success = true;
    result.data = param_obj;

    RCLCPP_INFO(node_->get_logger(), "Reset parameter '%s' on node '%s' to default value", param_name.c_str(),
                node_name.c_str());
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to reset parameter: ") + e.what();
  }

  return result;
}

ParameterResult ConfigurationManager::reset_all_parameters(const std::string & node_name) {
  ParameterResult result;

  try {
    // Ensure defaults are cached
    cache_default_values(node_name);

    // Look up default values
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    auto node_it = default_values_.find(node_name);
    if (node_it == default_values_.end()) {
      result.success = false;
      result.error_message = "No default values cached for node: " + node_name;
      return result;
    }

    auto client = get_param_client(node_name);
    if (!client->wait_for_service(2s)) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      return result;
    }

    // Collect all default parameters
    std::vector<rclcpp::Parameter> params_to_reset;
    for (const auto & [name, param] : node_it->second) {
      params_to_reset.push_back(param);
    }

    // Reset all parameters
    size_t reset_count = 0;
    size_t failed_count = 0;
    json failed_params = json::array();

    // Set parameters one by one to handle partial failures
    for (const auto & param : params_to_reset) {
      try {
        auto results = client->set_parameters({param});
        if (!results.empty() && results[0].successful) {
          reset_count++;
        } else {
          failed_count++;
          failed_params.push_back(param.get_name());
        }
      } catch (const std::exception &) {
        failed_count++;
        failed_params.push_back(param.get_name());
      }
    }

    json response;
    response["node_name"] = node_name;
    response["reset_count"] = reset_count;
    response["failed_count"] = failed_count;
    if (!failed_params.empty()) {
      response["failed_parameters"] = failed_params;
    }

    result.success = (failed_count == 0);
    result.data = response;

    if (failed_count > 0) {
      result.error_message = "Some parameters could not be reset";
    }

    RCLCPP_INFO(node_->get_logger(), "Reset %zu parameters on node '%s' (%zu failed)", reset_count, node_name.c_str(),
                failed_count);
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to reset parameters: ") + e.what();
  }

  return result;
}

}  // namespace ros2_medkit_gateway
