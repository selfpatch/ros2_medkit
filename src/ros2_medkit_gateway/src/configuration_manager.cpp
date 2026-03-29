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

#include <atomic>
#include <chrono>

using namespace std::chrono_literals;

namespace ros2_medkit_gateway {

// Thread-local param node counter for unique names
static std::atomic<int> param_node_counter{0};

ConfigurationManager::ConfigurationManager(rclcpp::Node * node) : node_(node) {
  // Get configurable timeout for parameter services (default 0.5 seconds)
  service_timeout_sec_ = node_->declare_parameter("parameter_service_timeout_sec", 0.5);

  // Get negative cache TTL (default 60 seconds)
  negative_cache_ttl_sec_ = node_->declare_parameter("parameter_service_negative_cache_sec", 60.0);

  // Store own node FQN for self-query detection
  own_node_fqn_ = node_->get_fully_qualified_name();

  RCLCPP_INFO(node_->get_logger(), "ConfigurationManager initialized (timeout=%.1fs, negative_cache=%.0fs)",
              service_timeout_sec_, negative_cache_ttl_sec_);
}

ConfigurationManager::~ConfigurationManager() {
  shutdown();
}

void ConfigurationManager::shutdown() {
  std::lock_guard<std::mutex> lock(param_nodes_registry_mutex_);
  for (auto & weak_node : param_nodes_registry_) {
    if (auto node = weak_node.lock()) {
      node.reset();
    }
  }
  param_nodes_registry_.clear();
}

void ConfigurationManager::register_param_node(std::shared_ptr<rclcpp::Node> node) {
  std::lock_guard<std::mutex> lock(param_nodes_registry_mutex_);
  param_nodes_registry_.push_back(node);
}

std::chrono::duration<double> ConfigurationManager::get_service_timeout() const {
  return std::chrono::duration<double>(service_timeout_sec_);
}

std::mutex & ConfigurationManager::get_node_mutex(const std::string & node_name) const {
  // Fast path: check if mutex exists
  {
    std::shared_lock<std::shared_mutex> lock(node_mutexes_mutex_);
    auto it = node_mutexes_.find(node_name);
    if (it != node_mutexes_.end()) {
      return *it->second;
    }
  }
  // Slow path: create new mutex
  std::unique_lock<std::shared_mutex> lock(node_mutexes_mutex_);
  auto [it, inserted] = node_mutexes_.try_emplace(node_name, std::make_unique<std::mutex>());
  return *it->second;
}

bool ConfigurationManager::is_node_unavailable(const std::string & node_name) const {
  std::shared_lock<std::shared_mutex> lock(negative_cache_mutex_);
  auto it = unavailable_nodes_.find(node_name);
  if (it == unavailable_nodes_.end()) {
    return false;
  }
  auto elapsed = std::chrono::steady_clock::now() - it->second;
  return elapsed < std::chrono::duration<double>(negative_cache_ttl_sec_);
}

void ConfigurationManager::mark_node_unavailable(const std::string & node_name) {
  std::unique_lock<std::shared_mutex> lock(negative_cache_mutex_);
  unavailable_nodes_[node_name] = std::chrono::steady_clock::now();

  // Lazy cleanup: remove expired entries to prevent unbounded growth
  if (unavailable_nodes_.size() > 100) {
    auto now = std::chrono::steady_clock::now();
    auto ttl = std::chrono::duration<double>(negative_cache_ttl_sec_);
    for (auto it = unavailable_nodes_.begin(); it != unavailable_nodes_.end();) {
      if (now - it->second > ttl) {
        it = unavailable_nodes_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

bool ConfigurationManager::is_self_node(const std::string & node_name) const {
  return node_name == own_node_fqn_;
}

ParameterResult ConfigurationManager::list_own_parameters() {
  ParameterResult result;
  try {
    auto params = node_->get_parameters(node_->list_parameters({}, 0).names);
    json params_array = json::array();
    for (const auto & param : params) {
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
    result.error_message = std::string("Failed to list own parameters: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }
  return result;
}

ParameterResult ConfigurationManager::get_own_parameter(const std::string & param_name) {
  ParameterResult result;
  try {
    if (!node_->has_parameter(param_name)) {
      result.success = false;
      result.error_message = "Parameter not found: " + param_name;
      result.error_code = ParameterErrorCode::NOT_FOUND;
      return result;
    }
    auto param = node_->get_parameter(param_name);
    auto descriptor = node_->describe_parameter(param_name);

    json param_obj;
    param_obj["name"] = param.get_name();
    param_obj["value"] = parameter_value_to_json(param.get_parameter_value());
    param_obj["type"] = parameter_type_to_string(param.get_type());
    param_obj["description"] = descriptor.description;
    param_obj["read_only"] = descriptor.read_only;

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to get own parameter: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }
  return result;
}

std::shared_ptr<rclcpp::SyncParametersClient> ConfigurationManager::get_param_client(const std::string & node_name) {
  // Create a thread-local param node to avoid executor conflicts.
  // SyncParametersClient spins its node internally - sharing one node
  // across threads causes "Node already added to executor" errors.
  // Nodes are registered for deterministic cleanup on shutdown.
  thread_local std::shared_ptr<rclcpp::Node> tl_param_node;
  thread_local std::map<std::string, std::shared_ptr<rclcpp::SyncParametersClient>> tl_clients;

  if (!tl_param_node) {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    options.use_global_arguments(false);
    int id = param_node_counter.fetch_add(1);
    tl_param_node = std::make_shared<rclcpp::Node>(
        "_param_client_" + std::to_string(id), options);
    register_param_node(tl_param_node);
  }

  auto it = tl_clients.find(node_name);
  if (it != tl_clients.end()) {
    return it->second;
  }

  auto client = std::make_shared<rclcpp::SyncParametersClient>(tl_param_node, node_name);
  tl_clients[node_name] = client;
  return client;
}

ParameterResult ConfigurationManager::list_parameters(const std::string & node_name) {
  // Self-query guard: use direct access for gateway's own node
  if (is_self_node(node_name)) {
    return list_own_parameters();
  }

  // Negative cache: skip nodes that recently had no parameter service
  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  // Per-node mutex: only block concurrent queries to the SAME node
  std::lock_guard<std::mutex> op_lock(get_node_mutex(node_name));
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(get_service_timeout())) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
      mark_node_unavailable(node_name);
      RCLCPP_WARN(node_->get_logger(), "Parameter service not available for node: '%s' (cached for %.0fs)",
                  node_name.c_str(), negative_cache_ttl_sec_);
      return result;
    }

    // Cache default values after confirming service is available
    cache_default_values(node_name);

    auto param_names = client->list_parameters({}, 0);

    std::vector<rclcpp::Parameter> parameters;
    parameters = client->get_parameters(param_names.names);

    if (parameters.empty() && !param_names.names.empty()) {
      RCLCPP_WARN(node_->get_logger(), "get_parameters returned empty, trying one by one for node: '%s'",
                  node_name.c_str());
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
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
    RCLCPP_ERROR(node_->get_logger(), "Exception in list_parameters for node '%s': %s", node_name.c_str(), e.what());
  }

  return result;
}

ParameterResult ConfigurationManager::get_parameter(const std::string & node_name, const std::string & param_name) {
  if (is_self_node(node_name)) {
    return get_own_parameter(param_name);
  }

  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  std::lock_guard<std::mutex> op_lock(get_node_mutex(node_name));
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(get_service_timeout())) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
      mark_node_unavailable(node_name);
      return result;
    }

    auto param_names = client->list_parameters({param_name}, 1);
    if (param_names.names.empty()) {
      result.success = false;
      result.error_message = "Parameter not found: " + param_name;
      result.error_code = ParameterErrorCode::NOT_FOUND;
      return result;
    }

    auto parameters = client->get_parameters({param_name});
    if (parameters.empty()) {
      result.success = false;
      result.error_message = "Failed to get parameter: " + param_name;
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
      return result;
    }

    const auto & param = parameters[0];
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
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }

  return result;
}

ParameterResult ConfigurationManager::set_parameter(const std::string & node_name, const std::string & param_name,
                                                    const json & value) {
  // Self-query guard: set parameter directly on own node
  if (is_self_node(node_name)) {
    ParameterResult result;
    try {
      auto current_value = node_->get_parameter(param_name).get_parameter_value();
      rclcpp::ParameterValue param_value = json_to_parameter_value(value, current_value.get_type());
      auto set_result = node_->set_parameter(rclcpp::Parameter(param_name, param_value));
      if (!set_result.successful) {
        result.success = false;
        result.error_message = set_result.reason;
        result.error_code = ParameterErrorCode::INVALID_VALUE;
        return result;
      }
      json param_obj;
      param_obj["name"] = param_name;
      param_obj["value"] = parameter_value_to_json(param_value);
      param_obj["type"] = parameter_type_to_string(param_value.get_type());
      result.success = true;
      result.data = param_obj;
    } catch (const std::exception & e) {
      result.success = false;
      result.error_message = std::string("Failed to set own parameter: ") + e.what();
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
    }
    return result;
  }

  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  std::lock_guard<std::mutex> op_lock(get_node_mutex(node_name));
  ParameterResult result;

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(get_service_timeout())) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
      mark_node_unavailable(node_name);
      return result;
    }

    auto current_params = client->get_parameters({param_name});
    rclcpp::ParameterType hint_type = rclcpp::ParameterType::PARAMETER_NOT_SET;
    if (!current_params.empty()) {
      hint_type = current_params[0].get_type();
    }

    rclcpp::ParameterValue param_value = json_to_parameter_value(value, hint_type);
    rclcpp::Parameter param(param_name, param_value);

    auto results = client->set_parameters({param});
    if (results.empty() || !results[0].successful) {
      result.success = false;
      result.error_message = results.empty() ? "Failed to set parameter" : results[0].reason;
      if (!results.empty()) {
        const auto & reason = results[0].reason;
        if (reason.find("read-only") != std::string::npos || reason.find("read only") != std::string::npos ||
            reason.find("is read_only") != std::string::npos) {
          result.error_code = ParameterErrorCode::READ_ONLY;
        } else if (reason.find("type") != std::string::npos) {
          result.error_code = ParameterErrorCode::TYPE_MISMATCH;
        } else {
          result.error_code = ParameterErrorCode::INVALID_VALUE;
        }
      } else {
        result.error_code = ParameterErrorCode::INTERNAL_ERROR;
      }
      return result;
    }

    json param_obj;
    param_obj["name"] = param_name;
    param_obj["value"] = parameter_value_to_json(param_value);
    param_obj["type"] = parameter_type_to_string(param_value.get_type());

    result.success = true;
    result.data = param_obj;
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to set parameter: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
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
      return rclcpp::ParameterValue(std::vector<std::string>{});
    }
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

  return rclcpp::ParameterValue(value.dump());
}

void ConfigurationManager::cache_default_values(const std::string & node_name) {
  {
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    if (default_values_.find(node_name) != default_values_.end()) {
      return;
    }
  }

  // Skip if node is in negative cache
  if (is_node_unavailable(node_name)) {
    return;
  }

  try {
    auto client = get_param_client(node_name);

    if (!client->wait_for_service(get_service_timeout())) {
      RCLCPP_WARN(node_->get_logger(), "Cannot cache defaults - service not available for node: '%s'",
                  node_name.c_str());
      mark_node_unavailable(node_name);
      return;
    }

    auto param_names = client->list_parameters({}, 0);

    std::vector<rclcpp::Parameter> parameters;
    parameters = client->get_parameters(param_names.names);

    if (parameters.empty() && !param_names.names.empty()) {
      for (const auto & name : param_names.names) {
        try {
          auto single_params = client->get_parameters({name});
          if (!single_params.empty()) {
            parameters.push_back(single_params[0]);
          }
        } catch (const std::exception &) {
        }
      }
    }

    std::map<std::string, rclcpp::Parameter> node_defaults;
    for (const auto & param : parameters) {
      node_defaults[param.get_name()] = param;
    }

    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      if (default_values_.find(node_name) == default_values_.end()) {
        default_values_[node_name] = std::move(node_defaults);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to cache defaults for node '%s': %s", node_name.c_str(), e.what());
  }
}

ParameterResult ConfigurationManager::reset_parameter(const std::string & node_name, const std::string & param_name) {
  // Self-query guard: reset via direct access
  if (is_self_node(node_name)) {
    ParameterResult result;
    std::lock_guard<std::mutex> lock(defaults_mutex_);
    auto node_it = default_values_.find(node_name);
    if (node_it == default_values_.end()) {
      result.success = false;
      result.error_message = "No default values cached for node: " + node_name;
      result.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
      return result;
    }
    auto param_it = node_it->second.find(param_name);
    if (param_it == node_it->second.end()) {
      result.success = false;
      result.error_message = "No default value for parameter: " + param_name;
      result.error_code = ParameterErrorCode::NOT_FOUND;
      return result;
    }
    auto set_result = node_->set_parameter(param_it->second);
    if (!set_result.successful) {
      result.success = false;
      result.error_message = set_result.reason;
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
      return result;
    }
    json param_obj;
    param_obj["name"] = param_name;
    param_obj["value"] = parameter_value_to_json(param_it->second.get_parameter_value());
    param_obj["type"] = parameter_type_to_string(param_it->second.get_type());
    param_obj["reset_to_default"] = true;
    result.success = true;
    result.data = param_obj;
    return result;
  }

  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  std::lock_guard<std::mutex> op_lock(get_node_mutex(node_name));
  ParameterResult result;

  try {
    cache_default_values(node_name);

    rclcpp::Parameter default_param;
    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      auto node_it = default_values_.find(node_name);
      if (node_it == default_values_.end()) {
        result.success = false;
        result.error_message = "No default values cached for node: " + node_name;
        result.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
        return result;
      }

      auto param_it = node_it->second.find(param_name);
      if (param_it == node_it->second.end()) {
        result.success = false;
        result.error_message = "No default value for parameter: " + param_name;
        result.error_code = ParameterErrorCode::NOT_FOUND;
        return result;
      }

      default_param = param_it->second;
    }

    auto client = get_param_client(node_name);
    if (!client->wait_for_service(get_service_timeout())) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
      mark_node_unavailable(node_name);
      return result;
    }

    auto results = client->set_parameters({default_param});
    if (results.empty() || !results[0].successful) {
      result.success = false;
      result.error_message = results.empty() ? "Failed to reset parameter" : results[0].reason;
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
      return result;
    }

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
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }

  return result;
}

ParameterResult ConfigurationManager::reset_all_parameters(const std::string & node_name) {
  if (is_node_unavailable(node_name)) {
    ParameterResult result;
    result.success = false;
    result.error_message = "Parameter service not available for node (cached): " + node_name;
    result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
    return result;
  }

  std::lock_guard<std::mutex> op_lock(get_node_mutex(node_name));
  ParameterResult result;

  try {
    cache_default_values(node_name);

    std::vector<rclcpp::Parameter> params_to_reset;
    {
      std::lock_guard<std::mutex> lock(defaults_mutex_);
      auto node_it = default_values_.find(node_name);
      if (node_it == default_values_.end()) {
        result.success = false;
        result.error_message = "No default values cached for node: " + node_name;
        result.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
        return result;
      }

      params_to_reset.reserve(node_it->second.size());
      for (const auto & [name, param] : node_it->second) {
        params_to_reset.push_back(param);
      }
    }

    auto client = get_param_client(node_name);
    if (!client->wait_for_service(get_service_timeout())) {
      result.success = false;
      result.error_message = "Parameter service not available for node: " + node_name;
      result.error_code = ParameterErrorCode::SERVICE_UNAVAILABLE;
      mark_node_unavailable(node_name);
      return result;
    }

    size_t reset_count = 0;
    size_t failed_count = 0;
    json failed_params = json::array();

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
      result.error_code = ParameterErrorCode::INTERNAL_ERROR;
    }

    RCLCPP_INFO(node_->get_logger(), "Reset %zu parameters on node '%s' (%zu failed)", reset_count, node_name.c_str(),
                failed_count);
  } catch (const std::exception & e) {
    result.success = false;
    result.error_message = std::string("Failed to reset parameters: ") + e.what();
    result.error_code = ParameterErrorCode::INTERNAL_ERROR;
  }

  return result;
}

}  // namespace ros2_medkit_gateway
