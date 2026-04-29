// Copyright 2026 bburda
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

#include "ros2_medkit_gateway/core/managers/configuration_manager.hpp"

#include <utility>

namespace ros2_medkit_gateway {

ConfigurationManager::ConfigurationManager(std::shared_ptr<ParameterTransport> transport)
  : transport_(std::move(transport)) {
}

ConfigurationManager::~ConfigurationManager() {
  shutdown();
}

void ConfigurationManager::shutdown() {
  if (shutdown_.exchange(true)) {
    return;  // Already shut down
  }
  if (transport_) {
    transport_->shutdown();
  }
}

ParameterResult ConfigurationManager::shut_down_result() {
  return {false, {}, "ConfigurationManager is shut down", ParameterErrorCode::SHUT_DOWN};
}

ParameterResult ConfigurationManager::list_parameters(const std::string & node_name) {
  if (shutdown_.load()) {
    return shut_down_result();
  }
  if (transport_->is_self_node(node_name)) {
    return transport_->list_own_parameters();
  }
  return transport_->list_parameters(node_name);
}

ParameterResult ConfigurationManager::get_parameter(const std::string & node_name, const std::string & param_name) {
  if (shutdown_.load()) {
    return shut_down_result();
  }
  if (transport_->is_self_node(node_name)) {
    return transport_->get_own_parameter(param_name);
  }
  return transport_->get_parameter(node_name, param_name);
}

ParameterResult ConfigurationManager::set_parameter(const std::string & node_name, const std::string & param_name,
                                                    const json & value) {
  if (shutdown_.load()) {
    return shut_down_result();
  }
  return transport_->set_parameter(node_name, param_name, value);
}

ParameterResult ConfigurationManager::reset_parameter(const std::string & node_name, const std::string & param_name) {
  if (shutdown_.load()) {
    return shut_down_result();
  }

  // Ask the transport for the cached default. Returns NO_DEFAULTS_CACHED /
  // NOT_FOUND / SERVICE_UNAVAILABLE / TIMEOUT etc. as appropriate.
  auto default_result = transport_->get_default(node_name, param_name);
  if (!default_result.success) {
    return default_result;
  }

  // Apply the default by composing set_parameter. The transport echoes the
  // committed descriptor in the result; we re-stamp the response so callers
  // observe the same shape they did before the refactor.
  const json default_value = default_result.data;
  auto set_result = transport_->set_parameter(node_name, param_name, default_value);
  if (!set_result.success) {
    return set_result;
  }

  if (set_result.data.is_object()) {
    set_result.data["reset_to_default"] = true;
  }
  return set_result;
}

ParameterResult ConfigurationManager::reset_all_parameters(const std::string & node_name) {
  if (shutdown_.load()) {
    return shut_down_result();
  }

  // Fetch all cached defaults for the node up front. Each entry is
  // {"name", "value", "type"}; we fan out via set_parameter and aggregate.
  auto defaults_result = transport_->list_defaults(node_name);
  if (!defaults_result.success) {
    return defaults_result;
  }

  size_t reset_count = 0;
  size_t failed_count = 0;
  json failed_params = json::array();

  if (defaults_result.data.is_array()) {
    for (const auto & entry : defaults_result.data) {
      if (!entry.is_object() || !entry.contains("name")) {
        ++failed_count;
        continue;
      }
      const std::string param_name = entry.at("name").get<std::string>();
      const json param_value = entry.value("value", json{});
      auto set_result = transport_->set_parameter(node_name, param_name, param_value);
      if (set_result.success) {
        ++reset_count;
      } else {
        ++failed_count;
        failed_params.push_back(param_name);
      }
    }
  }

  ParameterResult result;
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
  return result;
}

}  // namespace ros2_medkit_gateway
