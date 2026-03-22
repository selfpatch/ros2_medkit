// Copyright 2026 mfaferek93
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

#include "ros2_medkit_gateway/fault_manager_paths.hpp"

#include <stdexcept>

namespace ros2_medkit_gateway {

std::string normalize_fault_manager_namespace(const std::string & namespace_value) {
  if (namespace_value.empty() || namespace_value == "/") {
    return "";
  }

  std::string normalized = namespace_value;
  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }

  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }

  return normalized;
}

std::string get_fault_manager_namespace(rclcpp::Node * node) {
  if (!node) {
    throw std::invalid_argument("Fault manager namespace resolution requires a valid node");
  }

  constexpr auto kParameterName = "fault_manager_namespace";
  if (!node->has_parameter(kParameterName)) {
    node->declare_parameter<std::string>(kParameterName, "");
  }

  return normalize_fault_manager_namespace(node->get_parameter(kParameterName).as_string());
}

std::string build_fault_manager_base_path(const std::string & namespace_value) {
  return normalize_fault_manager_namespace(namespace_value) + "/fault_manager";
}

std::string build_fault_manager_base_path(rclcpp::Node * node) {
  return build_fault_manager_base_path(get_fault_manager_namespace(node));
}

std::string build_fault_manager_events_topic(const std::string & namespace_value) {
  return build_fault_manager_base_path(namespace_value) + "/events";
}

std::string build_fault_manager_events_topic(rclcpp::Node * node) {
  return build_fault_manager_events_topic(get_fault_manager_namespace(node));
}

}  // namespace ros2_medkit_gateway
