// Copyright 2026 sewon
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

#include <cctype>
#include <stdexcept>

namespace ros2_medkit_gateway {

std::string normalize_fault_manager_namespace(const std::string & namespace_value) {
  if (namespace_value.empty()) {
    return "";
  }

  std::string normalized = namespace_value;
  if (normalized.front() != '/') {
    normalized.insert(normalized.begin(), '/');
  }

  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }

  if (normalized == "/") {
    return "";
  }

  return normalized;
}

std::string get_fault_manager_namespace(rclcpp::Node * node) {
  if (!node) {
    throw std::invalid_argument("Fault manager namespace resolution requires a valid node");
  }

  constexpr auto kParameterName = "fault_manager_namespace";
  std::string namespace_value;
  if (!node->get_parameter(kParameterName, namespace_value)) {
    return "";
  }

  auto normalized = normalize_fault_manager_namespace(namespace_value);
  for (char c : normalized) {
    auto uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc) || c == '_' || c == '/') {
      continue;
    }

    RCLCPP_WARN(node->get_logger(),
                "Ignoring invalid fault_manager_namespace '%s': character '%c' is not allowed. "
                "Falling back to root fault manager path.",
                namespace_value.c_str(), c);
    return "";
  }

  return normalized;
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
