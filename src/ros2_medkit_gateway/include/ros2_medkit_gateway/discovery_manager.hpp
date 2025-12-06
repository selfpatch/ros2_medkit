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

#pragma once

#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/models.hpp"

namespace ros2_medkit_gateway {

class DiscoveryManager {
 public:
  explicit DiscoveryManager(rclcpp::Node * node);

  std::vector<Area> discover_areas();
  std::vector<Component> discover_components();

  /// Discover all services in the system with their types
  std::vector<ServiceInfo> discover_services();

  /// Discover all actions in the system with their types
  std::vector<ActionInfo> discover_actions();

  /// Find a service by component namespace and operation name
  std::optional<ServiceInfo> find_service(const std::string & component_ns, const std::string & operation_name) const;

  /// Find an action by component namespace and operation name
  std::optional<ActionInfo> find_action(const std::string & component_ns, const std::string & operation_name) const;

 private:
  std::string extract_area_from_namespace(const std::string & ns);

  /// Extract the last segment from a path (e.g., "/a/b/c" -> "c")
  std::string extract_name_from_path(const std::string & path);

  /// Check if a service path belongs to a component namespace
  bool path_belongs_to_namespace(const std::string & path, const std::string & ns) const;

  rclcpp::Node * node_;

  // Cached services and actions for lookup
  std::vector<ServiceInfo> cached_services_;
  std::vector<ActionInfo> cached_actions_;
};

}  // namespace ros2_medkit_gateway
