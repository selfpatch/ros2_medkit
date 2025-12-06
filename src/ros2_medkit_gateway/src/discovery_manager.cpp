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

#include "ros2_medkit_gateway/discovery_manager.hpp"

#include <algorithm>
#include <set>
#include <unordered_map>

namespace ros2_medkit_gateway {

DiscoveryManager::DiscoveryManager(rclcpp::Node * node) : node_(node) {
}

std::vector<Area> DiscoveryManager::discover_areas() {
  // Extract unique areas from namespaces
  std::set<std::string> area_set;

  // Get node graph interface
  auto node_graph = node_->get_node_graph_interface();

  // Iterate through all nodes to find namespaces
  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();

  for (const auto & name_and_ns : names_and_namespaces) {
    std::string ns = name_and_ns.second;
    std::string area = extract_area_from_namespace(ns);
    area_set.insert(area);
  }

  // Convert set to vector of Area structs
  std::vector<Area> areas;
  for (const auto & area_name : area_set) {
    Area area;
    area.id = area_name;
    area.namespace_path = (area_name == "root") ? "/" : "/" + area_name;
    areas.push_back(area);
  }

  return areas;
}

std::vector<Component> DiscoveryManager::discover_components() {
  std::vector<Component> components;

  // First, discover all services and actions to attach to components
  auto services = discover_services();
  auto actions = discover_actions();

  // Pre-group services by parent namespace for O(1) lookup
  // Key: parent namespace (e.g., "/powertrain/engine" for service "/powertrain/engine/calibrate")
  std::unordered_map<std::string, std::vector<ServiceInfo>> services_by_ns;
  for (const auto & svc : services) {
    // Extract parent namespace: everything before the last '/'
    size_t last_slash = svc.full_path.rfind('/');
    std::string parent_ns = (last_slash == 0) ? "/" : svc.full_path.substr(0, last_slash);
    services_by_ns[parent_ns].push_back(svc);
  }

  // Pre-group actions by parent namespace for O(1) lookup
  std::unordered_map<std::string, std::vector<ActionInfo>> actions_by_ns;
  for (const auto & act : actions) {
    size_t last_slash = act.full_path.rfind('/');
    std::string parent_ns = (last_slash == 0) ? "/" : act.full_path.substr(0, last_slash);
    actions_by_ns[parent_ns].push_back(act);
  }

  auto node_graph = node_->get_node_graph_interface();
  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();

  for (const auto & name_and_ns : names_and_namespaces) {
    const auto & name = name_and_ns.first;
    const auto & ns = name_and_ns.second;

    Component comp;
    comp.id = name;
    comp.namespace_path = ns;
    comp.fqn = (ns == "/") ? std::string("/").append(name) : std::string(ns).append("/").append(name);
    comp.area = extract_area_from_namespace(ns);

    // Attach services that belong to this component's namespace (O(1) lookup)
    auto svc_it = services_by_ns.find(comp.namespace_path);
    if (svc_it != services_by_ns.end()) {
      comp.services = svc_it->second;
    }

    // Attach actions that belong to this component's namespace (O(1) lookup)
    auto act_it = actions_by_ns.find(comp.namespace_path);
    if (act_it != actions_by_ns.end()) {
      comp.actions = act_it->second;
    }

    components.push_back(comp);
  }

  return components;
}

std::vector<ServiceInfo> DiscoveryManager::discover_services() {
  std::vector<ServiceInfo> services;

  // Use native rclcpp API to get service names and types
  auto service_names_and_types = node_->get_service_names_and_types();

  for (const auto & [service_path, types] : service_names_and_types) {
    // Skip internal ROS2 services (parameter services, etc.)
    if (service_path.find("/get_parameters") != std::string::npos ||
        service_path.find("/set_parameters") != std::string::npos ||
        service_path.find("/list_parameters") != std::string::npos ||
        service_path.find("/describe_parameters") != std::string::npos ||
        service_path.find("/get_parameter_types") != std::string::npos ||
        service_path.find("/set_parameters_atomically") != std::string::npos ||
        service_path.find("/get_type_description") != std::string::npos ||
        service_path.find("/_action/") != std::string::npos) {  // Skip action internal services
      continue;
    }

    ServiceInfo info;
    info.full_path = service_path;
    info.name = extract_name_from_path(service_path);
    info.type = types.empty() ? "" : types[0];

    services.push_back(info);
  }

  // Update cache for find operations
  cached_services_ = services;

  return services;
}

std::vector<ActionInfo> DiscoveryManager::discover_actions() {
  std::vector<ActionInfo> actions;

  // Use native rclcpp API to get action names and types
  // Note: This requires checking service endpoints for action patterns
  auto service_names_and_types = node_->get_service_names_and_types();

  // Actions expose services with /_action/send_goal, /_action/cancel_goal, /_action/get_result
  // We detect actions by looking for /_action/send_goal services
  std::set<std::string> discovered_action_paths;

  for (const auto & [service_path, types] : service_names_and_types) {
    const std::string action_suffix = "/_action/send_goal";
    if (service_path.length() > action_suffix.length() &&
        service_path.compare(service_path.length() - action_suffix.length(), action_suffix.length(), action_suffix) ==
            0) {
      // Extract action path by removing /_action/send_goal suffix
      std::string action_path = service_path.substr(0, service_path.length() - action_suffix.length());

      if (discovered_action_paths.count(action_path) == 0) {
        discovered_action_paths.insert(action_path);

        ActionInfo info;
        info.full_path = action_path;
        info.name = extract_name_from_path(action_path);

        // Derive action type from send_goal service type
        // Service type is like "example_interfaces/action/Fibonacci_SendGoal"
        // We need to extract "example_interfaces/action/Fibonacci"
        if (!types.empty()) {
          std::string srv_type = types[0];
          const std::string send_goal_suffix = "_SendGoal";
          if (srv_type.length() > send_goal_suffix.length() &&
              srv_type.compare(srv_type.length() - send_goal_suffix.length(), send_goal_suffix.length(),
                               send_goal_suffix) == 0) {
            info.type = srv_type.substr(0, srv_type.length() - send_goal_suffix.length());
          } else {
            info.type = srv_type;
          }
        }

        actions.push_back(info);
      }
    }
  }

  // Update cache for find operations
  cached_actions_ = actions;

  return actions;
}

std::optional<ServiceInfo> DiscoveryManager::find_service(const std::string & component_ns,
                                                          const std::string & operation_name) const {
  // Construct expected service path
  std::string expected_path = component_ns;
  if (!expected_path.empty() && expected_path.back() != '/') {
    expected_path += "/";
  }
  expected_path += operation_name;

  for (const auto & svc : cached_services_) {
    if (svc.full_path == expected_path || svc.name == operation_name) {
      // Check if service belongs to the component namespace
      if (path_belongs_to_namespace(svc.full_path, component_ns)) {
        return svc;
      }
    }
  }

  return std::nullopt;
}

std::optional<ActionInfo> DiscoveryManager::find_action(const std::string & component_ns,
                                                        const std::string & operation_name) const {
  // Construct expected action path
  std::string expected_path = component_ns;
  if (!expected_path.empty() && expected_path.back() != '/') {
    expected_path += "/";
  }
  expected_path += operation_name;

  for (const auto & act : cached_actions_) {
    if (act.full_path == expected_path || act.name == operation_name) {
      // Check if action belongs to the component namespace
      if (path_belongs_to_namespace(act.full_path, component_ns)) {
        return act;
      }
    }
  }

  return std::nullopt;
}

std::string DiscoveryManager::extract_area_from_namespace(const std::string & ns) {
  if (ns == "/" || ns.empty()) {
    return "root";
  }

  // Remove leading slash
  std::string cleaned = ns;
  if (cleaned[0] == '/') {
    cleaned = cleaned.substr(1);
  }

  // Get first segment
  auto pos = cleaned.find('/');
  if (pos != std::string::npos) {
    return cleaned.substr(0, pos);
  }

  return cleaned;
}

std::string DiscoveryManager::extract_name_from_path(const std::string & path) {
  if (path.empty()) {
    return "";
  }

  // Find last slash
  auto pos = path.rfind('/');
  if (pos != std::string::npos && pos < path.length() - 1) {
    return path.substr(pos + 1);
  }

  return path;
}

bool DiscoveryManager::path_belongs_to_namespace(const std::string & path, const std::string & ns) const {
  if (ns.empty() || ns == "/") {
    // Root namespace - check if path has only one segment after leading slash
    if (path.empty() || path[0] != '/') {
      return false;
    }
    std::string without_leading = path.substr(1);
    return without_leading.find('/') == std::string::npos;
  }

  // Check if path starts with namespace
  if (path.length() <= ns.length()) {
    return false;
  }

  // Path should start with ns and have exactly one more segment
  if (path.compare(0, ns.length(), ns) != 0) {
    return false;
  }

  // Check that after namespace there's a slash and then the service/action name (no more slashes)
  if (path[ns.length()] != '/') {
    return false;
  }

  std::string remainder = path.substr(ns.length() + 1);
  return remainder.find('/') == std::string::npos;
}

}  // namespace ros2_medkit_gateway
