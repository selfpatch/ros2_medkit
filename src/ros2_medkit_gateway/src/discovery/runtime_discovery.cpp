// Copyright 2025 selfpatch
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

#include "ros2_medkit_gateway/discovery/runtime_discovery.hpp"

#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"

#include <set>
#include <unordered_map>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Check if a service path matches an internal ROS2 service suffix.
 *
 * Uses exact suffix matching to avoid false positives. For example,
 * "/my_node/get_parameters" is internal, but "/my_node/get_parameters_backup" is not.
 *
 * @param service_path The full service path (e.g., "/my_node/get_parameters")
 * @param suffix The suffix to match (e.g., "/get_parameters")
 * @return true if the service_path ends with the suffix
 */
static bool matches_internal_suffix(const std::string & service_path, const std::string & suffix) {
  if (service_path.length() < suffix.length()) {
    return false;
  }
  return service_path.compare(service_path.length() - suffix.length(), suffix.length(), suffix) == 0;
}

// Helper function to check if a service path is internal ROS2 infrastructure
bool RuntimeDiscoveryStrategy::is_internal_service(const std::string & service_path) {
  // Use exact suffix matching to avoid false positives
  // e.g., "/my_node/get_parameters" is internal, but "/my_node/get_parameters_backup" is NOT
  return matches_internal_suffix(service_path, "/get_parameters") ||
         matches_internal_suffix(service_path, "/set_parameters") ||
         matches_internal_suffix(service_path, "/list_parameters") ||
         matches_internal_suffix(service_path, "/describe_parameters") ||
         matches_internal_suffix(service_path, "/get_parameter_types") ||
         matches_internal_suffix(service_path, "/set_parameters_atomically") ||
         matches_internal_suffix(service_path, "/get_type_description") ||
         service_path.find("/_action/") != std::string::npos;  // Action internal services
}

RuntimeDiscoveryStrategy::RuntimeDiscoveryStrategy(rclcpp::Node * node) : node_(node) {
}

void RuntimeDiscoveryStrategy::set_config(const RuntimeConfig & config) {
  config_ = config;
  RCLCPP_DEBUG(node_->get_logger(),
               "Runtime discovery config: synthetic_areas=%s, synthetic_components=%s, "
               "functions_from_namespaces=%s, grouping=%s",
               config_.create_synthetic_areas ? "true" : "false",
               config_.create_synthetic_components ? "true" : "false",
               config_.create_functions_from_namespaces ? "true" : "false",
               grouping_strategy_to_string(config_.grouping).c_str());
}

std::vector<Area> RuntimeDiscoveryStrategy::discover_areas() {
  if (!config_.create_synthetic_areas) {
    return {};
  }

  RCLCPP_WARN_ONCE(node_->get_logger(),
                   "create_synthetic_areas is enabled but deprecated. "
                   "Namespace grouping now creates Function entities instead of Areas. "
                   "Set discovery.runtime.create_synthetic_areas=false (the new default) "
                   "and use create_functions_from_namespaces=true instead.");

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

  // Also include areas from topic namespaces (for topic-based discovery)
  if (topic_sampler_) {
    auto topic_namespaces = topic_sampler_->discover_topic_namespaces();
    for (const auto & ns : topic_namespaces) {
      area_set.insert(ns);
    }
  }

  // Filter out topic "namespaces" that are actually root-namespace node names.
  // Root-namespace nodes publish topics with their node name as prefix
  // (e.g., /fault_manager publishes /fault_manager/events), which looks like
  // a topic in namespace "fault_manager". These nodes belong to area "root".
  // Only erase if the name was NOT also derived from actual node namespaces
  // (protects legitimate areas when a root node shares a name with a namespace).
  std::set<std::string> node_derived_areas;
  for (const auto & [n, ns] : names_and_namespaces) {
    node_derived_areas.insert(extract_area_from_namespace(ns));
  }
  for (const auto & [name, ns] : names_and_namespaces) {
    if (ns == "/" && node_derived_areas.find(name) == node_derived_areas.end()) {
      area_set.erase(name);
    }
  }

  // Convert set to vector of Area structs
  std::vector<Area> areas;
  for (const auto & area_name : area_set) {
    Area area;
    area.id = area_name;
    area.namespace_path = (area_name == "root") ? "/" : "/" + area_name;
    area.source = "heuristic";
    areas.push_back(area);
  }

  return areas;
}

std::vector<Component> RuntimeDiscoveryStrategy::discover_components() {
  return discover_components(discover_apps());
}

std::vector<Component> RuntimeDiscoveryStrategy::discover_components(const std::vector<App> & apps) {
  if (!config_.create_synthetic_components) {
    // Legacy mode: each App becomes its own Component (1:1 mapping)
    std::vector<Component> components;
    components.reserve(apps.size());
    for (const auto & app : apps) {
      Component comp;
      comp.id = app.id;
      comp.source = "heuristic";
      if (app.bound_fqn.has_value()) {
        comp.fqn = app.bound_fqn.value();
        auto slash_pos = comp.fqn.rfind('/');
        comp.namespace_path = (slash_pos == std::string::npos || slash_pos == 0) ? "/" : comp.fqn.substr(0, slash_pos);
        comp.area = extract_area_from_namespace(comp.namespace_path);
      }
      components.push_back(std::move(comp));
    }
    return components;
  }

  return discover_synthetic_components(apps);
}

std::vector<App> RuntimeDiscoveryStrategy::discover_apps() {
  std::vector<App> apps;

  // Pre-build service info map for schema lookups
  std::unordered_map<std::string, ServiceInfo> service_info_map;
  auto all_services = discover_services();
  for (const auto & svc : all_services) {
    service_info_map[svc.full_path] = svc;
  }

  // Pre-build action info map for schema lookups
  std::unordered_map<std::string, ActionInfo> action_info_map;
  auto all_actions = discover_actions();
  for (const auto & act : all_actions) {
    action_info_map[act.full_path] = act;
  }

  auto node_graph = node_->get_node_graph_interface();
  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();

  // Build topic map if not yet ready (first call or after manual refresh)
  if (topic_sampler_ && !topic_map_ready_) {
    refresh_topic_map();
  }

  // Deduplicate nodes - ROS 2 RMW can report duplicates for nodes with multiple interfaces
  std::set<std::string> seen_fqns;

  for (const auto & name_and_ns : names_and_namespaces) {
    const auto & name = name_and_ns.first;
    const auto & ns = name_and_ns.second;

    std::string fqn = (ns == "/") ? std::string("/").append(name) : std::string(ns).append("/").append(name);

    // Skip duplicate nodes
    if (seen_fqns.count(fqn) > 0) {
      RCLCPP_DEBUG(node_->get_logger(), "Skipping duplicate node: %s", fqn.c_str());
      continue;
    }
    seen_fqns.insert(fqn);

    App app;
    app.id = name;
    app.name = name;
    app.source = "heuristic";
    app.is_online = true;
    app.bound_fqn = fqn;

    std::string area = extract_area_from_namespace(ns);
    app.component_id = derive_component_id(name, area);

    // Introspect services and actions for this node
    try {
      auto node_services = node_->get_service_names_and_types_by_node(name, ns);
      for (const auto & [service_path, types] : node_services) {
        if (is_internal_service(service_path)) {
          continue;
        }
        auto it = service_info_map.find(service_path);
        if (it != service_info_map.end()) {
          app.services.push_back(it->second);
        } else {
          ServiceInfo info;
          info.full_path = service_path;
          info.name = extract_name_from_path(service_path);
          info.type = types.empty() ? "" : types[0];
          app.services.push_back(info);
        }
      }

      // Detect actions by checking for /_action/send_goal services
      for (const auto & [service_path, types] : node_services) {
        const std::string action_suffix = "/_action/send_goal";
        if (service_path.length() > action_suffix.length() &&
            service_path.compare(service_path.length() - action_suffix.length(), action_suffix.length(),
                                 action_suffix) == 0) {
          std::string action_path = service_path.substr(0, service_path.length() - action_suffix.length());
          auto it = action_info_map.find(action_path);
          if (it != action_info_map.end()) {
            app.actions.push_back(it->second);
          } else {
            ActionInfo info;
            info.full_path = action_path;
            info.name = extract_name_from_path(action_path);
            app.actions.push_back(info);
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(node_->get_logger(), "Could not get services for node '%s' in namespace '%s': %s", name.c_str(),
                   ns.c_str(), e.what());
    }

    // Populate topics from cached map
    if (topic_sampler_) {
      auto it = cached_topic_map_.find(fqn);
      if (it != cached_topic_map_.end()) {
        app.topics = it->second;
      }
    }

    apps.push_back(app);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Discovered %zu apps from runtime nodes", apps.size());
  return apps;
}

std::vector<Function> RuntimeDiscoveryStrategy::discover_functions() {
  if (!config_.create_functions_from_namespaces) {
    return {};
  }

  // Discover apps to get their IDs and namespaces
  auto apps = discover_apps();

  // Group apps by namespace (first segment), similar to discover_areas() logic
  std::map<std::string, std::vector<std::string>> ns_to_app_ids;
  for (const auto & app : apps) {
    std::string ns = "/";
    if (app.bound_fqn.has_value()) {
      const auto & fqn = app.bound_fqn.value();
      auto pos = fqn.rfind('/');
      ns = (pos == std::string::npos || pos == 0) ? "/" : fqn.substr(0, pos);
    }
    std::string area = extract_area_from_namespace(ns);
    ns_to_app_ids[area].push_back(app.id);
  }

  // Create Function entities from namespace groups
  std::vector<Function> functions;
  for (const auto & [ns_name, app_ids] : ns_to_app_ids) {
    if (app_ids.empty()) {
      continue;
    }

    Function func;
    func.id = ns_name;
    func.name = ns_name;
    func.source = "runtime";
    func.hosts = app_ids;

    RCLCPP_DEBUG(node_->get_logger(), "Created runtime function '%s' with %zu host apps", ns_name.c_str(),
                 app_ids.size());
    functions.push_back(std::move(func));
  }

  RCLCPP_DEBUG(node_->get_logger(), "Discovered %zu functions from namespace grouping", functions.size());
  return functions;
}

std::vector<ServiceInfo> RuntimeDiscoveryStrategy::discover_services() {
  std::vector<ServiceInfo> services;

  // Use native rclcpp API to get service names and types
  auto service_names_and_types = node_->get_service_names_and_types();

  for (const auto & [service_path, types] : service_names_and_types) {
    // Skip internal ROS2 services (parameter services, action internals, etc.)
    if (is_internal_service(service_path)) {
      continue;
    }

    ServiceInfo info;
    info.full_path = service_path;
    info.name = extract_name_from_path(service_path);
    info.type = types.empty() ? "" : types[0];

    // Enrich with schema info if TypeIntrospection is available
    // Service types: pkg/srv/Type -> Request: pkg/srv/Type_Request, Response: pkg/srv/Type_Response
    if (type_introspection_ && !info.type.empty()) {
      try {
        json type_info_json;
        auto request_info = type_introspection_->get_type_info(info.type + "_Request");
        auto response_info = type_introspection_->get_type_info(info.type + "_Response");
        type_info_json["request"] = request_info.schema;
        type_info_json["response"] = response_info.schema;
        info.type_info = type_info_json;
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(node_->get_logger(), "Could not get schema for service '%s': %s", info.type.c_str(), e.what());
      }
    }

    services.push_back(info);
  }

  // Update cache for find operations
  cached_services_ = services;

  return services;
}

std::vector<ActionInfo> RuntimeDiscoveryStrategy::discover_actions() {
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

        // Enrich with schema info if TypeIntrospection is available
        // Action types: pkg/action/Type -> Goal: pkg/action/Type_Goal, etc.
        if (type_introspection_ && !info.type.empty()) {
          try {
            json type_info_json;
            auto goal_info = type_introspection_->get_type_info(info.type + "_Goal");
            auto result_info = type_introspection_->get_type_info(info.type + "_Result");
            auto feedback_info = type_introspection_->get_type_info(info.type + "_Feedback");
            type_info_json["goal"] = goal_info.schema;
            type_info_json["result"] = result_info.schema;
            type_info_json["feedback"] = feedback_info.schema;
            info.type_info = type_info_json;
          } catch (const std::exception & e) {
            RCLCPP_DEBUG(node_->get_logger(), "Could not get schema for action '%s': %s", info.type.c_str(), e.what());
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

std::optional<ServiceInfo> RuntimeDiscoveryStrategy::find_service(const std::string & component_ns,
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

std::optional<ActionInfo> RuntimeDiscoveryStrategy::find_action(const std::string & component_ns,
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

void RuntimeDiscoveryStrategy::set_topic_sampler(NativeTopicSampler * sampler) {
  topic_sampler_ = sampler;
}

void RuntimeDiscoveryStrategy::set_type_introspection(TypeIntrospection * introspection) {
  type_introspection_ = introspection;
}

void RuntimeDiscoveryStrategy::refresh_topic_map() {
  if (!topic_sampler_) {
    return;
  }
  cached_topic_map_ = topic_sampler_->build_component_topic_map();
  topic_map_ready_ = true;
  RCLCPP_DEBUG(node_->get_logger(), "Topic map refreshed: %zu components", cached_topic_map_.size());
}

bool RuntimeDiscoveryStrategy::is_topic_map_ready() const {
  return topic_map_ready_;
}

std::string RuntimeDiscoveryStrategy::extract_area_from_namespace(const std::string & ns) {
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

std::string RuntimeDiscoveryStrategy::extract_name_from_path(const std::string & path) {
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

std::set<std::string> RuntimeDiscoveryStrategy::get_node_namespaces() {
  std::set<std::string> namespaces;

  auto node_graph = node_->get_node_graph_interface();
  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();

  for (const auto & name_and_ns : names_and_namespaces) {
    std::string node_name = name_and_ns.first;
    std::string ns = name_and_ns.second;
    std::string area = extract_area_from_namespace(ns);
    if (area != "root") {
      namespaces.insert(area);
    } else {
      // For root namespace nodes, add the node name to prevent
      // topic-based discovery from creating duplicate components
      // e.g., node /fault_manager publishes /fault_manager/events
      namespaces.insert(node_name);
    }
  }

  return namespaces;
}

std::vector<Component> RuntimeDiscoveryStrategy::discover_topic_components() {
  std::vector<Component> components;

  // Check policy - if IGNORE, don't create any topic-based entities
  if (config_.topic_only_policy == TopicOnlyPolicy::IGNORE) {
    RCLCPP_DEBUG(node_->get_logger(), "Topic-only policy is IGNORE, skipping topic-based discovery");
    return components;
  }

  // If CREATE_AREA_ONLY, areas are created in discover() but no components here
  if (config_.topic_only_policy == TopicOnlyPolicy::CREATE_AREA_ONLY) {
    RCLCPP_DEBUG(node_->get_logger(), "Topic-only policy is CREATE_AREA_ONLY, skipping component creation");
    return components;
  }

  if (!topic_sampler_) {
    RCLCPP_DEBUG(node_->get_logger(), "Topic sampler not set, skipping topic-based discovery");
    return components;
  }

  // Single graph query - get all namespaces and their topics at once (avoids N+1 queries)
  auto discovery_result = topic_sampler_->discover_topics_by_namespace();

  // Get namespaces that already have nodes (to avoid duplicates)
  auto node_namespaces = get_node_namespaces();

  RCLCPP_DEBUG(node_->get_logger(), "Topic-based discovery: %zu topic namespaces, %zu node namespaces",
               discovery_result.namespaces.size(), node_namespaces.size());

  for (const auto & ns : discovery_result.namespaces) {
    // Skip if there's already a node with this namespace
    if (node_namespaces.count(ns) > 0) {
      RCLCPP_DEBUG(node_->get_logger(), "Skipping namespace '%s' - already has nodes", ns.c_str());
      continue;
    }

    // Get topics from cached result (no additional graph query)
    std::string ns_prefix = "/" + ns;
    auto it = discovery_result.topics_by_ns.find(ns_prefix);
    if (it == discovery_result.topics_by_ns.end()) {
      continue;
    }

    // Check minimum topics threshold
    size_t topic_count = it->second.publishes.size() + it->second.subscribes.size();
    if (static_cast<int>(topic_count) < config_.min_topics_for_component) {
      RCLCPP_DEBUG(node_->get_logger(), "Skipping namespace '%s' - topic count %zu < min %d", ns.c_str(), topic_count,
                   config_.min_topics_for_component);
      continue;
    }

    Component comp;
    comp.id = ns;
    comp.namespace_path = "/" + ns;
    comp.fqn = "/" + ns;
    comp.area = ns;
    comp.source = "topic";
    comp.topics = it->second;

    RCLCPP_DEBUG(node_->get_logger(), "Created topic-based component '%s' with %zu topics", ns.c_str(),
                 comp.topics.publishes.size());

    components.push_back(comp);
  }

  RCLCPP_INFO(node_->get_logger(), "Discovered %zu topic-based components", components.size());
  return components;
}

bool RuntimeDiscoveryStrategy::path_belongs_to_namespace(const std::string & path, const std::string & ns) const {
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

std::vector<Component> RuntimeDiscoveryStrategy::discover_synthetic_components(const std::vector<App> & apps) {
  // Group runtime apps by their component_id (already derived during discover_apps)
  std::map<std::string, std::vector<const App *>> groups;

  for (const auto & app : apps) {
    groups[app.component_id].push_back(&app);
  }

  // Create synthetic components from groups
  std::vector<Component> result;
  for (const auto & [group_id, group_apps] : groups) {
    Component comp;
    comp.id = group_id;
    comp.source = "synthetic";
    comp.type = "ComponentGroup";

    // Use first app's FQN to derive namespace and area
    if (!group_apps.empty() && group_apps[0]->bound_fqn.has_value()) {
      const auto & fqn = group_apps[0]->bound_fqn.value();
      auto pos = fqn.rfind('/');
      comp.namespace_path = (pos == std::string::npos || pos == 0) ? "/" : fqn.substr(0, pos);
      comp.area = extract_area_from_namespace(comp.namespace_path);
      comp.fqn = "/" + group_id;
    }

    // Topics/services stay with Apps - synthetic components are just groupings

    RCLCPP_DEBUG(node_->get_logger(), "Created synthetic component '%s' with %zu apps", group_id.c_str(),
                 group_apps.size());
    result.push_back(comp);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Discovered %zu synthetic components from %zu nodes", result.size(), apps.size());
  return result;
}

std::string RuntimeDiscoveryStrategy::derive_component_id(const std::string & node_id, const std::string & area) {
  switch (config_.grouping) {
    case ComponentGroupingStrategy::NAMESPACE:
      return apply_component_name_pattern(area);
    case ComponentGroupingStrategy::NONE:
    default:
      return node_id;
  }
}

std::string RuntimeDiscoveryStrategy::apply_component_name_pattern(const std::string & area) {
  // Validate inputs to prevent unexpected empty component IDs
  if (area.empty()) {
    RCLCPP_WARN(node_->get_logger(), "apply_component_name_pattern called with empty area, using 'unknown'");
    return apply_component_name_pattern("unknown");
  }

  if (config_.synthetic_component_name_pattern.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Empty synthetic_component_name_pattern, using area directly: %s", area.c_str());
    return area;
  }

  std::string result = config_.synthetic_component_name_pattern;

  // Replace {area} placeholder
  const std::string placeholder = "{area}";
  size_t pos = result.find(placeholder);
  if (pos != std::string::npos) {
    result.replace(pos, placeholder.length(), area);
  } else {
    // Pattern doesn't contain {area} - all synthetic components will have same ID
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "synthetic_component_name_pattern '%s' doesn't contain {area} placeholder - "
                     "all synthetic components will have the same ID, which may cause collisions",
                     config_.synthetic_component_name_pattern.c_str());
  }

  return result;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
