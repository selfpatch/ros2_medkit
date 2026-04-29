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

#include "ros2_medkit_gateway/ros2/providers/ros2_runtime_introspection.hpp"

#include <algorithm>
#include <set>
#include <unordered_map>

namespace ros2_medkit_gateway::ros2 {

namespace {

/**
 * @brief Exact suffix match on a service path.
 *
 * Avoids false positives that a substring search would produce
 * (e.g. "/my_node/get_parameters" is internal but
 * "/my_node/get_parameters_backup" is not).
 */
bool matches_internal_suffix(const std::string & service_path, const std::string & suffix) {
  if (service_path.length() < suffix.length()) {
    return false;
  }
  return service_path.compare(service_path.length() - suffix.length(), suffix.length(), suffix) == 0;
}

}  // namespace

bool Ros2RuntimeIntrospection::is_internal_service(const std::string & service_path) {
  return matches_internal_suffix(service_path, "/get_parameters") ||
         matches_internal_suffix(service_path, "/set_parameters") ||
         matches_internal_suffix(service_path, "/list_parameters") ||
         matches_internal_suffix(service_path, "/describe_parameters") ||
         matches_internal_suffix(service_path, "/get_parameter_types") ||
         matches_internal_suffix(service_path, "/set_parameters_atomically") ||
         matches_internal_suffix(service_path, "/get_type_description") ||
         service_path.find("/_action/") != std::string::npos;
}

Ros2RuntimeIntrospection::Ros2RuntimeIntrospection(rclcpp::Node * node) : node_(node) {
}

void Ros2RuntimeIntrospection::set_config(const RuntimeConfig & config) {
  config_ = config;
  RCLCPP_DEBUG(node_->get_logger(), "Runtime introspection config: functions_from_namespaces=%s",
               config_.create_functions_from_namespaces ? "true" : "false");
}

IntrospectionResult Ros2RuntimeIntrospection::introspect(const IntrospectionInput & /*input*/) {
  IntrospectionResult result;
  // Areas and Components are never produced by runtime introspection.
  // Apps come from the live graph; Functions are derived from namespaces.
  auto apps = discover_apps();
  result.new_entities.functions = discover_functions(apps);
  result.new_entities.apps = std::move(apps);
  return result;
}

std::vector<App> Ros2RuntimeIntrospection::discover_apps() {
  std::vector<App> apps;

  std::unordered_map<std::string, ServiceInfo> service_info_map;
  auto all_services = discover_services();
  for (const auto & svc : all_services) {
    service_info_map[svc.full_path] = svc;
  }

  std::unordered_map<std::string, ActionInfo> action_info_map;
  auto all_actions = discover_actions();
  for (const auto & act : all_actions) {
    action_info_map[act.full_path] = act;
  }

  auto node_graph = node_->get_node_graph_interface();
  std::vector<std::pair<std::string, std::string>> names_and_namespaces;
  try {
    names_and_namespaces = node_graph->get_node_names_and_namespaces();
  } catch (const std::runtime_error & ex) {
    // rclcpp throws "rcl node's context is invalid" when get_node_names_*
    // is called after rclcpp::shutdown (e.g. refresh timer fires once
    // between SIGINT handling and the executor stopping). Swallow and
    // return empty so ~GatewayNode's shutdown path isn't aborted mid-run
    // by std::terminate; callers handle empty gracefully.
    RCLCPP_DEBUG(node_->get_logger(), "get_node_names_and_namespaces threw during shutdown: %s", ex.what());
    return {};
  }

  if (topic_data_provider_ && !topic_map_ready_) {
    refresh_topic_map();
  }

  std::set<std::string> seen_fqns;

  // Detect bare-name collisions across namespaces so IDs are only disambiguated
  // when necessary (preserves backward compatibility for the common case where
  // node names are unique). Count distinct namespaces per bare name to avoid
  // false positives from RMW duplicates (same FQN reported multiple times).
  std::unordered_map<std::string, std::set<std::string>> name_namespaces;
  for (const auto & name_and_ns : names_and_namespaces) {
    name_namespaces[name_and_ns.first].insert(name_and_ns.second);
  }

  for (const auto & name_and_ns : names_and_namespaces) {
    const auto & name = name_and_ns.first;
    const auto & ns = name_and_ns.second;

    std::string fqn = (ns == "/") ? std::string("/").append(name) : std::string(ns).append("/").append(name);

    if (seen_fqns.count(fqn) > 0) {
      RCLCPP_DEBUG(node_->get_logger(), "Skipping duplicate node: %s", fqn.c_str());
      continue;
    }
    seen_fqns.insert(fqn);

    App app;
    if (name_namespaces[name].size() > 1 && ns != "/") {
      std::string ns_prefix = ns.substr(1);  // Remove leading '/'
      std::replace(ns_prefix.begin(), ns_prefix.end(), '/', '_');
      app.id = ns_prefix;
      app.id += "_";
      app.id += name;
      RCLCPP_DEBUG(node_->get_logger(), "Name collision detected for '%s', using namespaced ID '%s'", name.c_str(),
                   app.id.c_str());
    } else {
      app.id = name;
    }
    app.name = name;
    app.source = "heuristic";
    app.is_online = true;
    app.bound_fqn = fqn;

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

    if (topic_data_provider_) {
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

std::vector<Function> Ros2RuntimeIntrospection::discover_functions() {
  if (!config_.create_functions_from_namespaces) {
    return {};
  }
  return discover_functions(discover_apps());
}

std::vector<Function> Ros2RuntimeIntrospection::discover_functions(const std::vector<App> & apps) {
  if (!config_.create_functions_from_namespaces) {
    return {};
  }

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

std::vector<ServiceInfo> Ros2RuntimeIntrospection::discover_services() {
  std::vector<ServiceInfo> services;

  // Swallow the "context invalid" throw that fires when the refresh timer
  // races rclcpp::shutdown during SIGINT; callers already handle an empty list.
  std::map<std::string, std::vector<std::string>> service_names_and_types;
  try {
    service_names_and_types = node_->get_service_names_and_types();
  } catch (const std::runtime_error & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "get_service_names_and_types threw during shutdown: %s", ex.what());
    return services;
  }

  for (const auto & [service_path, types] : service_names_and_types) {
    if (is_internal_service(service_path)) {
      continue;
    }

    ServiceInfo info;
    info.full_path = service_path;
    info.name = extract_name_from_path(service_path);
    info.type = types.empty() ? "" : types[0];

    // Service types: pkg/srv/Type -> Request/Response companions.
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

  cached_services_ = services;

  return services;
}

std::vector<ActionInfo> Ros2RuntimeIntrospection::discover_actions() {
  std::vector<ActionInfo> actions;

  // Actions are detected by looking for /_action/send_goal services (no native
  // rclcpp API for action names).
  std::map<std::string, std::vector<std::string>> service_names_and_types;
  try {
    service_names_and_types = node_->get_service_names_and_types();
  } catch (const std::runtime_error & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "get_service_names_and_types threw during shutdown: %s", ex.what());
    return actions;
  }

  std::set<std::string> discovered_action_paths;

  for (const auto & [service_path, types] : service_names_and_types) {
    const std::string action_suffix = "/_action/send_goal";
    if (service_path.length() > action_suffix.length() &&
        service_path.compare(service_path.length() - action_suffix.length(), action_suffix.length(), action_suffix) ==
            0) {
      std::string action_path = service_path.substr(0, service_path.length() - action_suffix.length());

      if (discovered_action_paths.count(action_path) == 0) {
        discovered_action_paths.insert(action_path);

        ActionInfo info;
        info.full_path = action_path;
        info.name = extract_name_from_path(action_path);

        // Service type is like "example_interfaces/action/Fibonacci_SendGoal";
        // strip the suffix to get "example_interfaces/action/Fibonacci".
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

        // Action types: pkg/action/Type -> Goal/Result/Feedback companions.
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

  cached_actions_ = actions;

  return actions;
}

std::optional<ServiceInfo> Ros2RuntimeIntrospection::find_service(const std::string & component_ns,
                                                                  const std::string & operation_name) const {
  std::string expected_path = component_ns;
  if (!expected_path.empty() && expected_path.back() != '/') {
    expected_path += "/";
  }
  expected_path += operation_name;

  for (const auto & svc : cached_services_) {
    if (svc.full_path == expected_path || svc.name == operation_name) {
      if (path_belongs_to_namespace(svc.full_path, component_ns)) {
        return svc;
      }
    }
  }

  return std::nullopt;
}

std::optional<ActionInfo> Ros2RuntimeIntrospection::find_action(const std::string & component_ns,
                                                                const std::string & operation_name) const {
  std::string expected_path = component_ns;
  if (!expected_path.empty() && expected_path.back() != '/') {
    expected_path += "/";
  }
  expected_path += operation_name;

  for (const auto & act : cached_actions_) {
    if (act.full_path == expected_path || act.name == operation_name) {
      if (path_belongs_to_namespace(act.full_path, component_ns)) {
        return act;
      }
    }
  }

  return std::nullopt;
}

void Ros2RuntimeIntrospection::set_topic_data_provider(TopicDataProvider * provider) {
  topic_data_provider_ = provider;
}

void Ros2RuntimeIntrospection::set_type_introspection(TypeIntrospection * introspection) {
  type_introspection_ = introspection;
}

void Ros2RuntimeIntrospection::refresh_topic_map() {
  if (!topic_data_provider_) {
    return;
  }
  cached_topic_map_ = topic_data_provider_->build_component_topic_map();
  topic_map_ready_ = true;
  RCLCPP_DEBUG(node_->get_logger(), "Topic map refreshed: %zu components", cached_topic_map_.size());
}

bool Ros2RuntimeIntrospection::is_topic_map_ready() const {
  return topic_map_ready_;
}

std::string Ros2RuntimeIntrospection::extract_area_from_namespace(const std::string & ns) {
  if (ns == "/" || ns.empty()) {
    return "root";
  }

  std::string cleaned = ns;
  if (cleaned[0] == '/') {
    cleaned = cleaned.substr(1);
  }

  auto pos = cleaned.find('/');
  if (pos != std::string::npos) {
    return cleaned.substr(0, pos);
  }

  return cleaned;
}

std::string Ros2RuntimeIntrospection::extract_name_from_path(const std::string & path) {
  if (path.empty()) {
    return "";
  }

  auto pos = path.rfind('/');
  if (pos != std::string::npos && pos < path.length() - 1) {
    return path.substr(pos + 1);
  }

  return path;
}

bool Ros2RuntimeIntrospection::path_belongs_to_namespace(const std::string & path, const std::string & ns) const {
  if (ns.empty() || ns == "/") {
    if (path.empty() || path[0] != '/') {
      return false;
    }
    std::string without_leading = path.substr(1);
    return without_leading.find('/') == std::string::npos;
  }

  if (path.length() <= ns.length()) {
    return false;
  }

  if (path.compare(0, ns.length(), ns) != 0) {
    return false;
  }

  if (path[ns.length()] != '/') {
    return false;
  }

  std::string remainder = path.substr(ns.length() + 1);
  return remainder.find('/') == std::string::npos;
}

}  // namespace ros2_medkit_gateway::ros2
