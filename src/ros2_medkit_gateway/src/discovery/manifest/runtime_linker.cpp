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

#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"

#include <algorithm>

namespace ros2_medkit_gateway {
namespace discovery {

RuntimeLinker::RuntimeLinker(rclcpp::Node * node) : node_(node) {
}

LinkingResult RuntimeLinker::link(const std::vector<App> & apps, const std::vector<Component> & runtime_components,
                                  const ManifestConfig & config) {
  LinkingResult result;

  // Build a map of node FQN -> Component for quick lookup
  std::unordered_map<std::string, const Component *> fqn_to_component;
  for (const auto & comp : runtime_components) {
    fqn_to_component[comp.fqn] = &comp;
  }

  // Track which runtime nodes have been matched
  std::set<std::string> matched_nodes;

  // Process each manifest app
  for (const auto & manifest_app : apps) {
    App linked_app = manifest_app;  // Copy
    linked_app.is_online = false;

    // External apps don't need ROS binding
    if (manifest_app.external) {
      result.linked_apps.push_back(linked_app);
      continue;
    }

    // If no ROS binding, app can't be linked
    if (!manifest_app.ros_binding.has_value() || manifest_app.ros_binding->is_empty()) {
      result.unlinked_app_ids.push_back(manifest_app.id);
      result.linked_apps.push_back(linked_app);
      continue;
    }

    const auto & binding = manifest_app.ros_binding.value();
    bool found = false;

    // Try to find matching runtime node
    for (const auto & comp : runtime_components) {
      // Extract node name and namespace from component
      std::string node_name = comp.id;
      std::string node_ns = comp.namespace_path;

      if (matches_binding(binding, comp.fqn, node_name, node_ns)) {
        // Match found!
        linked_app.bound_fqn = comp.fqn;
        linked_app.is_online = true;
        enrich_app(linked_app, comp);

        result.app_to_node[manifest_app.id] = comp.fqn;
        result.node_to_app[comp.fqn] = manifest_app.id;
        matched_nodes.insert(comp.fqn);
        found = true;

        log_debug("Linked app '" + manifest_app.id + "' to node '" + comp.fqn + "'");
        break;
      }

      // Try topic namespace matching
      if (!binding.topic_namespace.empty() && matches_topic_namespace(binding.topic_namespace, comp)) {
        linked_app.bound_fqn = comp.fqn;
        linked_app.is_online = true;
        enrich_app(linked_app, comp);

        result.app_to_node[manifest_app.id] = comp.fqn;
        result.node_to_app[comp.fqn] = manifest_app.id;
        matched_nodes.insert(comp.fqn);
        found = true;

        log_debug("Linked app '" + manifest_app.id + "' to node '" + comp.fqn + "' (topic namespace)");
        break;
      }
    }

    if (!found) {
      result.unlinked_app_ids.push_back(manifest_app.id);
      log_debug("App '" + manifest_app.id + "' not linked (no matching node)");
    }

    result.linked_apps.push_back(linked_app);
  }

  // Find orphan nodes (runtime nodes not matching any manifest app)
  for (const auto & comp : runtime_components) {
    if (matched_nodes.find(comp.fqn) == matched_nodes.end()) {
      result.orphan_nodes.push_back(comp.fqn);
    }
  }

  // Log summary
  log_info("Runtime linking: " + result.summary());

  // Handle orphan nodes according to policy
  if (!result.orphan_nodes.empty()) {
    switch (config.unmanifested_nodes) {
      case ManifestConfig::UnmanifestedNodePolicy::IGNORE:
        log_debug("Ignoring " + std::to_string(result.orphan_nodes.size()) + " orphan nodes");
        break;

      case ManifestConfig::UnmanifestedNodePolicy::WARN:
        for (const auto & orphan : result.orphan_nodes) {
          log_warn("Orphan node (not in manifest): " + orphan);
        }
        break;

      case ManifestConfig::UnmanifestedNodePolicy::ERROR:
        log_error("Orphan nodes detected with 'error' policy. Discovery will fail.");
        break;

      case ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN:
        log_info("Including " + std::to_string(result.orphan_nodes.size()) + " orphan nodes with source='orphan'");
        break;
    }
  }

  last_result_ = result;
  return result;
}

bool RuntimeLinker::matches_binding(const App::RosBinding & binding, const std::string & node_fqn,
                                    const std::string & node_name, const std::string & node_namespace) const {
  // Check node name match
  if (binding.node_name.empty()) {
    return false;
  }

  // Node name can be simple or with subpath (e.g., "local_costmap/local_costmap")
  // Check if binding.node_name matches node_name or is contained in fqn
  bool name_matches = (node_name == binding.node_name) || (node_fqn.find("/" + binding.node_name) != std::string::npos);

  if (!name_matches) {
    return false;
  }

  // Check namespace match
  if (binding.namespace_pattern == "*") {
    // Wildcard matches any namespace
    return true;
  }

  // Exact namespace match
  std::string expected_ns = binding.namespace_pattern;
  if (expected_ns.empty()) {
    expected_ns = "/";
  }

  // Normalize namespaces for comparison
  std::string actual_ns = node_namespace;
  if (actual_ns.empty()) {
    actual_ns = "/";
  }

  return actual_ns == expected_ns || actual_ns.find(expected_ns) == 0;  // Prefix match
}

bool RuntimeLinker::matches_topic_namespace(const std::string & topic_namespace, const Component & component) const {
  // Check if any topic starts with the given namespace
  for (const auto & topic : component.topics.publishes) {
    if (topic.find(topic_namespace) == 0) {
      return true;
    }
  }
  for (const auto & topic : component.topics.subscribes) {
    if (topic.find(topic_namespace) == 0) {
      return true;
    }
  }
  return false;
}

void RuntimeLinker::enrich_app(App & app, const Component & component) {
  // Copy topics
  app.topics = component.topics;

  // Copy services
  app.services = component.services;

  // Copy actions
  app.actions = component.actions;
}

bool RuntimeLinker::is_app_online(const std::string & app_id) const {
  return last_result_.app_to_node.find(app_id) != last_result_.app_to_node.end();
}

std::optional<std::string> RuntimeLinker::get_bound_node(const std::string & app_id) const {
  auto it = last_result_.app_to_node.find(app_id);
  if (it != last_result_.app_to_node.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<std::string> RuntimeLinker::get_app_for_node(const std::string & node_fqn) const {
  auto it = last_result_.node_to_app.find(node_fqn);
  if (it != last_result_.node_to_app.end()) {
    return it->second;
  }
  return std::nullopt;
}

void RuntimeLinker::log_info(const std::string & msg) const {
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  }
}

void RuntimeLinker::log_debug(const std::string & msg) const {
  if (node_) {
    RCLCPP_DEBUG(node_->get_logger(), "%s", msg.c_str());
  }
}

void RuntimeLinker::log_warn(const std::string & msg) const {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
  }
}

void RuntimeLinker::log_error(const std::string & msg) const {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
