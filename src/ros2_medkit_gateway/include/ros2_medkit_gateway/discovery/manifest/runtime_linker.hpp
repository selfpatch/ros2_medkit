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

#pragma once

#include "ros2_medkit_gateway/discovery/manifest/manifest.hpp"
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"

#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Result of runtime linking operation
 *
 * Contains all the information about how manifest Apps were linked
 * to runtime ROS 2 nodes, including orphan detection.
 */
struct LinkingResult {
  /// Apps with successfully bound ROS nodes (and those that failed)
  std::vector<App> linked_apps;

  /// App IDs that have no matching ROS node (offline or external)
  std::vector<std::string> unlinked_app_ids;

  /// ROS node FQNs that have no matching manifest App (orphans)
  std::vector<std::string> orphan_nodes;

  /// Mapping from App ID to bound node FQN
  std::unordered_map<std::string, std::string> app_to_node;

  /// Mapping from node FQN to App ID (reverse lookup)
  std::unordered_map<std::string, std::string> node_to_app;

  /// Check if linking produced any errors based on policy
  bool has_errors(ManifestConfig::UnmanifestedNodePolicy policy) const {
    return policy == ManifestConfig::UnmanifestedNodePolicy::ERROR && !orphan_nodes.empty();
  }

  /// Get statistics summary
  std::string summary() const {
    return std::to_string(app_to_node.size()) + " linked, " + std::to_string(unlinked_app_ids.size()) + " unlinked, " +
           std::to_string(orphan_nodes.size()) + " orphan nodes";
  }
};

/**
 * @brief Links manifest Apps to runtime ROS 2 nodes
 *
 * RuntimeLinker performs the binding between manifest-declared Apps
 * and actual ROS 2 nodes discovered at runtime. This enables:
 * - Stable App IDs from manifest
 * - Live data (topics, services, actions) from runtime nodes
 * - Detection of offline apps and orphan nodes
 *
 * Match priority:
 * 1. Exact match: node_name + namespace both match
 * 2. Wildcard namespace: node_name matches, namespace is "*"
 * 3. Topic namespace: topic_namespace prefix matches node's topics
 */
class RuntimeLinker {
 public:
  /**
   * @brief Constructor
   * @param node ROS node for logging (can be nullptr for testing)
   */
  explicit RuntimeLinker(rclcpp::Node * node = nullptr);

  /**
   * @brief Link manifest apps to runtime nodes
   *
   * @param apps Apps from manifest
   * @param runtime_components Components discovered from ROS graph
   * @param config Manifest config with orphan policy
   * @return LinkingResult with linked apps and orphan info
   */
  LinkingResult link(const std::vector<App> & apps, const std::vector<Component> & runtime_components,
                     const ManifestConfig & config);

  /**
   * @brief Check if a specific app is linked to a runtime node
   * @param app_id App identifier
   * @return true if app is online (linked to a node)
   */
  bool is_app_online(const std::string & app_id) const;

  /**
   * @brief Get the node FQN bound to an app
   * @param app_id App identifier
   * @return Node FQN if linked, nullopt otherwise
   */
  std::optional<std::string> get_bound_node(const std::string & app_id) const;

  /**
   * @brief Get the app ID for a node FQN
   * @param node_fqn Fully qualified node name
   * @return App ID if bound, nullopt otherwise
   */
  std::optional<std::string> get_app_for_node(const std::string & node_fqn) const;

  /**
   * @brief Get the last linking result
   */
  const LinkingResult & get_last_result() const {
    return last_result_;
  }

 private:
  /**
   * @brief Try to match a ROS node to an app's binding
   * @param binding The app's ROS binding configuration
   * @param node_fqn Fully qualified node name
   * @param node_name Node name (without namespace)
   * @param node_namespace Node namespace
   * @return true if binding matches
   */
  bool matches_binding(const App::RosBinding & binding, const std::string & node_fqn, const std::string & node_name,
                       const std::string & node_namespace) const;

  /**
   * @brief Try to match by topic namespace
   * @param topic_namespace Topic namespace pattern from binding
   * @param component Component with topic info
   * @return true if any topic matches the prefix
   */
  bool matches_topic_namespace(const std::string & topic_namespace, const Component & component) const;

  /**
   * @brief Enrich app with runtime data from matched component
   * @param app App to enrich (modified in place)
   * @param component Component with runtime data
   */
  void enrich_app(App & app, const Component & component);

  /**
   * @brief Log message at info level
   */
  void log_info(const std::string & msg) const;

  /**
   * @brief Log message at debug level
   */
  void log_debug(const std::string & msg) const;

  /**
   * @brief Log message at warning level
   */
  void log_warn(const std::string & msg) const;

  /**
   * @brief Log message at error level
   */
  void log_error(const std::string & msg) const;

  rclcpp::Node * node_;
  LinkingResult last_result_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
