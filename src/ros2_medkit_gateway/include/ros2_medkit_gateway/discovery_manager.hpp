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
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/models.hpp"
#include "ros2_medkit_gateway/native_topic_sampler.hpp"
#include "ros2_medkit_gateway/type_introspection.hpp"

namespace ros2_medkit_gateway {

class DiscoveryManager {
 public:
  explicit DiscoveryManager(rclcpp::Node * node);

  std::vector<Area> discover_areas();
  std::vector<Component> discover_components();

  /**
   * @brief Discover components from topic namespaces (topic-based discovery)
   *
   * Creates "virtual" components for topic namespaces that don't have
   * corresponding ROS 2 nodes. This is useful for systems like Isaac Sim
   * that publish topics without creating proper ROS 2 nodes.
   *
   * Example: Topics ["/carter1/odom", "/carter1/cmd_vel", "/carter2/odom"]
   * Creates components: carter1, carter2 (if no matching nodes exist)
   *
   * Components are created with:
   * - id: namespace name (e.g., "carter1")
   * - source: "topic" (to distinguish from node-based components)
   * - topics.publishes: all topics under this namespace
   *
   * @return Vector of topic-based components (excludes namespaces with existing nodes)
   */
  std::vector<Component> discover_topic_components();

  /// Discover all services in the system with their types
  std::vector<ServiceInfo> discover_services();

  /// Discover all actions in the system with their types
  std::vector<ActionInfo> discover_actions();

  /// Find a service by component namespace and operation name
  std::optional<ServiceInfo> find_service(const std::string & component_ns, const std::string & operation_name) const;

  /// Find an action by component namespace and operation name
  std::optional<ActionInfo> find_action(const std::string & component_ns, const std::string & operation_name) const;

  /**
   * @brief Set the topic sampler for component-topic mapping
   *
   * When set, discover_components() will populate the topics field
   * of each component with its publishes/subscribes lists.
   *
   * @param sampler Pointer to NativeTopicSampler (must outlive DiscoveryManager)
   */
  void set_topic_sampler(NativeTopicSampler * sampler);

  /**
   * @brief Set the type introspection for operation schema enrichment
   *
   * When set, discover_services() and discover_actions() will populate
   * the type_info field with schema information.
   *
   * @param introspection Pointer to TypeIntrospection (must outlive DiscoveryManager)
   */
  void set_type_introspection(TypeIntrospection * introspection);

  /**
   * @brief Refresh the cached topic map
   *
   * Call this to force a rebuild of the component-topic map.
   * The map is built once at startup and cached for performance.
   * Periodic background refresh will also update this cache.
   */
  void refresh_topic_map();

  /**
   * @brief Check if topic map has been built at least once
   * @return true if topic map is ready, false if not yet built
   */
  bool is_topic_map_ready() const;

 private:
  std::string extract_area_from_namespace(const std::string & ns);

  /// Extract the last segment from a path (e.g., "/a/b/c" -> "c")
  std::string extract_name_from_path(const std::string & path);

  /// Get set of namespaces that have ROS 2 nodes (for deduplication)
  std::set<std::string> get_node_namespaces();

  /// Check if a service path belongs to a component namespace
  bool path_belongs_to_namespace(const std::string & path, const std::string & ns) const;

  /// Check if a service path is an internal ROS2 service (parameter services, action internals, etc.)
  static bool is_internal_service(const std::string & service_path);

  rclcpp::Node * node_;
  NativeTopicSampler * topic_sampler_{nullptr};
  TypeIntrospection * type_introspection_{nullptr};

  // Cached services and actions for lookup
  std::vector<ServiceInfo> cached_services_;
  std::vector<ActionInfo> cached_actions_;

  // Cached topic map for performance (rebuilt on demand or periodically)
  std::map<std::string, ComponentTopics> cached_topic_map_;
  bool topic_map_ready_{false};
};

}  // namespace ros2_medkit_gateway
