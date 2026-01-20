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

#ifndef ROS2_MEDKIT_GATEWAY__DISCOVERY__RUNTIME_DISCOVERY_HPP_
#define ROS2_MEDKIT_GATEWAY__DISCOVERY__RUNTIME_DISCOVERY_HPP_

#include "ros2_medkit_gateway/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/discovery/discovery_strategy.hpp"
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/native_topic_sampler.hpp"
#include "ros2_medkit_gateway/type_introspection.hpp"

#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

// Forward declaration
struct DiscoveryConfig;

namespace discovery {

/**
 * @brief Runtime discovery strategy using ROS 2 graph introspection
 *
 * This is the current discovery behavior extracted into a strategy class.
 * It discovers entities by querying the ROS 2 node graph at runtime.
 *
 * Features:
 * - Discovers areas from node namespaces
 * - Discovers components from ROS 2 nodes
 * - Discovers topic-based "virtual" components for systems like Isaac Sim
 * - Enriches components with services, actions, and topics
 * - Exposes nodes as Apps
 * - Can create synthetic Components that group Apps
 *
 * @note Functions are not supported in runtime-only mode.
 *       Use ManifestDiscoveryStrategy for custom entity definitions.
 */
class RuntimeDiscoveryStrategy : public DiscoveryStrategy {
 public:
  /**
   * @brief Runtime discovery configuration options
   */
  struct RuntimeConfig {
    bool create_synthetic_components{true};
    ComponentGroupingStrategy grouping{};
    std::string synthetic_component_name_pattern{"{area}"};
    TopicOnlyPolicy topic_only_policy{TopicOnlyPolicy::CREATE_COMPONENT};
    int min_topics_for_component{1};
  };

  /**
   * @brief Construct runtime discovery strategy
   * @param node ROS 2 node for graph introspection (must outlive this strategy)
   */
  explicit RuntimeDiscoveryStrategy(rclcpp::Node * node);

  /**
   * @brief Set runtime discovery configuration
   * @param config Runtime options from DiscoveryConfig
   */
  void set_config(const RuntimeConfig & config);

  /// @copydoc DiscoveryStrategy::discover_areas
  std::vector<Area> discover_areas() override;

  /// @copydoc DiscoveryStrategy::discover_components
  std::vector<Component> discover_components() override;

  /// @copydoc DiscoveryStrategy::discover_apps
  /// @note Returns nodes as Apps in runtime discovery
  std::vector<App> discover_apps() override;

  /// @copydoc DiscoveryStrategy::discover_functions
  /// @note Returns empty vector - functions require manifest
  std::vector<Function> discover_functions() override;

  /// @copydoc DiscoveryStrategy::get_name
  std::string get_name() const override {
    return "runtime";
  }

  // =========================================================================
  // Runtime-specific methods (from current DiscoveryManager)
  // =========================================================================

  /**
   * @brief Discover node-based components (individual ROS 2 nodes)
   *
   * This returns the traditional component discovery where each node
   * becomes a Component. Used internally when synthetic components
   * are not enabled or for building Apps.
   *
   * @return Vector of node-based components
   */
  std::vector<Component> discover_node_components();

  /**
   * @brief Discover synthetic components (grouped by namespace)
   *
   * Creates aggregated Components that group multiple nodes by namespace.
   * Only used when create_synthetic_components is enabled.
   *
   * @return Vector of synthetic components
   */
  std::vector<Component> discover_synthetic_components();

  /**
   * @brief Discover components from topic namespaces (topic-based discovery)
   *
   * Creates "virtual" components for topic namespaces that don't have
   * corresponding ROS 2 nodes. This is useful for systems like Isaac Sim
   * that publish topics without creating proper ROS 2 nodes.
   *
   * @return Vector of topic-based components (excludes namespaces with existing nodes)
   */
  std::vector<Component> discover_topic_components();

  /**
   * @brief Discover all services in the system with their types
   * @return Vector of ServiceInfo with schema information
   */
  std::vector<ServiceInfo> discover_services();

  /**
   * @brief Discover all actions in the system with their types
   * @return Vector of ActionInfo with schema information
   */
  std::vector<ActionInfo> discover_actions();

  /**
   * @brief Find a service by component namespace and operation name
   * @param component_ns Component namespace
   * @param operation_name Service name
   * @return ServiceInfo if found, nullopt otherwise
   */
  std::optional<ServiceInfo> find_service(const std::string & component_ns, const std::string & operation_name) const;

  /**
   * @brief Find an action by component namespace and operation name
   * @param component_ns Component namespace
   * @param operation_name Action name
   * @return ActionInfo if found, nullopt otherwise
   */
  std::optional<ActionInfo> find_action(const std::string & component_ns, const std::string & operation_name) const;

  /**
   * @brief Set the topic sampler for component-topic mapping
   * @param sampler Pointer to NativeTopicSampler (must outlive this strategy)
   */
  void set_topic_sampler(NativeTopicSampler * sampler);

  /**
   * @brief Set the type introspection for operation schema enrichment
   * @param introspection Pointer to TypeIntrospection (must outlive this strategy)
   */
  void set_type_introspection(TypeIntrospection * introspection);

  /**
   * @brief Refresh the cached topic map
   */
  void refresh_topic_map();

  /**
   * @brief Check if topic map has been built at least once
   * @return true if topic map is ready, false if not yet built
   */
  bool is_topic_map_ready() const;

 private:
  /// Extract area ID from namespace (e.g., "/powertrain/engine" -> "powertrain")
  std::string extract_area_from_namespace(const std::string & ns);

  /// Extract the last segment from a path (e.g., "/a/b/c" -> "c")
  std::string extract_name_from_path(const std::string & path);

  /// Get set of namespaces that have ROS 2 nodes (for deduplication)
  std::set<std::string> get_node_namespaces();

  /// Check if a service path belongs to a component namespace
  bool path_belongs_to_namespace(const std::string & path, const std::string & ns) const;

  /// Check if a service path is an internal ROS2 service
  static bool is_internal_service(const std::string & service_path);

  /// Derive component ID for a node based on grouping strategy
  std::string derive_component_id(const Component & node);

  /// Apply naming pattern for synthetic component ID
  std::string apply_component_name_pattern(const std::string & area);

  rclcpp::Node * node_;
  RuntimeConfig config_;
  NativeTopicSampler * topic_sampler_{nullptr};
  TypeIntrospection * type_introspection_{nullptr};

  // Cached services and actions for lookup
  std::vector<ServiceInfo> cached_services_;
  std::vector<ActionInfo> cached_actions_;

  // Cached topic map for performance (rebuilt on demand or periodically)
  std::map<std::string, ComponentTopics> cached_topic_map_;
  bool topic_map_ready_{false};
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__DISCOVERY__RUNTIME_DISCOVERY_HPP_
