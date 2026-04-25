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

#include "ros2_medkit_gateway/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/discovery/discovery_strategy.hpp"
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/type_introspection.hpp"

#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
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
 * - Discovers Functions from node namespaces (functional grouping)
 * - Exposes nodes as Apps
 * - Enriches apps with services, actions, and topics
 *
 * Note: Synthetic/heuristic Area and Component creation has been removed.
 * Areas come from manifest only. Components come from HostInfoProvider
 * or manifest. Namespaces create Function entities.
 */
class RuntimeDiscoveryStrategy : public DiscoveryStrategy {
 public:
  /**
   * @brief Runtime discovery configuration options
   */
  struct RuntimeConfig {
    bool create_functions_from_namespaces{true};
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
  /// @note Always returns empty - Areas come from manifest only
  std::vector<Area> discover_areas() override;

  /// @copydoc DiscoveryStrategy::discover_components
  /// @note Always returns empty - Components come from HostInfoProvider or manifest
  std::vector<Component> discover_components() override;

  /// @copydoc DiscoveryStrategy::discover_apps
  /// @note Returns nodes as Apps in runtime discovery
  std::vector<App> discover_apps() override;

  /// @copydoc DiscoveryStrategy::discover_functions
  /// @note Creates Function entities from namespace grouping when enabled
  std::vector<Function> discover_functions() override;

  /**
   * @brief Create Function entities from pre-discovered apps
   *
   * Avoids redundant ROS 2 graph introspection when apps have already been
   * discovered in the same refresh cycle (e.g., refresh_cache() calls
   * discover_apps() before discover_functions()).
   *
   * @param apps Pre-discovered apps to group by namespace
   * @return Vector of Function entities
   */
  std::vector<Function> discover_functions(const std::vector<App> & apps);

  /// @copydoc DiscoveryStrategy::get_name
  std::string get_name() const override {
    return "runtime";
  }

  // =========================================================================
  // Runtime-specific methods (from current DiscoveryManager)
  // =========================================================================

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
   * @brief Set the topic data provider for component-topic mapping
   * @param provider Pointer to TopicDataProvider (must outlive this strategy)
   */
  void set_topic_data_provider(TopicDataProvider * provider);

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

  /// Check if a service path belongs to a component namespace
  bool path_belongs_to_namespace(const std::string & path, const std::string & ns) const;

  /// Check if a service path is an internal ROS2 service
  static bool is_internal_service(const std::string & service_path);

  rclcpp::Node * node_;
  RuntimeConfig config_;
  TopicDataProvider * topic_data_provider_{nullptr};
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
