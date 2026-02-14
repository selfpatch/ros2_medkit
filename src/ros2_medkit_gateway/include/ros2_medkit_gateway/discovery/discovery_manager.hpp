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

#include "ros2_medkit_gateway/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/discovery/discovery_strategy.hpp"
#include "ros2_medkit_gateway/discovery/hybrid_discovery.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/discovery/runtime_discovery.hpp"

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

// Forward declarations
class NativeTopicSampler;
class TypeIntrospection;

/**
 * @brief Configuration for discovery
 */
struct DiscoveryConfig {
  DiscoveryMode mode{DiscoveryMode::RUNTIME_ONLY};
  std::string manifest_path;
  bool manifest_strict_validation{true};

  /**
   * @brief Runtime (heuristic) discovery options
   *
   * These options control how the heuristic discovery strategy
   * maps ROS 2 graph entities to SOVD entities.
   */
  struct RuntimeOptions {
    /**
     * @brief Create synthetic Component entities that group Apps
     *
     * When true, Components are synthetic groupings (by namespace).
     * When false, each node is a Component (legacy behavior).
     * Default: true (new behavior for initial release)
     */
    bool create_synthetic_components{true};

    /**
     * @brief How to group nodes into synthetic components
     *
     * Only used when create_synthetic_components is true.
     * - NONE: Each node = 1 component
     * - NAMESPACE: Group by first namespace segment (area)
     */
    ComponentGroupingStrategy grouping{ComponentGroupingStrategy::NAMESPACE};

    /**
     * @brief Naming pattern for synthetic components
     *
     * Placeholders: {area}
     * Default: "{area}" - uses area name as component ID
     */
    std::string synthetic_component_name_pattern{"{area}"};

    /**
     * @brief Policy for handling topic-only namespaces
     *
     * When topics exist in a namespace without ROS 2 nodes:
     * - IGNORE: Don't create any entity
     * - CREATE_COMPONENT: Create component with source="topic" (default)
     * - CREATE_AREA_ONLY: Only create the area, no component
     */
    TopicOnlyPolicy topic_only_policy{TopicOnlyPolicy::CREATE_COMPONENT};

    /**
     * @brief Minimum number of topics to create a component
     *
     * Only applies when topic_only_policy is CREATE_COMPONENT.
     * Namespaces with fewer topics than this threshold are skipped.
     * Default: 1 (create component for any namespace with topics)
     */
    int min_topics_for_component{1};
  } runtime;
};

/**
 * @brief Orchestrates entity discovery using pluggable strategies
 *
 * This class delegates discovery to strategy implementations based on
 * the configured mode:
 * - RUNTIME_ONLY: Uses RuntimeDiscoveryStrategy (traditional ROS graph)
 * - MANIFEST_ONLY: Uses manifest as sole source of truth
 * - HYBRID: Combines manifest definitions with runtime linking
 *
 * The DiscoveryManager provides a unified interface for discovering:
 * - Areas: Logical groupings (ROS 2 namespaces or manifest areas)
 * - Components: Software/hardware units (ROS 2 nodes)
 * - Apps: Software applications (manifest-defined)
 * - Functions: Functional groupings (manifest-defined)
 *
 * @see discovery::RuntimeDiscoveryStrategy
 * @see discovery::HybridDiscoveryStrategy
 * @see discovery::ManifestManager
 */
class DiscoveryManager {
 public:
  /**
   * @brief Construct the discovery manager
   * @param node ROS 2 node for graph introspection (must outlive this manager)
   */
  explicit DiscoveryManager(rclcpp::Node * node);

  /**
   * @brief Initialize with configuration
   *
   * Loads manifest if configured, creates appropriate strategy.
   * For RUNTIME_ONLY mode, this is a no-op.
   *
   * @param config Discovery configuration
   * @return true if initialization succeeded
   */
  bool initialize(const DiscoveryConfig & config);

  // =========================================================================
  // Main discovery methods (delegate to strategy)
  // =========================================================================

  /**
   * @brief Discover all areas
   * @return Vector of discovered Area entities
   */
  std::vector<Area> discover_areas();

  /**
   * @brief Discover all components
   * @return Vector of discovered Component entities
   */
  std::vector<Component> discover_components();

  /**
   * @brief Discover all apps
   * @return Vector of discovered App entities (empty in runtime-only mode)
   */
  std::vector<App> discover_apps();

  /**
   * @brief Discover all functions
   * @return Vector of discovered Function entities (empty in runtime-only mode)
   */
  std::vector<Function> discover_functions();

  // =========================================================================
  // Entity lookup by ID
  // =========================================================================

  /**
   * @brief Get area by ID
   * @param id Area identifier
   * @return Area if found
   */
  std::optional<Area> get_area(const std::string & id);

  /**
   * @brief Get component by ID
   * @param id Component identifier
   * @return Component if found
   */
  std::optional<Component> get_component(const std::string & id);

  /**
   * @brief Get app by ID
   * @param id App identifier
   * @return App if found
   */
  std::optional<App> get_app(const std::string & id);

  /**
   * @brief Get function by ID
   * @param id Function identifier
   * @return Function if found
   */
  std::optional<Function> get_function(const std::string & id);

  // =========================================================================
  // Relationship queries
  // =========================================================================

  /**
   * @brief Get subareas of an area
   * @param area_id Parent area ID
   * @return Vector of child areas
   */
  std::vector<Area> get_subareas(const std::string & area_id);

  /**
   * @brief Get subcomponents of a component
   * @param component_id Parent component ID
   * @return Vector of child components
   */
  std::vector<Component> get_subcomponents(const std::string & component_id);

  /**
   * @brief Get components in an area
   * @param area_id Area ID
   * @return Vector of components in the area
   */
  std::vector<Component> get_components_for_area(const std::string & area_id);

  /**
   * @brief Get apps for a component
   * @param component_id Component ID
   * @return Vector of apps associated with the component
   */
  std::vector<App> get_apps_for_component(const std::string & component_id);

  /**
   * @brief Get host component IDs for a function
   * @param function_id Function ID
   * @return Vector of component IDs that host the function
   */
  std::vector<std::string> get_hosts_for_function(const std::string & function_id);

  // =========================================================================
  // Runtime-specific methods (delegate to runtime strategy)
  // =========================================================================

  /**
   * @brief Discover components from topic namespaces
   * @return Vector of topic-based components
   * @see discovery::RuntimeDiscoveryStrategy::discover_topic_components
   */
  std::vector<Component> discover_topic_components();

  /**
   * @brief Discover all services in the system
   * @return Vector of ServiceInfo with schema information
   */
  std::vector<ServiceInfo> discover_services();

  /**
   * @brief Discover all actions in the system
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
   * @param sampler Pointer to NativeTopicSampler (must outlive DiscoveryManager)
   */
  void set_topic_sampler(NativeTopicSampler * sampler);

  /**
   * @brief Set the type introspection for operation schema enrichment
   * @param introspection Pointer to TypeIntrospection (must outlive DiscoveryManager)
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

  // =========================================================================
  // Manifest management
  // =========================================================================

  /**
   * @brief Get the manifest manager
   * @return Pointer to manifest manager (nullptr if not using manifest)
   */
  discovery::ManifestManager * get_manifest_manager();

  /**
   * @brief Reload manifest from file
   *
   * Only works if a manifest was loaded during initialize().
   *
   * @return true if reload succeeded
   */
  bool reload_manifest();

  // =========================================================================
  // Status
  // =========================================================================

  /**
   * @brief Get current discovery mode
   * @return Active discovery mode
   */
  DiscoveryMode get_mode() const {
    return config_.mode;
  }

  /**
   * @brief Get the current discovery strategy name
   * @return Strategy name (e.g., "runtime", "manifest", "hybrid")
   */
  std::string get_strategy_name() const;

 private:
  /**
   * @brief Create and activate the appropriate strategy
   */
  void create_strategy();

  rclcpp::Node * node_;
  DiscoveryConfig config_;

  // Strategies
  std::unique_ptr<discovery::RuntimeDiscoveryStrategy> runtime_strategy_;
  std::unique_ptr<discovery::ManifestManager> manifest_manager_;
  std::unique_ptr<discovery::HybridDiscoveryStrategy> hybrid_strategy_;

  // Active strategy pointer (points to one of the above)
  discovery::DiscoveryStrategy * active_strategy_{nullptr};
};

}  // namespace ros2_medkit_gateway
