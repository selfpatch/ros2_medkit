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
#include "ros2_medkit_gateway/discovery/host_info_provider.hpp"
#include "ros2_medkit_gateway/discovery/hybrid_discovery.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/discovery/runtime_discovery.hpp"

#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

// Forward declarations
class TopicDataProvider;
class TypeIntrospection;
class IntrospectionProvider;

/**
 * @brief Configuration for discovery
 */
struct DiscoveryConfig {
  DiscoveryMode mode{DiscoveryMode::RUNTIME_ONLY};
  std::string manifest_path;
  bool manifest_strict_validation{true};
  bool manifest_enabled{true};  // enable/disable manifest layer in hybrid mode
  bool runtime_enabled{true};   // enable/disable runtime layer in hybrid mode
  /// Directory scanned for manifest fragment yaml files on every manifest
  /// load / reload. Empty = disabled. Fragments may contribute apps,
  /// components, and functions; top-level fields (areas, metadata, config,
  /// scripts, capabilities, lock_overrides) are reserved for the base
  /// manifest. See `ManifestManager::set_fragments_dir`.
  std::string manifest_fragments_dir;

  /**
   * @brief Runtime (heuristic) discovery options
   *
   * These options control how the heuristic discovery strategy
   * maps ROS 2 graph entities to SOVD entities.
   *
   * Note: Synthetic/heuristic Area and Component creation has been removed.
   * Areas come from manifest only. Components come from HostInfoProvider
   * or manifest. Namespaces create Function entities.
   */
  struct RuntimeOptions {
    /**
     * @brief Create Function entities from ROS 2 namespace grouping
     *
     * When true (default), namespaces are mapped to Function entities.
     * Each Function hosts the Apps in that namespace.
     * This is the SOVD-correct mapping: namespaces represent
     * functional grouping, not deployment topology.
     */
    bool create_functions_from_namespaces{true};

    /**
     * @brief Create a default Component from HostInfoProvider
     *
     * When true (default), a single host-level Component is created
     * from system info (hostname, OS, arch) instead of synthetic
     * per-namespace Components. All discovered Apps are linked to
     * this Component via the is-located-on relationship.
     * Only used in runtime_only mode.
     */
    bool default_component_enabled{true};

    /**
     * @brief Filter ROS 2 internal nodes from entity discovery
     *
     * When true (default), apps whose ID starts with underscore (_) are
     * filtered out. These are ROS 2 infrastructure nodes like
     * _param_client_node that should not appear as SOVD entities.
     * Applies to both local and peer-discovered apps.
     */
    bool filter_internal_nodes{true};
  } runtime;

  /**
   * @brief Merge pipeline configuration (hybrid mode only)
   */
  struct MergePipelineConfig {
    discovery::GapFillConfig gap_fill;
    /// Per-layer merge policy overrides: layer_name -> (field_group_name -> policy_name)
    /// e.g. {"manifest": {"live_data": "authoritative"}, "runtime": {"identity": "authoritative"}}
    std::map<std::string, std::map<std::string, std::string>> layer_policies;
  } merge_pipeline;
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

  /**
   * @brief Discover functions using pre-discovered apps
   *
   * Avoids redundant ROS 2 graph introspection when apps have already been
   * discovered in the same refresh cycle. Falls back to the no-arg overload
   * for modes that don't support this optimization (manifest-only, hybrid).
   *
   * @param apps Pre-discovered apps (used only in RUNTIME_ONLY mode)
   * @return Vector of discovered Function entities
   */
  std::vector<Function> discover_functions(const std::vector<App> & apps);

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
   * @param provider Pointer to TopicDataProvider (must outlive DiscoveryManager)
   */
  void set_topic_data_provider(TopicDataProvider * provider);

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
   * @brief Add a plugin layer to the merge pipeline
   *
   * Wraps an IntrospectionProvider as a PluginLayer and adds it to
   * the pipeline. Only works in HYBRID mode.
   *
   * @param plugin_name Name of the plugin
   * @param provider Non-owning pointer to IntrospectionProvider
   */
  void add_plugin_layer(const std::string & plugin_name, IntrospectionProvider * provider);

  /**
   * @brief Re-execute the merge pipeline (hybrid mode only)
   *
   * Call after adding plugin layers to trigger a single pipeline refresh.
   */
  void refresh_pipeline();

  /**
   * @brief Get the manifest manager
   * @return Pointer to manifest manager (nullptr if not using manifest)
   */
  discovery::ManifestManager * get_manifest_manager();

  // =========================================================================
  // Status
  // =========================================================================

  /**
   * @brief Check if a host info provider is active
   * @return true if default component is enabled and provider exists
   */
  bool has_host_info_provider() const;

  /**
   * @brief Get the default Component from HostInfoProvider
   * @return Component entity representing the local host, or nullopt if not enabled
   */
  std::optional<Component> get_default_component() const;

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

  /**
   * @brief Get the last merge pipeline report (hybrid mode only)
   * @return MergeReport if in hybrid mode, nullopt otherwise
   */
  std::optional<discovery::MergeReport> get_merge_report() const;

  /**
   * @brief Get the last linking result (hybrid mode only)
   * @return LinkingResult if in hybrid mode, nullopt otherwise
   */
  std::optional<discovery::LinkingResult> get_linking_result() const;

 private:
  /**
   * @brief Create and activate the appropriate strategy
   */
  void create_strategy();

  /**
   * @brief Apply user-configured merge policy overrides to a layer
   */
  template <typename LayerT>
  void apply_layer_policy_overrides(const std::string & layer_name, LayerT & layer);

  rclcpp::Node * node_;
  DiscoveryConfig config_;

  // Host info provider (created when default_component_enabled is true)
  std::unique_ptr<HostInfoProvider> host_info_provider_;

  // Strategies
  std::unique_ptr<discovery::RuntimeDiscoveryStrategy> runtime_strategy_;
  std::unique_ptr<discovery::ManifestManager> manifest_manager_;
  std::unique_ptr<discovery::HybridDiscoveryStrategy> hybrid_strategy_;

  // Active strategy pointer (points to one of the above)
  discovery::DiscoveryStrategy * active_strategy_{nullptr};
};

}  // namespace ros2_medkit_gateway
