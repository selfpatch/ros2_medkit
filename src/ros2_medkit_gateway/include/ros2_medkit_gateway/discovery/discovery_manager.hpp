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

#ifndef ROS2_MEDKIT_GATEWAY__DISCOVERY__DISCOVERY_MANAGER_HPP_
#define ROS2_MEDKIT_GATEWAY__DISCOVERY__DISCOVERY_MANAGER_HPP_

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
 * @brief Orchestrates entity discovery using pluggable strategies
 *
 * This class delegates discovery to strategy implementations.
 * Currently uses RuntimeDiscoveryStrategy; will support ManifestDiscoveryStrategy
 * and HybridDiscoveryStrategy in the future.
 *
 * The DiscoveryManager provides a unified interface for discovering:
 * - Areas: Logical groupings (ROS 2 namespaces)
 * - Components: Software/hardware units (ROS 2 nodes)
 * - Apps: Software applications (manifest-defined, stub for now)
 * - Functions: Functional groupings (manifest-defined, stub for now)
 *
 * @see discovery::RuntimeDiscoveryStrategy
 */
class DiscoveryManager {
 public:
  /**
   * @brief Construct the discovery manager
   * @param node ROS 2 node for graph introspection (must outlive this manager)
   */
  explicit DiscoveryManager(rclcpp::Node * node);

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

  /**
   * @brief Get the current discovery strategy name
   * @return Strategy name (e.g., "runtime", "manifest", "hybrid")
   */
  std::string get_strategy_name() const;

 private:
  rclcpp::Node * node_;
  std::unique_ptr<discovery::RuntimeDiscoveryStrategy> runtime_strategy_;
  // Future: std::unique_ptr<discovery::ManifestDiscoveryStrategy> manifest_strategy_;
};

}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__DISCOVERY__DISCOVERY_MANAGER_HPP_
