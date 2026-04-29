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

#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/common.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_serialization/type_introspection.hpp"

#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing IntrospectionProvider via ROS 2 graph queries.
 *
 * Owns the introspection of the live ROS 2 graph (node names, services, actions,
 * topics) that previously lived inside RuntimeDiscoveryStrategy. Exposes the
 * `IntrospectionProvider::introspect()` contract used by the merge pipeline so
 * built-in graph queries are treated identically to plugin-provided introspection.
 *
 * In addition to the IntrospectionProvider contract, this class exposes the
 * service/action discovery and topic-map management methods that the gateway's
 * data, operation, and discovery layers consume directly (independent of the
 * merge pipeline).
 *
 * Threading: the underlying rclcpp graph queries are not thread-safe across
 * concurrent introspection. Callers (DiscoveryManager / RuntimeLayer) serialize
 * access via the gateway's refresh lock.
 */
class Ros2RuntimeIntrospection : public IntrospectionProvider {
 public:
  /**
   * @brief Per-introspection configuration knobs.
   */
  struct RuntimeConfig {
    /// Map ROS 2 namespaces to Function entities when true.
    bool create_functions_from_namespaces{true};
  };

  /**
   * @param node Non-owning ROS node used for graph queries.
   */
  explicit Ros2RuntimeIntrospection(rclcpp::Node * node);

  ~Ros2RuntimeIntrospection() override = default;

  Ros2RuntimeIntrospection(const Ros2RuntimeIntrospection &) = delete;
  Ros2RuntimeIntrospection & operator=(const Ros2RuntimeIntrospection &) = delete;
  Ros2RuntimeIntrospection(Ros2RuntimeIntrospection &&) = delete;
  Ros2RuntimeIntrospection & operator=(Ros2RuntimeIntrospection &&) = delete;

  /// Update runtime introspection configuration.
  void set_config(const RuntimeConfig & config);

  // ---------------------------------------------------------------------------
  // IntrospectionProvider
  // ---------------------------------------------------------------------------

  /**
   * @brief Build new entities from the current ROS 2 graph.
   *
   * Returns Apps (one per node) and Functions (one per namespace) as
   * `new_entities`. Areas and Components are intentionally never produced by
   * runtime introspection - they come from the manifest layer or the
   * HostInfoProvider.
   */
  IntrospectionResult introspect(const IntrospectionInput & input) override;

  // ---------------------------------------------------------------------------
  // Direct access (used by RuntimeLayer for gap-fill linking and by managers
  // that need service/action/topic introspection independent of the pipeline).
  // ---------------------------------------------------------------------------

  /// Discover the live nodes as Apps. Always queries the ROS 2 graph; do not
  /// call from hot paths.
  std::vector<App> discover_apps();

  /// Group nodes by namespace into Function entities (no graph query).
  std::vector<Function> discover_functions(const std::vector<App> & apps);

  /// Group nodes by namespace into Function entities (queries the graph).
  std::vector<Function> discover_functions();

  /// Discover all services in the ROS 2 graph with schema enrichment.
  std::vector<ServiceInfo> discover_services();

  /// Discover all actions (synthesised from `/_action/send_goal` services).
  std::vector<ActionInfo> discover_actions();

  /// Find a service by component namespace and operation name (cache lookup).
  std::optional<ServiceInfo> find_service(const std::string & component_ns, const std::string & operation_name) const;

  /// Find an action by component namespace and operation name (cache lookup).
  std::optional<ActionInfo> find_action(const std::string & component_ns, const std::string & operation_name) const;

  /// Provide the topic data provider used to enrich Apps with publish/subscribe
  /// topics. Pointer must outlive this provider.
  void set_topic_data_provider(TopicDataProvider * provider);

  /// Provide the type introspection used to enrich service/action schemas.
  /// Pointer must outlive this provider.
  void set_type_introspection(ros2_medkit_serialization::TypeIntrospection * introspection);

  /// Rebuild the cached `<node_fqn> -> ComponentTopics` map from the topic
  /// data provider. Called by the gateway's refresh path.
  void refresh_topic_map();

  /// True once the topic map has been built at least once.
  bool is_topic_map_ready() const;

 private:
  /// Extract area ID from namespace ("/powertrain/engine" -> "powertrain").
  static std::string extract_area_from_namespace(const std::string & ns);

  /// Extract the last segment from a path ("/a/b/c" -> "c").
  static std::string extract_name_from_path(const std::string & path);

  /// True if the path lives directly under the given namespace.
  bool path_belongs_to_namespace(const std::string & path, const std::string & ns) const;

  /// True if a service path looks like internal ROS 2 infrastructure
  /// (parameter services, action internals, etc.).
  static bool is_internal_service(const std::string & service_path);

  rclcpp::Node * node_;
  RuntimeConfig config_;
  TopicDataProvider * topic_data_provider_{nullptr};
  ros2_medkit_serialization::TypeIntrospection * type_introspection_{nullptr};

  std::vector<ServiceInfo> cached_services_;
  std::vector<ActionInfo> cached_actions_;

  std::map<std::string, ComponentTopics> cached_topic_map_;
  bool topic_map_ready_{false};
};

}  // namespace ros2_medkit_gateway::ros2
