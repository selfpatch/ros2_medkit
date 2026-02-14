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

#include "ros2_medkit_gateway/discovery/discovery_strategy.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"
#include "ros2_medkit_gateway/discovery/runtime_discovery.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Hybrid discovery combining manifest and runtime discovery
 *
 * Uses manifest as source of truth for entity IDs and hierarchy while
 * linking to runtime ROS 2 nodes for live data (topics, services, actions).
 *
 * Behavior:
 * - Areas: From manifest (runtime areas not exposed unless orphan policy allows)
 * - Components: From manifest, enriched with runtime data if linked
 * - Apps: From manifest, bound to runtime nodes via RuntimeLinker
 * - Functions: From manifest only
 *
 * The hybrid strategy maintains a RuntimeLinker that binds manifest Apps
 * to actual ROS 2 nodes discovered at runtime.
 */
class HybridDiscoveryStrategy : public DiscoveryStrategy {
 public:
  /**
   * @brief Construct hybrid discovery strategy
   * @param node ROS 2 node for logging
   * @param manifest_manager Manifest manager (must be loaded before use)
   * @param runtime_strategy Runtime discovery strategy for ROS graph introspection
   */
  HybridDiscoveryStrategy(rclcpp::Node * node, ManifestManager * manifest_manager,
                          RuntimeDiscoveryStrategy * runtime_strategy);

  /**
   * @brief Discover areas from manifest
   * @return Areas defined in manifest
   */
  std::vector<Area> discover_areas() override;

  /**
   * @brief Discover components from manifest, linked to runtime
   * @return Components with runtime data if linked
   */
  std::vector<Component> discover_components() override;

  /**
   * @brief Discover apps from manifest, linked to runtime nodes
   * @return Apps with is_online and bound_fqn set
   */
  std::vector<App> discover_apps() override;

  /**
   * @brief Discover functions from manifest
   * @return Functions defined in manifest
   */
  std::vector<Function> discover_functions() override;

  /**
   * @brief Get strategy name
   * @return "hybrid"
   */
  std::string get_name() const override {
    return "hybrid";
  }

  /**
   * @brief Refresh runtime linking
   *
   * Call this after runtime discovery refresh to update app-node bindings.
   * This will re-run the RuntimeLinker with fresh runtime component data.
   */
  void refresh_linking();

  /**
   * @brief Get the last linking result
   * @return Reference to last linking result
   */
  const LinkingResult & get_linking_result() const {
    return linking_result_;
  }

  /**
   * @brief Get orphan nodes from last linking
   * @return Vector of node FQNs not bound to any manifest app
   */
  const std::vector<std::string> & get_orphan_nodes() const {
    return linking_result_.orphan_nodes;
  }

 private:
  /**
   * @brief Perform initial linking on construction
   */
  void perform_linking();

  /**
   * @brief Log message at info level
   */
  void log_info(const std::string & msg) const;

  rclcpp::Node * node_;
  ManifestManager * manifest_manager_;
  RuntimeDiscoveryStrategy * runtime_strategy_;
  RuntimeLinker linker_;
  LinkingResult linking_result_;
  mutable std::mutex mutex_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
