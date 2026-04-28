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

#include "ros2_medkit_gateway/core/discovery/discovery_layer.hpp"
#include "ros2_medkit_gateway/core/discovery/discovery_strategy.hpp"
#include "ros2_medkit_gateway/discovery/merge_pipeline.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Hybrid discovery using a MergePipeline
 *
 * Thin wrapper around MergePipeline that caches the merged result
 * and exposes it through the DiscoveryStrategy interface.
 * The pipeline orchestrates ManifestLayer, RuntimeLayer, and any
 * PluginLayers with per-field-group merge policies.
 */
class HybridDiscoveryStrategy : public DiscoveryStrategy {
 public:
  /**
   * @brief Construct hybrid discovery strategy
   * @param node ROS 2 node for logging
   * @param pipeline Pre-configured merge pipeline
   */
  HybridDiscoveryStrategy(rclcpp::Node * node, MergePipeline pipeline);

  std::vector<Area> discover_areas() override;
  std::vector<Component> discover_components() override;
  std::vector<App> discover_apps() override;
  std::vector<Function> discover_functions() override;

  std::string get_name() const override {
    return "hybrid";
  }

  /**
   * @brief Re-execute the pipeline and cache the result
   */
  void refresh();

  /**
   * @brief Get the last merge report (returned by value for thread safety)
   */
  MergeReport get_merge_report() const;

  /**
   * @brief Get the last linking result (returned by value for thread safety)
   */
  LinkingResult get_linking_result() const;

  /**
   * @brief Get orphan nodes from last linking
   */
  std::vector<std::string> get_orphan_nodes() const;

  /**
   * @brief Add a discovery layer to the pipeline.
   * @warning Construction-time only. Must be called before the first refresh()
   * (i.e., before EntityCache timer starts). Calling concurrently with refresh()
   * is a data race.
   */
  void add_layer(std::unique_ptr<DiscoveryLayer> layer);

 private:
  rclcpp::Node * node_;
  MergePipeline pipeline_;
  MergeResult cached_result_;
  mutable std::mutex mutex_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
