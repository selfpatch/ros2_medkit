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

#include "ros2_medkit_gateway/discovery/discovery_layer.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest.hpp"
#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"
#include "ros2_medkit_gateway/discovery/merge_types.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Result of a merge pipeline execution
 */
struct MergeResult {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
  MergeReport report;
};

/**
 * @brief Orchestrates multiple discovery layers with configurable merge policies
 *
 * Layers are added in priority order (first added = highest priority).
 * Each layer produces entities and declares per-field-group MergePolicy.
 * The pipeline merges entities by ID, resolving conflicts per policy.
 */
class MergePipeline {
 public:
  explicit MergePipeline(rclcpp::Logger logger = rclcpp::get_logger("merge_pipeline"));

  /**
   * @brief Add a discovery layer to the pipeline
   * @param layer Layer to add (priority = insertion order, first = highest)
   */
  void add_layer(std::unique_ptr<DiscoveryLayer> layer);

  /**
   * @brief Execute all layers and merge results
   * @return Merged entities with diagnostics report
   */
  MergeResult execute();

  /**
   * @brief Get the last merge report
   */
  const MergeReport & get_last_report() const {
    return last_report_;
  }

  /**
   * @brief Set RuntimeLinker for post-merge app-to-node binding
   * @param linker RuntimeLinker instance
   * @param config ManifestConfig needed by RuntimeLinker::link() for unmanifested node policy
   */
  void set_linker(std::unique_ptr<RuntimeLinker> linker, const ManifestConfig & config);

  /**
   * @brief Get the last linking result
   */
  const LinkingResult & get_linking_result() const {
    return linking_result_;
  }

 private:
  /// Merge a vector of entities from multiple layers by ID
  template <typename Entity>
  std::vector<Entity> merge_entities(const std::vector<std::pair<size_t, std::vector<Entity>>> & layer_entities,
                                     MergeReport & report);

  rclcpp::Logger logger_;
  std::vector<std::unique_ptr<DiscoveryLayer>> layers_;
  MergeReport last_report_;
  std::unique_ptr<RuntimeLinker> linker_;
  ManifestConfig manifest_config_;
  LinkingResult linking_result_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
