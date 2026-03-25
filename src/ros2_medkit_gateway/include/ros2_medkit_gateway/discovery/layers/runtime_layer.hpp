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
#include "ros2_medkit_gateway/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/discovery/runtime_discovery.hpp"

#include <unordered_map>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Discovery layer wrapping RuntimeDiscoveryStrategy
 *
 * Default policies: IDENTITY=FALLBACK, HIERARCHY=FALLBACK, LIVE_DATA=AUTH,
 * STATUS=AUTH, METADATA=ENRICH
 */
class RuntimeLayer : public DiscoveryLayer {
 public:
  explicit RuntimeLayer(RuntimeDiscoveryStrategy * runtime_strategy);

  std::string name() const override {
    return "runtime";
  }
  bool provides_runtime_apps() const override {
    return true;
  }
  LayerOutput discover() override;
  MergePolicy policy_for(FieldGroup group) const override;

  void set_policy(FieldGroup group, MergePolicy policy);
  void set_gap_fill_config(GapFillConfig config);

  /// Number of entities filtered by gap-fill config in last discover()
  size_t filtered_count() const override {
    return last_filtered_count_;
  }

  /// Return unfiltered runtime apps for post-merge linking.
  /// Gap-fill may exclude apps from discover() output, but the linker
  /// needs all runtime apps to bind manifest apps to live nodes.
  std::vector<App> get_linking_apps() const override {
    return linking_apps_;
  }

  /// Direct access to runtime services (for operation/data endpoints, not part of pipeline)
  std::vector<ServiceInfo> discover_services();
  std::vector<ActionInfo> discover_actions();

 private:
  RuntimeDiscoveryStrategy * runtime_strategy_;
  std::unordered_map<FieldGroup, MergePolicy> policies_;
  GapFillConfig gap_fill_config_;
  size_t last_filtered_count_{0};
  std::vector<App> linking_apps_;  ///< Unfiltered runtime apps for linker
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
