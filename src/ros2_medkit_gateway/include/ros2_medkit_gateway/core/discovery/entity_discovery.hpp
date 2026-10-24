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

#include "ros2_medkit_gateway/core/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/core/discovery/linking_result.hpp"
#include "ros2_medkit_gateway/core/discovery/manifest/manifest.hpp"
#include "ros2_medkit_gateway/core/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

#include <optional>
#include <string>

namespace ros2_medkit_gateway {

/// Entity lookups and discovery status the HTTP layer reads when the entity cache has no answer.
class EntityDiscovery {
 public:
  EntityDiscovery() = default;
  virtual ~EntityDiscovery() = default;
  EntityDiscovery(const EntityDiscovery &) = delete;
  EntityDiscovery & operator=(const EntityDiscovery &) = delete;
  EntityDiscovery(EntityDiscovery &&) = delete;
  EntityDiscovery & operator=(EntityDiscovery &&) = delete;

  virtual std::optional<Area> get_area(const std::string & id) = 0;
  virtual std::optional<Component> get_component(const std::string & id) = 0;
  virtual std::optional<App> get_app(const std::string & id) = 0;
  virtual std::optional<Function> get_function(const std::string & id) = 0;

  virtual DiscoveryMode get_mode() const = 0;
  virtual std::string get_strategy_name() const = 0;
  /// Hybrid mode only.
  virtual std::optional<discovery::MergeReport> get_merge_report() const = 0;
  /// Hybrid mode only.
  virtual std::optional<discovery::LinkingResult> get_linking_result() const = 0;
  /// Defaults when no manifest is loaded.
  virtual discovery::ManifestConfig get_manifest_config() const = 0;
};

}  // namespace ros2_medkit_gateway
