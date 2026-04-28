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

#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Snapshot of current discovery state, passed to introspection providers
 */
struct IntrospectionInput {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
};

/**
 * @brief New entity definitions an introspection provider can introduce
 */
struct NewEntities {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
};

/**
 * @brief Result returned by IntrospectionProvider::introspect()
 */
struct IntrospectionResult {
  /// Per-entity metadata for plugin-internal use. Key = entity_id.
  /// Plugins serve this data as SOVD vendor extension resources
  /// via get_routes() and register_capability().
  std::unordered_map<std::string, nlohmann::json> metadata;

  /// New entities discovered by this provider
  NewEntities new_entities;
};

/**
 * @brief Provider interface for platform-specific introspection
 *
 * Implementations enrich discovered entities with runtime metadata
 * and can discover new entities from non-ROS sources.
 *
 * @see GatewayPlugin for the base class
 * @see PluginManager for orchestration
 */
class IntrospectionProvider {
 public:
  virtual ~IntrospectionProvider() = default;

  /**
   * @brief Core introspection method
   *
   * Called during each discovery cycle by the merge pipeline.
   * Input contains entities from all higher-priority layers (manifest + runtime).
   *
   * @param input Snapshot of entities discovered by previous layers
   * @return Metadata enrichments and new entities
   */
  virtual IntrospectionResult introspect(const IntrospectionInput & input) = 0;
};

}  // namespace ros2_medkit_gateway
