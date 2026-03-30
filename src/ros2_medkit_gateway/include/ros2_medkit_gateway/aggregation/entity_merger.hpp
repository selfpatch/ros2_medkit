// Copyright 2026 Selfpatch GmbH
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

#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Merges entities from a remote peer gateway with local entities.
 *
 * Type-aware merge logic:
 * - Area: merge by ID (combine relations, no duplication)
 * - Function: merge by ID (combine hosts lists)
 * - Component: prefix remote ID with "peername__" on collision
 * - App: prefix remote ID with "peername__" on collision
 *
 * After merging, a routing table maps remote entity IDs (possibly prefixed)
 * to the peer name, enabling request forwarding.
 */
class EntityMerger {
 public:
  explicit EntityMerger(const std::string & peer_name);

  /**
   * @brief Merge local and remote Areas by ID.
   *
   * Same area ID from both local and remote yields one Area entity.
   * Remote-only areas are added with source tagged as "peer:<name>".
   */
  std::vector<Area> merge_areas(const std::vector<Area> & local, const std::vector<Area> & remote);

  /**
   * @brief Merge local and remote Functions by ID, combining hosts lists.
   *
   * Same function ID from both local and remote yields one Function with
   * the union of hosts from both sides.
   */
  std::vector<Function> merge_functions(const std::vector<Function> & local, const std::vector<Function> & remote);

  /**
   * @brief Merge local and remote Components with prefix on collision.
   *
   * If a remote Component has the same ID as a local one, the remote entity
   * gets its ID prefixed with "peername__".
   */
  std::vector<Component> merge_components(const std::vector<Component> & local, const std::vector<Component> & remote);

  /**
   * @brief Merge local and remote Apps with prefix on collision.
   *
   * If a remote App has the same ID as a local one, the remote entity
   * gets its ID prefixed with "peername__".
   */
  std::vector<App> merge_apps(const std::vector<App> & local, const std::vector<App> & remote);

  /**
   * @brief Get the routing table: entity_id -> peer_name for remote entities.
   *
   * Prefixed entities map their prefixed ID. Merged entities (Area/Function)
   * do not appear in the routing table. Non-colliding remote Components/Apps
   * still appear because they need request forwarding.
   */
  const std::unordered_map<std::string, std::string> & get_routing_table() const;

  /// Separator used for prefixing remote entity IDs on collision.
  static constexpr const char * SEPARATOR = "__";

 private:
  std::string peer_name_;
  std::unordered_map<std::string, std::string> routing_table_;

  /// Create a prefixed ID: "peername__original_id"
  std::string prefix_id(const std::string & id) const;

  /// Create the source tag for remote entities: "peer:<name>"
  std::string peer_source() const;
};

}  // namespace ros2_medkit_gateway
