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

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Merges entities from a remote peer gateway with local entities.
 *
 * Type-aware merge logic:
 * - Area: merge by ID (combine tags/description, no duplication)
 * - Function: merge by ID (combine hosts lists)
 * - Component: merge by ID (combine tags/description, no duplication)
 * - App: prefix remote ID with "peername__" on collision
 *
 * A remote entity whose ID a local entity of another type holds is prefixed
 * the same way, whatever its own type: the routing table is keyed by ID alone,
 * so routing that ID to the peer would hand the local entity's requests to it.
 *
 * After merging, a routing table maps remote entity IDs (possibly prefixed)
 * to the peer name, enabling request forwarding. Merged Areas and Functions do
 * NOT appear in the routing table; a merged Component does (see merge_components).
 */
class EntityMerger {
 public:
  /// IDs the merge target holds before this peer is merged into it, one set per
  /// entity type: the local gateway's own entities and every peer merged before.
  struct LocalIds {
    std::unordered_set<std::string> areas;
    std::unordered_set<std::string> components;
    std::unordered_set<std::string> apps;
    std::unordered_set<std::string> functions;
  };

  /// Original remote ID -> the prefixed ID it is served under, per entity type.
  struct Renames {
    std::unordered_map<std::string, std::string> areas;
    std::unordered_map<std::string, std::string> components;
    std::unordered_map<std::string, std::string> apps;
    std::unordered_map<std::string, std::string> functions;
  };

  /// A merger that compares each remote entity only with local entities of its
  /// own type, which is all a caller that merges one collection needs.
  explicit EntityMerger(const std::string & peer_name);

  /// A merger that also keeps every remote entity out of an ID a local entity
  /// of another type holds. The merge pass over a whole peer uses this one.
  EntityMerger(const std::string & peer_name, LocalIds local_ids);

  /// The IDs of the given entities, ready to pass to the constructor.
  static LocalIds collect_ids(const std::vector<Area> & areas, const std::vector<Component> & components,
                              const std::vector<App> & apps, const std::vector<Function> & functions);

  /**
   * @brief Merge local and remote Areas by ID.
   *
   * Same area ID from both local and remote yields one Area entity.
   * Remote-only areas are added with source tagged as "peer:<name>".
   * A remote subarea whose parent was prefixed names the prefixed parent.
   */
  std::vector<Area> merge_areas(const std::vector<Area> & local, const std::vector<Area> & remote);

  /**
   * @brief Merge local and remote Functions by ID, combining hosts lists.
   *
   * Same function ID from both local and remote yields one Function with
   * the union of hosts from both sides. A remote host that names an App or
   * Component this merger prefixed names the prefixed ID, so call this after
   * merge_apps and merge_components.
   */
  std::vector<Function> merge_functions(const std::vector<Function> & local, const std::vector<Function> & remote);

  /**
   * @brief Merge local and remote Components by ID.
   *
   * Same component ID from both local and remote yields one Component entity
   * with merged tags and description, and the remote's area when the local one
   * names none. The merged Component routes to the peer. A local entity of another type
   * holding the ID would be routed with it, so in that case the remote
   * Component is prefixed instead of merged. Remote-only components are added
   * with source tagged as "peer:<name>". References to an Area merge_areas
   * prefixed, or to a Component this call prefixed, name the prefixed ID.
   */
  std::vector<Component> merge_components(const std::vector<Component> & local, const std::vector<Component> & remote);

  /**
   * @brief Merge local and remote Apps with prefix on collision.
   *
   * If a remote App has the same ID as a local one, the remote entity
   * gets its ID prefixed with "peername__". A remote App located on a
   * Component merge_components prefixed names the prefixed Component.
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

  /// The remote IDs this merger prefixed, original -> prefixed.
  const Renames & get_renames() const;

  /// Remote entities left out because their prefixed ID is held locally too,
  /// as "<Type> '<id>'". Serving one would route an ID a local entity holds.
  const std::vector<std::string> & get_dropped() const;

  /// Separator used for prefixing remote entity IDs on collision.
  static constexpr const char * SEPARATOR = "__";

 private:
  enum class Kind { kArea, kComponent, kApp, kFunction };

  std::string peer_name_;
  LocalIds local_ids_;
  std::unordered_map<std::string, std::string> routing_table_;
  Renames renames_;
  std::vector<std::string> dropped_;

  /// Create a prefixed ID: "peername__original_id"
  std::string prefix_id(const std::string & id) const;

  /// Create the source tag for remote entities: "peer:<name>"
  std::string peer_source() const;

  /// True when a local entity of a type other than `kind` holds `id`.
  bool held_by_other_type(const std::string & id, Kind kind) const;

  /// True when a local entity of any type holds `id`.
  bool held_by_any_type(const std::string & id) const;
};

}  // namespace ros2_medkit_gateway
