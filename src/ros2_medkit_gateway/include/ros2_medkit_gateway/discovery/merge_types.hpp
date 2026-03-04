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

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Merge precedence policy for a discovery layer's field group
 */
enum class MergePolicy {
  AUTHORITATIVE,  ///< This layer's value wins over lower-priority layers
  ENRICHMENT,     ///< Fill empty fields only, don't override existing values
  FALLBACK        ///< Use only if no other layer provides the value
};

/**
 * @brief Logical groupings of entity fields with the same merge behavior
 */
enum class FieldGroup {
  IDENTITY,   ///< id, name, translation_id, description, tags
  HIERARCHY,  ///< area, component_id, parent_*, depends_on, hosts
  LIVE_DATA,  ///< topics, services, actions
  STATUS,     ///< is_online, bound_fqn
  METADATA    ///< source, x-medkit extensions, custom fields
};

/**
 * @brief Record of a merge conflict between two layers
 */
struct MergeConflict {
  std::string entity_id;
  FieldGroup field_group;
  std::string winning_layer;
  std::string losing_layer;
};

/**
 * @brief Diagnostics report from a merge pipeline execution
 */
struct MergeReport {
  std::vector<std::string> layers;
  std::vector<MergeConflict> conflicts;
  std::unordered_map<std::string, std::string> entity_source;  ///< entity id -> primary layer name
  size_t total_entities{0};
  size_t enriched_count{0};
  size_t conflict_count{0};
  size_t id_collision_count{0};

  nlohmann::json to_json() const {
    return {{"layers", layers},
            {"total_entities", total_entities},
            {"enriched_count", enriched_count},
            {"conflict_count", conflict_count},
            {"id_collisions", id_collision_count}};
  }
};

/**
 * @brief Controls what heuristic (runtime) discovery is allowed to create
 *
 * When manifest is present, runtime entities fill gaps. This struct
 * controls which entity types and namespaces are eligible for gap-fill.
 */
struct GapFillConfig {
  bool allow_heuristic_areas{true};
  bool allow_heuristic_components{true};
  bool allow_heuristic_apps{true};
  bool allow_heuristic_functions{false};
  std::vector<std::string> namespace_whitelist;
  std::vector<std::string> namespace_blacklist;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway

// Required: C++17 does not provide std::hash for enum class types
template <>
struct std::hash<ros2_medkit_gateway::discovery::FieldGroup> {
  size_t operator()(ros2_medkit_gateway::discovery::FieldGroup fg) const noexcept {
    return std::hash<int>{}(static_cast<int>(fg));
  }
};
