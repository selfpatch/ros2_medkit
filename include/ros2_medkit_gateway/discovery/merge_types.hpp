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
#include <optional>
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

inline const char * field_group_to_string(FieldGroup fg) {
  switch (fg) {
    case FieldGroup::IDENTITY:
      return "IDENTITY";
    case FieldGroup::HIERARCHY:
      return "HIERARCHY";
    case FieldGroup::LIVE_DATA:
      return "LIVE_DATA";
    case FieldGroup::STATUS:
      return "STATUS";
    case FieldGroup::METADATA:
      return "METADATA";
  }
  return "UNKNOWN";
}

/// Parse a FieldGroup from its lowercase string form (e.g. "identity", "live_data")
inline std::optional<FieldGroup> field_group_from_string(const std::string & s) {
  if (s == "identity") {
    return FieldGroup::IDENTITY;
  }
  if (s == "hierarchy") {
    return FieldGroup::HIERARCHY;
  }
  if (s == "live_data") {
    return FieldGroup::LIVE_DATA;
  }
  if (s == "status") {
    return FieldGroup::STATUS;
  }
  if (s == "metadata") {
    return FieldGroup::METADATA;
  }
  return std::nullopt;
}

/// Parse a MergePolicy from its lowercase string form (e.g. "authoritative", "enrichment", "fallback")
inline std::optional<MergePolicy> merge_policy_from_string(const std::string & s) {
  if (s == "authoritative") {
    return MergePolicy::AUTHORITATIVE;
  }
  if (s == "enrichment") {
    return MergePolicy::ENRICHMENT;
  }
  if (s == "fallback") {
    return MergePolicy::FALLBACK;
  }
  return std::nullopt;
}

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
  size_t filtered_by_gap_fill{0};

  nlohmann::json to_json() const {
    nlohmann::json conflict_list = nlohmann::json::array();
    for (const auto & c : conflicts) {
      conflict_list.push_back({{"entity_id", c.entity_id},
                               {"field_group", field_group_to_string(c.field_group)},
                               {"winning_layer", c.winning_layer},
                               {"losing_layer", c.losing_layer}});
    }
    return {{"layers", layers},
            {"total_entities", total_entities},
            {"enriched_count", enriched_count},
            {"conflict_count", conflict_count},
            {"conflicts", conflict_list},
            {"id_collisions", id_collision_count},
            {"filtered_by_gap_fill", filtered_by_gap_fill}};
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
