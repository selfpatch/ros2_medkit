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

#include <optional>
#include <string>

#include "ros2_medkit_gateway/models/entity_types.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Parsed entity path information from HTTP request URL
 *
 * Contains all extracted information from an entity-related HTTP request path,
 * supporting both top-level entities (apps, components, areas, functions) and
 * nested entities (subareas, subcomponents).
 */
struct EntityPathInfo {
  SovdEntityType type;        ///< APP, COMPONENT, AREA, FUNCTION, or UNKNOWN
  std::string entity_id;      ///< Entity identifier (e.g., "motor_controller")
  std::string resource_path;  ///< Remainder after entity (e.g., "/faults/MOTOR_ERR")
  std::string entity_path;    ///< Full entity path (e.g., "/apps/motor_controller")
  std::string parent_id;      ///< For nested entities: parent entity ID (e.g., "perception" for subarea)
  bool is_nested{false};      ///< True if this is a nested entity (subarea/subcomponent)
};

/**
 * @brief Parse entity path from HTTP request URL
 *
 * Extracts entity type, ID, and resource path from URLs like:
 * - /api/v1/apps/{id}/... -> APP, id, ...
 * - /api/v1/components/{id}/... -> COMPONENT, id, ...
 * - /api/v1/areas/{id}/... -> AREA, id, ...
 * - /api/v1/functions/{id}/... -> FUNCTION, id, ...
 * - /api/v1/areas/{parent}/subareas/{id}/... -> AREA, id, parent
 * - /api/v1/components/{parent}/subcomponents/{id}/... -> COMPONENT, id, parent
 *
 * @param request_path Full request path (e.g., "/api/v1/apps/motor/faults")
 * @return Parsed EntityPathInfo, or std::nullopt if path doesn't match any pattern
 */
std::optional<EntityPathInfo> parse_entity_path(const std::string & request_path);

/**
 * @brief Extract category from bulk-data URL
 *
 * Examples:
 * - "/bulk-data/rosbags/..." -> "rosbags"
 * - "/bulk-data/snapshots/..." -> "snapshots"
 *
 * @param path URL path containing bulk-data segment
 * @return Category string, or empty if not found
 */
std::string extract_bulk_data_category(const std::string & path);

/**
 * @brief Extract bulk-data ID (UUID) from URL
 *
 * Example: "/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000" -> UUID
 *
 * @param path URL path containing bulk-data segment
 * @return Bulk data ID (UUID), or empty if not found
 */
std::string extract_bulk_data_id(const std::string & path);

}  // namespace ros2_medkit_gateway
