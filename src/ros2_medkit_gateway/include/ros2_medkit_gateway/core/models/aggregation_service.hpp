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

#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief Service for aggregating resources across entity hierarchy
 *
 * Implements x-medkit aggregation extension while maintaining
 * SOVD compliance through explicit metadata.
 *
 * Aggregation rules:
 * - APP: No aggregation (leaf entity) - returns own resources
 * - COMPONENT: Aggregates from hosted Apps
 * - AREA: Aggregates from all Components in area (x-medkit extension)
 * - FUNCTION: Aggregates from all Apps implementing function
 *
 * All aggregated responses include x-medkit metadata:
 * {
 *   "x-medkit": {
 *     "aggregated": true,
 *     "aggregation_sources": ["entity_id_1", "entity_id_2"],
 *     "aggregation_level": "component" | "area" | "function"
 *   }
 * }
 */
class AggregationService {
 public:
  /**
   * @brief Construct aggregation service
   * @param cache Pointer to thread-safe entity cache (must outlive this service)
   */
  explicit AggregationService(const ThreadSafeEntityCache * cache);

  /**
   * @brief Get aggregated operations for any entity type
   *
   * Automatically determines aggregation behavior based on entity type.
   *
   * @param type Entity type
   * @param entity_id Entity identifier
   * @return Aggregated operations with source tracking
   */
  AggregatedOperations get_operations(SovdEntityType type, const std::string & entity_id) const;

  /**
   * @brief Get operations for entity by ID (auto-detects type)
   *
   * @param entity_id Entity identifier
   * @return Aggregated operations with source tracking
   */
  AggregatedOperations get_operations_by_id(const std::string & entity_id) const;

  /**
   * @brief Build x-medkit extension JSON for aggregated response
   *
   * Creates the x-medkit metadata object:
   * {
   *   "aggregated": true/false,
   *   "aggregation_sources": [...],
   *   "aggregation_level": "app" | "component" | "area" | "function"
   * }
   *
   * @param result Aggregation result
   * @return JSON object for x-medkit field
   */
  static nlohmann::json build_x_medkit(const AggregatedOperations & result);

  /**
   * @brief Check if entity type supports operations collection
   *
   * @param type Entity type
   * @return true if operations collection is supported
   */
  static bool supports_operations(SovdEntityType type);

  /**
   * @brief Check if operations should be aggregated for entity type
   *
   * Returns true for COMPONENT, AREA, and FUNCTION.
   * Returns false for APP (leaf entity).
   *
   * @param type Entity type
   * @return true if aggregation applies
   */
  static bool should_aggregate(SovdEntityType type);

  /**
   * @brief Get child app IDs for an aggregated entity
   *
   * Resolves the leaf App entity IDs that contribute resources to
   * an aggregated entity (Function, Area, or Component).
   *
   * - FUNCTION: returns app IDs from the function's hosts list
   * - AREA: returns app IDs from all components in the area
   * - COMPONENT: returns app IDs hosted by the component
   * - APP: returns the app's own ID
   *
   * @param type Entity type
   * @param entity_id Entity identifier
   * @return Vector of child app IDs
   */
  std::vector<std::string> get_child_app_ids(SovdEntityType type, const std::string & entity_id) const;

  /**
   * @brief Build x-medkit extension JSON for any aggregated collection response
   *
   * Creates a standardized x-medkit metadata object with:
   * - aggregated: true/false
   * - aggregation_sources: [...] (child app IDs)
   * - aggregation_level: "app" | "component" | "area" | "function"
   *
   * @param type Entity type (determines aggregation_level)
   * @param entity_id Entity identifier (used to resolve sources)
   * @return JSON object for x-medkit field
   */
  nlohmann::json build_collection_x_medkit(SovdEntityType type, const std::string & entity_id) const;

 private:
  const ThreadSafeEntityCache * cache_;
};

}  // namespace ros2_medkit_gateway
