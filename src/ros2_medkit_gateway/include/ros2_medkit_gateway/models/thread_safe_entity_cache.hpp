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

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"

#include <chrono>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief Reference to an entity in the cache
 */
struct EntityRef {
  SovdEntityType type{SovdEntityType::UNKNOWN};
  size_t index{0};
};

/**
 * @brief Aggregated operations result from entity hierarchy
 */
struct AggregatedOperations {
  std::vector<ServiceInfo> services;
  std::vector<ActionInfo> actions;
  std::vector<std::string> source_ids;  ///< Entity IDs that contributed
  std::string aggregation_level;        ///< "app" | "component" | "area" | "function"
  bool is_aggregated{false};            ///< true if collected from sub-entities

  bool empty() const {
    return services.empty() && actions.empty();
  }
  size_t total_count() const {
    return services.size() + actions.size();
  }
};

/**
 * @brief Cache statistics
 */
struct EntityCacheStats {
  size_t area_count{0};
  size_t component_count{0};
  size_t app_count{0};
  size_t function_count{0};
  size_t total_operations{0};
  std::chrono::system_clock::time_point last_update;
};

/**
 * @brief Thread-safe, index-optimized cache for SOVD entities
 *
 * Design principles:
 * 1. Primary storage in vectors (cache-friendly, predictable memory)
 * 2. Hash indexes for O(1) lookup by ID
 * 3. Relationship indexes for O(1) aggregation queries
 * 4. Reader-writer lock for concurrent access
 * 5. Batch updates to minimize lock contention
 *
 * Thread Safety:
 * - Multiple readers can access concurrently (shared lock)
 * - Writers get exclusive access (unique lock)
 * - Readers never block each other
 * - Writers block all readers and other writers
 *
 * Usage:
 * - Discovery thread calls update_*() methods (writer)
 * - HTTP handlers call get_*() methods (reader)
 */
class ThreadSafeEntityCache {
 public:
  ThreadSafeEntityCache() = default;

  // =========================================================================
  // Writer methods (exclusive lock) - called by discovery thread
  // =========================================================================

  /**
   * @brief Atomic batch update of all entities
   *
   * This is the preferred update method as it:
   * 1. Minimizes lock hold time
   * 2. Ensures readers never see partial state
   * 3. Rebuilds all indexes in one pass
   *
   * @param areas All discovered areas
   * @param components All discovered components
   * @param apps All discovered apps
   * @param functions All discovered functions
   */
  void update_all(std::vector<Area> areas, std::vector<Component> components, std::vector<App> apps,
                  std::vector<Function> functions);

  /**
   * @brief Incremental update for single entity type
   *
   * Use sparingly - prefer update_all() for full refresh.
   * Each call acquires exclusive lock and rebuilds relevant indexes.
   */
  void update_areas(std::vector<Area> areas);
  void update_components(std::vector<Component> components);
  void update_apps(std::vector<App> apps);
  void update_functions(std::vector<Function> functions);

  // =========================================================================
  // Reader methods (shared lock) - called by HTTP handlers
  // =========================================================================

  // --- List all entities (returns copy) ---
  std::vector<Area> get_areas() const;
  std::vector<Component> get_components() const;
  std::vector<App> get_apps() const;
  std::vector<Function> get_functions() const;

  // --- Get single entity by ID (O(1) lookup) ---
  std::optional<Area> get_area(const std::string & id) const;
  std::optional<Component> get_component(const std::string & id) const;
  std::optional<App> get_app(const std::string & id) const;
  std::optional<Function> get_function(const std::string & id) const;

  // --- Check existence (O(1)) ---
  bool has_area(const std::string & id) const;
  bool has_component(const std::string & id) const;
  bool has_app(const std::string & id) const;
  bool has_function(const std::string & id) const;

  // --- Resolve any entity by ID ---
  std::optional<EntityRef> find_entity(const std::string & id) const;
  SovdEntityType get_entity_type(const std::string & id) const;

  // =========================================================================
  // Relationship queries (O(1) via indexes)
  // =========================================================================

  /**
   * @brief Get Apps hosted on a Component
   * @return Vector of App IDs (empty if component not found)
   */
  std::vector<std::string> get_apps_for_component(const std::string & component_id) const;

  /**
   * @brief Get Components in an Area
   * @return Vector of Component IDs (empty if area not found)
   */
  std::vector<std::string> get_components_for_area(const std::string & area_id) const;

  /**
   * @brief Get Apps implementing a Function
   * @return Vector of App IDs (empty if function not found)
   */
  std::vector<std::string> get_apps_for_function(const std::string & function_id) const;

  /**
   * @brief Get subareas of an Area
   * @return Vector of Area IDs (empty if area not found or no subareas)
   */
  std::vector<std::string> get_subareas(const std::string & area_id) const;

  // =========================================================================
  // Aggregation methods (uses relationship indexes)
  // =========================================================================

  /**
   * @brief Aggregate operations for an App (no aggregation, returns own ops)
   */
  AggregatedOperations get_app_operations(const std::string & app_id) const;

  /**
   * @brief Aggregate operations for a Component
   *
   * Returns: Component's own operations + all operations from hosted Apps.
   * Deduplicates by operation full_path.
   */
  AggregatedOperations get_component_operations(const std::string & component_id) const;

  /**
   * @brief Aggregate operations for an Area (x-medkit extension)
   *
   * Returns: All operations from all Components in the Area.
   * Recursive through Component→App hierarchy.
   *
   * Note: This is a ros2_medkit extension. SOVD spec doesn't allow
   * Area to have resource collections.
   */
  AggregatedOperations get_area_operations(const std::string & area_id) const;

  /**
   * @brief Aggregate operations for a Function
   *
   * Returns: All operations from all Apps implementing this Function.
   */
  AggregatedOperations get_function_operations(const std::string & function_id) const;

  // =========================================================================
  // Operation lookup (O(1) via operation index)
  // =========================================================================

  /**
   * @brief Find which entity owns an operation by full path
   * @param operation_path e.g., "/nav2/navigate_to_pose"
   * @return EntityRef if found
   */
  std::optional<EntityRef> find_operation_owner(const std::string & operation_path) const;

  // =========================================================================
  // Diagnostics
  // =========================================================================

  /**
   * @brief Get cache statistics
   */
  EntityCacheStats get_stats() const;

  /**
   * @brief Validate internal consistency
   *
   * Checks:
   * 1. All indexes point to valid vector entries
   * 2. Relationship indexes are consistent
   * 3. No duplicate IDs within entity type
   *
   * @return Empty string if valid, error message otherwise
   */
  std::string validate() const;

  /**
   * @brief Get last update timestamp
   */
  std::chrono::system_clock::time_point get_last_update() const;

 private:
  mutable std::shared_mutex mutex_;

  // Primary storage
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
  std::vector<Function> functions_;

  // Timestamp
  std::chrono::system_clock::time_point last_update_;

  // Primary indexes (ID → vector index)
  std::unordered_map<std::string, size_t> area_index_;
  std::unordered_map<std::string, size_t> component_index_;
  std::unordered_map<std::string, size_t> app_index_;
  std::unordered_map<std::string, size_t> function_index_;

  // Relationship indexes (parent ID → child vector indexes)
  std::unordered_map<std::string, std::vector<size_t>> component_to_apps_;
  std::unordered_map<std::string, std::vector<size_t>> area_to_components_;
  std::unordered_map<std::string, std::vector<size_t>> area_to_subareas_;
  std::unordered_map<std::string, std::vector<size_t>> function_to_apps_;

  // Operation index (operation full_path → owning entity)
  std::unordered_map<std::string, EntityRef> operation_index_;

  // Internal helpers (called under lock)
  void rebuild_all_indexes();
  void rebuild_area_index();
  void rebuild_component_index();
  void rebuild_app_index();
  void rebuild_function_index();
  void rebuild_relationship_indexes();
  void rebuild_operation_index();

  // Aggregation helpers (called under shared lock)
  void collect_operations_from_apps(const std::vector<size_t> & app_indexes,
                                    std::unordered_set<std::string> & seen_paths, AggregatedOperations & result) const;
  void collect_operations_from_component(size_t comp_index, std::unordered_set<std::string> & seen_paths,
                                         AggregatedOperations & result) const;
};

}  // namespace ros2_medkit_gateway
