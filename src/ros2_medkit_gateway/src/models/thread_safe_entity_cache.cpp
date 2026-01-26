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

#include "ros2_medkit_gateway/models/thread_safe_entity_cache.hpp"

#include <algorithm>
#include <mutex>
#include <sstream>

namespace ros2_medkit_gateway {

// ============================================================================
// Writer methods (exclusive lock)
// ============================================================================

void ThreadSafeEntityCache::update_all(std::vector<Area> areas, std::vector<Component> components,
                                       std::vector<App> apps, std::vector<Function> functions) {
  std::unique_lock lock(mutex_);

  areas_ = std::move(areas);
  components_ = std::move(components);
  apps_ = std::move(apps);
  functions_ = std::move(functions);
  last_update_ = std::chrono::system_clock::now();

  rebuild_all_indexes();
}

void ThreadSafeEntityCache::update_areas(std::vector<Area> areas) {
  std::unique_lock lock(mutex_);
  areas_ = std::move(areas);
  last_update_ = std::chrono::system_clock::now();
  rebuild_area_index();
  rebuild_relationship_indexes();
}

void ThreadSafeEntityCache::update_components(std::vector<Component> components) {
  std::unique_lock lock(mutex_);
  components_ = std::move(components);
  last_update_ = std::chrono::system_clock::now();
  rebuild_component_index();
  rebuild_relationship_indexes();
  rebuild_operation_index();
}

void ThreadSafeEntityCache::update_apps(std::vector<App> apps) {
  std::unique_lock lock(mutex_);
  apps_ = std::move(apps);
  last_update_ = std::chrono::system_clock::now();
  rebuild_app_index();
  rebuild_relationship_indexes();
  rebuild_operation_index();
}

void ThreadSafeEntityCache::update_functions(std::vector<Function> functions) {
  std::unique_lock lock(mutex_);
  functions_ = std::move(functions);
  last_update_ = std::chrono::system_clock::now();
  rebuild_function_index();
  rebuild_relationship_indexes();
}

// ============================================================================
// Reader methods - List all
// ============================================================================

std::vector<Area> ThreadSafeEntityCache::get_areas() const {
  std::shared_lock lock(mutex_);
  return areas_;
}

std::vector<Component> ThreadSafeEntityCache::get_components() const {
  std::shared_lock lock(mutex_);
  return components_;
}

std::vector<App> ThreadSafeEntityCache::get_apps() const {
  std::shared_lock lock(mutex_);
  return apps_;
}

std::vector<Function> ThreadSafeEntityCache::get_functions() const {
  std::shared_lock lock(mutex_);
  return functions_;
}

// ============================================================================
// Reader methods - Get by ID
// ============================================================================

std::optional<Area> ThreadSafeEntityCache::get_area(const std::string & id) const {
  std::shared_lock lock(mutex_);
  auto it = area_index_.find(id);
  if (it != area_index_.end() && it->second < areas_.size()) {
    return areas_[it->second];
  }
  return std::nullopt;
}

std::optional<Component> ThreadSafeEntityCache::get_component(const std::string & id) const {
  std::shared_lock lock(mutex_);
  auto it = component_index_.find(id);
  if (it != component_index_.end() && it->second < components_.size()) {
    return components_[it->second];
  }
  return std::nullopt;
}

std::optional<App> ThreadSafeEntityCache::get_app(const std::string & id) const {
  std::shared_lock lock(mutex_);
  auto it = app_index_.find(id);
  if (it != app_index_.end() && it->second < apps_.size()) {
    return apps_[it->second];
  }
  return std::nullopt;
}

std::optional<Function> ThreadSafeEntityCache::get_function(const std::string & id) const {
  std::shared_lock lock(mutex_);
  auto it = function_index_.find(id);
  if (it != function_index_.end() && it->second < functions_.size()) {
    return functions_[it->second];
  }
  return std::nullopt;
}

// ============================================================================
// Reader methods - Check existence
// ============================================================================

bool ThreadSafeEntityCache::has_area(const std::string & id) const {
  std::shared_lock lock(mutex_);
  return area_index_.count(id) > 0;
}

bool ThreadSafeEntityCache::has_component(const std::string & id) const {
  std::shared_lock lock(mutex_);
  return component_index_.count(id) > 0;
}

bool ThreadSafeEntityCache::has_app(const std::string & id) const {
  std::shared_lock lock(mutex_);
  return app_index_.count(id) > 0;
}

bool ThreadSafeEntityCache::has_function(const std::string & id) const {
  std::shared_lock lock(mutex_);
  return function_index_.count(id) > 0;
}

// ============================================================================
// Reader methods - Find any entity
// ============================================================================

std::optional<EntityRef> ThreadSafeEntityCache::find_entity(const std::string & id) const {
  std::shared_lock lock(mutex_);

  // Search order: Component, App, Area, Function
  if (auto it = component_index_.find(id); it != component_index_.end()) {
    return EntityRef{SovdEntityType::COMPONENT, it->second};
  }
  if (auto it = app_index_.find(id); it != app_index_.end()) {
    return EntityRef{SovdEntityType::APP, it->second};
  }
  if (auto it = area_index_.find(id); it != area_index_.end()) {
    return EntityRef{SovdEntityType::AREA, it->second};
  }
  if (auto it = function_index_.find(id); it != function_index_.end()) {
    return EntityRef{SovdEntityType::FUNCTION, it->second};
  }

  return std::nullopt;
}

SovdEntityType ThreadSafeEntityCache::get_entity_type(const std::string & id) const {
  auto ref = find_entity(id);
  return ref ? ref->type : SovdEntityType::UNKNOWN;
}

// ============================================================================
// Relationship queries
// ============================================================================

std::vector<std::string> ThreadSafeEntityCache::get_apps_for_component(const std::string & component_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  auto it = component_to_apps_.find(component_id);
  if (it != component_to_apps_.end()) {
    result.reserve(it->second.size());
    for (size_t idx : it->second) {
      if (idx < apps_.size()) {
        result.push_back(apps_[idx].id);
      }
    }
  }
  return result;
}

std::vector<std::string> ThreadSafeEntityCache::get_components_for_area(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  auto it = area_to_components_.find(area_id);
  if (it != area_to_components_.end()) {
    result.reserve(it->second.size());
    for (size_t idx : it->second) {
      if (idx < components_.size()) {
        result.push_back(components_[idx].id);
      }
    }
  }
  return result;
}

std::vector<std::string> ThreadSafeEntityCache::get_apps_for_function(const std::string & function_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  auto it = function_to_apps_.find(function_id);
  if (it != function_to_apps_.end()) {
    result.reserve(it->second.size());
    for (size_t idx : it->second) {
      if (idx < apps_.size()) {
        result.push_back(apps_[idx].id);
      }
    }
  }
  return result;
}

std::vector<std::string> ThreadSafeEntityCache::get_subareas(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  std::vector<std::string> result;

  auto it = area_to_subareas_.find(area_id);
  if (it != area_to_subareas_.end()) {
    result.reserve(it->second.size());
    for (size_t idx : it->second) {
      if (idx < areas_.size()) {
        result.push_back(areas_[idx].id);
      }
    }
  }
  return result;
}

// ============================================================================
// Aggregation methods
// ============================================================================

AggregatedOperations ThreadSafeEntityCache::get_app_operations(const std::string & app_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "app";
  result.is_aggregated = false;

  auto it = app_index_.find(app_id);
  if (it == app_index_.end() || it->second >= apps_.size()) {
    return result;
  }

  const auto & app = apps_[it->second];
  result.services = app.services;
  result.actions = app.actions;
  result.source_ids.push_back(app_id);

  return result;
}

AggregatedOperations ThreadSafeEntityCache::get_component_operations(const std::string & component_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "component";

  auto comp_it = component_index_.find(component_id);
  if (comp_it == component_index_.end() || comp_it->second >= components_.size()) {
    return result;
  }

  std::unordered_set<std::string> seen_paths;

  // Add component's own operations first
  collect_operations_from_component(comp_it->second, seen_paths, result);

  // Add operations from hosted apps
  auto apps_it = component_to_apps_.find(component_id);
  if (apps_it != component_to_apps_.end()) {
    collect_operations_from_apps(apps_it->second, seen_paths, result);
    // Mark as aggregated if we collected from apps
    if (!apps_it->second.empty()) {
      result.is_aggregated = true;
    }
  }

  return result;
}

AggregatedOperations ThreadSafeEntityCache::get_area_operations(const std::string & area_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "area";
  result.is_aggregated = true;  // Area operations are always aggregated

  auto area_it = area_index_.find(area_id);
  if (area_it == area_index_.end()) {
    return result;
  }

  std::unordered_set<std::string> seen_paths;

  // Get all components in this area
  auto comps_it = area_to_components_.find(area_id);
  if (comps_it != area_to_components_.end()) {
    for (size_t comp_idx : comps_it->second) {
      if (comp_idx >= components_.size()) {
        continue;
      }

      const auto & comp = components_[comp_idx];

      // Add component's own operations
      collect_operations_from_component(comp_idx, seen_paths, result);

      // Add operations from component's apps
      auto apps_it = component_to_apps_.find(comp.id);
      if (apps_it != component_to_apps_.end()) {
        collect_operations_from_apps(apps_it->second, seen_paths, result);
      }
    }
  }

  return result;
}

AggregatedOperations ThreadSafeEntityCache::get_function_operations(const std::string & function_id) const {
  std::shared_lock lock(mutex_);
  AggregatedOperations result;
  result.aggregation_level = "function";
  result.is_aggregated = true;  // Function operations are always aggregated

  auto func_it = function_index_.find(function_id);
  if (func_it == function_index_.end()) {
    return result;
  }

  std::unordered_set<std::string> seen_paths;

  // Get all apps implementing this function
  auto apps_it = function_to_apps_.find(function_id);
  if (apps_it != function_to_apps_.end()) {
    collect_operations_from_apps(apps_it->second, seen_paths, result);
  }

  return result;
}

// ============================================================================
// Operation lookup
// ============================================================================

std::optional<EntityRef> ThreadSafeEntityCache::find_operation_owner(const std::string & operation_path) const {
  std::shared_lock lock(mutex_);
  auto it = operation_index_.find(operation_path);
  if (it != operation_index_.end()) {
    return it->second;
  }
  return std::nullopt;
}

// ============================================================================
// Diagnostics
// ============================================================================

EntityCacheStats ThreadSafeEntityCache::get_stats() const {
  std::shared_lock lock(mutex_);
  EntityCacheStats stats;
  stats.area_count = areas_.size();
  stats.component_count = components_.size();
  stats.app_count = apps_.size();
  stats.function_count = functions_.size();
  stats.total_operations = operation_index_.size();
  stats.last_update = last_update_;
  return stats;
}

std::string ThreadSafeEntityCache::validate() const {
  std::shared_lock lock(mutex_);
  std::ostringstream errors;

  // Check area index
  for (const auto & [id, idx] : area_index_) {
    if (idx >= areas_.size()) {
      errors << "Area index out of bounds: " << id << " -> " << idx << "\n";
    } else if (areas_[idx].id != id) {
      errors << "Area index mismatch: " << id << " -> " << areas_[idx].id << "\n";
    }
  }

  // Check component index
  for (const auto & [id, idx] : component_index_) {
    if (idx >= components_.size()) {
      errors << "Component index out of bounds: " << id << " -> " << idx << "\n";
    } else if (components_[idx].id != id) {
      errors << "Component index mismatch: " << id << " -> " << components_[idx].id << "\n";
    }
  }

  // Check app index
  for (const auto & [id, idx] : app_index_) {
    if (idx >= apps_.size()) {
      errors << "App index out of bounds: " << id << " -> " << idx << "\n";
    } else if (apps_[idx].id != id) {
      errors << "App index mismatch: " << id << " -> " << apps_[idx].id << "\n";
    }
  }

  // Check function index
  for (const auto & [id, idx] : function_index_) {
    if (idx >= functions_.size()) {
      errors << "Function index out of bounds: " << id << " -> " << idx << "\n";
    } else if (functions_[idx].id != id) {
      errors << "Function index mismatch: " << id << " -> " << functions_[idx].id << "\n";
    }
  }

  // Check for duplicate IDs
  std::unordered_set<std::string> seen_ids;
  for (const auto & area : areas_) {
    if (!seen_ids.insert(area.id).second) {
      errors << "Duplicate area ID: " << area.id << "\n";
    }
  }
  for (const auto & comp : components_) {
    if (!seen_ids.insert(comp.id).second) {
      errors << "Duplicate component ID (or conflicts with area): " << comp.id << "\n";
    }
  }
  for (const auto & app : apps_) {
    if (!seen_ids.insert(app.id).second) {
      errors << "Duplicate app ID (or conflicts with other entity): " << app.id << "\n";
    }
  }
  for (const auto & func : functions_) {
    if (!seen_ids.insert(func.id).second) {
      errors << "Duplicate function ID (or conflicts with other entity): " << func.id << "\n";
    }
  }

  return errors.str();
}

std::chrono::system_clock::time_point ThreadSafeEntityCache::get_last_update() const {
  std::shared_lock lock(mutex_);
  return last_update_;
}

// ============================================================================
// Index rebuild helpers
// ============================================================================

void ThreadSafeEntityCache::rebuild_all_indexes() {
  rebuild_area_index();
  rebuild_component_index();
  rebuild_app_index();
  rebuild_function_index();
  rebuild_relationship_indexes();
  rebuild_operation_index();
}

void ThreadSafeEntityCache::rebuild_area_index() {
  area_index_.clear();
  area_index_.reserve(areas_.size());
  for (size_t i = 0; i < areas_.size(); ++i) {
    area_index_[areas_[i].id] = i;
  }
}

void ThreadSafeEntityCache::rebuild_component_index() {
  component_index_.clear();
  component_index_.reserve(components_.size());
  for (size_t i = 0; i < components_.size(); ++i) {
    component_index_[components_[i].id] = i;
  }
}

void ThreadSafeEntityCache::rebuild_app_index() {
  app_index_.clear();
  app_index_.reserve(apps_.size());
  for (size_t i = 0; i < apps_.size(); ++i) {
    app_index_[apps_[i].id] = i;
  }
}

void ThreadSafeEntityCache::rebuild_function_index() {
  function_index_.clear();
  function_index_.reserve(functions_.size());
  for (size_t i = 0; i < functions_.size(); ++i) {
    function_index_[functions_[i].id] = i;
  }
}

void ThreadSafeEntityCache::rebuild_relationship_indexes() {
  component_to_apps_.clear();
  area_to_components_.clear();
  area_to_subareas_.clear();
  function_to_apps_.clear();

  // Build component_to_apps from apps' component_id
  for (size_t i = 0; i < apps_.size(); ++i) {
    const auto & app = apps_[i];
    if (!app.component_id.empty()) {
      component_to_apps_[app.component_id].push_back(i);
    }
  }

  // Build area_to_components from components' area
  for (size_t i = 0; i < components_.size(); ++i) {
    const auto & comp = components_[i];
    if (!comp.area.empty()) {
      area_to_components_[comp.area].push_back(i);
    }
  }

  // Build area_to_subareas from areas' parent_area_id
  for (size_t i = 0; i < areas_.size(); ++i) {
    const auto & area = areas_[i];
    if (!area.parent_area_id.empty()) {
      area_to_subareas_[area.parent_area_id].push_back(i);
    }
  }

  // Build function_to_apps (functions have a hosts field which is vector of app IDs)
  // Note: Function.hosts contains app IDs that implement this function
  for (const auto & func : functions_) {
    for (const auto & app_id : func.hosts) {
      auto app_it = app_index_.find(app_id);
      if (app_it != app_index_.end()) {
        function_to_apps_[func.id].push_back(app_it->second);
      }
    }
  }
}

void ThreadSafeEntityCache::rebuild_operation_index() {
  operation_index_.clear();

  // Index operations from components
  for (size_t i = 0; i < components_.size(); ++i) {
    const auto & comp = components_[i];
    for (const auto & svc : comp.services) {
      operation_index_[svc.full_path] = {SovdEntityType::COMPONENT, i};
    }
    for (const auto & act : comp.actions) {
      operation_index_[act.full_path] = {SovdEntityType::COMPONENT, i};
    }
  }

  // Index operations from apps (apps take priority over components for same path)
  for (size_t i = 0; i < apps_.size(); ++i) {
    const auto & app = apps_[i];
    for (const auto & svc : app.services) {
      operation_index_[svc.full_path] = {SovdEntityType::APP, i};
    }
    for (const auto & act : app.actions) {
      operation_index_[act.full_path] = {SovdEntityType::APP, i};
    }
  }
}

// ============================================================================
// Aggregation helpers
// ============================================================================

void ThreadSafeEntityCache::collect_operations_from_apps(const std::vector<size_t> & app_indexes,
                                                         std::unordered_set<std::string> & seen_paths,
                                                         AggregatedOperations & result) const {
  for (size_t idx : app_indexes) {
    if (idx >= apps_.size()) {
      continue;
    }
    const auto & app = apps_[idx];
    result.source_ids.push_back(app.id);

    for (const auto & svc : app.services) {
      if (seen_paths.insert(svc.full_path).second) {
        result.services.push_back(svc);
      }
    }
    for (const auto & act : app.actions) {
      if (seen_paths.insert(act.full_path).second) {
        result.actions.push_back(act);
      }
    }
  }
}

void ThreadSafeEntityCache::collect_operations_from_component(size_t comp_index,
                                                              std::unordered_set<std::string> & seen_paths,
                                                              AggregatedOperations & result) const {
  if (comp_index >= components_.size()) {
    return;
  }

  const auto & comp = components_[comp_index];
  result.source_ids.push_back(comp.id);

  for (const auto & svc : comp.services) {
    if (seen_paths.insert(svc.full_path).second) {
      result.services.push_back(svc);
    }
  }
  for (const auto & act : comp.actions) {
    if (seen_paths.insert(act.full_path).second) {
      result.actions.push_back(act);
    }
  }
}

// ============================================================================
// Data aggregation methods
// ============================================================================

AggregatedData ThreadSafeEntityCache::get_entity_data(const std::string & entity_id) const {
  // find_entity takes its own lock, so we don't need one here
  auto entity_ref = find_entity(entity_id);
  if (!entity_ref) {
    return {};
  }

  // Each get_*_data method takes its own lock
  switch (entity_ref->type) {
    case SovdEntityType::APP:
      return get_app_data(entity_id);
    case SovdEntityType::COMPONENT:
      return get_component_data(entity_id);
    case SovdEntityType::AREA:
      return get_area_data(entity_id);
    case SovdEntityType::FUNCTION:
      return get_function_data(entity_id);
    default:
      return {};
  }
}

AggregatedData ThreadSafeEntityCache::get_app_data(const std::string & app_id) const {
  std::shared_lock lock(mutex_);

  auto it = app_index_.find(app_id);
  if (it == app_index_.end()) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "app";
  result.is_aggregated = false;

  std::unordered_set<std::string> seen_topics;
  collect_topics_from_app(it->second, seen_topics, result);

  return result;
}

AggregatedData ThreadSafeEntityCache::get_component_data(const std::string & component_id) const {
  std::shared_lock lock(mutex_);

  auto comp_it = component_index_.find(component_id);
  if (comp_it == component_index_.end()) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "component";

  std::unordered_set<std::string> seen_topics;

  // Collect from component itself
  collect_topics_from_component(comp_it->second, seen_topics, result);

  // Collect from hosted apps
  auto apps_it = component_to_apps_.find(component_id);
  if (apps_it != component_to_apps_.end()) {
    collect_topics_from_apps(apps_it->second, seen_topics, result);
    result.is_aggregated = !apps_it->second.empty();
  }

  return result;
}

AggregatedData ThreadSafeEntityCache::get_area_data(const std::string & area_id) const {
  std::shared_lock lock(mutex_);

  auto area_it = area_index_.find(area_id);
  if (area_it == area_index_.end()) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "area";
  result.is_aggregated = true;
  result.source_ids.push_back(area_id);

  std::unordered_set<std::string> seen_topics;

  // Collect from all components in this area
  auto comps_it = area_to_components_.find(area_id);
  if (comps_it != area_to_components_.end()) {
    for (size_t comp_idx : comps_it->second) {
      collect_topics_from_component(comp_idx, seen_topics, result);

      // Also collect from apps hosted on each component
      if (comp_idx < components_.size()) {
        auto apps_it = component_to_apps_.find(components_[comp_idx].id);
        if (apps_it != component_to_apps_.end()) {
          collect_topics_from_apps(apps_it->second, seen_topics, result);
        }
      }
    }
  }

  return result;
}

AggregatedData ThreadSafeEntityCache::get_function_data(const std::string & function_id) const {
  std::shared_lock lock(mutex_);

  auto func_it = function_index_.find(function_id);
  if (func_it == function_index_.end()) {
    return {};
  }

  AggregatedData result;
  result.aggregation_level = "function";
  result.is_aggregated = true;
  result.source_ids.push_back(function_id);

  std::unordered_set<std::string> seen_topics;

  // Collect from all apps implementing this function
  auto apps_it = function_to_apps_.find(function_id);
  if (apps_it != function_to_apps_.end()) {
    collect_topics_from_apps(apps_it->second, seen_topics, result);
  }

  return result;
}

// ============================================================================
// Data aggregation helpers
// ============================================================================

void ThreadSafeEntityCache::collect_topics_from_app(size_t app_index, std::unordered_set<std::string> & seen_topics,
                                                    AggregatedData & result) const {
  if (app_index >= apps_.size()) {
    return;
  }

  const auto & app = apps_[app_index];
  result.source_ids.push_back(app.id);

  // Publishers
  for (const auto & topic : app.topics.publishes) {
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "publish"});
    } else {
      // Topic already seen as subscriber - update direction to "both"
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "subscribe") {
          t.direction = "both";
          break;
        }
      }
    }
  }

  // Subscribers
  for (const auto & topic : app.topics.subscribes) {
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "subscribe"});
    } else {
      // Topic already seen - might be both pub and sub, update direction
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "publish") {
          t.direction = "both";
          break;
        }
      }
    }
  }
}

void ThreadSafeEntityCache::collect_topics_from_apps(const std::vector<size_t> & app_indexes,
                                                     std::unordered_set<std::string> & seen_topics,
                                                     AggregatedData & result) const {
  for (size_t idx : app_indexes) {
    collect_topics_from_app(idx, seen_topics, result);
  }
}

void ThreadSafeEntityCache::collect_topics_from_component(size_t comp_index,
                                                          std::unordered_set<std::string> & seen_topics,
                                                          AggregatedData & result) const {
  if (comp_index >= components_.size()) {
    return;
  }

  const auto & comp = components_[comp_index];
  result.source_ids.push_back(comp.id);

  // Publishers
  for (const auto & topic : comp.topics.publishes) {
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "publish"});
    } else {
      // Topic already seen as subscriber - update direction to "both"
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "subscribe") {
          t.direction = "both";
          break;
        }
      }
    }
  }

  // Subscribers
  for (const auto & topic : comp.topics.subscribes) {
    auto [_, inserted] = seen_topics.insert(topic);
    if (inserted) {
      result.topics.push_back({topic, "", "subscribe"});
    } else {
      // Topic already seen - might be both pub and sub, update direction
      for (auto & t : result.topics) {
        if (t.name == topic && t.direction == "publish") {
          t.direction = "both";
          break;
        }
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
