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

#include "ros2_medkit_gateway/lock_manager.hpp"

#include <algorithm>

namespace ros2_medkit_gateway {

LockManager::LockManager(const ThreadSafeEntityCache & cache, LockConfig config)
  : cache_(cache), config_(std::move(config)) {
}

std::string LockManager::generate_lock_id() {
  uint64_t id = next_id_.fetch_add(1, std::memory_order_relaxed);
  return "lock_" + std::to_string(id);
}

EntityLockConfig LockManager::get_entity_config(const std::string & entity_id, const std::string & entity_type) const {
  // Entity-specific override takes precedence
  auto entity_it = config_.entity_overrides.find(entity_id);
  if (entity_it != config_.entity_overrides.end()) {
    return entity_it->second;
  }

  // Type-level default
  auto type_it = config_.type_defaults.find(entity_type);
  if (type_it != config_.type_defaults.end()) {
    return type_it->second;
  }

  // Global defaults
  return EntityLockConfig{};
}

std::string LockManager::get_entity_type_string(const std::string & entity_id) const {
  auto entity_type = cache_.get_entity_type(entity_id);
  switch (entity_type) {
    case SovdEntityType::AREA:
      return "area";
    case SovdEntityType::COMPONENT:
      return "component";
    case SovdEntityType::APP:
      return "app";
    default:
      return "unknown";
  }
}

bool LockManager::lock_covers_collection(const LockInfo & lock, const std::string & collection) {
  // Empty scopes means all collections are locked
  if (lock.scopes.empty()) {
    return true;
  }
  return std::find(lock.scopes.begin(), lock.scopes.end(), collection) != lock.scopes.end();
}

tl::expected<LockInfo, LockError> LockManager::acquire(const std::string & entity_id, const std::string & client_id,
                                                       const std::vector<std::string> & scopes, int expiration_seconds,
                                                       bool break_lock) {
  if (!config_.enabled) {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled on this gateway", 403, std::nullopt});
  }

  // Validate scopes
  const auto & valid_scopes = valid_lock_scopes();
  for (const auto & scope : scopes) {
    if (valid_scopes.find(scope) == valid_scopes.end()) {
      return tl::make_unexpected(LockError{"invalid-scope", "Unknown scope: " + scope, 400, std::nullopt});
    }
  }

  // Resolve entity type for config lookup
  std::string entity_type = get_entity_type_string(entity_id);
  EntityLockConfig entity_config = get_entity_config(entity_id, entity_type);

  // Determine effective max expiration
  int max_exp = entity_config.max_expiration > 0 ? entity_config.max_expiration : config_.default_max_expiration;

  // Validate expiration
  if (expiration_seconds <= 0) {
    return tl::make_unexpected(LockError{"invalid-expiration", "Expiration must be greater than 0", 400, std::nullopt});
  }
  if (expiration_seconds > max_exp) {
    return tl::make_unexpected(LockError{"invalid-expiration",
                                         "Expiration exceeds maximum allowed (" + std::to_string(max_exp) + "s)", 400,
                                         std::nullopt});
  }

  std::unique_lock<std::shared_mutex> write_lock(mutex_);

  // Check for existing lock
  auto existing_it = locks_.find(entity_id);
  if (existing_it != locks_.end()) {
    const auto & existing = existing_it->second;

    if (!break_lock) {
      return tl::make_unexpected(LockError{
          "lock-conflict", "Entity is already locked by client '" + existing.client_id + "'", 409, existing.lock_id});
    }

    // break_lock requested - check if the existing lock is breakable
    if (!existing.breakable) {
      return tl::make_unexpected(
          LockError{"lock-not-breakable", "Existing lock is not breakable", 409, existing.lock_id});
    }

    // Remove existing lock
    lock_id_index_.erase(existing.lock_id);
    locks_.erase(existing_it);
  }

  // Create new lock
  auto now = std::chrono::steady_clock::now();
  LockInfo info;
  info.lock_id = generate_lock_id();
  info.entity_id = entity_id;
  info.client_id = client_id;
  info.scopes = scopes;
  info.breakable = entity_config.breakable;
  info.created_at = now;
  info.expires_at = now + std::chrono::seconds(expiration_seconds);
  info.expiration_seconds = expiration_seconds;

  // Insert into primary storage and secondary index
  lock_id_index_[info.lock_id] = entity_id;
  locks_[entity_id] = info;

  return info;
}

tl::expected<void, LockError> LockManager::release(const std::string & entity_id, const std::string & client_id) {
  if (!config_.enabled) {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled on this gateway", 403, std::nullopt});
  }

  std::unique_lock<std::shared_mutex> write_lock(mutex_);

  auto it = locks_.find(entity_id);
  if (it == locks_.end()) {
    return tl::make_unexpected(
        LockError{"lock-not-found", "No lock exists for entity '" + entity_id + "'", 404, std::nullopt});
  }

  if (it->second.client_id != client_id) {
    return tl::make_unexpected(
        LockError{"lock-not-owner", "Lock is owned by a different client", 403, it->second.lock_id});
  }

  // Remove from secondary index and primary storage
  lock_id_index_.erase(it->second.lock_id);
  locks_.erase(it);

  return {};
}

tl::expected<LockInfo, LockError> LockManager::extend(const std::string & entity_id, const std::string & client_id,
                                                      int additional_seconds) {
  if (!config_.enabled) {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled on this gateway", 403, std::nullopt});
  }

  if (additional_seconds <= 0) {
    return tl::make_unexpected(
        LockError{"invalid-expiration", "Extension duration must be greater than 0", 400, std::nullopt});
  }

  std::unique_lock<std::shared_mutex> write_lock(mutex_);

  auto it = locks_.find(entity_id);
  if (it == locks_.end()) {
    return tl::make_unexpected(
        LockError{"lock-not-found", "No lock exists for entity '" + entity_id + "'", 404, std::nullopt});
  }

  if (it->second.client_id != client_id) {
    return tl::make_unexpected(
        LockError{"lock-not-owner", "Lock is owned by a different client", 403, it->second.lock_id});
  }

  // Extend from now
  auto now = std::chrono::steady_clock::now();
  it->second.expires_at = now + std::chrono::seconds(additional_seconds);
  it->second.expiration_seconds = additional_seconds;

  return it->second;
}

std::optional<LockInfo> LockManager::get_lock(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> read_lock(mutex_);

  auto it = locks_.find(entity_id);
  if (it == locks_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<LockInfo> LockManager::get_lock_by_id(const std::string & lock_id) const {
  std::shared_lock<std::shared_mutex> read_lock(mutex_);

  auto idx_it = lock_id_index_.find(lock_id);
  if (idx_it == lock_id_index_.end()) {
    return std::nullopt;
  }

  auto lock_it = locks_.find(idx_it->second);
  if (lock_it == locks_.end()) {
    return std::nullopt;  // Should not happen, but defensive
  }
  return lock_it->second;
}

LockAccessResult LockManager::check_access(const std::string & entity_id, const std::string & client_id,
                                           const std::string & collection) const {
  if (!config_.enabled) {
    return LockAccessResult{true, "", "", ""};
  }

  std::shared_lock<std::shared_mutex> read_lock(mutex_);

  // Phase 1: Check lock_required - if configured, a lock must be held by the client
  if (config_.lock_required) {
    auto it = locks_.find(entity_id);
    if (it == locks_.end() || it->second.client_id != client_id) {
      return LockAccessResult{false, "", "A lock is required to access this entity", "lock-required"};
    }
  }

  // Phase 2: Check lock conflict - walk up parent chain
  // Check the entity itself
  auto check_entity = [&](const std::string & check_id) -> std::optional<LockAccessResult> {
    auto it = locks_.find(check_id);
    if (it == locks_.end()) {
      return std::nullopt;
    }
    const auto & lock = it->second;

    // Owner always has access
    if (lock.client_id == client_id) {
      return std::nullopt;
    }

    // Check if the lock covers the requested collection
    if (lock_covers_collection(lock, collection)) {
      return LockAccessResult{false, lock.lock_id, "Entity '" + check_id + "' is locked by another client",
                              "lock-conflict"};
    }

    return std::nullopt;
  };

  // Check direct entity
  if (auto result = check_entity(entity_id)) {
    return *result;
  }

  // Walk up parent chain: App -> Component -> Area
  // If the entity is an App, check its parent Component
  auto app = cache_.get_app(entity_id);
  if (app.has_value() && !app->component_id.empty()) {
    if (auto result = check_entity(app->component_id)) {
      return *result;
    }

    // Check the Component's parent Area
    auto component = cache_.get_component(app->component_id);
    if (component.has_value() && !component->area.empty()) {
      if (auto result = check_entity(component->area)) {
        return *result;
      }
    }
  }

  // If the entity is a Component, check its parent Area
  auto component = cache_.get_component(entity_id);
  if (component.has_value() && !component->area.empty()) {
    if (auto result = check_entity(component->area)) {
      return *result;
    }
  }

  return LockAccessResult{true, "", "", ""};
}

std::vector<ExpiredLockInfo> LockManager::cleanup_expired() {
  std::vector<ExpiredLockInfo> expired;

  std::unique_lock<std::shared_mutex> write_lock(mutex_);

  auto now = std::chrono::steady_clock::now();
  auto it = locks_.begin();
  while (it != locks_.end()) {
    if (it->second.expires_at <= now) {
      expired.push_back(ExpiredLockInfo{it->second.lock_id, it->second.entity_id, it->second.client_id});
      lock_id_index_.erase(it->second.lock_id);
      it = locks_.erase(it);
    } else {
      ++it;
    }
  }

  return expired;
}

}  // namespace ros2_medkit_gateway
