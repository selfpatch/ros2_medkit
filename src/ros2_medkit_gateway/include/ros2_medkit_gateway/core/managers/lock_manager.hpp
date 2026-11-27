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

#include <atomic>
#include <chrono>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {

// =========================================================================
// Configuration types
// =========================================================================

/**
 * @brief Per-entity or per-type lock configuration
 *
 * Allows overriding lock behavior for specific entity types or individual
 * entities (e.g., making certain components non-breakable).
 */
struct EntityLockConfig {
  std::vector<std::string> required_scopes;  ///< Scopes that must be locked (empty = no requirement)
  bool breakable = true;                     ///< Whether locks on this entity can be broken
  int max_expiration = 0;                    ///< Max expiration seconds (0 = use global default)
};

/**
 * @brief Global lock subsystem configuration
 */
struct LockConfig {
  bool enabled = true;                ///< Whether locking is enabled at all
  int default_max_expiration = 3600;  ///< Default max lock expiration in seconds
  int cleanup_interval = 30;          ///< Interval in seconds for expired lock cleanup

  /// Per-entity-type defaults, keyed by lock_type_keyword ("area", "component", "app", "function")
  std::unordered_map<std::string, EntityLockConfig> type_defaults;

  /// Per-entity overrides, keyed by the entity's type and id: a manifest's lock section configures the
  /// entity that declares it, not an entity of another type that shares its id.
  std::unordered_map<MemberRef, EntityLockConfig, MemberRefHash> entity_overrides;
};

/// The key type_defaults is configured under for an entity type: "area", "component", "app", "function", or
/// "unknown".
const char * lock_type_keyword(SovdEntityType type);

// =========================================================================
// Lock data types
// =========================================================================

/**
 * @brief Information about an active lock
 */
struct LockInfo {
  std::string lock_id;                                  ///< Unique lock identifier ("lock_<uuid>")
  std::string entity_id;                                ///< Locked entity ID
  SovdEntityType entity_type{SovdEntityType::UNKNOWN};  ///< Type of the locked entity (the route that took it)
  std::string client_id;                                ///< Client that holds the lock
  std::vector<std::string> scopes;                      ///< Locked resource collections (empty = all)
  bool breakable = true;                                ///< Whether this lock can be broken
  std::chrono::steady_clock::time_point created_at;     ///< When the lock was acquired
  std::chrono::steady_clock::time_point expires_at;     ///< When the lock expires
  int expiration_seconds = 0;                           ///< Original requested expiration
};

/**
 * @brief Error returned from lock operations
 */
struct LockError {
  std::string code;                             ///< Error code (e.g., "lock-conflict")
  std::string message;                          ///< Human-readable message
  int status_code = 409;                        ///< HTTP status code for handler use
  std::optional<std::string> existing_lock_id;  ///< Lock ID causing conflict (if applicable)
};

/**
 * @brief Result of an access check against locks
 */
struct LockAccessResult {
  bool allowed = true;            ///< Whether access is permitted
  std::string denied_by_lock_id;  ///< Lock ID that denied access
  std::string denied_reason;      ///< Human-readable reason
  std::string denied_code;        ///< Error code for the denial
};

/**
 * @brief Information about an expired lock (returned from cleanup)
 */
struct ExpiredLockInfo {
  std::string lock_id;                                  ///< Lock ID that expired
  std::string entity_id;                                ///< Entity that was locked
  SovdEntityType entity_type{SovdEntityType::UNKNOWN};  ///< Type of the entity that was locked
  std::string client_id;                                ///< Client that held the lock
  std::vector<std::string> scopes;                      ///< Lock scopes (empty = all collections)
};

// =========================================================================
// Valid lock scopes
// =========================================================================

/**
 * @brief Set of valid scope names for lock operations
 *
 * These correspond to resource collections that can be individually locked.
 */
inline const std::unordered_set<std::string> & valid_lock_scopes() {
  static const std::unordered_set<std::string> scopes = {
      "data", "operations", "configurations", "faults", "modes", "scripts", "bulk-data", "logs", "cyclic-subscriptions",
  };
  return scopes;
}

// =========================================================================
// LockManager
// =========================================================================

/**
 * @brief Transport-agnostic lock manager for SOVD entity locking
 *
 * Manages exclusive locks on entities with optional scope restrictions.
 * Supports parent-child lock propagation: a lock on a Component denies
 * access to its Apps, and a lock on an Area denies access to its Components
 * and their Apps.
 *
 * Thread safety: uses shared_mutex (shared_lock for reads, unique_lock for writes).
 * All public methods are thread-safe.
 *
 * An entity is named by its type and its id: an App and a Component may share
 * an id, and each has its own lock and its own configuration. Every call names
 * the type of the route it serves.
 *
 * Design:
 * - Primary storage: unordered_map<(entity type, entity_id), LockInfo>
 * - Secondary index: unordered_map<lock_id, (entity type, entity_id)>
 * - One lock per entity (acquire on already-locked entity returns error unless break_lock=true)
 * - Empty scopes = all collections locked
 * - Parent propagation via EntityCache lookups (App->Component->Area)
 */
class LockManager {
 public:
  /**
   * @brief Construct a LockManager
   * @param cache Reference to the entity cache for parent-chain lookups
   * @param config Lock configuration
   */
  LockManager(const ThreadSafeEntityCache & cache, LockConfig config);

  /**
   * @brief Acquire a lock on an entity
   * @param entity_type Type of the entity to lock
   * @param entity_id Entity to lock
   * @param client_id Client requesting the lock
   * @param scopes Resource collections to lock (empty = all)
   * @param expiration_seconds Lock duration in seconds
   * @param break_lock If true, forcefully replace existing lock
   * @return LockInfo on success, LockError on failure
   */
  tl::expected<LockInfo, LockError> acquire(SovdEntityType entity_type, const std::string & entity_id,
                                            const std::string & client_id, const std::vector<std::string> & scopes,
                                            int expiration_seconds, bool break_lock = false);

  /**
   * @brief Release a lock
   * @param entity_type Type of the entity to unlock
   * @param entity_id Entity to unlock
   * @param client_id Client requesting release (must match lock owner)
   * @return Empty on success, LockError on failure
   */
  tl::expected<void, LockError> release(SovdEntityType entity_type, const std::string & entity_id,
                                        const std::string & client_id);

  /**
   * @brief Extend a lock's expiration
   * @param entity_type Type of the locked entity
   * @param entity_id Locked entity
   * @param client_id Client requesting extension (must match lock owner)
   * @param additional_seconds Seconds to add from now
   * @return Updated LockInfo on success, LockError on failure
   */
  tl::expected<LockInfo, LockError> extend(SovdEntityType entity_type, const std::string & entity_id,
                                           const std::string & client_id, int additional_seconds);

  /**
   * @brief Get lock info for an entity
   * @param entity_type Type of the entity to query
   * @param entity_id Entity to query
   * @return LockInfo if locked, nullopt otherwise
   */
  std::optional<LockInfo> get_lock(SovdEntityType entity_type, const std::string & entity_id) const;

  /**
   * @brief Get lock info by lock ID
   * @param lock_id Lock identifier
   * @return LockInfo if found, nullopt otherwise
   */
  std::optional<LockInfo> get_lock_by_id(const std::string & lock_id) const;

  /**
   * @brief Check if a client can access a resource collection on an entity
   *
   * Two-phase check:
   * 1. If lock_required is configured and no lock is held by client, deny
   * 2. Check lock conflict: walks up parent chain (App->Component->Area)
   *    checking for locks that cover the requested collection
   *
   * @param entity_type Type of the entity being accessed
   * @param entity_id Entity being accessed
   * @param client_id Client requesting access (empty = anonymous)
   * @param collection Resource collection being accessed
   * @return LockAccessResult indicating whether access is allowed
   */
  LockAccessResult check_access(SovdEntityType entity_type, const std::string & entity_id,
                                const std::string & client_id, const std::string & collection) const;

  /**
   * @brief Remove expired locks
   * @return List of expired locks that were removed
   */
  std::vector<ExpiredLockInfo> cleanup_expired();

  /**
   * @brief Get the lock configuration
   */
  const LockConfig & config() const {
    return config_;
  }

 private:
  /// Generate a unique lock ID
  std::string generate_lock_id();

  /// Get effective lock config for an entity (entity override > type default > global)
  EntityLockConfig get_entity_config(const MemberRef & entity) const;

  /// Check if a lock conflicts with access to a given collection (empty scopes = all locked)
  static bool lock_covers_collection(const LockInfo & lock, const std::string & collection);

  const ThreadSafeEntityCache & cache_;
  LockConfig config_;

  mutable std::shared_mutex mutex_;

  /// Primary storage: (entity type, entity_id) -> LockInfo
  std::unordered_map<MemberRef, LockInfo, MemberRefHash> locks_;

  /// Secondary index: lock_id -> (entity type, entity_id)
  std::unordered_map<std::string, MemberRef> lock_id_index_;

  /// Counter for unique lock ID generation
  std::atomic<uint64_t> next_id_{1};
};

}  // namespace ros2_medkit_gateway
