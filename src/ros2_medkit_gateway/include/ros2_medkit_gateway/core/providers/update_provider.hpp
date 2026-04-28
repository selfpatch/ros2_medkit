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

#include "ros2_medkit_gateway/core/providers/update_types.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Provider interface for software update backends
 *
 * Typed provider interface that plugins implement alongside GatewayPlugin
 * via multiple inheritance. Replaces the old UpdateBackend abstract class.
 *
 * Reuses all types from update_types.hpp (UpdateFilter, UpdateBackendErrorInfo,
 * UpdateProgressReporter, etc.).
 *
 * @par Thread Safety
 * - CRUD methods (list_updates, get_update, register_update, delete_update)
 *   are called WITHOUT holding UpdateManager's mutex. They may be called
 *   concurrently with each other and with prepare/execute running in a
 *   background thread. If the plugin shares state, it must provide its own
 *   synchronization.
 *   Note: delete_update has a partial guard - UpdateManager checks/sets a
 *   Deleting sentinel under lock before calling delete_update, but the
 *   plugin call itself runs outside the lock.
 * - prepare() and execute() run in a background std::async thread. They may
 *   run concurrently with CRUD calls from the HTTP thread.
 * - The UpdateProgressReporter passed to prepare/execute is already
 *   thread-safe - plugins may call set_progress/set_sub_progress freely.
 * - Exceptions thrown from prepare/execute are caught by UpdateManager and
 *   mapped to Failed status. Plugins should prefer returning
 *   tl::make_unexpected() for expected errors.
 *
 * @see GatewayPlugin for the base class
 * @see UpdateManager for the subsystem that uses this
 */
class UpdateProvider {
 public:
  virtual ~UpdateProvider() = default;

  // ---- CRUD (metadata storage owned by plugin) ----

  /// List all registered update package IDs, optionally filtered
  virtual tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter & filter) = 0;

  /// Get full metadata for a specific update package as JSON
  virtual tl::expected<nlohmann::json, UpdateBackendErrorInfo> get_update(const std::string & id) = 0;

  /// Register a new update package from JSON metadata
  virtual tl::expected<void, UpdateBackendErrorInfo> register_update(const nlohmann::json & metadata) = 0;

  /// Delete an update package
  virtual tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & id) = 0;

  // ---- Async operations (called in background thread by UpdateManager) ----

  /// Prepare an update (download, verify, check dependencies).
  /// Reporter is optional - plugin may call reporter.set_progress() etc.
  virtual tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & id,
                                                             UpdateProgressReporter & reporter) = 0;

  /// Execute an update (install). Only called after prepare succeeds.
  virtual tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & id,
                                                             UpdateProgressReporter & reporter) = 0;

  // ---- Capability queries ----

  /// Check whether a package supports automated mode (prepare + execute)
  virtual tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & id) = 0;
};

}  // namespace ros2_medkit_gateway
