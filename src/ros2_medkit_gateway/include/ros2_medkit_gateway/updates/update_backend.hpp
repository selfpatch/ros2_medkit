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

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Filter criteria for listing update packages
struct UpdateFilter {
  std::optional<std::string> origin;          // "remote" | "proximity"
  std::optional<std::string> target_version;  // Filter by target version
};

/// Status of an update operation
enum class UpdateStatus { Pending, InProgress, Completed, Failed };

/// Detailed progress for a sub-step of an update operation
struct UpdateSubProgress {
  std::string name;
  int progress;  // 0-100
};

/// Full status information for an update operation
struct UpdateStatusInfo {
  UpdateStatus status = UpdateStatus::Pending;
  std::optional<int> progress;                                 // 0-100
  std::optional<std::vector<UpdateSubProgress>> sub_progress;  // Detailed per-step progress
  std::optional<std::string> error_message;                    // Set when status == Failed
};

/// Internal phase tracking for update lifecycle
enum class UpdatePhase { None, Preparing, Prepared, Executing, Executed, Failed, Deleting };

/// Error codes for backend return values
enum class UpdateBackendError {
  NotFound,       // Package does not exist
  AlreadyExists,  // Duplicate ID on registration
  InvalidInput,   // Malformed metadata
  Internal        // Unexpected error
};

/// Typed error for backend return values
struct UpdateBackendErrorInfo {
  UpdateBackendError code;
  std::string message;
};

/**
 * @brief Thread-safe reporter for update progress.
 *
 * Passed to UpdateBackend::prepare/execute. The plugin MAY use it to report
 * fine-grained progress. If not used, UpdateManager still tracks base status
 * (Pending -> InProgress -> Completed/Failed) automatically.
 */
class UpdateProgressReporter {
 public:
  UpdateProgressReporter(UpdateStatusInfo & status, std::mutex & mutex) : status_(status), mutex_(mutex) {
  }

  void set_progress(int percent) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.progress = percent;
  }

  void set_sub_progress(std::vector<UpdateSubProgress> steps) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.sub_progress = std::move(steps);
  }

 private:
  UpdateStatusInfo & status_;
  std::mutex & mutex_;
};

/**
 * @brief Abstract base class for software update backends (plugin interface).
 *
 * Implementations handle the actual update logic: metadata storage, package
 * preparation, and execution. The gateway's UpdateManager handles async
 * lifecycle and status tracking.
 *
 * Plugins can be loaded at compile-time (subclass and pass to UpdateManager)
 * or at runtime (.so loaded via dlopen with extern "C" factory function).
 *
 * For runtime loading, the .so must export:
 *   extern "C" UpdateBackend* create_update_backend();
 *
 * @par Thread Safety
 * - CRUD methods (list_updates, get_update, register_update, delete_update)
 *   are called WITHOUT holding UpdateManager's mutex. They may be called
 *   concurrently with each other and with prepare/execute running in a
 *   background thread. If the backend shares state, it must provide its own
 *   synchronization.
 *   Note: delete_update has a partial guard - UpdateManager checks/sets a
 *   Deleting sentinel under lock before calling delete_update, but the
 *   backend call itself runs outside the lock.
 * - prepare() and execute() run in a background std::async thread. They may
 *   run concurrently with CRUD calls from the HTTP thread.
 * - The UpdateProgressReporter passed to prepare/execute is already
 *   thread-safe - plugins may call set_progress/set_sub_progress freely.
 * - Exceptions thrown from prepare/execute are caught by UpdateManager and
 *   mapped to Failed status. Plugins should prefer returning
 *   tl::make_unexpected() for expected errors.
 */
class UpdateBackend {
 public:
  virtual ~UpdateBackend() = default;

  // ---- CRUD (plugin owns all metadata storage) ----

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

  /// Check whether a package supports automated mode (prepare + execute)
  virtual tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & id) = 0;

  /// Optional: receive plugin-specific configuration from YAML
  virtual void configure(const nlohmann::json & /* config */) {
  }
};

}  // namespace ros2_medkit_gateway
