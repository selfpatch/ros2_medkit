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
enum class UpdatePhase { None, Preparing, Prepared, Executing, Executed, Failed };

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
 */
class UpdateBackend {
 public:
  virtual ~UpdateBackend() = default;

  // ---- CRUD (plugin owns all metadata storage) ----

  /// List all registered update package IDs, optionally filtered
  virtual tl::expected<std::vector<std::string>, std::string> list_updates(const UpdateFilter & filter) = 0;

  /// Get full metadata for a specific update package as JSON
  virtual tl::expected<nlohmann::json, std::string> get_update(const std::string & id) = 0;

  /// Register a new update package from JSON metadata
  virtual tl::expected<void, std::string> register_update(const nlohmann::json & metadata) = 0;

  /// Delete an update package
  virtual tl::expected<void, std::string> delete_update(const std::string & id) = 0;

  // ---- Async operations (called in background thread by UpdateManager) ----

  /// Prepare an update (download, verify, check dependencies).
  /// Reporter is optional - plugin may call reporter.set_progress() etc.
  virtual tl::expected<void, std::string> prepare(const std::string & id, UpdateProgressReporter & reporter) = 0;

  /// Execute an update (install). Only called after prepare succeeds.
  virtual tl::expected<void, std::string> execute(const std::string & id, UpdateProgressReporter & reporter) = 0;

  /// Check whether a package supports automated mode (prepare + execute)
  virtual tl::expected<bool, std::string> supports_automated(const std::string & id) = 0;

  /// Optional: receive plugin-specific configuration from YAML
  virtual void configure(const nlohmann::json & /* config */) {
  }
};

}  // namespace ros2_medkit_gateway
