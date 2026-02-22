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
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/updates/update_backend.hpp"

namespace ros2_medkit_gateway {

/// Error codes for UpdateManager operations - replaces string matching
enum class UpdateErrorCode {
  NotFound,        // Package does not exist
  AlreadyExists,   // Duplicate package ID on registration
  InProgress,      // Operation already running for this package
  NotPrepared,     // Execute called before prepare completed
  NotAutomated,    // Package does not support automated mode
  InvalidRequest,  // Missing fields or invalid input
  Deleting,        // Package is being deleted
  NoBackend,       // No update backend loaded
  Internal         // Unexpected backend error
};

/// Typed error for UpdateManager operations
struct UpdateError {
  UpdateErrorCode code;
  std::string message;
};

/**
 * @brief Manages software update lifecycle with pluggable backend.
 *
 * Handles async operations (prepare/execute) in background threads,
 * tracks status automatically, and delegates to UpdateBackend for
 * actual work. Without a backend, all operations return errors.
 */
class UpdateManager {
 public:
  /// Construct with optional backend. Pass nullptr for 501 mode.
  explicit UpdateManager(std::unique_ptr<UpdateBackend> backend, void * plugin_handle = nullptr);
  ~UpdateManager();

  // Prevent copy/move (owns async tasks)
  UpdateManager(const UpdateManager &) = delete;
  UpdateManager & operator=(const UpdateManager &) = delete;

  /// Check if a backend is loaded
  bool has_backend() const;

  // ---- CRUD (direct delegation to backend) ----
  tl::expected<std::vector<std::string>, UpdateError> list_updates(const UpdateFilter & filter);
  tl::expected<nlohmann::json, UpdateError> get_update(const std::string & id);
  tl::expected<void, UpdateError> register_update(const nlohmann::json & metadata);
  tl::expected<void, UpdateError> delete_update(const std::string & id);

  // ---- Async operations ----
  tl::expected<void, UpdateError> start_prepare(const std::string & id);
  tl::expected<void, UpdateError> start_execute(const std::string & id);
  tl::expected<void, UpdateError> start_automated(const std::string & id);

  // ---- Status ----
  tl::expected<UpdateStatusInfo, UpdateError> get_status(const std::string & id);

 private:
  std::unique_ptr<UpdateBackend> backend_;
  void * plugin_handle_ = nullptr;  // dlopen handle, closed in destructor

  struct PackageState {
    UpdatePhase phase = UpdatePhase::None;
    UpdateStatusInfo status;
    std::future<void> active_task;
  };

  // unique_ptr for pointer stability: references to PackageState (held by
  // UpdateProgressReporter in background threads) remain valid even if the
  // map rehashes due to concurrent insertions.
  std::unordered_map<std::string, std::unique_ptr<PackageState>> states_;
  mutable std::mutex mutex_;
  std::atomic<bool> stopped_{false};

  /// Check if a task is still running
  bool is_task_active(const std::string & id) const;

  /// Run prepare in background thread
  void run_prepare(const std::string & id);

  /// Run execute in background thread
  void run_execute(const std::string & id);

  /// Run automated (prepare + execute) in background thread
  void run_automated(const std::string & id);
};

}  // namespace ros2_medkit_gateway
