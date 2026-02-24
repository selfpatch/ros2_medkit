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
 * Passed to UpdateProvider::prepare/execute. The plugin MAY use it to report
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

}  // namespace ros2_medkit_gateway
