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

/// Status of an update operation (SOVD-compliant enum values)
enum class UpdateStatus { Pending, InProgress, Completed, Failed };

/// Lifecycle phase - exposed as SOVD vendor extension `x-medkit-phase`.
/// Differentiates "prepare completed" from "execute completed" which share
/// status=completed in the SOVD standard enum.
enum class UpdatePhase { None, Preparing, Prepared, Executing, Executed, Failed, Deleting };

/// Detailed progress for a sub-step of an update operation
struct UpdateSubProgress {
  std::string name;
  int progress;  // 0-100
};

/// Full status information for an update operation
struct UpdateStatusInfo {
  UpdateStatus status = UpdateStatus::Pending;
  UpdatePhase phase = UpdatePhase::None;
  std::optional<int> progress;                                 // 0-100
  std::optional<std::vector<UpdateSubProgress>> sub_progress;  // Detailed per-step progress
  std::optional<std::string> error_message;                    // Set when status == Failed
};

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

/// Serialize UpdatePhase to its `x-medkit-phase` string value.
inline const char * update_phase_to_string(UpdatePhase phase) {
  switch (phase) {
    case UpdatePhase::None:
      return "none";
    case UpdatePhase::Preparing:
      return "preparing";
    case UpdatePhase::Prepared:
      return "prepared";
    case UpdatePhase::Executing:
      return "executing";
    case UpdatePhase::Executed:
      return "executed";
    case UpdatePhase::Failed:
      return "failed";
    case UpdatePhase::Deleting:
      return "deleting";
  }
  return "none";
}

/// Serialize UpdateStatusInfo to SOVD-compliant JSON.
/// Adds vendor extension `x-medkit-phase` to distinguish prepare-completed from
/// execute-completed (both report SOVD status=completed).
inline nlohmann::json update_status_to_json(const UpdateStatusInfo & status) {
  nlohmann::json j;
  switch (status.status) {
    case UpdateStatus::Pending:
      j["status"] = "pending";
      break;
    case UpdateStatus::InProgress:
      j["status"] = "inProgress";
      break;
    case UpdateStatus::Completed:
      j["status"] = "completed";
      break;
    case UpdateStatus::Failed:
      j["status"] = "failed";
      break;
  }
  j["x-medkit-phase"] = update_phase_to_string(status.phase);
  if (status.progress.has_value()) {
    j["progress"] = *status.progress;
  }
  if (status.sub_progress.has_value()) {
    j["sub_progress"] = nlohmann::json::array();
    for (const auto & sp : *status.sub_progress) {
      j["sub_progress"].push_back({{"name", sp.name}, {"progress", sp.progress}});
    }
  }
  if (status.error_message.has_value()) {
    j["error"] = *status.error_message;
  }
  return j;
}

}  // namespace ros2_medkit_gateway
