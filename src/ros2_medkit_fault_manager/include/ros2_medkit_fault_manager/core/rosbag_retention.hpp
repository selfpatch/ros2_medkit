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

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/core/log_sink.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager::core {

/// Retention settings of the rosbag recordings (`snapshots.rosbag.*`).
struct RosbagRetentionLimits {
  size_t max_bags_per_fault{1};                  ///< Recordings kept per fault code, 0 = unlimited
  size_t max_total_bytes{500ULL * 1024 * 1024};  ///< Quota over every recording
  bool auto_cleanup{true};                       ///< A cleared fault loses its recordings
};

enum class StoreResult {
  kStored,            ///< The rows were applied. Also when the cap removed every new row.
  kAllFaultsCleared,  ///< Every fault of the recording is cleared. Nothing was stored.
  kStoreFailed,       ///< The cap read or the insert failed. Nothing was stored.
};

struct StoreOutcome {
  StoreResult result{StoreResult::kStored};
  /// The quota read or an eviction threw. The rows stay stored, and so do the evictions before it.
  bool quota_failed{false};
  std::string error;  ///< Exception text of the store failure or of the quota failure
  /// Directories no row names any more, each once. Never the path of a recording that
  /// kAllFaultsCleared or kStoreFailed left without rows: the caller discards that one.
  std::vector<std::string> unlink_paths;
};

struct RemovalOutcome {
  size_t removed_rows{0};
  std::vector<std::string> unlink_paths;  ///< Directories no row names any more, each once
};

/// Every rosbag retention decision of the fault manager, on FaultStorage::apply_rosbag_changes().
///
/// Each public method holds one mutex for its whole decision. Lock order: the caller's outer
/// locks -> this mutex -> the storage's own lock. Under it only storage calls and logging run:
/// no other lock and no file system call. Nothing here unlinks a file. Each method returns the
/// paths the caller may remove.
class RosbagRetention {
 public:
  RosbagRetention(FaultStorage & storage, RosbagRetentionLimits limits, LogSink & log_sink, std::string logger_name);

  RosbagRetention(const RosbagRetention &) = delete;
  RosbagRetention & operator=(const RosbagRetention &) = delete;
  RosbagRetention(RosbagRetention &&) = delete;
  RosbagRetention & operator=(RosbagRetention &&) = delete;
  ~RosbagRetention() = default;

  /// Store the rows of one recording. In one hold of the mutex: drop the rows of faults the
  /// storage holds as cleared, apply the per-fault cap together with the insert, then the quota.
  /// Precondition: no stored row names the recording's path, except links that @p rows upsert.
  /// Never throws.
  StoreOutcome store(std::vector<RosbagFileInfo> rows);

  /// Evict whole recordings, oldest first, until the total is at or under @p max_bytes. Never
  /// evicts @p keep_recording_id.
  /// @throws the first storage failure; the evictions before it stay applied
  /// @return The paths to unlink, in eviction order
  std::vector<std::string> evict_over_quota(size_t max_bytes, const std::string & keep_recording_id = {});

  /// Remove every row of @p fault_code. The caller decides whether a clear removes recordings.
  /// @throws the storage failure; no row changed then
  RemovalOutcome on_fault_cleared(const std::string & fault_code);

  /// Remove every row of a recording whose directory is gone.
  /// @throws the storage failure; no row changed then
  RemovalOutcome forget_recording(const std::string & recording_id);

 private:
  struct Eviction {
    std::vector<std::string> evicted_paths;
    std::vector<std::string> unlink_paths;
  };

  /// Whether a fault keeps its row: false only under auto_cleanup with a cap of exactly 1, for a
  /// fault the storage holds as cleared. A fault that cannot be read keeps its row.
  bool row_still_wanted(const std::string & fault_code);

  /// The insert and the per-fault cap removals of @p rows, as one change set.
  /// @throws a failure of the cap read
  RosbagChangeSet cap_changes(const std::vector<RosbagFileInfo> & rows);

  /// Quota eviction. @p out holds what was applied, also when this throws.
  void evict(size_t max_bytes, const std::string & keep_recording_id, Eviction & out);

  /// Apply @p changes and count @p rows as removed.
  RemovalOutcome remove(const std::vector<RosbagFileInfo> & rows, const RosbagChangeSet & changes);

  FaultStorage & storage_;
  const RosbagRetentionLimits limits_;
  LogSink & log_sink_;
  const std::string logger_name_;
  std::mutex mutex_;
};

}  // namespace ros2_medkit_fault_manager::core
