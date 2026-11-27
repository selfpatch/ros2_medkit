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

#include "ros2_medkit_fault_manager/core/rosbag_retention.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <optional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace ros2_medkit_fault_manager::core {

namespace {

/// Append @p freed to @p out, skipping a path already listed.
void append_unique(std::vector<FreedRecording> & out, const std::vector<FreedRecording> & freed) {
  for (const auto & entry : freed) {
    const bool listed = std::any_of(out.begin(), out.end(), [&entry](const FreedRecording & known) {
      return known.path == entry.path;
    });
    if (!listed) {
      out.push_back(entry);
    }
  }
}

/// The freed recordings of @p paths, each with its recording id from @p recording_ids.
std::vector<FreedRecording> freed(const std::vector<std::string> & paths,
                                  const std::unordered_map<std::string, std::string> & recording_ids,
                                  RemovalReason reason) {
  std::vector<FreedRecording> out;
  out.reserve(paths.size());
  for (const auto & path : paths) {
    const auto known = recording_ids.find(path);
    const std::string id =
        known != recording_ids.end() && !known->second.empty() ? known->second : rosbag_recording_id(path);
    out.push_back({path, id, reason});
  }
  return out;
}

bool is_empty(const RosbagChangeSet & changes) {
  return changes.insert.empty() && changes.remove.empty() && changes.remove_recordings.empty();
}

}  // namespace

RosbagRetention::RosbagRetention(FaultStorage & storage, RosbagRetentionLimits limits, LogSink & log_sink,
                                 std::string logger_name)
  : storage_(storage), limits_(limits), log_sink_(log_sink), logger_name_(std::move(logger_name)) {
}

StoreOutcome RosbagRetention::store(std::vector<RosbagFileInfo> rows) {
  std::lock_guard<std::mutex> lock(mutex_);
  StoreOutcome outcome;

  // A clear either shows here as cleared, or its removal waits for this mutex and takes the rows.
  rows.erase(std::remove_if(rows.begin(), rows.end(),
                            [this](const RosbagFileInfo & row) {
                              return !row_still_wanted(row.fault_code);
                            }),
             rows.end());
  if (rows.empty()) {
    outcome.result = StoreResult::kAllFaultsCleared;
    return outcome;
  }

  RosbagChangeSet changes;
  std::unordered_map<std::string, std::string> recording_ids;
  try {
    changes = cap_changes(rows, recording_ids);
    if (!is_empty(changes)) {
      outcome.unlink_paths =
          freed(storage_.apply_rosbag_changes(changes).orphaned_paths, recording_ids, RemovalReason::kCap);
    }
  } catch (const std::exception & e) {
    outcome.result = StoreResult::kStoreFailed;
    outcome.error = e.what();
    return outcome;
  } catch (...) {
    outcome.result = StoreResult::kStoreFailed;
    outcome.error = "unknown exception";
    return outcome;
  }

  // A path the cap left without any new row has no row at all: nothing else names a new recording.
  std::set<std::string> inserted_paths;
  for (const auto & row : changes.insert) {
    inserted_paths.insert(row.file_path);
  }
  for (const auto & row : rows) {
    if (inserted_paths.count(row.file_path) == 0) {
      append_unique(outcome.unlink_paths, freed({row.file_path}, recording_ids, RemovalReason::kCap));
    }
  }

  const std::string & bag_path = rows.front().file_path;
  const std::string & keep_recording_id = rows.front().recording_id;
  Eviction eviction;
  try {
    try {
      evict(limits_.max_total_bytes, keep_recording_id, eviction);
    } catch (...) {
      append_unique(outcome.unlink_paths, eviction.unlink_paths);
      throw;
    }
    append_unique(outcome.unlink_paths, eviction.unlink_paths);
    // Everything older is gone by now, so what is left over the quota is the kept recording alone.
    if (!keep_recording_id.empty() && storage_.get_total_rosbag_storage_bytes() > limits_.max_total_bytes) {
      logf(log_sink_, LogLevel::kWarn, logger_name_,
           "Recording '%s' alone exceeds snapshots.rosbag.max_total_storage_mb (%zu MB); it is kept, and the "
           "storage stays over the limit until a newer recording replaces it",
           keep_recording_id.c_str(), limits_.max_total_bytes / (1024 * 1024));
    }
  } catch (const std::exception & e) {
    outcome.quota_failed = true;
    outcome.error = e.what();
    logf(log_sink_, LogLevel::kWarn, logger_name_, "Failed to enforce the rosbag storage limit after storing '%s': %s",
         bag_path.c_str(), e.what());
  } catch (...) {
    outcome.quota_failed = true;
    outcome.error = "unknown exception";
    logf(log_sink_, LogLevel::kWarn, logger_name_,
         "Failed to enforce the rosbag storage limit after storing '%s': unknown exception", bag_path.c_str());
  }
  return outcome;
}

std::vector<std::string> RosbagRetention::evict_over_quota(size_t max_bytes, const std::string & keep_recording_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  Eviction eviction;
  evict(max_bytes, keep_recording_id, eviction);
  std::vector<std::string> paths;
  paths.reserve(eviction.unlink_paths.size());
  for (const auto & entry : eviction.unlink_paths) {
    paths.push_back(entry.path);
  }
  return paths;
}

RemovalOutcome RosbagRetention::on_fault_cleared(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto rows = storage_.get_rosbag_files(fault_code);
  RosbagChangeSet changes;
  for (const auto & row : rows) {
    changes.remove.push_back({row.fault_code, row.file_path});
  }
  return remove(rows, changes, RemovalReason::kCleared);
}

RemovalOutcome RosbagRetention::forget_recording(const std::string & recording_id) {
  // An empty id names rows whose backfill did not finish, not one recording.
  if (recording_id.empty()) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto rows = storage_.get_rosbag_files_by_recording(recording_id);
  RosbagChangeSet changes;
  changes.remove_recordings.push_back(recording_id);
  return remove(rows, changes, RemovalReason::kStale);
}

bool RosbagRetention::row_still_wanted(const std::string & fault_code) {
  // Without auto_cleanup, or with a history configured, a cleared fault keeps its recording.
  if (!limits_.auto_cleanup || limits_.max_bags_per_fault != 1) {
    return true;
  }
  std::optional<fd::FaultRecord> fault;
  try {
    fault = storage_.get_fault(fault_code);
  } catch (const std::exception & e) {
    logf(log_sink_, LogLevel::kWarn, logger_name_,
         "Could not read fault '%s' before storing its bag, keeping its row: %s", fault_code.c_str(), e.what());
    return true;
  } catch (...) {
    logf(log_sink_, LogLevel::kWarn, logger_name_,
         "Could not read fault '%s' before storing its bag, keeping its row: unknown exception", fault_code.c_str());
    return true;
  }
  // Absent is not cleared: only a fault the storage holds AND reports as cleared loses its row.
  return !fault.has_value() || fault->status != fd::fault_status::kCleared;
}

RosbagChangeSet RosbagRetention::cap_changes(const std::vector<RosbagFileInfo> & rows,
                                             std::unordered_map<std::string, std::string> & recording_ids) {
  for (const auto & row : rows) {
    recording_ids.emplace(row.file_path, row.recording_id);
  }
  RosbagChangeSet changes;
  const size_t cap = limits_.max_bags_per_fault;
  if (cap == 0) {
    changes.insert = rows;
    return changes;
  }

  // One candidate per link, ordered as the backends order rows: created_at_ns, then insertion.
  // A stored row ranks by its place in the read; a new row ranks after every stored one; a row
  // that upserts a stored link keeps the stored rank.
  struct Candidate {
    int64_t created_at_ns;
    size_t rank;
    std::string file_path;
    bool stored;
    bool incoming;
  };
  std::set<std::pair<std::string, std::string>> dropped;
  std::set<std::string> done;
  for (const auto & first : rows) {
    const std::string & code = first.fault_code;
    if (!done.insert(code).second) {
      continue;
    }
    const auto stored = storage_.get_rosbag_files(code);  // newest first
    std::vector<Candidate> candidates;
    size_t rank = 0;
    for (auto it = stored.rbegin(); it != stored.rend(); ++it) {
      recording_ids.emplace(it->file_path, it->recording_id);
      candidates.push_back({it->created_at_ns, rank++, it->file_path, true, false});
    }
    for (const auto & row : rows) {
      if (row.fault_code != code) {
        continue;
      }
      auto same_link = std::find_if(candidates.begin(), candidates.end(), [&row](const Candidate & c) {
        return c.file_path == row.file_path;
      });
      if (same_link != candidates.end()) {
        same_link->created_at_ns = row.created_at_ns;
        same_link->incoming = true;
      } else {
        candidates.push_back({row.created_at_ns, rank++, row.file_path, false, true});
      }
    }
    if (candidates.size() <= cap) {
      continue;
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      return std::tie(a.created_at_ns, a.rank) > std::tie(b.created_at_ns, b.rank);  // newest first
    });
    for (size_t i = cap; i < candidates.size(); ++i) {
      if (candidates[i].stored) {
        changes.remove.push_back({code, candidates[i].file_path});
      }
      if (candidates[i].incoming) {
        dropped.emplace(code, candidates[i].file_path);
      }
    }
  }

  for (const auto & row : rows) {
    if (dropped.count({row.fault_code, row.file_path}) == 0) {
      changes.insert.push_back(row);
    }
  }
  return changes;
}

void RosbagRetention::evict(size_t max_bytes, const std::string & keep_recording_id, Eviction & out) {
  size_t current_bytes = storage_.get_total_rosbag_storage_bytes();
  if (current_bytes <= max_bytes) {
    return;
  }

  // Rows are links: a burst shares one recording and one fault holds several. Group them back
  // into bags, so a shared bag is evicted whole and its bytes are freed once.
  std::vector<std::string> paths_oldest_first;
  std::unordered_map<std::string, std::string> recording_by_path;
  std::unordered_map<std::string, size_t> bytes_by_path;
  for (const auto & row : storage_.get_all_rosbag_files()) {
    // The rows arrive oldest first, so a path's first row gives the recording's age.
    if (recording_by_path.emplace(row.file_path, row.recording_id).second) {
      paths_oldest_first.push_back(row.file_path);
    }
    // The storage counts the largest row per path.
    auto & bytes = bytes_by_path[row.file_path];
    bytes = std::max(bytes, row.size_bytes);
  }

  for (const auto & path : paths_oldest_first) {
    if (current_bytes <= max_bytes) {
      break;
    }
    if (!keep_recording_id.empty() && recording_by_path[path] == keep_recording_id) {
      continue;
    }
    // By recording, not by fault code: a fault's newer recordings must stay.
    RosbagChangeSet changes;
    changes.remove_recordings.push_back(recording_by_path[path]);
    append_unique(out.unlink_paths, freed(storage_.apply_rosbag_changes(changes).orphaned_paths, recording_by_path,
                                          RemovalReason::kQuota));
    // Saturate: the quota must never be met by underflow.
    current_bytes -= std::min(current_bytes, bytes_by_path[path]);
  }
}

RemovalOutcome RosbagRetention::remove(const std::vector<RosbagFileInfo> & rows, const RosbagChangeSet & changes,
                                       RemovalReason reason) {
  RemovalOutcome outcome;
  if (rows.empty()) {
    return outcome;
  }
  std::unordered_map<std::string, std::string> recording_ids;
  for (const auto & row : rows) {
    recording_ids.emplace(row.file_path, row.recording_id);
  }
  outcome.unlink_paths = freed(storage_.apply_rosbag_changes(changes).orphaned_paths, recording_ids, reason);
  outcome.removed_rows = rows.size();
  return outcome;
}

}  // namespace ros2_medkit_fault_manager::core
