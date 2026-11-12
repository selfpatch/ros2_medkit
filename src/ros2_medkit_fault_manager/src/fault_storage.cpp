// Copyright 2025 mfaferek93
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

#include "ros2_medkit_fault_manager/fault_storage.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <tuple>

namespace ros2_medkit_fault_manager {

std::string rosbag_recording_id(const std::string & file_path) {
  std::filesystem::path p(file_path);
  if (!p.has_filename()) {
    p = p.parent_path();  // tolerate a trailing slash
  }
  return p.filename().string();
}

int32_t clamp_debounce_counter(int32_t counter, const DebounceConfig & config) {
  // Manual min/max rather than std::clamp: well-defined even if a bad config has lo > hi
  // (sanitize_debounce_config normally prevents that, but the storage layer must never UB).
  if (counter < config.confirmation_threshold) {
    return config.confirmation_threshold;
  }
  if (counter > config.healing_threshold) {
    return config.healing_threshold;
  }
  return counter;
}

std::string compute_debounce_status(int32_t counter, const std::string & current_status,
                                    const DebounceConfig & config) {
  if (counter <= config.confirmation_threshold) {
    return fd::fault_status::kConfirmed;
  }
  if (config.healing_enabled && counter >= config.healing_threshold) {
    return fd::fault_status::kHealed;
  }
  // Hysteresis latch: a confirmed or healed fault stays put until the counter reaches the opposite
  // threshold (handled above). A single opposite-direction report must not flip it to a pending state.
  if (current_status == fd::fault_status::kConfirmed || current_status == fd::fault_status::kHealed) {
    return current_status;
  }
  if (counter < 0) {
    return fd::fault_status::kPrefailed;
  }
  if (counter > 0) {
    return fd::fault_status::kPrepassed;
  }
  return current_status;  // counter == 0 keeps the current status (avoids flapping at the boundary)
}

bool is_near_miss(bool is_failed_event, const std::string & resulting_status) {
  return is_failed_event && resulting_status != fd::fault_status::kConfirmed;
}

bool sanitize_debounce_config(DebounceConfig & config) {
  bool valid = true;
  if (config.confirmation_threshold >= 0) {
    config.confirmation_threshold = -1;
    valid = false;
  }
  // healing_threshold == 0 is valid: it means "heal on a single PASSED event" (the counter reaches
  // the threshold at 0). Only a negative threshold is rejected.
  if (config.healing_threshold < 0) {
    config.healing_threshold = 3;
    valid = false;
  }
  return valid;
}

fd::FaultRecord FaultState::to_record() const {
  fd::FaultRecord msg;
  msg.fault_code = fault_code;
  msg.severity = severity;
  msg.description = description;
  msg.first_occurred_ns = first_occurred_ns;
  msg.last_occurred_ns = last_occurred_ns;
  msg.last_passed_ns = last_passed_ns;
  msg.occurrence_count = occurrence_count;
  msg.status = status;
  msg.reporting_sources.assign(reporting_sources.begin(), reporting_sources.end());
  return msg;
}

void InMemoryFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig InMemoryFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void InMemoryFaultStorage::update_status(FaultState & state, const DebounceConfig & config) {
  // CLEARED faults are handled in report_fault_event() before this is called.
  // Delegate to the shared state machine so both backends behave identically.
  state.status = compute_debounce_status(state.debounce_counter, state.status, config);
}

bool InMemoryFaultStorage::report_fault_event(const FaultId & id, fd::ReportKind kind, uint8_t severity,
                                              const std::string & description, const std::string & source_id,
                                              int64_t timestamp_ns, const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool is_failed = (kind == fd::ReportKind::kFailed);

  auto it = faults_.find(id);
  if (it == faults_.end()) {
    // New record - only create one for FAILED events. A PASSED event for a record that
    // does not exist has nothing to heal.
    if (!is_failed) {
      return false;  // PASSED event for non-existent record is ignored
    }

    FaultState state;
    state.fault_code = id.fault_code;
    state.owner = id.owner;
    state.reporting_sources.insert(source_id);
    state.severity = severity;
    state.description = description;
    state.first_occurred_ns = timestamp_ns;
    state.last_occurred_ns = timestamp_ns;
    state.last_failed_ns = timestamp_ns;
    state.occurrence_count = 1;
    state.debounce_counter = -1;  // First FAILED event

    // CRITICAL severity bypasses debounce and confirms immediately
    if (config.critical_immediate_confirm && severity == fd::severity::kCritical) {
      state.status = fd::fault_status::kConfirmed;
    } else {
      // Set status based on debounce counter vs threshold
      update_status(state, config);
    }

    if (is_near_miss(true, state.status)) {
      record_near_miss(state, config, severity, source_id, timestamp_ns);
    }

    faults_.emplace(id, std::move(state));
    return true;
  }

  // Existing record - update
  auto & state = it->second;

  // CLEARED faults can be reactivated by FAILED events
  if (state.status == fd::fault_status::kCleared) {
    if (!is_failed) {
      // PASSED events for CLEARED faults are ignored
      return false;
    }
    // FAILED event reactivates the fault - reset debounce counter and first-seen:
    // this is a new outage cycle, not a continuation of the one that just cleared.
    state.debounce_counter = -1;
    state.first_occurred_ns = timestamp_ns;
    state.last_failed_ns = timestamp_ns;
    state.last_occurred_ns = timestamp_ns;
    state.reporting_sources.insert(source_id);
    if (state.occurrence_count < std::numeric_limits<uint32_t>::max()) {
      ++state.occurrence_count;
    }
    if (severity > state.severity) {
      state.severity = severity;
    }
    if (!description.empty()) {
      state.description = description;
    }
    // Immediate confirmation reads the stored severity: a CRITICAL record confirms on any FAILED report.
    if (config.critical_immediate_confirm && state.severity == fd::severity::kCritical) {
      state.status = fd::fault_status::kConfirmed;
    } else {
      update_status(state, config);
    }
    if (is_near_miss(true, state.status)) {
      record_near_miss(state, config, severity, source_id, timestamp_ns);
    }
    return true;  // Reactivation treated as new occurrence for event publishing
  }

  // Bring a counter left outside this config's band back into range before applying the report.
  // The band a record resolves to can change under it (per-entity overrides when several sources
  // share the counter, a reloaded threshold file, a global config that moved between runs), which
  // leaves a stored value clamped to the old ceiling sitting above the new one. The SQLite
  // backend clamps on read for the same reason, and the two backends have to agree on the counter
  // they record and on the status it produces.
  state.debounce_counter = clamp_debounce_counter(state.debounce_counter, config);

  if (is_failed) {
    // last_occurred_ns tracks occurrences only. A PASSED event is the fault ENDING, not
    // occurring; bumping it there makes a long-stale CONFIRMED fault look freshly
    // active to operators. The PASSED instant is kept in last_passed_ns.
    state.last_occurred_ns = timestamp_ns;
    state.last_failed_ns = timestamp_ns;

    // occurrence_count is NOT bumped here: this is a still-active fault being
    // re-reported (level-triggered poller, or debounce building toward
    // confirmation), the same continuous occurrence. It only increments on a
    // genuine edge - new fault (above) or reactivation after CLEARED (above).

    // Decrement towards confirmation, clamped to the thresholds (a long FAILED burst can't
    // run the counter off to INT32_MIN and delay later healing).
    state.debounce_counter = clamp_debounce_counter(state.debounce_counter - 1, config);

    // Add source if not already present
    state.reporting_sources.insert(source_id);

    // Update severity if higher
    if (severity > state.severity) {
      state.severity = severity;
    }

    // Update description if provided
    if (!description.empty()) {
      state.description = description;
    }

    // Immediate confirmation reads the stored severity, so it also overrides the HEALED latch.
    if (config.critical_immediate_confirm && state.severity == fd::severity::kCritical) {
      state.debounce_counter = std::min(state.debounce_counter, -1);
      state.status = fd::fault_status::kConfirmed;
      return false;
    }
  } else {
    // PASSED event
    state.last_passed_ns = timestamp_ns;

    // Increment towards healing, clamped to the thresholds (a heal heartbeat can't run the
    // counter off to INT32_MAX and then delay a real fault from confirming).
    state.debounce_counter = clamp_debounce_counter(state.debounce_counter + 1, config);
  }

  // Update status based on debounce counter
  update_status(state, config);

  if (is_near_miss(is_failed, state.status)) {
    record_near_miss(state, config, severity, source_id, timestamp_ns);
  }

  return false;
}

std::vector<FaultEntry> InMemoryFaultStorage::list_fault_entries(bool filter_by_severity, uint8_t severity,
                                                                 const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Determine which statuses to include
  std::set<std::string> status_filter;
  if (statuses.empty()) {
    // Default: only CONFIRMED faults (excludes PREFAILED, CLEARED, HEALED)
    status_filter.insert(fd::fault_status::kConfirmed);
  } else {
    for (const auto & s : statuses) {
      // Only add valid statuses (invalid ones are silently ignored)
      if (s == fd::fault_status::kPrefailed || s == fd::fault_status::kPrepassed || s == fd::fault_status::kConfirmed ||
          s == fd::fault_status::kHealed || s == fd::fault_status::kCleared) {
        status_filter.insert(s);
      }
    }
    // If all provided statuses were invalid, default to CONFIRMED for consistency
    if (status_filter.empty()) {
      status_filter.insert(fd::fault_status::kConfirmed);
    }
  }

  std::vector<FaultEntry> result;
  result.reserve(faults_.size());

  for (const auto & [id, state] : faults_) {
    // Filter by status
    if (status_filter.find(state.status) == status_filter.end()) {
      continue;
    }

    // Filter by severity if requested
    if (filter_by_severity && state.severity != severity) {
      continue;
    }

    result.push_back(FaultEntry{id, state.to_record()});
  }

  return result;
}

std::optional<fd::FaultRecord> InMemoryFaultStorage::get_fault(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(id);
  if (it == faults_.end()) {
    return std::nullopt;
  }

  return it->second.to_record();
}

std::vector<FaultId> InMemoryFaultStorage::get_ids_by_code(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // FaultId orders by code first, so the records of one code are one contiguous range
  // and this is a range scan. Owner order inside it is the map's, which is what makes
  // an ambiguity message list the owners the same way every time.
  std::vector<FaultId> result;
  for (auto it = faults_.lower_bound(FaultId{fault_code, ""}); it != faults_.end(); ++it) {
    if (it->first.fault_code != fault_code) {
      break;
    }
    result.push_back(it->first);
  }
  return result;
}

bool InMemoryFaultStorage::clear_fault(const FaultId & id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(id);
  if (it == faults_.end()) {
    return false;
  }

  // Acknowledging a record drops its value snapshots, unless a history was asked
  // for: with recordings retained past a clear, deleting the readings that go with
  // them leaves a record holding bags whose matching values are gone. Scoped to the
  // record, so another owner's readings for the same code stay where they are.
  if (!retain_snapshots_on_clear_) {
    snapshots_.erase(std::remove_if(snapshots_.begin(), snapshots_.end(),
                                    [&id](const SnapshotData & s) {
                                      return s.fault_code == id.fault_code && s.owner == id.owner;
                                    }),
                     snapshots_.end());
  }

  it->second.status = fd::fault_status::kCleared;
  return true;
}

size_t InMemoryFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.size();
}

bool InMemoryFaultStorage::contains(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.find(id) != faults_.end();
}

std::vector<FaultId> InMemoryFaultStorage::check_time_based_confirmation(int64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<FaultId> confirmed;
  if (config_.auto_confirm_after_sec <= 0.0) {
    return confirmed;  // Time-based confirmation disabled
  }

  const int64_t threshold_ns = std::llround(config_.auto_confirm_after_sec * 1e9);

  for (auto & [id, state] : faults_) {
    if (state.status == fd::fault_status::kPrefailed) {
      const int64_t age_ns = now_ns - state.last_failed_ns;
      if (age_ns >= threshold_ns) {
        state.status = fd::fault_status::kConfirmed;
        confirmed.push_back(id);
      }
    }
  }

  return confirmed;
}

void InMemoryFaultStorage::set_max_snapshots_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_snapshots_per_fault_ = max_count;
}

void InMemoryFaultStorage::set_retain_snapshots_on_clear(bool retain) {
  std::lock_guard<std::mutex> lock(mutex_);
  retain_snapshots_on_clear_ = retain;
}

bool InMemoryFaultStorage::retains_snapshots_on_clear() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retain_snapshots_on_clear_;
}

void InMemoryFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  store_snapshots({snapshot});
}

void InMemoryFaultStorage::store_snapshots(const std::vector<SnapshotData> & snapshots) {
  if (snapshots.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  const FaultId id{snapshots.front().fault_code, snapshots.front().owner};

  // Built beside the live vector and swapped in, the shape store_rosbag_files
  // uses: a capture is all-or-nothing, so a throw partway must not leave half of
  // it stored.
  auto updated = snapshots_;
  updated.insert(updated.end(), snapshots.begin(), snapshots.end());

  if (max_snapshots_per_fault_ > 0) {
    // Evict whole capture sets, oldest first, until this record fits.
    //
    // The old rule counted rows and rejected the NEW row once full, so a capture
    // that straddled the cap was stored in part: some topics present, the rest
    // silently absent, indistinguishable from "that topic was not publishing".
    // Keep-newest also stops this cap from opposing the rosbag one.
    const auto rows_for_fault = [&updated, &id]() {
      return static_cast<size_t>(std::count_if(updated.begin(), updated.end(), [&id](const SnapshotData & s) {
        return s.fault_code == id.fault_code && s.owner == id.owner;
      }));
    };

    // One capture can exceed the cap on its own, so the newest set is exempt from
    // eviction: half a freeze frame reads exactly like "those topics were silent".
    // SqliteFaultStorage carries the same exemption as `AND capture_id <> ?2`, and
    // the two backends have to agree or the evidence depends on storage_type.
    int64_t newest = 0;
    bool have_newest = false;
    for (const auto & s : updated) {
      if (s.fault_code == id.fault_code && s.owner == id.owner && (!have_newest || s.capture_id > newest)) {
        newest = s.capture_id;
        have_newest = true;
      }
    }

    while (have_newest && rows_for_fault() > max_snapshots_per_fault_) {
      bool found = false;
      int64_t oldest = 0;
      for (const auto & s : updated) {
        if (s.fault_code == id.fault_code && s.owner == id.owner && s.capture_id != newest &&
            (!found || s.capture_id < oldest)) {
          oldest = s.capture_id;
          found = true;
        }
      }
      if (!found) {
        break;  // only the newest capture is left and it is over the cap on its own
      }
      updated.erase(std::remove_if(updated.begin(), updated.end(),
                                   [&id, oldest](const SnapshotData & s) {
                                     return s.fault_code == id.fault_code && s.owner == id.owner &&
                                            s.capture_id == oldest;
                                   }),
                    updated.end());
    }
  }

  snapshots_.swap(updated);
}

std::vector<SnapshotData> InMemoryFaultStorage::get_snapshots(const FaultId & id,
                                                              const std::string & topic_filter) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<SnapshotData> result;
  for (const auto & snapshot : snapshots_) {
    if (snapshot.fault_code == id.fault_code && snapshot.owner == id.owner) {
      if (topic_filter.empty() || snapshot.topic == topic_filter) {
        result.push_back(snapshot);
      }
    }
  }
  // Newest capture first, matching SqliteFaultStorage's ORDER BY. Insertion order
  // would put the OLDEST set first here, and the reader folds rows into a per-topic
  // map, so the values a client sees would depend on storage_type.
  std::stable_sort(result.begin(), result.end(), [](const SnapshotData & a, const SnapshotData & b) {
    return std::tie(b.capture_id, b.captured_at_ns) < std::tie(a.capture_id, a.captured_at_ns);
  });
  return result;
}

int64_t InMemoryFaultStorage::get_max_capture_id() const {
  std::lock_guard<std::mutex> lock(mutex_);

  int64_t max_id = 0;
  for (const auto & snapshot : snapshots_) {
    max_id = std::max(max_id, snapshot.capture_id);
  }
  return max_id;
}

void InMemoryFaultStorage::store_freeze_frame(const FreezeFrameData & frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  freeze_frames_[FaultId{frame.fault_code, frame.owner}] = frame;
}

std::optional<FreezeFrameData> InMemoryFaultStorage::get_freeze_frame(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = freeze_frames_.find(id);
  if (it == freeze_frames_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void InMemoryFaultStorage::record_near_miss(const FaultState & state, const DebounceConfig & config, uint8_t severity,
                                            const std::string & source_id, int64_t timestamp_ns) {
  auto & series = near_misses_[FaultId{state.fault_code, state.owner}];

  NearMissRecord record;
  record.fault_code = state.fault_code;
  record.owner = state.owner;
  record.occurred_at_ns = timestamp_ns;
  record.debounce_counter = state.debounce_counter;
  record.confirmation_threshold = config.confirmation_threshold;
  record.severity = severity;
  record.source_id = source_id;
  record.resulting_status = state.status;
  series.push_back(std::move(record));

  // Evict oldest-first: a series frozen at boot says nothing about a trend.
  if (max_near_misses_per_fault_ > 0 && series.size() > max_near_misses_per_fault_) {
    using DiffType = std::vector<NearMissRecord>::difference_type;
    const auto excess = static_cast<DiffType>(series.size() - max_near_misses_per_fault_);
    series.erase(series.begin(), series.begin() + excess);
  }
}

size_t InMemoryFaultStorage::set_max_near_misses_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_near_misses_per_fault_ = max_count;

  if (max_count == 0) {
    return 0;  // Unlimited
  }

  // Apply the bound to what is already stored, so a series built under a larger bound does not
  // stay over the new one until the next near miss for that code happens to arrive.
  using DiffType = std::vector<NearMissRecord>::difference_type;
  size_t evicted = 0;
  for (auto & [id, series] : near_misses_) {
    (void)id;
    if (series.size() > max_count) {
      const size_t excess = series.size() - max_count;
      series.erase(series.begin(), series.begin() + static_cast<DiffType>(excess));
      evicted += excess;
    }
  }
  return evicted;
}

std::vector<NearMissRecord> InMemoryFaultStorage::get_near_misses(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = near_misses_.find(id);
  if (it == near_misses_.end()) {
    return {};
  }
  return it->second;
}

void InMemoryFaultStorage::set_max_rosbags_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_rosbags_per_fault_ = max_count;
}

void InMemoryFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  // Through the batch path deliberately: with a per-fault cap a single store is
  // insert + trim, and both must publish together.
  store_rosbag_files({info});
}

void InMemoryFaultStorage::store_rosbag_files(const std::vector<RosbagFileInfo> & infos) {
  if (infos.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Built beside the live vector and swapped in, because swap cannot throw. The
  // caller reads a throw here as "no row was written" and removes the recording, so
  // a batch that stored half a burst and then threw would leave those rows naming a
  // bag that is gone. Copying costs one allocation per stored recording, not per
  // message. std::vector::swap is noexcept for the default allocator, exactly as
  // std::map::swap was.
  auto updated = rosbag_files_;
  uint64_t next_seq = rosbag_seq_;
  std::set<std::string> evicted;

  for (const auto & in : infos) {
    RosbagFileInfo row = in;
    if (row.recording_id.empty()) {
      row.recording_id = rosbag_recording_id(row.file_path);
    }

    // Upsert on the (record, recording) LINK - the same grain as the SQLite unique
    // index. Re-storing the same link refreshes it; a link to a different recording
    // appends, which is the feature.
    auto it = std::find_if(updated.begin(), updated.end(), [&row](const RosbagRow & r) {
      return r.info.fault_code == row.fault_code && r.info.owner == row.owner && r.info.file_path == row.file_path;
    });
    if (it != updated.end()) {
      it->info = row;
    } else {
      updated.push_back(RosbagRow{row, ++next_seq});
    }

    if (max_rosbags_per_fault_ == 0) {
      continue;
    }

    // Keep the newest N recordings of this record, oldest evicted first - the same
    // direction as evict_bags_over_quota, so the two eviction owners never need a
    // tiebreak. At N = 1 this is the pre-#620 behaviour exactly.
    std::vector<size_t> mine;
    for (size_t i = 0; i < updated.size(); ++i) {
      if (updated[i].info.fault_code == row.fault_code && updated[i].info.owner == row.owner) {
        mine.push_back(i);
      }
    }
    if (mine.size() <= max_rosbags_per_fault_) {
      continue;
    }
    std::sort(mine.begin(), mine.end(), [&updated](size_t a, size_t b) {
      return std::tie(updated[a].info.created_at_ns, updated[a].seq) <
             std::tie(updated[b].info.created_at_ns, updated[b].seq);
    });
    std::vector<size_t> doomed(mine.begin(),
                               mine.begin() + static_cast<std::ptrdiff_t>(mine.size() - max_rosbags_per_fault_));
    std::sort(doomed.rbegin(), doomed.rend());  // descending, so erase stays valid
    for (size_t idx : doomed) {
      evicted.insert(updated[idx].info.file_path);
      updated.erase(updated.begin() + static_cast<std::ptrdiff_t>(idx));
    }
  }

  rosbag_files_.swap(updated);
  rosbag_seq_ = next_seq;

  // Unlinked only once every row is in, the way the SQLite backend unlinks after its
  // COMMIT: a throw above must leave the old rows pointing at bags that still exist.
  // Referencing is decided on the finished batch, not on the state before it - two
  // records of one burst can share the bag being evicted, and checking row by row
  // beforehand would find it still held by a sibling that a later iteration then
  // evicts, leaking the directory.
  for (const auto & path : evicted) {
    if (path_referenced(path)) {
      continue;
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    // Ignore errors - file may already be deleted
  }
}

std::optional<RosbagFileInfo> InMemoryFaultStorage::get_rosbag_file(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Newest, matching the SQLite backend's ORDER BY created_at_ns DESC, id DESC.
  const RosbagRow * best = nullptr;
  for (const auto & row : rosbag_files_) {
    if (row.info.fault_code != id.fault_code || row.info.owner != id.owner) {
      continue;
    }
    if (best == nullptr || std::tie(best->info.created_at_ns, best->seq) < std::tie(row.info.created_at_ns, row.seq)) {
      best = &row;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return best->info;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::get_rosbag_files(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<const RosbagRow *> mine;
  for (const auto & row : rosbag_files_) {
    if (row.info.fault_code == id.fault_code && row.info.owner == id.owner) {
      mine.push_back(&row);
    }
  }
  std::sort(mine.begin(), mine.end(), [](const RosbagRow * a, const RosbagRow * b) {
    return std::tie(b->info.created_at_ns, b->seq) < std::tie(a->info.created_at_ns, a->seq);  // newest first
  });

  std::vector<RosbagFileInfo> result;
  result.reserve(mine.size());
  for (const auto * row : mine) {
    result.push_back(row->info);
  }
  return result;
}

std::vector<RosbagFileInfo>
InMemoryFaultStorage::get_rosbag_files_by_recording(const std::string & recording_id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;
  for (const auto & row : rosbag_files_) {
    if (row.info.recording_id == recording_id) {
      result.push_back(row.info);
    }
  }
  std::sort(result.begin(), result.end(), [](const RosbagFileInfo & a, const RosbagFileInfo & b) {
    return std::tie(a.fault_code, a.owner) < std::tie(b.fault_code, b.owner);
  });
  return result;
}

bool InMemoryFaultStorage::delete_rosbag_file(const FaultId & id) {
  std::lock_guard<std::mutex> lock(mutex_);

  // ALL recordings of this record. Used by auto_cleanup on clear, where dropping the
  // record's whole black-box history is the intent.
  std::set<std::string> touched;
  const size_t before = rosbag_files_.size();
  for (auto it = rosbag_files_.begin(); it != rosbag_files_.end();) {
    if (it->info.fault_code == id.fault_code && it->info.owner == id.owner) {
      touched.insert(it->info.file_path);
      it = rosbag_files_.erase(it);
    } else {
      ++it;
    }
  }
  if (rosbag_files_.size() == before) {
    return false;
  }

  for (const auto & path : touched) {
    if (path_referenced(path)) {
      continue;  // a sibling record of the burst still holds it
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    // Ignore errors - file may already be deleted
  }
  return true;
}

size_t InMemoryFaultStorage::delete_rosbag_recording(const std::string & recording_id) {
  // Mirrors SqliteFaultStorage: an empty id addresses a set, not a recording.
  if (recording_id.empty()) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  std::set<std::string> touched;
  size_t removed = 0;
  for (auto it = rosbag_files_.begin(); it != rosbag_files_.end();) {
    if (it->info.recording_id == recording_id) {
      touched.insert(it->info.file_path);
      it = rosbag_files_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }

  for (const auto & path : touched) {
    if (path_referenced(path)) {
      continue;
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
  return removed;
}

bool InMemoryFaultStorage::path_referenced(const std::string & file_path) const {
  return std::any_of(rosbag_files_.begin(), rosbag_files_.end(), [&](const RosbagRow & row) {
    return row.info.file_path == file_path;
  });
}

size_t InMemoryFaultStorage::get_total_rosbag_storage_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Sum per bag, not per record: one recording can back a burst of correlated
  // faults, and double-counting it would evict bags that still fit the quota.
  // Rows sharing a path can disagree on size while a recording is being
  // finalised, so take the largest - matching the SQLite backend's MAX() and
  // never under-reporting what is on disk.
  std::map<std::string, size_t> bytes_per_path;
  for (const auto & row : rosbag_files_) {
    auto & bytes = bytes_per_path[row.info.file_path];
    bytes = std::max(bytes, row.info.size_bytes);
  }

  size_t total = 0;
  for (const auto & [path, bytes] : bytes_per_path) {
    total += bytes;
  }
  return total;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<const RosbagRow *> rows;
  rows.reserve(rosbag_files_.size());
  for (const auto & row : rosbag_files_) {
    rows.push_back(&row);
  }

  // Oldest first, seq breaking the created_at_ns ties a burst guarantees - the
  // SQLite side orders by created_at_ns ASC, id ASC for the same reason.
  std::sort(rows.begin(), rows.end(), [](const RosbagRow * a, const RosbagRow * b) {
    return std::tie(a->info.created_at_ns, a->seq) < std::tie(b->info.created_at_ns, b->seq);
  });

  std::vector<RosbagFileInfo> result;
  result.reserve(rows.size());
  for (const auto * row : rows) {
    result.push_back(row->info);
  }
  return result;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<const RosbagRow *> rows;
  for (const auto & row : rosbag_files_) {
    // The entity owns the row when it is among the reporting sources of the row's record.
    auto fault_it = faults_.find(FaultId{row.info.fault_code, row.info.owner});
    if (fault_it == faults_.end()) {
      continue;
    }
    const auto & sources = fault_it->second.reporting_sources;
    if (sources.find(entity_fqn) != sources.end()) {
      rows.push_back(&row);
    }
  }

  std::sort(rows.begin(), rows.end(), [](const RosbagRow * a, const RosbagRow * b) {
    return std::tie(b->info.created_at_ns, b->seq) < std::tie(a->info.created_at_ns, a->seq);  // newest first
  });

  std::vector<RosbagFileInfo> result;
  result.reserve(rows.size());
  for (const auto * row : rows) {
    result.push_back(row->info);
  }
  return result;
}

std::vector<fd::FaultRecord> InMemoryFaultStorage::get_all_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<fd::FaultRecord> result;
  result.reserve(faults_.size());

  for (const auto & [id, state] : faults_) {
    result.push_back(state.to_record());
  }

  return result;
}

std::vector<FaultId> InMemoryFaultStorage::reclassify_healed_as_cleared(const KeepHealedFn & keep_healed) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<FaultId> reclassified;
  for (auto & [id, state] : faults_) {
    if (state.status == fd::fault_status::kHealed && !(keep_healed && keep_healed(id.fault_code))) {
      state.status = fd::fault_status::kCleared;
      reclassified.push_back(id);
    }
  }

  // Reclassified rows must match CLEARED semantics, snapshots included - the SQLite backend drops
  // them here, and a backend that kept them would answer a snapshot query differently for the
  // same sequence of calls. The near-miss series is retained, as it is on clear_fault.
  if (!retain_snapshots_on_clear_ && !reclassified.empty()) {
    const std::set<FaultId> affected(reclassified.begin(), reclassified.end());
    snapshots_.erase(std::remove_if(snapshots_.begin(), snapshots_.end(),
                                    [&affected](const SnapshotData & s) {
                                      return affected.count(FaultId{s.fault_code, s.owner}) > 0;
                                    }),
                     snapshots_.end());
  }

  return reclassified;
}

}  // namespace ros2_medkit_fault_manager
