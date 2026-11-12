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

#pragma once

#include <sqlite3.h>

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/core/log_sink.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// SQLite-based fault storage implementation with persistence
/// Thread-safe implementation using mutex protection
class SqliteFaultStorage : public FaultStorage {
 public:
  /// Create SQLite fault storage
  /// @param db_path Path to SQLite database file. Use ":memory:" for in-memory database.
  /// @throws std::runtime_error if database cannot be opened or initialized
  /// @param log_sink Receives storage log lines; must outlive the storage
  SqliteFaultStorage(const std::string & db_path, core::LogSink & log_sink);

  /// Destructor - closes database connection
  ~SqliteFaultStorage() override;

  // Non-copyable, non-movable (owns SQLite connection)
  SqliteFaultStorage(const SqliteFaultStorage &) = delete;
  SqliteFaultStorage & operator=(const SqliteFaultStorage &) = delete;
  SqliteFaultStorage(SqliteFaultStorage &&) = delete;
  SqliteFaultStorage & operator=(SqliteFaultStorage &&) = delete;

  void set_debounce_config(const DebounceConfig & config) override;
  DebounceConfig get_debounce_config() const override;

  bool report_fault_event(const FaultId & id, fd::ReportKind kind, uint8_t severity, const std::string & description,
                          const std::string & source_id, int64_t timestamp_ns, const DebounceConfig & config) override;

  std::vector<FaultEntry> list_fault_entries(bool filter_by_severity, uint8_t severity,
                                             const std::vector<std::string> & statuses) const override;

  std::optional<fd::FaultRecord> get_fault(const FaultId & id) const override;

  std::vector<FaultId> get_ids_by_code(const std::string & fault_code) const override;

  bool clear_fault(const FaultId & id) override;

  size_t size() const override;

  bool contains(const FaultId & id) const override;

  std::vector<FaultId> check_time_based_confirmation(int64_t now_ns) override;

  void set_max_snapshots_per_fault(size_t max_count) override;
  void set_retain_snapshots_on_clear(bool retain) override;
  bool retains_snapshots_on_clear() const override;

  void set_max_rosbags_per_fault(size_t max_count) override;

  void store_snapshot(const SnapshotData & snapshot) override;
  void store_snapshots(const std::vector<SnapshotData> & snapshots) override;
  std::vector<SnapshotData> get_snapshots(const FaultId & id, const std::string & topic_filter = "") const override;
  int64_t get_max_capture_id() const override;

  void store_freeze_frame(const FreezeFrameData & frame) override;
  std::optional<FreezeFrameData> get_freeze_frame(const FaultId & id) const override;

  size_t set_max_near_misses_per_fault(size_t max_count) override;
  std::vector<NearMissRecord> get_near_misses(const FaultId & id) const override;

  void store_rosbag_file(const RosbagFileInfo & info) override;
  void store_rosbag_files(const std::vector<RosbagFileInfo> & infos) override;
  std::optional<RosbagFileInfo> get_rosbag_file(const FaultId & id) const override;
  std::vector<RosbagFileInfo> get_rosbag_files(const FaultId & id) const override;
  std::vector<RosbagFileInfo> get_rosbag_files_by_recording(const std::string & recording_id) const override;
  bool delete_rosbag_file(const FaultId & id) override;
  size_t delete_rosbag_recording(const std::string & recording_id) override;
  size_t delete_rosbag_files(const std::vector<FaultId> & ids) override;
  size_t get_total_rosbag_storage_bytes() const override;
  std::vector<RosbagFileInfo> get_all_rosbag_files() const override;
  std::vector<RosbagFileInfo> list_rosbags_for_entity(const std::string & entity_fqn) const override;
  std::vector<fd::FaultRecord> get_all_faults() const override;
  std::vector<FaultId> reclassify_healed_as_cleared(const KeepHealedFn & keep_healed = {}) override;

  /// Get the database path
  const std::string & db_path() const {
    return db_path_;
  }

 private:
  /// Initialize database schema
  void initialize_schema();

  /// Whether rosbag_files still carries the legacy column-level UNIQUE on
  /// fault_code. Detected from the schema itself (PRAGMA index_list, origin 'u')
  /// rather than from a version counter, so a fresh database, a migrated one and
  /// one migrated by a later release all answer correctly with no bookkeeping.
  bool rosbag_files_has_unique_constraint() const;

  /// Rebuild rosbag_files without the legacy UNIQUE(fault_code), so one fault can
  /// hold several recordings. SQLite cannot drop a column constraint in place and
  /// CREATE TABLE IF NOT EXISTS is a no-op on an existing database, so this is the
  /// documented table-rebuild procedure. Idempotent; no filesystem side effects.
  void migrate_rosbag_files_drop_unique();

  /// Add and backfill recording_id on databases that predate it. Backfill runs in
  /// C++ through rosbag_recording_id() so the basename rule has one implementation.
  void migrate_rosbag_files_add_recording_id();

  /// Move a database whose fault identity is the bare fault_code onto (fault_code, owner).
  ///
  /// Detected per table by PRAGMA table_info lacking `owner`, which is what makes it safe
  /// to re-run on every open. faults and freeze_frames are REBUILT (their identity is a
  /// PRIMARY KEY, which ALTER TABLE cannot change and CREATE TABLE IF NOT EXISTS would
  /// silently skip), while snapshots, rosbag_files and near_misses only gain a column.
  /// A legacy database holds one record per fault code, fed by every source, so every row
  /// becomes the shared record of its code and every child row attaches to it. The rows keep
  /// their reporting_sources as written. All of it runs in one transaction.
  void migrate_faults_add_owner();

  /// Whether @p table exists, asked of sqlite_master. Used to tell a leftover scratch table
  /// apart from a clean start, which DROP TABLE IF EXISTS on its own cannot report.
  bool table_exists(const char * table) const;

  /// Drop one of the migration's scratch tables, warning first when it was really there.
  /// `faults_new` and `freeze_frames_new` are reserved for this procedure, so debris under
  /// those names is always an unfinished rebuild and never somebody's data.
  void drop_scratch_table(const char * table);

  /// Whether @p table already carries the `owner` column. The four tables are created
  /// by four independent CREATE TABLE IF NOT EXISTS statements, so a database can hold
  /// a legacy one next to one this release just created: each is probed on its own.
  bool table_has_owner(const char * table) const;

  /// Whether any row at all still references @p file_path. One recording can back
  /// several records of the same burst, so the bag must only be unlinked once the
  /// last of them is gone. Caller holds mutex_.
  bool path_referenced(const std::string & file_path) const;

  /// store_rosbag_file body without taking mutex_. Caller holds mutex_ and
  /// unlinks the returned replaced-bag path once the row change is durable.
  /// @return file_paths whose last row for this record the per-record cap evicted.
  /// The caller unlinks each only after the commit, and only if path_referenced()
  /// still says nobody holds it.
  std::vector<std::string> store_rosbag_file_locked(const RosbagFileInfo & info);

  /// report_fault_event body without taking mutex_ or opening a transaction. Caller holds mutex_
  /// and wraps the call, so the fault row and any near-miss row commit together.
  bool report_fault_event_locked(const FaultId & id, fd::ReportKind kind, uint8_t severity,
                                 const std::string & description, const std::string & source_id, int64_t timestamp_ns,
                                 const DebounceConfig & config);

  /// Append one entry to the near-miss series and evict the oldest entries beyond
  /// max_near_misses_per_fault_. Caller holds mutex_ and has already written the fault row.
  /// @param id The record that nearly confirmed; it scopes the series
  /// @param occurred_at_ns Timestamp of the report
  /// @param debounce_counter Counter value after the report
  /// @param config Debounce config the report was evaluated against
  /// @param severity Severity carried by the report
  /// @param source_id Source that sent the report
  /// @param resulting_status Fault status after the report was applied
  void record_near_miss_locked(const FaultId & id, int64_t occurred_at_ns, int32_t debounce_counter,
                               const DebounceConfig & config, uint8_t severity, const std::string & source_id,
                               const std::string & resulting_status);

  /// Run a plain SQL statement or throw with the SQLite error. Caller holds mutex_.
  void exec_or_throw(const char * sql);

  /// Serialize vector of strings to JSON array string, the form of faults.reporting_sources.
  static std::string serialize_json_array(const std::vector<std::string> & vec);

  std::string db_path_;
  core::LogSink & log_sink_;
  sqlite3 * db_{nullptr};
  mutable std::mutex mutex_;
  DebounceConfig config_;
  size_t max_snapshots_per_fault_{0};    ///< 0 = unlimited
  size_t max_near_misses_per_fault_{0};  ///< 0 = unlimited
  bool retain_snapshots_on_clear_{false};
  /// Defaults to 1, the pre-#620 behaviour: a new recording replaces the old one.
  /// 0 = unlimited, bounded only by max_total_storage_mb.
  size_t max_rosbags_per_fault_{1};
};

}  // namespace ros2_medkit_fault_manager
