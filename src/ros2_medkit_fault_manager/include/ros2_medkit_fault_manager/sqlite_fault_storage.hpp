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

#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// Owner given to a record migrated out of a database that recorded no readable
/// reporting source.
///
/// A migrated record is never left with an empty owner. It would be unreachable
/// through the source_id every single-record service takes, and it would leave the
/// child backfill looking for work on every open. The empty owner means one thing
/// only, a child row (freeze frame, snapshot, rosbag link) not yet assigned to a
/// record, and keeping that meaning unambiguous is what this constant is for.
inline constexpr const char * kLegacyOwner = "legacy";

/// SQLite-based fault storage implementation with persistence
/// Thread-safe implementation using mutex protection
class SqliteFaultStorage : public FaultStorage {
 public:
  /// Create SQLite fault storage
  /// @param db_path Path to SQLite database file. Use ":memory:" for in-memory database.
  /// @throws std::runtime_error if database cannot be opened or initialized
  explicit SqliteFaultStorage(const std::string & db_path);

  /// Destructor - closes database connection
  ~SqliteFaultStorage() override;

  // Non-copyable, non-movable (owns SQLite connection)
  SqliteFaultStorage(const SqliteFaultStorage &) = delete;
  SqliteFaultStorage & operator=(const SqliteFaultStorage &) = delete;
  SqliteFaultStorage(SqliteFaultStorage &&) = delete;
  SqliteFaultStorage & operator=(SqliteFaultStorage &&) = delete;

  void set_debounce_config(const DebounceConfig & config) override;
  DebounceConfig get_debounce_config() const override;

  bool report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                          const std::string & description, const std::string & source_id,
                          const rclcpp::Time & timestamp, const DebounceConfig & config) override;

  std::vector<ros2_medkit_msgs::msg::Fault> list_faults(bool filter_by_severity, uint8_t severity,
                                                        const std::vector<std::string> & statuses) const override;

  std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const FaultId & id) const override;

  std::vector<ros2_medkit_msgs::msg::Fault> get_faults_by_code(const std::string & fault_code) const override;

  bool clear_fault(const FaultId & id) override;

  size_t size() const override;

  bool contains(const FaultId & id) const override;

  std::vector<FaultId> check_time_based_confirmation(const rclcpp::Time & current_time) override;

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
  std::vector<ros2_medkit_msgs::msg::Fault> get_all_faults() const override;
  std::vector<FaultId> reclassify_healed_as_cleared() override;

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
  /// silently skip), while snapshots and rosbag_files only gain a column. The fault owner
  /// comes from the legacy row's first reporting source, empty when none can be read: that
  /// is the only per-source fact the old schema holds, so a row with several sources folds
  /// onto its first one rather than inventing per-source counters, statuses and timestamps
  /// that were never recorded.
  ///
  /// It never refuses to finish. A row whose content cannot be read leaves an empty owner
  /// and a log line, because a migration that aborts on a row takes the whole database with
  /// it: the rebuild rolls back and the next open reaches the same row again, forever.
  void migrate_faults_add_owner();

  /// Copy every legacy `faults` row into `faults_new`, deriving the owner in C++.
  ///
  /// In C++ rather than one INSERT ... SELECT because the derivation must not depend on the
  /// column being valid JSON, and SQL's json_extract raises rather than returning NULL on
  /// text it cannot parse. Caller holds the migration transaction.
  void copy_legacy_fault_rows();

  /// Give every ownerless child row the owner of its fault, where that is unambiguous.
  ///
  /// Runs on every open, not only when a column was just added: a child table can be left
  /// with empty owners by an open interrupted between the faults rebuild and the backfill,
  /// or by a database whose child table gained the column while faults was still legacy.
  /// Such a row is invisible to every (code, owner) read, so leaving it is losing evidence.
  /// A code carrying several owners is NOT resolved - guessing would hand one owner another
  /// owner's evidence - and the row keeps its empty owner with a log line naming it.
  void backfill_child_owners();

  /// Whether any child row is waiting for an owner the database can actually prove, so the
  /// backfill above has work to do. Deliberately the backfill's own condition and not just
  /// "owner is empty": a row whose code has several owners or none can never be assigned,
  /// and treating it as pending would re-enter the write transaction on every open forever.
  /// Tables without the column yet are skipped: the migration adds it.
  bool has_assignable_child_rows() const;

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
  bool report_fault_event_locked(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                 const std::string & description, const std::string & source_id,
                                 const rclcpp::Time & timestamp, const DebounceConfig & config);

  /// Append one entry to the near-miss series and evict the oldest entries beyond
  /// max_near_misses_per_fault_. Caller holds mutex_ and has already written the fault row.
  /// @param fault_code The fault code that nearly confirmed
  /// @param occurred_at_ns Timestamp of the report
  /// @param debounce_counter Counter value after the report
  /// @param config Debounce config the report was evaluated against
  /// @param severity Severity carried by the report
  /// @param source_id Reporting source, which is the record's owner and scopes the series
  /// @param resulting_status Fault status after the report was applied
  void record_near_miss_locked(const std::string & fault_code, int64_t occurred_at_ns, int32_t debounce_counter,
                               const DebounceConfig & config, uint8_t severity, const std::string & source_id,
                               const std::string & resulting_status);

  /// Run a plain SQL statement or throw with the SQLite error. Caller holds mutex_.
  void exec_or_throw(const char * sql);

  /// Serialize vector of strings to JSON array string. The faults table keeps
  /// reporting_sources as the serialized form of owner, so a reader of the raw table
  /// still sees the field it always saw.
  static std::string serialize_json_array(const std::vector<std::string> & vec);

  std::string db_path_;
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
