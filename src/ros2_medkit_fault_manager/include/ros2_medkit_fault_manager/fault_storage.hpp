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

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_fault_manager {

/// Debounce configuration for fault filtering.
///
/// Status lifecycle uses a bounded counter (AUTOSAR-DEM-style) plus a hysteresis latch:
/// FAILED events decrement the counter, PASSED events increment it, and the counter is always
/// clamped to [confirmation_threshold, healing_threshold]. CONFIRMED and HEALED latch - they
/// persist until the counter reaches the opposite threshold, so a single opposite-direction report
/// cannot flip them. One consequence is a latch delay: a fault that becomes active again is not
/// immediately back in the default (CONFIRMED-only) list - it can take up to
/// (healing_threshold - confirmation_threshold) reports to re-confirm. During that window
/// last_occurred still reflects the activity; occurrence_count does not (it only counts
/// the edge that started this occurrence, not every report within it).
struct DebounceConfig {
  /// Confirmation threshold (typically negative). Fault is CONFIRMED when counter <= this value,
  /// and the debounce counter is clamped to this lower bound so a long burst of FAILED events
  /// cannot drive it past confirmation.
  /// Default: -1 (immediate confirmation - first FAILED event confirms the fault).
  /// Set to lower values (e.g., -3) for debounce filtering.
  int32_t confirmation_threshold{-1};

  /// Whether healing is enabled. When true, faults can transition to HEALED status.
  bool healing_enabled{false};

  /// Healing threshold (non-negative; 0 means heal on a single PASSED event). When healing is enabled,
  /// the fault is HEALED once the counter reaches this value. The counter is always clamped to this
  /// upper bound, even when healing is
  /// disabled, so a heal heartbeat cannot drive it off to INT32_MAX; healing_enabled only controls
  /// whether reaching the bound produces a HEALED status.
  /// Default: 3 (3 more PASSED than FAILED events to heal).
  int32_t healing_threshold{3};

  /// Whether CRITICAL severity bypasses debounce and confirms immediately.
  bool critical_immediate_confirm{true};

  /// Time-based auto-confirmation. If > 0, PREFAILED faults older than this are auto-confirmed.
  /// 0.0 = disabled.
  double auto_confirm_after_sec{0.0};
};

/// Clamp the debounce counter into [confirmation_threshold, healing_threshold].
int32_t clamp_debounce_counter(int32_t counter, const DebounceConfig & config);

/// Compute the debounce status from the counter and the current status (the current status drives
/// the CONFIRMED/HEALED hysteresis latch). Does NOT apply the CRITICAL immediate-confirm bypass -
/// callers handle that. This is the single source of truth shared by both storage backends.
std::string compute_debounce_status(int32_t counter, const std::string & current_status, const DebounceConfig & config);

/// Whether a just-applied report counts as a near miss: a FAILED report that moved the debounce
/// counter without leaving the fault CONFIRMED - the fault nearly happened. PASSED reports move the
/// counter in the healing direction (the fault receding), so they never qualify. Single source of
/// truth shared by both storage backends.
/// @param is_failed_event Whether the report was FAILED (as opposed to PASSED)
/// @param resulting_status The fault status after the report was applied
bool is_near_miss(bool is_failed_event, const std::string & resulting_status);

/// Validate a (merged) debounce config in place, enforcing confirmation_threshold < 0 <= healing_threshold
/// (healing_threshold == 0 means heal on a single PASSED event). Offending fields are reset to safe
/// defaults (-1 / 3). Returns true if the config was already valid.
bool sanitize_debounce_config(DebounceConfig & config);

/// Identity of one fault record: a fault code plus the reporting source that owns it.
///
/// The owner is the source_id the ReportFault call carried. Two sources reporting one
/// fault_code are two records, and every piece of per-fault state (status, debounce
/// counter, occurrence count, timestamps, severity, freeze frame, snapshots, near
/// misses, rosbag links) belongs to one record. A clear or a heal driven by one owner
/// never touches another owner's record of the same code.
///
/// An aggregate on purpose: FaultId{code, owner} is the only way to build one, so no
/// call site can silently pass a bare code where a record identity is required.
struct FaultId {
  std::string fault_code;
  std::string owner;

  bool operator==(const FaultId & other) const {
    return fault_code == other.fault_code && owner == other.owner;
  }

  /// Ordered by code first, so the records of one code are contiguous in a map and
  /// get_faults_by_code is a range scan rather than a full sweep.
  bool operator<(const FaultId & other) const {
    return std::tie(fault_code, owner) < std::tie(other.fault_code, other.owner);
  }
};

/// Internal fault state of one record, stored in memory
struct FaultState {
  std::string fault_code;
  /// Reporting source that owns this record. Half of the record identity.
  std::string owner;
  uint8_t severity{0};
  std::string description;
  rclcpp::Time first_occurred;
  rclcpp::Time last_occurred;
  uint32_t occurrence_count{0};  ///< Count of genuine occurrences (new fault + each re-raise after CLEARED)
  std::string status;

  // Debounce state (internal, not exposed in Fault.msg)
  int32_t debounce_counter{0};      ///< FAILED decrements (-1), PASSED increments (+1)
  rclcpp::Time last_failed_time{};  ///< Timestamp of last FAILED event
  rclcpp::Time last_passed_time{};  ///< Timestamp of last PASSED event

  /// Convert to ROS 2 message
  ros2_medkit_msgs::msg::Fault to_msg() const;
};

/// Event type alias for convenience
using EventType = ros2_medkit_msgs::srv::ReportFault::Request;

/// Snapshot data captured when a fault is confirmed
/// One captured topic value. A confirmation writes one of these PER TOPIC, and
/// all of them share a capture_id: they are one reading of the machine taken at
/// one moment and only mean anything together.
struct SnapshotData {
  std::string fault_code;
  std::string owner;  ///< Reporting source that owns the record this reading belongs to
  std::string topic;
  std::string message_type;
  std::string data;  ///< JSON-encoded message data
  int64_t captured_at_ns{0};
  /// Groups the rows of one capture. Monotonic within a fault manager process;
  /// 0 on rows written before the field existed, which read as one legacy set.
  int64_t capture_id{0};
};

/// Compact freeze-frame captured when a fault confirms: a single JSON object mapping
/// each captured topic to its latest value at confirmation time. Unlike per-topic
/// snapshots, a freeze-frame is keyed by the record identity (one row per
/// (fault_code, owner) pair) and is RETAINED across clear_fault, so the confirmed-state
/// record persists after acknowledgement. A row exists only for fault codes with a
/// configured capture set. A fault code with no capture configured gets no row at all
/// (lookup returns nullopt, never an empty {}).
struct FreezeFrameData {
  std::string fault_code;
  std::string owner;  ///< Reporting source that owns the record this frame belongs to
  std::string data;   ///< Compact JSON object: {"<topic>": <value>, ...}
  int64_t captured_at_ns{0};
};

/// Derive a recording's public identity from its bag path: the directory basename,
/// `fault_<CODE>_<millis>`. Faults of one burst share a recording and therefore share
/// this id.
///
/// Safe as a URL path segment because fault codes are validated to
/// `[A-Za-z0-9_.-]` with no `..` before a bag is ever named after one, which makes
/// that validation load-bearing for a second reason. The gateway reads the same value
/// off the wire with its own copy of this rule; keep the two in step.
std::string rosbag_recording_id(const std::string & file_path);

/// One entry of the near-miss series of one fault record.
///
/// A near miss is a FAILED report that moved the debounce counter WITHOUT the fault
/// ending up CONFIRMED - the fault nearly happened. PASSED reports move the counter
/// too, but in the healing direction (the fault receding), so they are not near misses.
/// This is the debounce sense of the term and is unrelated to any scoring sense used
/// elsewhere in the product.
///
/// The series is append-only: one entry per qualifying report, never updated in place,
/// and RETAINED across clear_fault, because acknowledging a fault cycle must not erase
/// the record of how often that record approached confirmation. It is bounded per
/// record (see set_max_near_misses_per_fault) and evicts the OLDEST entries first, so a
/// long-running appliance keeps the recent series rather than freezing it at boot.
struct NearMissRecord {
  std::string fault_code;
  int64_t occurred_at_ns{0};    ///< Timestamp of the report that moved the counter
  int32_t debounce_counter{0};  ///< Counter value AFTER this report

  /// Confirmation threshold the report was evaluated against. The counter it belongs to is
  /// this record's own, so with per-entity overrides both are the reporting source's and the
  /// value is the distance to confirmation for the record.
  int32_t confirmation_threshold{0};

  uint8_t severity{0};    ///< Severity carried by the report
  std::string source_id;  ///< Reporting source, which is the record's owner

  /// Fault status after the report was applied. Never CONFIRMED - that is what makes the report a
  /// near miss. It separates a counter climbing from a resting state (PREFAILED) from one walking
  /// back down under the HEALED latch, which is on its way to a fault that does confirm. Without
  /// it the two are indistinguishable and the series cannot answer how often the code approached
  /// confirmation WITHOUT becoming a fault.
  std::string resulting_status;
};

/// One row = one LINK: a fault claiming a recording. Several faults of a burst link to
/// one recording (same file_path, same recording_id), and one fault can link to several
/// recordings over time. Bytes are owned by file_path, not by the row: a bag is unlinked
/// only when its last row goes.
struct RosbagFileInfo {
  std::string fault_code;
  std::string owner;         ///< Reporting source that owns the record holding this link
  std::string recording_id;  ///< Basename of file_path; shared by every row of a burst
  std::string file_path;
  std::string format;        ///< "sqlite3" or "mcap"
  double duration_sec{0.0};  ///< Total duration of recorded data
  size_t size_bytes{0};      ///< File size in bytes
  int64_t created_at_ns{0};  ///< Timestamp when bag was created
};

/// Abstract interface for fault storage backends
class FaultStorage {
 public:
  virtual ~FaultStorage() = default;

  /// Set debounce configuration
  virtual void set_debounce_config(const DebounceConfig & config) = 0;

  /// Get current debounce configuration
  virtual DebounceConfig get_debounce_config() const = 0;

  /// Report a fault event (FAILED or PASSED)
  /// @param fault_code Global fault identifier
  /// @param event_type EVENT_FAILED (0) or EVENT_PASSED (1)
  /// @param severity Fault severity level (only used for FAILED events)
  /// @param description Human-readable description (only used for FAILED events)
  /// @param source_id Reporting source identifier. Together with fault_code it names the
  ///        record this event applies to, and it creates that record when none exists.
  /// @param timestamp Current time for tracking
  /// @param config Debounce configuration to apply for this event (resolved per-entity by the node)
  /// @return true if this is a new occurrence (new record or reactivated CLEARED record),
  ///         false if an existing active record was updated
  virtual bool report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                  const std::string & description, const std::string & source_id,
                                  const rclcpp::Time & timestamp, const DebounceConfig & config) = 0;

  /// Get faults matching filter criteria
  /// @param filter_by_severity Whether to filter by severity
  /// @param severity Severity level to filter (if filter_by_severity is true)
  /// @param statuses List of statuses to include (empty = CONFIRMED only)
  /// @return Vector of matching faults
  virtual std::vector<ros2_medkit_msgs::msg::Fault> list_faults(bool filter_by_severity, uint8_t severity,
                                                                const std::vector<std::string> & statuses) const = 0;

  /// Get a single fault record
  /// @param id The record to look up
  /// @return The fault if found, nullopt otherwise
  virtual std::optional<ros2_medkit_msgs::msg::Fault> get_fault(const FaultId & id) const = 0;

  /// Every record carrying @p fault_code, one per owning reporting source.
  ///
  /// The resolution step behind an unscoped service call: with exactly one record the
  /// call applies to it, with several it is ambiguous and the owners are the answer.
  /// Returns them ordered by owner, so an ambiguity message is stable.
  /// @param fault_code The fault code to look up
  /// @return Every record carrying the code, empty when none does
  virtual std::vector<ros2_medkit_msgs::msg::Fault> get_faults_by_code(const std::string & fault_code) const = 0;

  /// Clear one fault record (manual acknowledgment). Drops that record's per-topic snapshots.
  /// The freeze-frame and the near-miss series are RETAINED, because they outlive a single fault
  /// cycle and cannot be reconstructed afterwards. Another owner's record of the same code is
  /// untouched, snapshots included.
  /// @param id The record to clear
  /// @return true if the record was found and cleared, false if not found
  virtual bool clear_fault(const FaultId & id) = 0;

  /// Get total number of stored fault records
  virtual size_t size() const = 0;

  /// Check if a fault record exists
  virtual bool contains(const FaultId & id) const = 0;

  /// Check and confirm PREFAILED records that have been pending too long (time-based confirmation)
  /// @param current_time Current timestamp for age calculation
  /// @return The records that were confirmed by this call (so the caller can audit each).
  virtual std::vector<FaultId> check_time_based_confirmation(const rclcpp::Time & current_time) = 0;

  /// Set maximum snapshots per fault record (0 = unlimited)
  virtual void set_max_snapshots_per_fault(size_t /*max_count*/) {
  }

  /// Cap on RECORDINGS retained per fault record (0 = unlimited).
  ///
  /// Enforced inside store_rosbag_file(s), atomically with the insert: past the cap
  /// the fault's OLDEST recordings lose their row, and a bag whose last referencing
  /// row goes is unlinked once the store is durable.
  ///
  /// Keep-newest, the same direction as set_max_snapshots_per_fault. The two caps
  /// differ in unit, not in policy: that one evicts whole capture SETS, this one
  /// evicts RECORDINGS. A bag is evidence about a machine you are about to inspect,
  /// so the recent one wins; and keep-newest at 1 is exactly the pre-#620
  /// behaviour, which is what makes the default a no-op.
  virtual void set_max_rosbags_per_fault(size_t /*max_count*/) {
  }

  /// Whether acknowledging a fault keeps the value snapshots it captured.
  ///
  /// Off by default, which is the historical behaviour: clear_fault deletes them.
  /// Turn it on together with a rosbag history, or acknowledging leaves the fault
  /// holding recordings whose matching readings are gone - evidence that no longer
  /// lines up. Growth stays bounded by the per-fault cap either way, so clearing
  /// was never the only thing holding the table down.
  virtual void set_retain_snapshots_on_clear(bool /*retain*/) {
  }

  /// What set_retain_snapshots_on_clear was last given. Callers that write evidence
  /// need it to tell "the acknowledgement deleted these on purpose" from "these
  /// were meant to survive it".
  virtual bool retains_snapshots_on_clear() const {
    return false;
  }

  /// Store one capture as a unit.
  ///
  /// The per-fault cap applies to whole capture sets: past it the OLDEST set is
  /// dropped entire, instead of the newest capture being truncated topic by topic
  /// as it is written. Writing row by row is what produced freeze frames with
  /// some topics silently missing and nothing on the wire saying which.
  /// @param snapshots Every row of one capture; they share a capture_id
  virtual void store_snapshots(const std::vector<SnapshotData> & snapshots) {
    for (const auto & snapshot : snapshots) {
      store_snapshot(snapshot);
    }
  }

  /// Store a snapshot captured when a fault was confirmed
  /// @param snapshot The snapshot data to store
  virtual void store_snapshot(const SnapshotData & snapshot) = 0;

  /// Get snapshots of one fault record, NEWEST capture set first.
  ///
  /// Ordered by capture_id descending, then captured_at_ns descending, in every
  /// backend. Readers fold rows into a per-topic map, so insertion order would let
  /// an older capture's values win on one backend and not the other.
  /// @param id The record to get snapshots for
  /// @param topic_filter Optional topic filter (empty = all topics)
  /// @return Vector of snapshots of the record
  virtual std::vector<SnapshotData> get_snapshots(const FaultId & id, const std::string & topic_filter = "") const = 0;

  /// Highest capture_id any stored snapshot holds, across every fault (0 when none).
  ///
  /// Capture ids are minted by a process-local counter, so a restart would otherwise
  /// hand out ids BELOW the ones already on disk: the eviction that protects
  /// MAX(capture_id) would then guard an old set and drop the one just written.
  /// SnapshotCapture seeds its counter from this at construction.
  virtual int64_t get_max_capture_id() const {
    return 0;
  }

  /// Store the compact freeze-frame captured for a fault record (JSON dict of topic values).
  /// Keyed by the record identity carried on @p frame: a later capture for the same record
  /// replaces the frame, and another owner's record of the same code keeps its own. The frame
  /// is retained across clear_fault so the confirmed-state record survives acknowledgement.
  /// Storage is bounded by the number of distinct records (one row per record, replaced in
  /// place), and rows are never evicted. Records themselves are never deleted (clear_fault only
  /// flips status), so there is currently no delete hook to tie eviction to.
  /// @param frame The freeze-frame to store, carrying fault_code and owner
  virtual void store_freeze_frame(const FreezeFrameData & frame) = 0;

  /// Get the freeze-frame captured for a fault record, if any.
  /// @param id The record to look up
  /// @return The freeze-frame if one was captured, nullopt otherwise (including records
  ///         with no capture configured, which never get a row)
  virtual std::optional<FreezeFrameData> get_freeze_frame(const FaultId & id) const = 0;

  /// Set the maximum number of near-miss entries retained per fault record.
  ///
  /// Entries beyond the bound are evicted oldest-first, including entries already stored when the
  /// bound is applied. 0 means unlimited, and so does any bound larger than the storage backend
  /// can express.
  ///
  /// @return How many already-stored entries this call evicted. A bound applied by mistake
  ///         deletes history that cannot be recovered, and the storage layer has no logger, so
  ///         the caller is the one that can report it.
  virtual size_t set_max_near_misses_per_fault(size_t /*max_count*/) {
    return 0;
  }

  /// Get the near-miss series of one fault record, oldest entry first.
  /// The series survives clear_fault. An unknown or never-near-missed record returns empty.
  /// @param id The record to look up
  /// @return The retained near-miss entries in chronological order
  virtual std::vector<NearMissRecord> get_near_misses(const FaultId & id) const = 0;

  /// Store rosbag file metadata for a fault record
  /// @param info The rosbag file info to store, carrying fault_code and owner (replaces any
  ///        existing link between that record and the same file_path)
  virtual void store_rosbag_file(const RosbagFileInfo & info) = 0;

  /// Store one row per record of a burst that shares a recording.
  ///
  /// Implementations MUST be all-or-nothing. The caller treats a throw as "no row
  /// was written" and discards the recording, so a batch that stored some rows and
  /// then threw would leave those rows naming a bag that has just been removed:
  /// unreadable for good, and still charged against the storage quota, which sums
  /// rows. Both in-tree backends satisfy this - SQLite through a transaction, the
  /// in-memory one by publishing the whole batch at once.
  ///
  /// The default below does NOT satisfy it and exists only so a backend that cannot
  /// fail per row keeps compiling. Override it in anything that can.
  ///
  /// @param infos The rows to store (typically all pointing at one file_path)
  virtual void store_rosbag_files(const std::vector<RosbagFileInfo> & infos) {
    for (const auto & info : infos) {
      store_rosbag_file(info);
    }
  }

  /// The MOST RECENT recording of a fault record, or nullopt.
  ///
  /// A record can hold several recordings, so "the" recording is a choice: newest
  /// wins, because a black box is evidence about the machine you are about to
  /// inspect. Implementations must order deterministically - an unordered pick
  /// serves an arbitrary recording, which no test catches reliably.
  /// @param id The record to get rosbag for
  /// @return Rosbag file info if exists, nullopt otherwise
  virtual std::optional<RosbagFileInfo> get_rosbag_file(const FaultId & id) const = 0;

  /// Every recording of a fault record, newest first.
  virtual std::vector<RosbagFileInfo> get_rosbag_files(const FaultId & id) const = 0;

  /// Every row of one recording - one per record the recording covers. Backs the
  /// bulk-data download and the entity authorization scope check, both of which
  /// start from a recording id and need the faults behind it.
  virtual std::vector<RosbagFileInfo> get_rosbag_files_by_recording(const std::string & recording_id) const = 0;

  /// Delete one whole recording: every fault's link to it, and the bag. This is the
  /// right unit for quota eviction and for a bag that has vanished from disk - both
  /// are facts about the recording, not about one fault that happens to reference it.
  /// @return number of rows removed
  virtual size_t delete_rosbag_recording(const std::string & recording_id) = 0;

  /// Delete rosbag rows and the actual file for one fault record. Records from one
  /// burst can share a recording, and the file is unlinked only with the last row
  /// that references it.
  /// @param id The record to delete rosbags for
  /// @return true if at least one row was deleted, false if none was found
  virtual bool delete_rosbag_file(const FaultId & id) = 0;

  /// Delete the rows of several records (typically the whole burst behind one
  /// recording). Backends with real transactions (SQLite) remove the rows
  /// atomically and unlink the file only after the commit, so a crash mid-delete
  /// never leaves a row pointing at a removed bag. Default: plain loop.
  /// @param ids The records to delete rosbag rows for
  /// @return Number of records for which rows were actually deleted
  virtual size_t delete_rosbag_files(const std::vector<FaultId> & ids) {
    size_t deleted = 0;
    for (const auto & id : ids) {
      if (delete_rosbag_file(id)) {
        ++deleted;
      }
    }
    return deleted;
  }

  /// Get total size of all stored rosbag files in bytes, counting a shared
  /// recording once regardless of how many faults reference it
  /// @return Total size in bytes
  virtual size_t get_total_rosbag_storage_bytes() const = 0;

  /// Get all rosbag files ordered by creation time (oldest first)
  /// @return Vector of rosbag file info
  virtual std::vector<RosbagFileInfo> get_all_rosbag_files() const = 0;

  /// Get rosbags of every record owned by an entity
  /// @param entity_fqn The entity's fully qualified name to filter by
  /// @return Vector of rosbag file info for records this entity owns
  virtual std::vector<RosbagFileInfo> list_rosbags_for_entity(const std::string & entity_fqn) const = 0;

  /// Get all stored fault records regardless of status (for filtering)
  /// @return Vector of all records in storage
  virtual std::vector<ros2_medkit_msgs::msg::Fault> get_all_faults() const = 0;

  /// One-time startup cleanup: reclassify HEALED records as CLEARED. Called when healing is
  /// disabled, so a HEALED row left by a previous (healing-enabled) run does not behave
  /// inconsistently under the latch. Default is a no-op (in-memory storage starts empty).
  /// @return the reclassified records, so the caller can audit each transition. Identities,
  ///         not codes: one code can hold a HEALED record for one owner and an untouched
  ///         CONFIRMED one for another, and auditing by code would claim both moved.
  virtual std::vector<FaultId> reclassify_healed_as_cleared() {
    return {};
  }

 protected:
  FaultStorage() = default;
  FaultStorage(const FaultStorage &) = default;
  FaultStorage & operator=(const FaultStorage &) = default;
  FaultStorage(FaultStorage &&) = default;
  FaultStorage & operator=(FaultStorage &&) = default;
};

/// Thread-safe in-memory fault storage implementation
class InMemoryFaultStorage : public FaultStorage {
 public:
  InMemoryFaultStorage() = default;

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

  void store_snapshot(const SnapshotData & snapshot) override;
  void store_snapshots(const std::vector<SnapshotData> & snapshots) override;
  std::vector<SnapshotData> get_snapshots(const FaultId & id, const std::string & topic_filter = "") const override;
  int64_t get_max_capture_id() const override;

  void store_freeze_frame(const FreezeFrameData & frame) override;
  std::optional<FreezeFrameData> get_freeze_frame(const FaultId & id) const override;

  void set_max_rosbags_per_fault(size_t max_count) override;

  size_t set_max_near_misses_per_fault(size_t max_count) override;
  std::vector<NearMissRecord> get_near_misses(const FaultId & id) const override;

  void store_rosbag_file(const RosbagFileInfo & info) override;
  /// All-or-nothing, as the base class requires: the batch is built beside the live
  /// map and swapped in, so a throw leaves the store exactly as it was.
  void store_rosbag_files(const std::vector<RosbagFileInfo> & infos) override;
  std::optional<RosbagFileInfo> get_rosbag_file(const FaultId & id) const override;
  std::vector<RosbagFileInfo> get_rosbag_files(const FaultId & id) const override;
  std::vector<RosbagFileInfo> get_rosbag_files_by_recording(const std::string & recording_id) const override;
  bool delete_rosbag_file(const FaultId & id) override;
  size_t delete_rosbag_recording(const std::string & recording_id) override;
  size_t get_total_rosbag_storage_bytes() const override;
  std::vector<RosbagFileInfo> get_all_rosbag_files() const override;
  std::vector<RosbagFileInfo> list_rosbags_for_entity(const std::string & entity_fqn) const override;
  std::vector<ros2_medkit_msgs::msg::Fault> get_all_faults() const override;
  std::vector<FaultId> reclassify_healed_as_cleared() override;

 private:
  /// Update fault status based on debounce counter and given config
  void update_status(FaultState & state, const DebounceConfig & config);

  /// Append one entry to the near-miss series for @p state and evict the oldest entries beyond
  /// max_near_misses_per_fault_. Caller holds mutex_ and has already applied the report to @p state.
  void record_near_miss(const FaultState & state, const DebounceConfig & config, uint8_t severity,
                        const std::string & source_id, const rclcpp::Time & timestamp);

  /// Whether a fault other than @p fault_code still references @p file_path.
  /// One recording can back several faults of the same burst, so the bag must
  /// only be unlinked once the last of them is gone. Caller holds mutex_.

  /// Whether any row at all still references @p file_path. Caller holds mutex_.
  bool path_referenced(const std::string & file_path) const;

  mutable std::mutex mutex_;
  std::map<FaultId, FaultState> faults_;
  std::vector<SnapshotData> snapshots_;
  std::map<FaultId, FreezeFrameData> freeze_frames_;  ///< record -> freeze-frame (retained across clear)
  /// One entry per LINK, mirroring the flat SQLite table rather than a map keyed by
  /// the record - a record holds several recordings now, and a recording several
  /// records.
  ///
  /// `seq` is the in-memory twin of SQLite's autoincrement id and is load-bearing,
  /// not decoration: a burst stamps ONE created_at_ns across every row it writes, so
  /// ties are guaranteed. SQLite breaks them by id; without seq the two backends
  /// would order differently and the parity tests would go flaky instead of failing
  /// honestly.
  struct RosbagRow {
    RosbagFileInfo info;
    uint64_t seq{0};
  };
  std::vector<RosbagRow> rosbag_files_;
  uint64_t rosbag_seq_{0};
  /// Defaults to 1, the pre-#620 behaviour: a new recording replaces the old one.
  /// A backend constructed directly (tests, embedders) therefore behaves exactly as
  /// it always did until someone opts into a history. 0 = unlimited.
  size_t max_rosbags_per_fault_{1};
  std::map<FaultId, std::vector<NearMissRecord>> near_misses_;  ///< record -> series (retained across clear)
  DebounceConfig config_;
  size_t max_snapshots_per_fault_{0};  ///< 0 = unlimited
  bool retain_snapshots_on_clear_{false};
  size_t max_near_misses_per_fault_{0};  ///< 0 = unlimited
};

}  // namespace ros2_medkit_fault_manager
