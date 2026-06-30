// Copyright 2026 mfaferek93, bburda
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

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_fault_manager {

/// A single fault state-transition to record in the audit log.
///
/// `transition` is one of the kTransition* constants below. The remaining
/// fields describe the fault at the moment of the transition; all of them feed
/// the canonical serialization that the hash chain is computed over, so an edit
/// to a stored row that does not also recompute the chain breaks verify().
struct AuditEvent {
  std::string fault_code;
  std::string transition;     ///< one of the kTransition* constants below
  uint8_t severity{0};        ///< severity at the time of the transition
  std::string status;         ///< resulting fault status (e.g. CONFIRMED)
  std::string source_id;      ///< reporting source that drove the transition
  std::string description;    ///< human-readable description
  int64_t occurred_at_ns{0};  ///< wall-clock timestamp of the transition
};

/// Canonical transition kinds. Stored verbatim, so they are part of the hash.
constexpr const char * kTransitionOccurred = "occurred";
constexpr const char * kTransitionConfirmed = "confirmed";
constexpr const char * kTransitionCleared = "cleared";
/// Auto-recovery: a fault reached the healing threshold via PASSED events. Kept
/// distinct from kTransitionCleared so an automatic recovery is not mistaken for
/// a manual clear in the timeline.
constexpr const char * kTransitionHealed = "healed";
/// Audit-log lifecycle markers (CIR (EU) 2024/2690 sec. 3.2: activation /
/// deactivation of logging). Appended directly, independent of the per-fault
/// transition filter, so the log records its own start and stop.
constexpr const char * kTransitionLoggingActivated = "logging_activated";
constexpr const char * kTransitionLoggingDeactivated = "logging_deactivated";
// NOTE: there is deliberately no "ack" kind. The open fault_manager has no
// acknowledge action separate from clearing: ~/clear_fault IS the acknowledge,
// and it is recorded as kTransitionCleared (clear == ack). A separate "ack" kind
// would never be written, so defining it would only mislead readers of the log.

/// One immutable, hash-chained row read back from the audit log.
struct AuditRecord {
  int64_t seq{0};
  AuditEvent event;
  std::string prev_hash;
  std::string record_hash;
};

/// Persisted head of the hash chain.
struct ChainHead {
  int64_t seq{0};           ///< 0 when the chain is empty
  std::string record_hash;  ///< genesis hash when the chain is empty
};

/// Result of verifying the persisted chain.
struct AuditVerifyResult {
  bool ok{true};
  int64_t checked{0};  ///< number of records walked
  int64_t bad_seq{0};  ///< seq of the first offending record (0 if ok)
  std::string error;   ///< human-readable reason when !ok
};

/// Append-only, hash-chained audit log of fault state transitions.
///
/// Each appended row stores `record_hash = sha256(prev_hash + canonical(event))`
/// using OpenSSL's EVP SHA-256 (the same primitive the gateway links). The hash
/// is computed once at insert and never recomputed. A persisted chain head lets
/// the chain resume across process restarts, and rotation seals a segment by
/// persisting an anchor (its final seq + hash) before pruning so the surviving
/// history stays verifiable.
///
/// The table is treated as append-only: this class only ever INSERTs rows (and,
/// on rotation, deletes a sealed prefix). It never UPDATEs an existing record.
/// BEFORE UPDATE / BEFORE DELETE triggers reject out-of-band edits (the guarded
/// rotation prune excepted) as defense-in-depth.
///
/// Threat model: the hash chain is UNKEYED and the head/anchors live in the same
/// writable file. verify() catches edits or deletions that did not also recompute
/// the chain (casual or accidental tampering), but anyone with write access to the
/// file can recompute the whole chain (and drop the triggers) and forge a
/// consistent history. True tamper-proofing needs a key/signature over the head or
/// external anchoring; that is out of scope here.
class FaultAuditLog {
 public:
  /// Open (or create) the audit log database.
  /// @param db_path SQLite path. Use ":memory:" for an in-memory log.
  /// @param retention_max_records Max records to retain before rotation seals
  ///        and prunes the oldest segment. 0 disables rotation (unlimited).
  /// @throws std::runtime_error if the database cannot be opened or initialized.
  explicit FaultAuditLog(const std::string & db_path, int64_t retention_max_records = 0);

  ~FaultAuditLog();

  FaultAuditLog(const FaultAuditLog &) = delete;
  FaultAuditLog & operator=(const FaultAuditLog &) = delete;
  FaultAuditLog(FaultAuditLog &&) = delete;
  FaultAuditLog & operator=(FaultAuditLog &&) = delete;

  /// Append one transition. Computes the chained hash, inserts the row, and
  /// advances the persisted head, atomically.
  /// @return the monotonic seq assigned to the new record.
  int64_t append(const AuditEvent & event);

  /// Walk the persisted chain oldest-first and validate every link.
  AuditVerifyResult verify() const;

  /// Read records oldest-first.
  /// @param limit Max records to return (0 = all).
  /// @param after_seq Only return records with seq > after_seq (0 = from start).
  std::vector<AuditRecord> read(int64_t limit = 0, int64_t after_seq = 0) const;

  /// Current persisted chain head.
  ChainHead head() const;

  /// Number of records currently retained (excludes pruned/sealed rows).
  int64_t record_count() const;

  /// Deterministic canonical serialization of an event at a given seq.
  /// Stable field order so verify is reproducible across processes.
  static std::string canonicalize(int64_t seq, const AuditEvent & event);

  /// Genesis hash used as prev_hash for the very first record.
  static std::string genesis_hash();

  /// SHA-256 of `data` as a lowercase hex string (OpenSSL EVP).
  static std::string sha256_hex(const std::string & data);

  const std::string & db_path() const {
    return db_path_;
  }

 private:
  void initialize_schema();
  /// Read the persisted chain head row (audit_chain_head id=1) straight from the
  /// DB. Returns nullopt when the row is absent. verify() relies on this so a
  /// deleted head row is treated as tampering rather than silently recovered.
  std::optional<ChainHead> read_head_row_locked() const;
  ChainHead load_head_locked() const;
  void store_head_locked(const ChainHead & head_record);
  /// Seal + prune the oldest segment if the retained count exceeds the limit.
  void rotate_if_needed_locked();

  std::string db_path_;
  int64_t retention_max_records_{0};
  sqlite3 * db_{nullptr};
  mutable std::mutex mutex_;
  ChainHead head_;  ///< cached head, kept in sync with the head table
};

}  // namespace ros2_medkit_fault_manager
