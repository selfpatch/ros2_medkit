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
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_fault_detection/fault_types.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_config.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"
#include "ros2_medkit_fault_manager/core/log_sink.hpp"

namespace ros2_medkit_fault_manager::core {

namespace fd = ros2_medkit::fault_detection;

/// Receives every fault event the core emits. Called with the core mutex held; implementations must not
/// call back into the core (that would deadlock).
class EventSink {
 public:
  EventSink() = default;
  EventSink(const EventSink &) = delete;
  EventSink & operator=(const EventSink &) = delete;
  EventSink(EventSink &&) = delete;
  EventSink & operator=(EventSink &&) = delete;
  virtual ~EventSink() = default;

  virtual void publish(const fd::FaultEventRecord & event) = 0;
};

/// Snapshot and rosbag capture the core triggers. Called with the core mutex held; must not call back into the core.
class CaptureHooks {
 public:
  CaptureHooks() = default;
  CaptureHooks(const CaptureHooks &) = delete;
  CaptureHooks & operator=(const CaptureHooks &) = delete;
  CaptureHooks(CaptureHooks &&) = delete;
  CaptureHooks & operator=(CaptureHooks &&) = delete;
  virtual ~CaptureHooks() = default;

  virtual void on_confirmed(const std::string & code) = 0;
  virtual void on_prefailed(const std::string & code) = 0;
  virtual void on_cleared(const std::string & code) = 0;
  /// Drops the recapture cooldown of @p code. Called for a cleared code and its auto-cleared symptoms
  /// before any of them is processed.
  virtual void forget_cooldown(const std::string & code) = 0;
};

/// Wall-clock source, nanoseconds since the Unix epoch.
class WallClock {
 public:
  WallClock() = default;
  WallClock(const WallClock &) = delete;
  WallClock & operator=(const WallClock &) = delete;
  WallClock(WallClock &&) = delete;
  WallClock & operator=(WallClock &&) = delete;
  virtual ~WallClock() = default;

  virtual int64_t now_ns() const = 0;
};

/// Longest accepted fault code. Equals the bound the gateway enforces on `{fault_code}` routes.
inline constexpr size_t kMaxFaultCodeLength = 256;

/// Empty when @p fault_code is well formed, else the reason it is not.
std::string validate_fault_code(const std::string & fault_code);

/// Response text for a request the fault storage server could not serve.
std::string storage_unavailable(const std::exception & e);

struct ReportOutcome {
  bool accepted{false};
  std::string reject_reason;
};

struct ClearOutcome {
  bool success{false};
  std::string message;
  std::vector<std::string> auto_cleared_codes;
};

/// ListFaults.srv request.
struct ListQuery {
  bool filter_by_severity{false};
  uint8_t severity{0};
  std::vector<std::string> statuses;
  bool include_muted{false};
  bool include_clusters{false};
};

/// MutedFaultInfo.msg field for field.
struct MutedFault {
  std::string fault_code;
  std::string root_cause_code;
  std::string rule_id;
  uint32_t delay_ms{0};
};

/// ClusterInfo.msg field for field. Times are ns since the epoch.
struct ClusterSummary {
  std::string cluster_id;
  std::string rule_id;
  std::string rule_name;
  std::string label;
  std::string representative_code;
  std::string representative_severity;
  std::vector<std::string> fault_codes;
  uint32_t count{0};
  int64_t first_at_ns{0};
  int64_t last_at_ns{0};
};

/// ListFaults.srv response field for field.
struct ListOutcome {
  std::vector<fd::FaultRecord> faults;
  uint32_t muted_count{0};
  std::vector<MutedFault> muted_faults;
  uint32_t cluster_count{0};
  std::vector<ClusterSummary> clusters;
};

/// Snapshot.msg field for field.
struct SnapshotEntry {
  std::string type;  ///< kSnapshotFreezeFrame or kSnapshotRosbag
  std::string name;
  std::string data;
  std::string topic;
  std::string message_type;
  std::string bulk_data_id;
  uint64_t size_bytes{0};
  double duration_sec{0.0};
  std::string format;
  int64_t captured_at_ns{0};
};

inline constexpr const char * kSnapshotFreezeFrame = "freeze_frame";
inline constexpr const char * kSnapshotRosbag = "rosbag";

/// GetFault.srv response field for field.
struct FaultDetail {
  bool success{false};
  std::string error_message;
  fd::FaultRecord fault;
  int64_t first_occurrence_ns{0};
  int64_t last_occurrence_ns{0};
  std::vector<SnapshotEntry> snapshots;
};

/// Report, clear and query sequences of the fault manager, without any ROS handle. report, clear,
/// list, get, list_for_entity and record_transition run under one internal mutex, so calls are
/// serialised as the node's executor serialises them. The sink and the hooks run inside that
/// critical section. The accessors below return internals without the lock; they are for the
/// node's timers until the tick seam moves those into the core.
class FaultManagerCore {
 public:
  FaultManagerCore(FaultManagerConfig config, CoreParts parts, EventSink & events, CaptureHooks & hooks,
                   const WallClock & clock, LogSink & log);
  FaultManagerCore(const FaultManagerCore &) = delete;
  FaultManagerCore & operator=(const FaultManagerCore &) = delete;
  FaultManagerCore(FaultManagerCore &&) = delete;
  FaultManagerCore & operator=(FaultManagerCore &&) = delete;
  ~FaultManagerCore() = default;

  ReportOutcome report(const fd::FaultReport & report);
  ClearOutcome clear(const std::string & code, bool skip_correlation_auto_clear);
  ListOutcome list(const ListQuery & query);
  FaultDetail get(const std::string & code);
  /// Throws FaultStorage::IgnorableConnectionException when the storage is unreachable.
  std::vector<fd::FaultRecord> list_for_entity(const std::string & entity_id);

  /// Appends one audit event. No-op when the log is off, or when only confirmations are logged and
  /// this is not one. Throws when audit_log.fail_closed is set and the append fails.
  void record_transition(const char * transition, const fd::FaultRecord & fault, const std::string & source_id,
                         int64_t occurred_at_ns);

  /// True when @p entity_id equals a source or is the last path segment of one.
  static bool matches_entity(const std::vector<std::string> & reporting_sources, const std::string & entity_id);

  FaultStorage & storage() {
    return *storage_;
  }
  FaultAuditLog * audit_log() const {
    return audit_.get();
  }
  correlation::CorrelationEngine * correlation() {
    return correlation_.get();
  }
  const FaultCodeThresholdResolver * fault_code_resolver() const {
    return fault_code_resolver_.get();
  }
  const DebounceConfig & global_debounce() const {
    return global_debounce_;
  }

  uint64_t audit_dropped_writes() const {
    return audit_dropped_writes_.load(std::memory_order_relaxed);
  }
  bool audit_healthy() const {
    return audit_healthy_.load(std::memory_order_relaxed);
  }

 private:
  /// The first debounce policy seen for a fault code, and who reported it.
  struct DebouncePolicyWitness {
    DebounceConfig config;
    std::string source_id;
    bool warned{false};
  };

  ReportOutcome report_locked(const fd::FaultReport & report);
  ClearOutcome clear_locked(const std::string & code, bool skip_correlation_auto_clear);
  ListOutcome list_locked(const ListQuery & query);
  FaultDetail get_locked(const std::string & code);

  DebounceConfig resolve_config(const std::string & source_id, const std::string & fault_code) const;
  void warn_on_conflicting_debounce_policy(const std::string & fault_code, const std::string & source_id,
                                           const DebounceConfig & resolved);
  void audit_transition(const char * transition, const fd::FaultRecord & fault, const std::string & source_id,
                        int64_t occurred_at_ns);
  void publish_event(const char * event_type, const fd::FaultRecord & fault,
                     const std::vector<std::string> & auto_cleared_codes = {});

  FaultManagerConfig config_;
  std::unique_ptr<FaultStorage> storage_;
  std::unique_ptr<FaultAuditLog> audit_;
  std::unique_ptr<correlation::CorrelationEngine> correlation_;
  DebounceConfig global_debounce_;
  std::unique_ptr<EntityThresholdResolver> entity_resolver_;
  std::unique_ptr<FaultCodeThresholdResolver> fault_code_resolver_;
  EventSink & events_;
  CaptureHooks & hooks_;
  const WallClock & clock_;
  LogSink & log_;
  bool audit_confirmed_only_{false};
  bool audit_fail_closed_{false};

  std::mutex mutex_;
  std::unordered_map<std::string, DebouncePolicyWitness> debounce_policy_witness_;
  std::atomic<uint64_t> audit_dropped_writes_{0};
  std::atomic<bool> audit_healthy_{true};
};

}  // namespace ros2_medkit_fault_manager::core
