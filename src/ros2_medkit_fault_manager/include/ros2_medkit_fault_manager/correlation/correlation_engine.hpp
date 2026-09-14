// Copyright 2026 mfaferek93
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

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/correlation/pattern_matcher.hpp"
#include "ros2_medkit_fault_manager/correlation/types.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {
namespace correlation {

/// Result of processing a fault through the correlation engine
struct ProcessFaultResult {
  /// Whether this fault should be muted (not published as event)
  bool should_mute{false};

  /// Whether this fault was identified as a root cause
  bool is_root_cause{false};

  /// For symptoms: the root cause fault code
  std::string root_cause_code;

  /// The rule ID that matched (if any)
  std::string rule_id;

  /// Delay from root cause in milliseconds (for symptoms)
  uint32_t delay_ms{0};

  /// For auto-cluster: cluster ID if this fault triggered or joined a cluster
  std::string cluster_id;

  /// For auto-cluster: fault codes that should be retroactively muted
  /// When cluster reaches min_count threshold, previous non-representative faults
  /// that were added before threshold was reached need to be muted retroactively
  std::vector<std::string> retroactive_mute_codes;
};

/// Result of clearing a fault
struct ProcessClearResult {
  /// The symptom RECORDS that should be auto-cleared. Records, not codes: a root
  /// cause of one owner must never auto-clear another owner's record of the same
  /// symptom code.
  std::vector<FaultId> auto_cleared_symptoms;
};

/// Information about a muted record (for ListFaults response).
/// Shape unchanged: MutedFaultInfo carries the CODE, so a muted (code, owner) appears
/// on the wire as its code.
struct MutedFaultData {
  std::string fault_code;
  std::string root_cause_code;
  std::string rule_id;
  uint32_t delay_ms{0};
};

/// Information about an active cluster (for ListFaults response)
struct ClusterData {
  std::string cluster_id;
  std::string rule_id;
  std::string rule_name;
  std::string label;
  std::string representative_code;
  std::string representative_severity;
  std::vector<std::string> fault_codes;
  std::chrono::system_clock::time_point first_at;
  std::chrono::system_clock::time_point last_at;
};

/// Main correlation engine
///
/// Processes incoming faults and determines:
/// - Whether they should be muted (symptoms of a root cause)
/// - Whether they are root causes that should collect symptoms
/// - Whether they form part of an auto-detected cluster
///
/// Thread-safe: all public methods can be called from multiple threads.
class CorrelationEngine {
 public:
  /// Create correlation engine from configuration
  /// @param config Correlation configuration (must be enabled and valid)
  explicit CorrelationEngine(const CorrelationConfig & config);

  /// Process an incoming fault record
  ///
  /// Rules match on the fault CODE - a rule says which codes are root causes and which
  /// are symptoms, and says nothing about who reports them. Every RELATION the engine
  /// then forms (root to symptom, muting, cluster membership, auto-clear cascade) is
  /// between records of the SAME owner, so a root cause of owner X never mutes or
  /// auto-clears a symptom of owner Y.
  /// @param id The fault record
  /// @param severity The fault severity (for representative selection)
  /// @param timestamp When the fault occurred
  /// @return Processing result indicating whether to mute, correlations, etc.
  ProcessFaultResult process_fault(const FaultId & id, const std::string & severity,
                                   std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now());

  /// Process a fault record being cleared
  /// @param id The record being cleared
  /// @return Result with the symptom records to auto-clear, all of the same owner
  ProcessClearResult process_clear(const FaultId & id);

  /// Get all currently muted faults
  /// @return List of muted fault data
  std::vector<MutedFaultData> get_muted_faults() const;

  /// Get count of muted faults
  uint32_t get_muted_count() const;

  /// Whether a fault record is currently muted as a symptom.
  /// @param id Record to test
  /// @return True while the record is suppressed by a root cause of the same owner
  bool is_muted(const FaultId & id) const;

  /// Get all active clusters
  /// @return List of cluster data
  std::vector<ClusterData> get_clusters() const;

  /// Get count of active clusters
  uint32_t get_cluster_count() const;

  /// Clean up expired pending root causes and clusters
  /// Called periodically to remove old state
  void cleanup_expired();

 private:
  /// Check if the record's CODE matches a root cause pattern in any hierarchical rule
  /// @return Rule ID if matched, empty optional otherwise
  std::optional<std::string> try_as_root_cause(const std::string & fault_code);

  /// Check if the record is a symptom of a pending root cause OF THE SAME OWNER
  /// @return ProcessFaultResult with correlation info if matched
  std::optional<ProcessFaultResult> try_as_symptom(const FaultId & id, std::chrono::steady_clock::time_point timestamp);

  /// Check if the record's code matches an auto-cluster rule. A cluster holds records
  /// of one owner, so the pending-cluster key carries the owner alongside the rule id.
  /// @return ProcessFaultResult with cluster info if matched
  std::optional<ProcessFaultResult> try_auto_cluster(const FaultId & id, const std::string & severity,
                                                     std::chrono::steady_clock::time_point timestamp);

  /// Generate unique cluster ID
  std::string generate_cluster_id(const std::string & rule_id);

  CorrelationConfig config_;
  std::unique_ptr<PatternMatcher> matcher_;

  /// Active root causes waiting for symptoms
  struct PendingRootCause {
    FaultId fault_id;
    std::string rule_id;
    std::chrono::steady_clock::time_point timestamp;
    uint32_t window_ms;
  };
  std::vector<PendingRootCause> pending_root_causes_;

  /// Mapping from a root cause RECORD to its symptom RECORDS, all of the same owner
  std::map<FaultId, std::vector<FaultId>> root_to_symptoms_;

  /// Muted records (record -> data)
  std::map<FaultId, MutedFaultData> muted_faults_;

  /// Active clusters (cluster_id -> data)
  std::map<std::string, ClusterData> active_clusters_;

  /// Mapping from a record to its cluster ID (for records in clusters)
  std::map<FaultId, std::string> fault_to_cluster_;

  /// Pending cluster with steady_clock timestamp for window tracking
  struct PendingCluster {
    ClusterData data;
    std::string owner;  ///< Every member record of this cluster has this owner
    std::chrono::steady_clock::time_point steady_first_at;
    std::map<std::string, std::string> fault_severities;  ///< fault_code -> severity
  };

  /// Pending clusters being formed ((rule_id, owner) -> cluster data). Keyed by owner
  /// too, so one rule forms one cluster per owner and membership never crosses owners.
  /// Once min_count is reached, moved to active_clusters_
  std::map<std::pair<std::string, std::string>, PendingCluster> pending_clusters_;

  /// Counter for cluster ID generation
  uint64_t cluster_counter_{0};

  mutable std::mutex mutex_;
};

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
