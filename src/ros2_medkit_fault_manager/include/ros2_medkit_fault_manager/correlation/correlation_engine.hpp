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

  /// For auto-cluster: records that should be retroactively muted
  /// When cluster reaches min_count threshold, previous non-representative records
  /// that were added before threshold was reached need to be muted retroactively
  std::vector<FaultId> retroactive_mute_ids;
};

/// Result of clearing a fault
struct ProcessClearResult {
  /// The symptom RECORDS that should be auto-cleared. Records, not codes: a clear
  /// acts on the exact records the root cause muted, whoever owns them.
  std::vector<FaultId> auto_cleared_symptoms;
};

/// Information about a muted record (for ListFaults response).
/// Carries both halves of the record identity: two owners muted on one code are two
/// entries of that code, told apart by owner.
struct MutedFaultData {
  std::string fault_code;
  /// Reporting source that owns the muted record. Filled from the map key, so it
  /// cannot drift from the record this entry describes.
  std::string owner;
  std::string root_cause_code;
  std::string rule_id;
  uint32_t delay_ms{0};
};

/// Information about an active cluster (for ListFaults response).
/// A member is a record: fault_owners[i] is the owner of fault_codes[i], and
/// (representative_code, representative_owner) is the representative record.
struct ClusterData {
  std::string cluster_id;
  std::string rule_id;
  std::string rule_name;
  std::string label;
  std::string representative_code;
  std::string representative_owner;
  std::string representative_severity;
  std::vector<std::string> fault_codes;
  std::vector<std::string> fault_owners;
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
  /// Rules match on the fault CODE, whoever owns the record: a root cause of one owner
  /// mutes and auto-clears matching symptom records of any owner. Every RELATION the
  /// engine forms (root to symptom, muting, cluster membership) holds exact records.
  /// @param id The fault record
  /// @param severity The fault severity (for representative selection)
  /// @param timestamp When the fault occurred
  /// @return Processing result indicating whether to mute, correlations, etc.
  ProcessFaultResult process_fault(const FaultId & id, const std::string & severity,
                                   std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now());

  /// Process a fault record being cleared
  /// @param id The record being cleared
  /// @return Result with the symptom records to auto-clear
  ProcessClearResult process_clear(const FaultId & id);

  /// Get all currently muted faults
  /// @return List of muted fault data
  std::vector<MutedFaultData> get_muted_faults() const;

  /// Get count of muted faults
  uint32_t get_muted_count() const;

  /// Whether a fault record is currently muted as a symptom.
  /// @param id Record to test
  /// @return True while the record is suppressed by a root cause
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

  /// Check if the record is a symptom of a pending root cause
  /// @return ProcessFaultResult with correlation info if matched
  std::optional<ProcessFaultResult> try_as_symptom(const FaultId & id, std::chrono::steady_clock::time_point timestamp);

  /// Check if the record's code matches an auto-cluster rule. A cluster holds the records
  /// of any owner that match the rule inside its window.
  /// @return ProcessFaultResult with cluster info if matched
  std::optional<ProcessFaultResult> try_auto_cluster(const FaultId & id, const std::string & severity,
                                                     std::chrono::steady_clock::time_point timestamp);

  /// Generate unique cluster ID
  std::string generate_cluster_id(const std::string & rule_id);

  /// A cluster as the engine keeps it: members and the representative are records.
  struct Cluster {
    std::string cluster_id;
    std::string rule_id;
    std::string rule_name;
    std::string label;
    FaultId representative;
    std::string representative_severity;
    std::vector<FaultId> members;
    std::chrono::system_clock::time_point first_at;
    std::chrono::system_clock::time_point last_at;
  };

  /// Record a re-reported member's severity and update the representative if it changes
  void update_member_severity(const CorrelationRule & rule, Cluster & cluster,
                              std::map<FaultId, std::string> & severities, const FaultId & id,
                              const std::string & severity) const;

  /// Reassign cluster representative from the remaining members and their severities
  void reassign_representative(Cluster & cluster, const std::map<FaultId, std::string> & severities) const;

  static ClusterData to_cluster_data(const Cluster & cluster);

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

  /// Mapping from a root cause RECORD to its symptom RECORDS
  std::map<FaultId, std::vector<FaultId>> root_to_symptoms_;

  /// Muted records (record -> data)
  std::map<FaultId, MutedFaultData> muted_faults_;

  /// Active clusters (cluster_id -> cluster)
  std::map<std::string, Cluster> active_clusters_;

  /// Per-record severities for active clusters (cluster_id -> (record -> severity))
  std::map<std::string, std::map<FaultId, std::string>> active_cluster_severities_;

  /// Mapping from a record to its cluster ID (for records in clusters)
  std::map<FaultId, std::string> fault_to_cluster_;

  /// Pending cluster with steady_clock timestamp for window tracking
  struct PendingCluster {
    Cluster data;
    std::chrono::steady_clock::time_point steady_first_at;
    std::map<FaultId, std::string> fault_severities;  ///< record -> severity
  };

  /// Pending clusters being formed (rule_id -> cluster data).
  /// Once min_count is reached, moved to active_clusters_
  std::map<std::string, PendingCluster> pending_clusters_;

  /// Counter for cluster ID generation
  uint64_t cluster_counter_{0};

  mutable std::mutex mutex_;
};

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
