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

#include "ros2_medkit_fault_manager/correlation/correlation_engine.hpp"

#include <algorithm>
#include <sstream>

namespace ros2_medkit_fault_manager {
namespace correlation {

CorrelationEngine::CorrelationEngine(const CorrelationConfig & config)
  : config_(config), matcher_(std::make_unique<PatternMatcher>(config.patterns)) {
}

ProcessFaultResult CorrelationEngine::process_fault(const FaultId & id, const std::string & severity,
                                                    std::chrono::steady_clock::time_point timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Rules match on the CODE; every relation formed below is between records of the
  // same owner.
  const std::string & fault_code = id.fault_code;
  ProcessFaultResult result;

  // First, clean up expired entries
  // (This could also be done periodically via cleanup_expired())
  auto now = std::chrono::steady_clock::now();
  pending_root_causes_.erase(
      std::remove_if(pending_root_causes_.begin(), pending_root_causes_.end(),
                     [now](const PendingRootCause & prc) {
                       auto elapsed =
                           std::chrono::duration_cast<std::chrono::milliseconds>(now - prc.timestamp).count();
                       return elapsed > static_cast<int64_t>(prc.window_ms);
                     }),
      pending_root_causes_.end());

  // Check if this record is a symptom of an existing root cause of the same owner
  auto symptom_result = try_as_symptom(id, timestamp);
  if (symptom_result) {
    return *symptom_result;
  }

  // Check if this fault is a root cause
  auto root_cause_rule = try_as_root_cause(fault_code);
  if (root_cause_rule) {
    result.is_root_cause = true;
    result.rule_id = *root_cause_rule;

    // Find the rule to get window_ms
    for (const auto & rule : config_.rules) {
      if (rule.id == *root_cause_rule) {
        // Add to pending root causes
        PendingRootCause prc;
        prc.fault_id = id;
        prc.rule_id = rule.id;
        prc.timestamp = timestamp;
        prc.window_ms = rule.window_ms;
        pending_root_causes_.push_back(prc);

        // Initialize symptom list
        root_to_symptoms_[id] = {};
        break;
      }
    }

    return result;
  }

  // Check if this record matches an auto-cluster rule
  auto cluster_result = try_auto_cluster(id, severity, timestamp);
  if (cluster_result) {
    return *cluster_result;
  }

  // No correlation found
  return result;
}

ProcessClearResult CorrelationEngine::process_clear(const FaultId & id) {
  std::lock_guard<std::mutex> lock(mutex_);

  const std::string & fault_code = id.fault_code;
  ProcessClearResult result;

  // Check if this record is a root cause with symptoms. The lookup is by record, so
  // clearing owner X's root cause never reaches owner Y's symptoms.
  auto it = root_to_symptoms_.find(id);
  if (it != root_to_symptoms_.end()) {
    // Find the rule to check auto_clear_with_root
    for (const auto & prc : pending_root_causes_) {
      if (prc.fault_id == id) {
        for (const auto & rule : config_.rules) {
          if (rule.id == prc.rule_id && rule.auto_clear_with_root) {
            result.auto_cleared_symptoms = it->second;
            break;
          }
        }
        break;
      }
    }

    // Also check finalized root causes (not in pending anymore)
    if (result.auto_cleared_symptoms.empty()) {
      for (const auto & rule : config_.rules) {
        if (rule.mode == CorrelationMode::HIERARCHICAL && rule.auto_clear_with_root) {
          // Check if fault matches this rule's root cause
          if (matcher_->matches_any(fault_code, rule.root_cause_codes)) {
            result.auto_cleared_symptoms = it->second;
            break;
          }
        }
      }
    }

    // Clean up muted records
    for (const auto & symptom_id : it->second) {
      muted_faults_.erase(symptom_id);
    }

    // Remove from root_to_symptoms
    root_to_symptoms_.erase(it);
  }

  // Remove from pending root causes
  pending_root_causes_.erase(std::remove_if(pending_root_causes_.begin(), pending_root_causes_.end(),
                                            [&id](const PendingRootCause & prc) {
                                              return prc.fault_id == id;
                                            }),
                             pending_root_causes_.end());

  // Check if this record is part of a cluster
  auto cluster_it = fault_to_cluster_.find(id);
  if (cluster_it != fault_to_cluster_.end()) {
    const std::string cluster_id = cluster_it->second;

    // Clean up pending_clusters_ when fault is cleared before min_count (#127)
    for (auto pending_it = pending_clusters_.begin(); pending_it != pending_clusters_.end();) {
      auto & pending_cluster = pending_it->second.data;
      if (pending_cluster.cluster_id != cluster_id) {
        ++pending_it;
        continue;
      }

      auto & codes = pending_cluster.fault_codes;
      codes.erase(std::remove(codes.begin(), codes.end(), fault_code), codes.end());
      pending_it->second.fault_severities.erase(fault_code);

      if (codes.empty()) {
        pending_it = pending_clusters_.erase(pending_it);
        continue;
      }

      // Reassign representative if the cleared fault was the representative
      if (pending_cluster.representative_code == fault_code) {
        for (const auto & rule : config_.rules) {
          if (rule.id == pending_it->first.first) {
            switch (rule.representative) {
              case Representative::FIRST: {
                auto & sevs = pending_it->second.fault_severities;
                const std::string & first_code = codes.front();
                pending_cluster.representative_code = first_code;
                auto sev_it = sevs.find(first_code);
                pending_cluster.representative_severity = (sev_it != sevs.end()) ? sev_it->second : "";
                break;
              }
              case Representative::HIGHEST_SEVERITY: {
                auto & sevs = pending_it->second.fault_severities;
                std::string best_code = codes.front();
                int best_rank = -1;
                for (const auto & code : codes) {
                  auto sev_it = sevs.find(code);
                  int rank = (sev_it != sevs.end()) ? severity_rank(sev_it->second) : 0;
                  if (rank > best_rank) {
                    best_rank = rank;
                    best_code = code;
                  }
                }
                pending_cluster.representative_code = best_code;
                auto best_sev_it = sevs.find(best_code);
                pending_cluster.representative_severity = (best_sev_it != sevs.end()) ? best_sev_it->second : "";
                break;
              }
              case Representative::MOST_RECENT: {
                auto & sevs = pending_it->second.fault_severities;
                const std::string & most_recent_code = codes.back();
                pending_cluster.representative_code = most_recent_code;
                auto sev_it = sevs.find(most_recent_code);
                pending_cluster.representative_severity = (sev_it != sevs.end()) ? sev_it->second : "";
                break;
              }
            }
            break;
          }
        }
      }

      ++pending_it;
    }

    // Remove fault from active cluster
    auto active_it = active_clusters_.find(cluster_id);
    if (active_it != active_clusters_.end()) {
      auto & active_cluster = active_it->second;
      auto & codes = active_cluster.fault_codes;
      codes.erase(std::remove(codes.begin(), codes.end(), fault_code), codes.end());

      if (codes.empty()) {
        active_clusters_.erase(active_it);
      } else if (active_cluster.representative_code == fault_code) {
        // Sync representative from pending cluster (already updated above)
        for (const auto & [pending_key, pending] : pending_clusters_) {
          if (pending.data.cluster_id == cluster_id) {
            active_cluster.representative_code = pending.data.representative_code;
            active_cluster.representative_severity = pending.data.representative_severity;
            break;
          }
        }
      }
    }

    fault_to_cluster_.erase(cluster_it);
  }

  // Remove from muted records if it was a symptom
  muted_faults_.erase(id);

  return result;
}

std::vector<MutedFaultData> CorrelationEngine::get_muted_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<MutedFaultData> result;
  result.reserve(muted_faults_.size());

  for (const auto & [muted_id, data] : muted_faults_) {
    result.push_back(data);
  }

  return result;
}

uint32_t CorrelationEngine::get_muted_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<uint32_t>(muted_faults_.size());
}

bool CorrelationEngine::is_muted(const FaultId & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return muted_faults_.find(id) != muted_faults_.end();
}

std::vector<ClusterData> CorrelationEngine::get_clusters() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<ClusterData> result;
  result.reserve(active_clusters_.size());

  for (const auto & [id, data] : active_clusters_) {
    result.push_back(data);
  }

  return result;
}

uint32_t CorrelationEngine::get_cluster_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<uint32_t>(active_clusters_.size());
}

void CorrelationEngine::cleanup_expired() {
  std::lock_guard<std::mutex> lock(mutex_);

  auto now = std::chrono::steady_clock::now();

  // Clean up expired pending root causes
  pending_root_causes_.erase(
      std::remove_if(pending_root_causes_.begin(), pending_root_causes_.end(),
                     [now](const PendingRootCause & prc) {
                       auto elapsed =
                           std::chrono::duration_cast<std::chrono::milliseconds>(now - prc.timestamp).count();
                       return elapsed > static_cast<int64_t>(prc.window_ms);
                     }),
      pending_root_causes_.end());

  // Clean up expired pending clusters
  std::vector<std::pair<std::string, std::string>> expired_pending;
  for (const auto & [key, pending] : pending_clusters_) {
    // Find rule to get window_ms
    for (const auto & rule : config_.rules) {
      if (rule.id == key.first) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - pending.steady_first_at).count();
        if (elapsed > static_cast<int64_t>(rule.window_ms)) {
          expired_pending.push_back(key);
        }
        break;
      }
    }
  }

  for (const auto & key : expired_pending) {
    auto it = pending_clusters_.find(key);
    if (it != pending_clusters_.end()) {
      if (active_clusters_.find(it->second.data.cluster_id) == active_clusters_.end()) {
        for (const auto & fault_code : it->second.data.fault_codes) {
          fault_to_cluster_.erase(FaultId{fault_code, it->second.owner});
        }
      }
      pending_clusters_.erase(it);
    }
  }
}

std::optional<std::string> CorrelationEngine::try_as_root_cause(const std::string & fault_code) {
  for (const auto & rule : config_.rules) {
    if (rule.mode != CorrelationMode::HIERARCHICAL) {
      continue;
    }

    if (matcher_->matches_any(fault_code, rule.root_cause_codes)) {
      return rule.id;
    }
  }

  return std::nullopt;
}

std::optional<ProcessFaultResult> CorrelationEngine::try_as_symptom(const FaultId & id,
                                                                    std::chrono::steady_clock::time_point timestamp) {
  const std::string & fault_code = id.fault_code;
  for (const auto & prc : pending_root_causes_) {
    // A root cause only explains its OWN reporter's faults. Without this the first
    // owner to report a root-cause code would mute every other owner's symptom.
    if (prc.fault_id.owner != id.owner) {
      continue;
    }
    // Find the rule
    for (const auto & rule : config_.rules) {
      if (rule.id != prc.rule_id || rule.mode != CorrelationMode::HIERARCHICAL) {
        continue;
      }

      // Check if fault matches any symptom pattern
      bool matches_symptom = false;
      for (const auto & pattern_id : rule.symptom_pattern_ids) {
        if (matcher_->matches(fault_code, pattern_id)) {
          matches_symptom = true;
          break;
        }
      }

      // Also check inline symptom codes (direct codes with wildcard support)
      if (!matches_symptom && !rule.inline_symptom_codes.empty()) {
        matches_symptom = matcher_->matches_any(fault_code, rule.inline_symptom_codes);
      }

      if (!matches_symptom) {
        continue;
      }

      // Check time window
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - prc.timestamp).count();
      if (elapsed > static_cast<int64_t>(rule.window_ms)) {
        continue;
      }

      // This fault is a symptom!
      ProcessFaultResult result;
      result.should_mute = rule.mute_symptoms;
      result.root_cause_code = prc.fault_id.fault_code;
      result.rule_id = rule.id;
      result.delay_ms = static_cast<uint32_t>(elapsed);

      // Track the symptom (avoid duplicates)
      auto & symptoms = root_to_symptoms_[prc.fault_id];
      if (std::find(symptoms.begin(), symptoms.end(), id) == symptoms.end()) {
        symptoms.push_back(id);
      }

      if (rule.mute_symptoms) {
        MutedFaultData muted;
        muted.fault_code = fault_code;
        muted.root_cause_code = prc.fault_id.fault_code;
        muted.rule_id = rule.id;
        muted.delay_ms = result.delay_ms;
        muted_faults_[id] = muted;
      }

      return result;
    }
  }

  return std::nullopt;
}

std::optional<ProcessFaultResult> CorrelationEngine::try_auto_cluster(const FaultId & id, const std::string & severity,
                                                                      std::chrono::steady_clock::time_point timestamp) {
  const std::string & fault_code = id.fault_code;
  for (const auto & rule : config_.rules) {
    if (rule.mode != CorrelationMode::AUTO_CLUSTER) {
      continue;
    }

    // Check if fault matches any pattern
    bool matches = false;
    for (const auto & pattern_id : rule.match_pattern_ids) {
      if (matcher_->matches(fault_code, pattern_id)) {
        matches = true;
        break;
      }
    }

    if (!matches) {
      continue;
    }

    auto now_system = std::chrono::system_clock::now();

    // Check if we have a pending cluster for this rule AND this owner. One rule forms
    // one cluster per owner, so a burst on owner A never counts owner B's faults
    // towards min_count and never mutes them as non-representative members.
    const auto pending_key = std::make_pair(rule.id, id.owner);
    auto pending_it = pending_clusters_.find(pending_key);
    if (pending_it != pending_clusters_.end()) {
      // Check if within time window using steady_clock timestamp
      auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - pending_it->second.steady_first_at).count();

      if (elapsed > static_cast<int64_t>(rule.window_ms)) {
        // Window expired, start new cluster
        pending_clusters_.erase(pending_it);
        pending_it = pending_clusters_.end();
      }
    }

    if (pending_it == pending_clusters_.end()) {
      // Start new pending cluster
      PendingCluster pending;
      pending.owner = id.owner;
      pending.steady_first_at = timestamp;
      pending.data.cluster_id = generate_cluster_id(rule.id);
      pending.data.rule_id = rule.id;
      pending.data.rule_name = rule.name;
      pending.data.label = rule.name;  // Use rule name as label
      pending.data.representative_code = fault_code;
      pending.data.representative_severity = severity;
      pending.data.fault_codes.push_back(fault_code);
      pending.fault_severities[fault_code] = severity;
      pending.data.first_at = now_system;
      pending.data.last_at = now_system;

      pending_clusters_[pending_key] = pending;
      fault_to_cluster_[id] = pending.data.cluster_id;

      // Not enough faults yet for a cluster
      ProcessFaultResult result;
      result.cluster_id = pending.data.cluster_id;
      // Don't mute - first fault is the representative
      return result;
    }

    // Add to existing pending cluster
    auto & pending = pending_it->second;
    auto & cluster = pending.data;

    // Check for duplicate
    if (std::find(cluster.fault_codes.begin(), cluster.fault_codes.end(), fault_code) != cluster.fault_codes.end()) {
      // Already in cluster - ensure consistent muting for duplicates
      ProcessFaultResult result;
      result.cluster_id = cluster.cluster_id;
      if (rule.show_as_single && fault_code != cluster.representative_code &&
          cluster.fault_codes.size() >= rule.min_count) {
        result.should_mute = true;
      }
      return result;
    }

    cluster.fault_codes.push_back(fault_code);
    pending.fault_severities[fault_code] = severity;
    cluster.last_at = now_system;
    fault_to_cluster_[id] = cluster.cluster_id;

    // Update representative based on rule's representative selection
    bool update_representative = false;
    switch (rule.representative) {
      case Representative::FIRST:
        // Keep first fault as representative
        break;
      case Representative::MOST_RECENT:
        update_representative = true;
        break;
      case Representative::HIGHEST_SEVERITY:
        if (severity_rank(severity) > severity_rank(cluster.representative_severity)) {
          update_representative = true;
        }
        break;
    }

    if (update_representative) {
      cluster.representative_code = fault_code;
      cluster.representative_severity = severity;
    }

    ProcessFaultResult result;
    result.cluster_id = cluster.cluster_id;

    // Check if cluster threshold reached
    if (cluster.fault_codes.size() >= rule.min_count) {
      // Check if cluster is newly activated (first time reaching threshold)
      bool newly_activated = (active_clusters_.find(cluster.cluster_id) == active_clusters_.end());

      // Move to active clusters
      if (newly_activated) {
        active_clusters_[cluster.cluster_id] = cluster;

        // Retroactively mute all non-representative faults added before threshold
        if (rule.show_as_single) {
          for (const auto & code : cluster.fault_codes) {
            if (code != cluster.representative_code && code != fault_code) {
              result.retroactive_mute_codes.push_back(code);
            }
          }
        }
      } else {
        // Update existing
        active_clusters_[cluster.cluster_id] = cluster;
      }

      // Mute non-representative faults
      if (rule.show_as_single && fault_code != cluster.representative_code) {
        result.should_mute = true;
      }
    }

    return result;
  }

  return std::nullopt;
}

std::string CorrelationEngine::generate_cluster_id(const std::string & rule_id) {
  ++cluster_counter_;
  std::ostringstream oss;
  oss << rule_id << "_" << cluster_counter_;
  return oss.str();
}

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
