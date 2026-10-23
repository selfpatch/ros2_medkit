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

#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <unordered_set>
#include <utility>

namespace ros2_medkit_fault_manager::core {

namespace {

bool is_valid_severity(uint8_t severity) {
  return severity <= fd::severity::kCritical;
}

std::string extract_topic_name(const std::string & topic_path) {
  auto pos = topic_path.rfind('/');
  if (pos != std::string::npos && pos < topic_path.length() - 1) {
    return topic_path.substr(pos + 1);
  }
  return topic_path;
}

}  // namespace

std::string validate_fault_code(const std::string & fault_code) {
  if (fault_code.empty()) {
    return "fault_code cannot be empty";
  }

  if (fault_code.length() > kMaxFaultCodeLength) {
    return "fault_code exceeds maximum length of " + std::to_string(kMaxFaultCodeLength);
  }

  // Allow only alphanumeric, underscore, hyphen, and dot.
  // This prevents path traversal (../) and header injection (\r\n, ").
  for (char c : fault_code) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {
      return "fault_code contains invalid character '" + std::string(1, c) +
             "'. Only alphanumeric, underscore, hyphen, and dot are allowed";
    }
  }

  if (fault_code.find("..") != std::string::npos) {
    return "fault_code cannot contain '..'";
  }

  return "";
}

std::string storage_unavailable(const std::exception & e) {
  return std::string("Fault storage unavailable: ") + e.what();
}

FaultManagerCore::FaultManagerCore(FaultManagerConfig config, CoreParts parts, EventSink & events, CaptureHooks & hooks,
                                   const WallClock & clock, LogSink & log)
  : config_(std::move(config))
  , storage_(std::move(parts.storage))
  , audit_(std::move(parts.audit))
  , correlation_(std::move(parts.correlation))
  , global_debounce_(parts.global_debounce)
  , entity_resolver_(std::move(parts.entity_resolver))
  , fault_code_resolver_(std::move(parts.fault_code_resolver))
  , events_(events)
  , hooks_(hooks)
  , clock_(clock)
  , log_(log)
  , audit_confirmed_only_(config_.audit_log_transitions == "confirmed_only")
  , audit_fail_closed_(config_.audit_log_fail_closed) {
}

ReportOutcome FaultManagerCore::report(const fd::FaultReport & report) {
  std::lock_guard<std::mutex> lock(mutex_);
  return report_locked(report);
}

ClearOutcome FaultManagerCore::clear(const std::string & code, bool skip_correlation_auto_clear) {
  std::lock_guard<std::mutex> lock(mutex_);
  return clear_locked(code, skip_correlation_auto_clear);
}

ListOutcome FaultManagerCore::list(const ListQuery & query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return list_locked(query);
}

FaultDetail FaultManagerCore::get(const std::string & code) {
  std::lock_guard<std::mutex> lock(mutex_);
  return get_locked(code);
}

std::vector<fd::FaultRecord> FaultManagerCore::list_for_entity(const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<fd::FaultRecord> result;
  for (auto & fault : storage_->get_all_faults()) {
    if (matches_entity(fault.reporting_sources, entity_id)) {
      result.push_back(std::move(fault));
    }
  }
  return result;
}

void FaultManagerCore::record_transition(const char * transition, const fd::FaultRecord & fault,
                                         const std::string & source_id, int64_t occurred_at_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  audit_transition(transition, fault, source_id, occurred_at_ns);
}

bool FaultManagerCore::matches_entity(const std::vector<std::string> & reporting_sources,
                                      const std::string & entity_id) {
  for (const auto & source : reporting_sources) {
    if (source == entity_id) {
      return true;
    }

    // FQN suffix match: source ends with "/" + entity_id,
    // e.g. source="/powertrain/motor_controller" matches entity_id="motor_controller".
    std::string suffix = "/" + entity_id;
    if (source.size() > suffix.size() && source.substr(source.size() - suffix.size()) == suffix) {
      return true;
    }
  }
  return false;
}

ReportOutcome FaultManagerCore::report_locked(const fd::FaultReport & report) {
  const std::string & logger = config_.logger_name;
  ReportOutcome outcome;
  const auto reject = [&](const std::string & reason) {
    outcome.reject_reason = reason;
    return outcome;
  };

  std::string validation_error = validate_fault_code(report.fault_code);
  if (!validation_error.empty()) {
    logf(log_, LogLevel::kWarn, logger, "ReportFault rejected: %s", validation_error.c_str());
    return reject(validation_error);
  }

  if (report.kind != fd::ReportKind::kFailed && report.kind != fd::ReportKind::kPassed) {
    logf(log_, LogLevel::kWarn, logger, "ReportFault rejected: invalid event_type %d", static_cast<int>(report.kind));
    return reject("invalid event_type");
  }

  const bool failed = report.kind == fd::ReportKind::kFailed;
  if (failed && !is_valid_severity(report.severity)) {
    logf(log_, LogLevel::kWarn, logger, "ReportFault rejected: invalid severity %d for FAILED event", report.severity);
    return reject("invalid severity");
  }

  if (report.source_id.empty()) {
    logf(log_, LogLevel::kWarn, logger, "ReportFault rejected: source_id cannot be empty");
    return reject("source_id cannot be empty");
  }

  const auto connection_lost = [&](const FaultStorage::IgnorableConnectionException & e) {
    logf(log_, LogLevel::kWarn, logger, "Failed to connect with the fault storage server: %s", e.what());
  };

  // Get status before update (if fault exists)
  std::optional<fd::FaultRecord> fault_before;
  try {
    fault_before = storage_->get_fault(report.fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    connection_lost(e);
    return outcome;
  }
  std::string status_before = fault_before ? fault_before->status : "";

  // Resolve the debounce config: global, then the entity override matching
  // source_id, then the fault code's own override.
  auto resolved_config = resolve_config(report.source_id, report.fault_code);

  const int64_t event_time_ns = clock_.now_ns();
  bool is_new = false;
  try {
    is_new = storage_->report_fault_event(report.fault_code, report.kind, report.severity, report.description,
                                          report.source_id, event_time_ns, resolved_config);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    connection_lost(e);
    return outcome;
  }

  outcome.accepted = true;

  // Get updated fault state to publish event
  std::optional<fd::FaultRecord> fault_after;
  try {
    fault_after = storage_->get_fault(report.fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    connection_lost(e);
    return outcome;
  }

  if (fault_after) {
    // Only for codes the store holds: a PASSED for an unknown code writes nothing, so it must
    // not add an entry here either (the diagnostic bridge sends PASSED for every OK status).
    warn_on_conflicting_debounce_policy(report.fault_code, report.source_id, resolved_config);

    // Process through correlation engine (if enabled). Only FAILED events correlate.
    bool should_mute = false;
    if (correlation_ && failed) {
      auto correlation_result =
          correlation_->process_fault(report.fault_code, correlation::severity_to_string(report.severity));

      should_mute = correlation_result.should_mute;

      if (correlation_result.is_root_cause) {
        logf(log_, LogLevel::kDebug, logger, "Fault %s identified as root cause (rule=%s)", report.fault_code.c_str(),
             correlation_result.rule_id.c_str());
      } else if (should_mute) {
        logf(log_, LogLevel::kDebug, logger, "Fault %s muted as symptom of %s (rule=%s, delay=%ums)",
             report.fault_code.c_str(), correlation_result.root_cause_code.c_str(), correlation_result.rule_id.c_str(),
             correlation_result.delay_ms);
      } else if (!correlation_result.cluster_id.empty()) {
        logf(log_, LogLevel::kDebug, logger, "Fault %s added to cluster %s", report.fault_code.c_str(),
             correlation_result.cluster_id.c_str());
        // Log retroactively muted faults when cluster becomes active
        if (!correlation_result.retroactive_mute_codes.empty()) {
          logf(log_, LogLevel::kDebug, logger, "Cluster %s activated: retroactively muting %zu faults",
               correlation_result.cluster_id.c_str(), correlation_result.retroactive_mute_codes.size());
        }
      }
    }

    // Determine event type based on status transition
    bool just_confirmed = false;
    if (is_new && fault_after->status == fd::fault_status::kConfirmed) {
      // New fault immediately confirmed (e.g., CRITICAL severity or threshold=-1)
      if (!should_mute) {
        publish_event(fd::fault_event_type::kConfirmed, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && status_before != fd::fault_status::kConfirmed &&
               fault_after->status == fd::fault_status::kConfirmed) {
      // Existing fault transitioned to CONFIRMED
      if (!should_mute) {
        publish_event(fd::fault_event_type::kConfirmed, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && fault_after->status == fd::fault_status::kConfirmed) {
      // Fault was already CONFIRMED, data updated (last_occurred, severity, sources).
      // Not occurrence_count: a re-report inside one occurrence does not touch it.
      if (!should_mute) {
        publish_event(fd::fault_event_type::kUpdated, *fault_after);
      }
    }
    // Note: PREFAILED/PREPASSED status changes don't emit events (debounce in progress)

    // A PASSED event can drive an existing fault past the healing threshold to
    // HEALED. That is the fault's END and must be audited (with a distinct kind
    // and source) so the timeline is complete, not just occurred+confirmed.
    const bool just_healed =
        !is_new && status_before != fd::fault_status::kHealed && fault_after->status == fd::fault_status::kHealed;

    // The end of a fault must reach the event stream too, or an SSE consumer keeps
    // showing a fault that is over. The cleared event type is reused so existing
    // consumers act on it; the payload's status = HEALED distinguishes auto-heal from
    // an acknowledged clear.
    if (just_healed) {
      publish_event(fd::fault_event_type::kCleared, *fault_after);
    }

    // Append tamper-evident audit records for the transitions that just happened.
    // Recorded regardless of correlation muting: muting affects display, not the
    // fact that the state transition occurred.
    if (is_new) {
      audit_transition(kTransitionOccurred, *fault_after, report.source_id, event_time_ns);
    }
    if (just_confirmed) {
      audit_transition(kTransitionConfirmed, *fault_after, report.source_id, event_time_ns);
    }
    if (just_healed) {
      audit_transition(kTransitionHealed, *fault_after, "auto_heal", event_time_ns);
    }

    if (just_confirmed) {
      hooks_.on_confirmed(report.fault_code);
    }

    // PREFAILED starts the lazy_start rosbag capture
    bool just_prefailed = (is_new && fault_after->status == fd::fault_status::kPrefailed) ||
                          (!is_new && status_before != fd::fault_status::kPrefailed &&
                           fault_after->status == fd::fault_status::kPrefailed);
    if (just_prefailed) {
      hooks_.on_prefailed(report.fault_code);
    }
  }

  if (failed) {
    if (is_new) {
      logf(log_, LogLevel::kInfo, logger, "New fault reported: %s (severity=%d, source=%s)", report.fault_code.c_str(),
           report.severity, report.source_id.c_str());
    } else {
      logf(log_, LogLevel::kDebug, logger, "Fault updated: %s (source=%s)", report.fault_code.c_str(),
           report.source_id.c_str());
    }
  } else {
    logf(log_, LogLevel::kDebug, logger, "PASSED event for fault: %s (source=%s)", report.fault_code.c_str(),
         report.source_id.c_str());
  }
  return outcome;
}

ClearOutcome FaultManagerCore::clear_locked(const std::string & code, bool skip_correlation_auto_clear) {
  const std::string & logger = config_.logger_name;
  ClearOutcome outcome;

  std::string validation_error = validate_fault_code(code);
  if (!validation_error.empty()) {
    outcome.message = validation_error;
    return outcome;
  }

  const auto connection_lost = [&](const FaultStorage::IgnorableConnectionException & e) {
    logf(log_, LogLevel::kWarn, logger, "Failed to connect with the fault storage server: %s", e.what());
  };

  // Storage first: a clear that the storage refuses must not change the correlation state.
  bool cleared = false;
  try {
    cleared = storage_->clear_fault(code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    connection_lost(e);
    outcome.message = storage_unavailable(e);
    return outcome;
  }

  // Then the correlation engine (to get the auto-clear list).
  // `skip_correlation_auto_clear` lets the caller opt out of cascade-clearing
  // correlated symptom fault codes. Per-entity DELETE routes set it to true
  // so they cannot reach across entity boundaries via the correlation graph.
  std::vector<std::string> auto_cleared_codes;
  if (correlation_ && !skip_correlation_auto_clear) {
    auto clear_result = correlation_->process_clear(code);
    auto_cleared_codes = clear_result.auto_cleared_codes;
  }

  outcome.success = cleared;
  if (!cleared) {
    outcome.message = "Fault not found: " + code;
    logf(log_, LogLevel::kWarn, logger, "Attempted to clear non-existent fault: %s", code.c_str());
    return outcome;
  }

  // Evict cooldown tracking for the cleared fault and the auto-cleared symptoms
  hooks_.forget_cooldown(code);
  for (const auto & symptom_code : auto_cleared_codes) {
    hooks_.forget_cooldown(symptom_code);
  }

  // Auto-clear correlated symptoms
  for (const auto & symptom_code : auto_cleared_codes) {
    try {
      storage_->clear_fault(symptom_code);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      connection_lost(e);
      outcome.message = storage_unavailable(e);
      return outcome;
    }
    if (audit_) {
      std::optional<fd::FaultRecord> symptom;
      try {
        symptom = storage_->get_fault(symptom_code);
      } catch (const FaultStorage::IgnorableConnectionException & e) {
        connection_lost(e);
        outcome.message = storage_unavailable(e);
        return outcome;
      }
      if (symptom) {
        audit_transition(kTransitionCleared, *symptom, "clear_service", clock_.now_ns());
      }
    }
    logf(log_, LogLevel::kDebug, logger, "Auto-cleared symptom: %s (root cause: %s)", symptom_code.c_str(),
         code.c_str());
    // Also cleanup rosbag for auto-cleared faults
    hooks_.on_cleared(symptom_code);
  }

  outcome.auto_cleared_codes = auto_cleared_codes;
  if (auto_cleared_codes.empty()) {
    outcome.message = "Fault cleared: " + code;
  } else {
    outcome.message =
        "Fault cleared: " + code + " (auto-cleared " + std::to_string(auto_cleared_codes.size()) + " symptoms)";
  }
  logf(log_, LogLevel::kInfo, logger, "Fault cleared: %s (auto-cleared %zu symptoms)", code.c_str(),
       auto_cleared_codes.size());

  // Cleanup rosbag for the main fault (auto_cleanup handled inside RosbagCapture)
  hooks_.on_cleared(code);

  // Publish the cleared event - get the cleared fault to include in the event
  std::optional<fd::FaultRecord> fault;
  try {
    fault = storage_->get_fault(code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    connection_lost(e);
    outcome.message = storage_unavailable(e);
    return outcome;
  }
  if (fault) {
    publish_event(fd::fault_event_type::kCleared, *fault, auto_cleared_codes);
    audit_transition(kTransitionCleared, *fault, "clear_service", clock_.now_ns());
  }
  return outcome;
}

ListOutcome FaultManagerCore::list_locked(const ListQuery & query) {
  const std::string & logger = config_.logger_name;
  ListOutcome outcome;
  try {
    outcome.faults = storage_->list_faults(query.filter_by_severity, query.severity, query.statuses);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    logf(log_, LogLevel::kWarn, logger, "Failed to connect with the fault storage server: %s", e.what());
    return outcome;
  }

  // Include correlation data if engine is enabled
  if (correlation_) {
    // Always include counts
    outcome.muted_count = correlation_->get_muted_count();
    outcome.cluster_count = correlation_->get_cluster_count();

    auto muted_faults = correlation_->get_muted_faults();

    if (query.include_muted) {
      outcome.muted_faults.reserve(muted_faults.size());
      for (const auto & muted : muted_faults) {
        MutedFault info;
        info.fault_code = muted.fault_code;
        info.root_cause_code = muted.root_cause_code;
        info.rule_id = muted.rule_id;
        info.delay_ms = muted.delay_ms;
        outcome.muted_faults.push_back(info);
      }
    } else {
      // Build a set of muted codes for O(1) lookup.
      std::unordered_set<std::string> muted_codes;
      for (const auto & muted : muted_faults) {
        muted_codes.insert(muted.fault_code);
      }

      // Remove all faults whose code is muted, in one pass.
      auto & faults = outcome.faults;
      faults.erase(std::remove_if(faults.begin(), faults.end(),
                                  [&muted_codes](const fd::FaultRecord & fault) {
                                    return muted_codes.count(fault.fault_code) > 0;
                                  }),
                   faults.end());
    }

    if (query.include_clusters) {
      auto clusters = correlation_->get_clusters();
      outcome.clusters.reserve(clusters.size());
      for (const auto & cluster : clusters) {
        ClusterSummary info;
        info.cluster_id = cluster.cluster_id;
        info.rule_id = cluster.rule_id;
        info.rule_name = cluster.rule_name;
        info.label = cluster.label;
        info.representative_code = cluster.representative_code;
        info.representative_severity = cluster.representative_severity;
        info.fault_codes = cluster.fault_codes;
        info.count = static_cast<uint32_t>(cluster.fault_codes.size());
        info.first_at_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(cluster.first_at.time_since_epoch()).count();
        info.last_at_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(cluster.last_at.time_since_epoch()).count();
        outcome.clusters.push_back(info);
      }
    }
  }

  logf(log_, LogLevel::kDebug, logger, "ListFaults returned %zu faults (muted=%u, clusters=%u)", outcome.faults.size(),
       outcome.muted_count, outcome.cluster_count);
  return outcome;
}

FaultDetail FaultManagerCore::get_locked(const std::string & code) {
  const std::string & logger = config_.logger_name;
  FaultDetail detail;
  logf(log_, LogLevel::kDebug, logger, "GetFault request for: %s", code.c_str());

  std::string validation_error = validate_fault_code(code);
  if (!validation_error.empty()) {
    detail.error_message = validation_error;
    return detail;
  }

  // Each storage failure ends the call with what has been filled so far and success = false.
  const auto unavailable = [&](const FaultStorage::IgnorableConnectionException & e) {
    logf(log_, LogLevel::kWarn, logger, "Failed to connect with the fault storage server: %s", e.what());
    detail.success = false;
    detail.error_message = storage_unavailable(e);
    return detail;
  };

  std::optional<fd::FaultRecord> fault;
  try {
    fault = storage_->get_fault(code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    return unavailable(e);
  }
  if (!fault) {
    detail.error_message = "Fault not found: " + code;
    return detail;
  }

  detail.success = true;
  detail.fault = *fault;
  detail.first_occurrence_ns = fault->first_occurred_ns;
  detail.last_occurrence_ns = fault->last_occurred_ns;

  // Get freeze frame snapshots from storage
  std::vector<SnapshotData> stored_snapshots = {};
  try {
    stored_snapshots = storage_->get_snapshots(code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    return unavailable(e);
  }
  for (const auto & stored_snapshot : stored_snapshots) {
    SnapshotEntry snapshot;
    snapshot.type = kSnapshotFreezeFrame;
    snapshot.name = extract_topic_name(stored_snapshot.topic);
    snapshot.data = stored_snapshot.data;
    snapshot.topic = stored_snapshot.topic;
    snapshot.message_type = stored_snapshot.message_type;
    snapshot.captured_at_ns = stored_snapshot.captured_at_ns;
    detail.snapshots.push_back(snapshot);
  }

  // Per-topic snapshots are deleted on clear_fault unless snapshots.retain_on_clear is set, but
  // the compact freeze-frame row is retained either way. When no snapshots remain, serve the
  // retained frame so the confirmed-state record stays observable after acknowledgement.
  //
  // With retention on, snapshots never run out, so "none left" would never fire and the frame -
  // which is the state at the MOST RECENT confirmation - would stay hidden behind snapshots of
  // earlier occurrences. Serve it in that case too.
  if (stored_snapshots.empty() || storage_->retains_snapshots_on_clear()) {
    std::optional<FreezeFrameData> frame;
    try {
      frame = storage_->get_freeze_frame(code);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      return unavailable(e);
    }
    if (frame) {
      SnapshotEntry snapshot;
      snapshot.type = kSnapshotFreezeFrame;
      snapshot.name = "freeze_frame";
      snapshot.data = frame->data;
      // topic/message_type left empty: the frame aggregates values from multiple topics
      snapshot.captured_at_ns = frame->captured_at_ns;
      detail.snapshots.push_back(snapshot);
    }
  }

  // One entry per recording, newest first. A fault that keeps re-confirming leaves a
  // trail of black boxes, and every one of them needs its own addressable id here -
  // this list is the only place the fault itself advertises them.
  try {
    for (const auto & rosbag_info : storage_->get_rosbag_files(code)) {
      SnapshotEntry rosbag_snapshot;
      rosbag_snapshot.type = kSnapshotRosbag;
      rosbag_snapshot.name = "rosbag_" + rosbag_info.recording_id;
      // The RECORDING, not the fault code: several recordings of one fault would
      // otherwise all carry the same id and collapse into one download.
      rosbag_snapshot.bulk_data_id = rosbag_info.recording_id;
      rosbag_snapshot.size_bytes = rosbag_info.size_bytes;
      rosbag_snapshot.duration_sec = rosbag_info.duration_sec;
      rosbag_snapshot.format = rosbag_info.format;
      rosbag_snapshot.captured_at_ns = rosbag_info.created_at_ns;
      detail.snapshots.push_back(rosbag_snapshot);
    }
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    return unavailable(e);
  }

  logf(log_, LogLevel::kDebug, logger, "GetFault returned fault '%s' with %zu snapshots", code.c_str(),
       detail.snapshots.size());
  return detail;
}

DebounceConfig FaultManagerCore::resolve_config(const std::string & source_id, const std::string & fault_code) const {
  DebounceConfig config = global_debounce_;
  if (entity_resolver_) {
    config = entity_resolver_->resolve(source_id, global_debounce_);
  }
  // The fault code is the last layer: it is what the debounce counter belongs to, so an
  // override on it settles the policy whichever source reported.
  if (fault_code_resolver_) {
    config = fault_code_resolver_->resolve(fault_code, config);
  }
  // Defensive: the merged config is validated at load time, but never hand the storage backend a
  // config that violates confirmation_threshold < 0 <= healing_threshold (the counter would stick).
  sanitize_debounce_config(config);
  return config;
}

void FaultManagerCore::warn_on_conflicting_debounce_policy(const std::string & fault_code,
                                                           const std::string & source_id,
                                                           const DebounceConfig & resolved) {
  auto [it, inserted] =
      debounce_policy_witness_.try_emplace(fault_code, DebouncePolicyWitness{resolved, source_id, false});
  if (inserted || it->second.warned || debounce_policy_equal(it->second.config, resolved)) {
    return;
  }

  // One counter, two policies: the report that arrives decides the transition, so the other
  // source's debounce policy is bypassed for as long as both report this code.
  it->second.warned = true;
  logf(log_, LogLevel::kWarn, config_.logger_name,
       "Fault code '%s' is debounced two ways: '%s' resolves confirmation=%d healing_enabled=%s healing=%d, "
       "'%s' resolves confirmation=%d healing_enabled=%s healing=%d. The debounce counter belongs to the fault "
       "code, so whichever source reports decides the transition and the other policy is bypassed. Pin "
       "confirmation_threshold, healing_enabled and healing_threshold for this code in "
       "fault_thresholds.config_file.",
       fault_code.c_str(), it->second.source_id.c_str(), it->second.config.confirmation_threshold,
       it->second.config.healing_enabled ? "true" : "false", it->second.config.healing_threshold, source_id.c_str(),
       resolved.confirmation_threshold, resolved.healing_enabled ? "true" : "false", resolved.healing_threshold);
}

void FaultManagerCore::audit_transition(const char * transition, const fd::FaultRecord & fault,
                                        const std::string & source_id, int64_t occurred_at_ns) {
  if (!audit_) {
    return;
  }
  if (audit_confirmed_only_ && std::string(transition) != kTransitionConfirmed) {
    return;
  }

  AuditEvent event;
  event.fault_code = fault.fault_code;
  event.transition = transition;
  event.severity = fault.severity;
  event.status = fault.status;
  event.source_id = source_id;
  event.description = fault.description;
  event.occurred_at_ns = occurred_at_ns;

  try {
    audit_->append(event);
  } catch (const std::exception & e) {
    // A dropped append is a hole in the chain that verify() can never see (it
    // proves nothing was deleted, not that everything was written). The fault-
    // state transition that triggered this has ALREADY committed to the fault
    // store, which is a SEPARATE SQLite database: there is no cross-DB atomicity,
    // so this failure does NOT (and cannot) roll that change back. All we can do
    // is make the broken audit loud and observable via the health signals.
    audit_dropped_writes_.fetch_add(1, std::memory_order_relaxed);
    audit_healthy_.store(false, std::memory_order_relaxed);
    const uint64_t dropped = audit_dropped_writes_.load(std::memory_order_relaxed);
    logf(log_, LogLevel::kError, config_.logger_name,
         "Failed to append audit record for '%s' (%s): %s [audit dropped_writes=%" PRIu64 "]", fault.fault_code.c_str(),
         transition, e.what(), dropped);
    if (audit_fail_closed_) {
      // Explicit fail-FAST: surface the broken audit to the caller so a
      // compliance-strict deployment learns the chain is now incomplete and an
      // operator must act. Rethrowing propagates that signal ONLY - the fault-
      // state change already committed (see above) and is NOT rolled back; this
      // is not atomicity.
      logf(log_, LogLevel::kFatal, config_.logger_name,
           "audit_log.fail_closed is set: audit append failed for '%s' (%s); the audit chain is now "
           "incomplete and requires operator action. The already-committed fault-state change is NOT "
           "rolled back (the fault store is a separate database).",
           fault.fault_code.c_str(), transition);
      throw;
    }
  }
}

void FaultManagerCore::publish_event(const char * event_type, const fd::FaultRecord & fault,
                                     const std::vector<std::string> & auto_cleared_codes) {
  fd::FaultEventRecord event;
  event.event_type = event_type;
  event.fault = fault;
  event.timestamp_ns = clock_.now_ns();
  event.auto_cleared_codes = auto_cleared_codes;

  events_.publish(event);

  logf(log_, LogLevel::kDebug, config_.logger_name, "Published fault event: %s for fault_code=%s", event_type,
       fault.fault_code.c_str());
}

}  // namespace ros2_medkit_fault_manager::core
