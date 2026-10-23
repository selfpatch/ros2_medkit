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

#include "ros2_medkit_fault_manager/fault_manager_node.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"
#include "ros2_medkit_fault_manager/ros2/fault_msg_conversions.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_fault_manager/time_utils.hpp"
#include "ros2_medkit_msgs/msg/cluster_info.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/extended_data_records.hpp"
#include "ros2_medkit_msgs/msg/muted_fault_info.hpp"
#include "ros2_medkit_msgs/msg/snapshot.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// Maximum allowed length for fault_code.
///
/// Matches the bound the gateway enforces on every route carrying
/// `{fault_code}` and publishes as that parameter's `maxLength`. The two are
/// one contract seen from two nodes, so they have to be the same number: while
/// this was the lower of the two, a code the document called well-formed was
/// admitted by the gateway and then refused here, and the refusal reached the
/// client as a server error.
///
/// Nothing downstream constrains it further. Every column holding a fault code
/// is SQLite `TEXT`, which is unbounded, and every `fault_code` field in the
/// message and service definitions is an unbounded `string`. `RosbagCapture`
/// is the one consumer with a limit of its own - it names a directory after
/// the code - and it bounds its own path component rather than relying on this
/// value.
constexpr size_t kMaxFaultCodeLength = 256;

/// Validate fault_code format
/// @param fault_code The fault code to validate
/// @return Empty string if valid, error message if invalid
std::string validate_fault_code(const std::string & fault_code) {
  if (fault_code.empty()) {
    return "fault_code cannot be empty";
  }

  if (fault_code.length() > kMaxFaultCodeLength) {
    return "fault_code exceeds maximum length of " + std::to_string(kMaxFaultCodeLength);
  }

  // Allow only alphanumeric, underscore, hyphen, and dot
  // This prevents path traversal (../) and header injection (\r\n, ")
  for (char c : fault_code) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {
      return "fault_code contains invalid character '" + std::string(1, c) +
             "'. Only alphanumeric, underscore, hyphen, and dot are allowed";
    }
  }

  // Prevent path traversal patterns
  if (fault_code.find("..") != std::string::npos) {
    return "fault_code cannot contain '..'";
  }

  return "";  // Valid
}

/// Validate a recording id (the bag directory basename, `fault_<CODE>_<millis>`).
///
/// Same alphabet and the same no-`..` rule as a fault code, because the id ends up
/// as a path segment and a filesystem lookup. The LENGTH bound is different: a
/// recording id is `fault_` + a fault code + `_` + a millisecond stamp, so a
/// maximal fault code produces ~148 characters and would be rejected by the
/// fault-code validator it is derived from.
std::string validate_recording_id(const std::string & recording_id) {
  constexpr size_t kMaxRecordingIdLength = kMaxFaultCodeLength + 32;

  if (recording_id.empty()) {
    return "recording_id cannot be empty";
  }
  if (recording_id.length() > kMaxRecordingIdLength) {
    return "recording_id exceeds maximum length of " + std::to_string(kMaxRecordingIdLength);
  }
  for (char c : recording_id) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {
      return "recording_id contains invalid character '" + std::string(1, c) +
             "'. Only alphanumeric, underscore, hyphen, and dot are allowed";
    }
  }
  if (recording_id.find("..") != std::string::npos) {
    return "recording_id cannot contain '..'";
  }
  return "";  // Valid
}

/// Response text for a request the fault storage server could not serve.
std::string storage_unavailable(const std::exception & e) {
  return std::string("Fault storage unavailable: ") + e.what();
}

}  // namespace

FaultManagerNode::FaultManagerNode(const rclcpp::NodeOptions & options) : Node("fault_manager", options) {
  config_ = declare_config();
  config_.logger_name = get_logger().get_name();
  for (const auto & notice : config_.validate()) {
    log_sink_.log(notice.level, notice.logger, notice.text);
  }

  storage_type_ = config_.storage_type;
  confirmation_threshold_ = static_cast<int32_t>(config_.confirmation_threshold);
  healing_enabled_ = config_.healing_enabled;
  healing_threshold_ = static_cast<int32_t>(config_.healing_threshold);
  auto_confirm_after_sec_ = config_.auto_confirm_after_sec;
  snapshot_recapture_cooldown_sec_ = config_.snapshots_recapture_cooldown_sec;
  capture_pool_size_ = static_cast<int>(config_.snapshots_capture_pool_size);
  capture_queue_depth_ = static_cast<int>(config_.snapshots_capture_queue_depth);
  capture_queue_full_policy_ = config_.snapshots_capture_queue_full_policy == "drop_oldest"
                                   ? QueueFullPolicy::kDropOldest
                                   : QueueFullPolicy::kRejectNewest;
  audit_confirmed_only_ = (config_.audit_log_transitions == "confirmed_only");
  audit_fail_closed_ = config_.audit_log_fail_closed;

  RCLCPP_INFO(get_logger(), "Capture concurrency: pool_size=%d, queue_depth=%d, policy=%s", capture_pool_size_,
              capture_queue_depth_,
              capture_queue_full_policy_ == QueueFullPolicy::kRejectNewest ? "reject_newest" : "drop_oldest");

  auto parts = core::build_core_parts(config_, log_sink_);
  storage_ = std::move(parts.storage);
  audit_log_ = std::move(parts.audit);
  correlation_engine_ = std::move(parts.correlation);
  global_config_ = parts.global_debounce;
  threshold_resolver_ = std::move(parts.entity_resolver);
  fault_code_resolver_ = std::move(parts.fault_code_resolver);

  // Create event publisher for SSE streaming
  event_publisher_ = create_publisher<ros2_medkit_msgs::msg::FaultEvent>("~/events", rclcpp::QoS(100).reliable());

  // Runs after the override files are loaded, so a code with its own healing_enabled: true keeps
  // its HEALED status. Each flipped fault is audited, or the audit log's verify() would not see it.
  core::reclassify_stale_healed(*storage_, fault_code_resolver_.get(), global_config_, audit_log_ != nullptr, log_sink_,
                                config_.logger_name, [this](const fd::FaultRecord & fault, int64_t reclassified_at_ns) {
                                  audit_transition(kTransitionCleared, fault, "startup_reclassify", reclassified_at_ns);
                                });

  // Create service servers
  report_fault_srv_ = create_service<ros2_medkit_msgs::srv::ReportFault>(
      "~/report_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
                               const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response) {
        handle_report_fault(request, response);
      });

  list_faults_srv_ = create_service<ros2_medkit_msgs::srv::ListFaults>(
      "~/list_faults", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & request,
                              const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & response) {
        handle_list_faults(request, response);
      });

  get_fault_srv_ = create_service<ros2_medkit_msgs::srv::GetFault>(
      "~/get_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                            const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response) {
        handle_get_fault(request, response);
      });

  clear_fault_srv_ = create_service<ros2_medkit_msgs::srv::ClearFault>(
      "~/clear_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
                              const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
        handle_clear_fault(request, response);
      });

  get_snapshots_srv_ = create_service<ros2_medkit_msgs::srv::GetSnapshots>(
      "~/get_snapshots", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Request> & request,
                                const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Response> & response) {
        handle_get_snapshots(request, response);
      });

  get_rosbag_srv_ = create_service<ros2_medkit_msgs::srv::GetRosbag>(
      "~/get_rosbag", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Request> & request,
                             const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Response> & response) {
        handle_get_rosbag(request, response);
      });

  list_rosbags_srv_ = create_service<ros2_medkit_msgs::srv::ListRosbags>(
      "~/list_rosbags", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Request> & request,
                               const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Response> & response) {
        handle_list_rosbags(request, response);
      });

  list_faults_for_entity_srv_ = create_service<ros2_medkit_msgs::srv::ListFaultsForEntity>(
      "~/list_faults_for_entity",
      [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> & request,
             const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> & response) {
        handle_list_faults_for_entity(request, response);
      });

  // Initialize snapshot capture
  auto snapshot_config = create_snapshot_config();
  if (snapshot_config.enabled) {
    snapshot_capture_ = std::make_shared<SnapshotCapture>(this, storage_.get(), snapshot_config);
  }

  // Initialize rosbag capture if enabled
  if (snapshot_config.rosbag.enabled) {
    rosbag_capture_ = std::make_shared<RosbagCapture>(this, storage_.get(), snapshot_config.rosbag, snapshot_config);

    // The recapture cooldown gates the capture job as a whole, bags included, so it
    // puts a floor under how often a history can actually grow: a fault that
    // re-confirms faster than the cooldown keeps ONE recording however high the cap
    // is. That is the flapping fault the cap exists for, so say it out loud at
    // startup rather than leaving an operator to infer it from a short history.
    if (snapshot_config.rosbag.max_bags_per_fault != 1 && snapshot_recapture_cooldown_sec_ > 0.0) {
      RCLCPP_WARN(get_logger(),
                  "snapshots.rosbag.max_bags_per_fault is %zu but snapshots.recapture_cooldown_sec is %.1fs, so a "
                  "fault that returns sooner than that still keeps one recording. Lower the cooldown to collect a "
                  "history of fast-repeating faults.",
                  snapshot_config.rosbag.max_bags_per_fault, snapshot_recapture_cooldown_sec_);
    }
  }

  // Drive captures through a bounded pool instead of one thread per fault.
  if (snapshot_capture_ || rosbag_capture_) {
    auto snap = snapshot_capture_;
    auto bag = rosbag_capture_;
    // RosbagCapture is single-writer (one shared ring buffer + one active writer +
    // one recording state machine), so on_fault_confirmed() is not safe for the
    // pool_size concurrent calls the pool contract allows. Serialize just the rosbag
    // leg under this mutex; snapshot capture is independent and stays parallel.
    auto rosbag_mutex = std::make_shared<std::mutex>();
    capture_pool_ = std::make_unique<CaptureThreadPool>(
        static_cast<std::size_t>(capture_pool_size_), static_cast<std::size_t>(capture_queue_depth_),
        capture_queue_full_policy_, get_logger(), [snap, bag, rosbag_mutex](const std::string & fault_code) {
          if (snap) {
            snap->capture(fault_code);
          }
          if (bag) {
            std::lock_guard<std::mutex> bag_lock(*rosbag_mutex);
            bag->on_fault_confirmed(fault_code);
          }
        });
  }

  // Create correlation cleanup timer if correlation is enabled
  if (correlation_engine_) {
    auto cleanup_interval_sec = declare_parameter<double>("correlation.cleanup_interval_sec", 5.0);
    if (!(std::isfinite(cleanup_interval_sec) && cleanup_interval_sec > 0.0)) {
      RCLCPP_WARN(get_logger(), "correlation.cleanup_interval_sec must be positive, got %.2f. Using default 5.0s",
                  cleanup_interval_sec);
      cleanup_interval_sec = 5.0;
    }
    auto cleanup_interval_ms = static_cast<int64_t>(cleanup_interval_sec * 1000);
    correlation_cleanup_timer_ = create_wall_timer(std::chrono::milliseconds(cleanup_interval_ms), [this]() {
      correlation_engine_->cleanup_expired();
    });
  }

  // Create auto-confirmation timer if enabled
  if (auto_confirm_after_sec_ > 0.0) {
    auto_confirm_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std::vector<std::string> confirmed = {};
      try {
        confirmed = storage_->check_time_based_confirmation(get_wall_clock_ns());
      } catch (const FaultStorage::IgnorableConnectionException & e) {
        RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
        return;
      }
      if (confirmed.empty()) {
        return;
      }
      // Audit every timer-driven PREFAILED->CONFIRMED transition. Without this the
      // confirmations are invisible to the audit log's verify().
      const int64_t confirmed_at_ns = get_wall_clock_ns();
      for (const auto & fault_code : confirmed) {
        std::optional<fd::FaultRecord> fault;
        try {
          fault = storage_->get_fault(fault_code);
        } catch (const FaultStorage::IgnorableConnectionException & e) {
          RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
          return;
        }
        if (fault) {
          audit_transition(kTransitionConfirmed, *fault, "auto_confirm_timer", confirmed_at_ns);
          // A timer-driven confirmation is a confirmation: it has to reach the
          // event stream and the black box exactly like one raised by a report,
          // or subscribers see no alarm and no recording is ever made for it.
          //
          // Muting is checked for the same reason the report path checks it: a
          // symptom suppressed by a root cause must not be announced, or SSE and
          // the trigger subscribers see an alarm that the fault list hides.
          // Capture is deliberately not gated, matching the report path, where
          // just_confirmed is set regardless of muting.
          if (!correlation_engine_ || !correlation_engine_->is_muted(fault_code)) {
            publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault);
          }
          capture_on_confirm(fault_code);
        }
      }
      RCLCPP_INFO(get_logger(), "Auto-confirmed %zu PREFAILED fault(s) due to time threshold", confirmed.size());
    });
    RCLCPP_INFO(get_logger(),
                "FaultManager node started (storage=%s, confirmation_threshold=%d, "
                "healing=%s, healing_threshold=%d, auto_confirm_after=%.1fs)",
                storage_type_.c_str(), global_config_.confirmation_threshold,
                global_config_.healing_enabled ? "enabled" : "disabled", global_config_.healing_threshold,
                auto_confirm_after_sec_);
  } else {
    RCLCPP_INFO(get_logger(),
                "FaultManager node started (storage=%s, confirmation_threshold=%d, healing=%s, "
                "healing_threshold=%d)",
                storage_type_.c_str(), global_config_.confirmation_threshold,
                global_config_.healing_enabled ? "enabled" : "disabled", global_config_.healing_threshold);
  }
}

FaultManagerNode::~FaultManagerNode() {
  // Join capture workers FIRST so no worker is mid-capture when rosbag tears
  // down. RosbagCapture::stop() is not a barrier against an in-flight
  // on_fault_confirmed() (it clears subscriptions/buffers without serializing
  // against the capture path). See issue #441.
  if (capture_pool_) {
    capture_pool_->shutdown();
  }
  if (rosbag_capture_) {
    rosbag_capture_->stop();
  }

  // Close the chain with a "logging deactivated" marker. Best-effort and appended
  // directly (recorded even in confirmed_only mode); a failure here must not throw
  // out of the destructor.
  if (audit_log_) {
    try {
      AuditEvent marker;
      marker.fault_code = "__audit__";
      marker.transition = kTransitionLoggingDeactivated;
      marker.status = "INACTIVE";
      marker.source_id = "fault_manager";
      marker.description = "audit logging deactivated";
      marker.occurred_at_ns = get_wall_clock_ns();
      audit_log_->append(marker);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to append audit 'logging_deactivated' marker: %s", e.what());
    }
  }
}

void FaultManagerNode::audit_transition(const char * transition, const fd::FaultRecord & fault,
                                        const std::string & source_id, int64_t occurred_at_ns) {
  if (!audit_log_) {
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
    audit_log_->append(event);
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
    RCLCPP_ERROR(get_logger(), "Failed to append audit record for '%s' (%s): %s [audit dropped_writes=%" PRIu64 "]",
                 fault.fault_code.c_str(), transition, e.what(), dropped);
    if (audit_fail_closed_) {
      // Explicit fail-FAST: surface the broken audit to the caller so a
      // compliance-strict deployment learns the chain is now incomplete and an
      // operator must act. Rethrowing propagates that signal ONLY - the fault-
      // state change already committed (see above) and is NOT rolled back; this
      // is not atomicity.
      RCLCPP_FATAL(get_logger(),
                   "audit_log.fail_closed is set: audit append failed for '%s' (%s); the audit chain is now "
                   "incomplete and requires operator action. The already-committed fault-state change is NOT "
                   "rolled back (the fault store is a separate database).",
                   fault.fault_code.c_str(), transition);
      throw;
    }
  }
}

void FaultManagerNode::capture_on_confirm(const std::string & fault_code) {
  // Capture snapshots/rosbag when a fault confirms, via the bounded pool.
  // Both callers - the report handler and the auto-confirm timer - run on the
  // node's single-threaded executor, so confirmations are already serialized;
  // last_capture_mutex_ only guards last_capture_times_ itself (the cooldown
  // check, the update below, and the expired-entry sweep). It is taken solely
  // when the cooldown is enabled, since the map is otherwise never touched.
  if (!capture_pool_) {
    return;
  }
  const bool cooldown_enabled = snapshot_recapture_cooldown_sec_ > 0.0;
  std::unique_lock<std::mutex> cd_lock(last_capture_mutex_, std::defer_lock);
  if (cooldown_enabled) {
    cd_lock.lock();
  }

  bool on_cooldown = false;
  if (cooldown_enabled) {
    const auto cooldown = std::chrono::duration<double>(snapshot_recapture_cooldown_sec_);
    const auto sweep_now = std::chrono::steady_clock::now();
    // Bound the map (issue #441): a storm of distinct fault codes would otherwise
    // leave one permanent entry per code. Entries older than the cooldown can never
    // gate a capture again, so drop them while we hold the lock.
    for (auto it = last_capture_times_.begin(); it != last_capture_times_.end();) {
      if (sweep_now - it->second >= cooldown) {
        it = last_capture_times_.erase(it);
      } else {
        ++it;
      }
    }
    auto it = last_capture_times_.find(fault_code);
    if (it != last_capture_times_.end()) {
      on_cooldown = (sweep_now - it->second) < cooldown;
    }
  }

  if (on_cooldown) {
    RCLCPP_DEBUG(get_logger(), "Skipping capture for '%s' - cooldown active", fault_code.c_str());
  } else {
    const EnqueueOutcome outcome = capture_pool_->enqueue(fault_code);
    const auto now = std::chrono::steady_clock::now();
    // RCLCPP_WARN_THROTTLE needs a non-const Clock lvalue (Humble/Lyrical
    // compat); mirror rosbag_capture.cpp's local-copy pattern. Cast the
    // uint64_t counter to unsigned long long + %llu to avoid -Wuseless-cast
    // (uint64_t == unsigned long on LP64).
    rclcpp::Clock throttle_clock(*get_clock());
    switch (outcome.result) {
      case EnqueueResult::kAccepted:
        if (cooldown_enabled) {
          last_capture_times_[fault_code] = now;
        }
        break;
      case EnqueueResult::kEvictedOldest:
        if (cooldown_enabled) {
          last_capture_times_[fault_code] = now;
          if (outcome.evicted_code) {
            last_capture_times_.erase(*outcome.evicted_code);  // keep evicted fault retriable
          }
        }
        RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock, 2000,
                             "Capture queue full (drop_oldest): evicted pending '%s' for '%s' "
                             "(pool=%d, queue=%d, total_dropped=%llu)",
                             outcome.evicted_code ? outcome.evicted_code->c_str() : "?", fault_code.c_str(),
                             capture_pool_size_, capture_queue_depth_,
                             static_cast<unsigned long long>(capture_pool_->dropped_captures()));
        break;
      case EnqueueResult::kDroppedNewest:
        // Cooldown NOT recorded: capture still possible if the fault later
        // heals/clears and re-confirms.
        RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock, 2000,
                             "Capture queue full (reject_newest): dropped capture for '%s' "
                             "(pool=%d, queue=%d, total_dropped=%llu)",
                             fault_code.c_str(), capture_pool_size_, capture_queue_depth_,
                             static_cast<unsigned long long>(capture_pool_->dropped_captures()));
        break;
      case EnqueueResult::kRejectedShuttingDown:
        RCLCPP_DEBUG(get_logger(), "Capture pool shutting down; skipped capture for '%s'", fault_code.c_str());
        break;
    }
  }
}

void FaultManagerNode::handle_report_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: %s", validation_error.c_str());
    return;
  }

  // Validate event_type
  if (request->event_type != ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED &&
      request->event_type != ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: invalid event_type %d", request->event_type);
    return;
  }

  // For FAILED events, validate severity
  if (request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED &&
      !is_valid_severity(request->severity)) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: invalid severity %d for FAILED event", request->severity);
    return;
  }

  // Validate source_id
  if (request->source_id.empty()) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: source_id cannot be empty");
    return;
  }

  // Get status before update (if fault exists)
  std::optional<fd::FaultRecord> fault_before;
  try {
    fault_before = storage_->get_fault(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    return;
  }
  std::string status_before = fault_before ? fault_before->status : "";

  // Resolve the debounce config: global, then the entity override matching
  // source_id, then the fault code's own override.
  auto resolved_config = resolve_config(request->source_id, request->fault_code);

  // Report the fault event (use wall clock time, not sim time, for proper timestamps)
  const int64_t event_time_ns = get_wall_clock_ns();
  bool is_new = false;
  const auto kind = request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED
                        ? fd::ReportKind::kFailed
                        : fd::ReportKind::kPassed;
  try {
    is_new = storage_->report_fault_event(request->fault_code, kind, request->severity, request->description,
                                          request->source_id, event_time_ns, resolved_config);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    return;
  }

  response->accepted = true;

  // Get updated fault state to publish event
  std::optional<fd::FaultRecord> fault_after;
  try {
    fault_after = storage_->get_fault(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    return;
  }
  if (fault_after) {
    // Only for codes the store holds: a PASSED for an unknown code writes nothing, so it must
    // not add an entry here either (the diagnostic bridge sends PASSED for every OK status).
    warn_on_conflicting_debounce_policy(request->fault_code, request->source_id, resolved_config);

    // Process through correlation engine (if enabled)
    // Only process FAILED events with correlation
    bool should_mute = false;
    if (correlation_engine_ && request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED) {
      auto correlation_result =
          correlation_engine_->process_fault(request->fault_code, correlation::severity_to_string(request->severity));

      should_mute = correlation_result.should_mute;

      if (correlation_result.is_root_cause) {
        RCLCPP_DEBUG(get_logger(), "Fault %s identified as root cause (rule=%s)", request->fault_code.c_str(),
                     correlation_result.rule_id.c_str());
      } else if (should_mute) {
        RCLCPP_DEBUG(get_logger(), "Fault %s muted as symptom of %s (rule=%s, delay=%ums)", request->fault_code.c_str(),
                     correlation_result.root_cause_code.c_str(), correlation_result.rule_id.c_str(),
                     correlation_result.delay_ms);
      } else if (!correlation_result.cluster_id.empty()) {
        RCLCPP_DEBUG(get_logger(), "Fault %s added to cluster %s", request->fault_code.c_str(),
                     correlation_result.cluster_id.c_str());
        // Log retroactively muted faults when cluster becomes active
        if (!correlation_result.retroactive_mute_codes.empty()) {
          RCLCPP_DEBUG(get_logger(), "Cluster %s activated: retroactively muting %zu faults",
                       correlation_result.cluster_id.c_str(), correlation_result.retroactive_mute_codes.size());
        }
      }
    }

    // Determine event type based on status transition
    bool just_confirmed = false;
    if (is_new && fault_after->status == fd::fault_status::kConfirmed) {
      // New fault immediately confirmed (e.g., CRITICAL severity or threshold=-1)
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && status_before != fd::fault_status::kConfirmed &&
               fault_after->status == fd::fault_status::kConfirmed) {
      // Existing fault transitioned to CONFIRMED
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && fault_after->status == fd::fault_status::kConfirmed) {
      // Fault was already CONFIRMED, data updated (last_occurred, severity, sources).
      // Not occurrence_count: a re-report inside one occurrence does not touch it.
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_UPDATED, *fault_after);
      }
    }
    // Note: PREFAILED/PREPASSED status changes don't emit events (debounce in progress)

    // A PASSED event can drive an existing fault past the healing threshold to
    // HEALED. That is the fault's END and must be audited (with a distinct kind
    // and source) so the timeline is complete, not just occurred+confirmed.
    const bool just_healed =
        !is_new && status_before != fd::fault_status::kHealed && fault_after->status == fd::fault_status::kHealed;

    // The end of a fault must reach the event stream too, or an SSE consumer keeps
    // showing a fault that is over. EVENT_CLEARED (not a new event name) so existing
    // consumers act on it; the payload's status = HEALED distinguishes auto-heal from
    // an acknowledged clear.
    if (just_healed) {
      publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CLEARED, *fault_after);
    }

    // Append tamper-evident audit records for the transitions that just happened.
    // Recorded regardless of correlation muting: muting affects display, not the
    // fact that the state transition occurred.
    if (is_new) {
      audit_transition(kTransitionOccurred, *fault_after, request->source_id, event_time_ns);
    }
    if (just_confirmed) {
      audit_transition(kTransitionConfirmed, *fault_after, request->source_id, event_time_ns);
    }
    if (just_healed) {
      audit_transition(kTransitionHealed, *fault_after, "auto_heal", event_time_ns);
    }

    if (just_confirmed) {
      capture_on_confirm(request->fault_code);
    }

    // Handle PREFAILED state for lazy_start rosbag capture
    bool just_prefailed = (is_new && fault_after->status == fd::fault_status::kPrefailed) ||
                          (!is_new && status_before != fd::fault_status::kPrefailed &&
                           fault_after->status == fd::fault_status::kPrefailed);
    if (just_prefailed && rosbag_capture_) {
      rosbag_capture_->on_fault_prefailed(request->fault_code);
    }
  }

  if (request->event_type == ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED) {
    if (is_new) {
      RCLCPP_INFO(get_logger(), "New fault reported: %s (severity=%d, source=%s)", request->fault_code.c_str(),
                  request->severity, request->source_id.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "Fault updated: %s (source=%s)", request->fault_code.c_str(),
                   request->source_id.c_str());
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "PASSED event for fault: %s (source=%s)", request->fault_code.c_str(),
                 request->source_id.c_str());
  }
}

void FaultManagerNode::handle_list_faults(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & response) {
  try {
    const auto records = storage_->list_faults(request->filter_by_severity, request->severity, request->statuses);
    response->faults.reserve(records.size());
    for (const auto & record : records) {
      response->faults.push_back(ros2::to_msg(record));
    }
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    return;
  }

  // Include correlation data if engine is enabled
  if (correlation_engine_) {
    // Always include counts
    response->muted_count = correlation_engine_->get_muted_count();
    response->cluster_count = correlation_engine_->get_cluster_count();

    auto muted_faults = correlation_engine_->get_muted_faults();

    // Include muted faults details if requested
    if (request->include_muted) {
      response->muted_faults.reserve(muted_faults.size());
      for (const auto & muted : muted_faults) {
        ros2_medkit_msgs::msg::MutedFaultInfo info;
        info.fault_code = muted.fault_code;
        info.root_cause_code = muted.root_cause_code;
        info.rule_id = muted.rule_id;
        info.delay_ms = muted.delay_ms;
        response->muted_faults.push_back(info);
      }
    } else {
      // Build a set of muted codes for O(1) lookup.
      std::unordered_set<std::string> muted_codes;
      for (const auto & muted : muted_faults) {
        muted_codes.insert(muted.fault_code);
      }

      // Remove all faults whose code is muted, in one pass.
      auto & faults = response->faults;
      faults.erase(std::remove_if(faults.begin(), faults.end(),
                                  [&muted_codes](const ros2_medkit_msgs::msg::Fault & fault) {
                                    return muted_codes.count(fault.fault_code) > 0;
                                  }),
                   faults.end());
    }

    // Include cluster details if requested
    if (request->include_clusters) {
      auto clusters = correlation_engine_->get_clusters();
      response->clusters.reserve(clusters.size());
      for (const auto & cluster : clusters) {
        ros2_medkit_msgs::msg::ClusterInfo info;
        info.cluster_id = cluster.cluster_id;
        info.rule_id = cluster.rule_id;
        info.rule_name = cluster.rule_name;
        info.label = cluster.label;
        info.representative_code = cluster.representative_code;
        info.representative_severity = cluster.representative_severity;
        info.fault_codes = cluster.fault_codes;
        info.count = static_cast<uint32_t>(cluster.fault_codes.size());
        // Convert chrono time_points to ROS time
        auto first_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(cluster.first_at.time_since_epoch()).count();
        auto last_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(cluster.last_at.time_since_epoch()).count();
        info.first_at = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
        info.last_at = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);
        response->clusters.push_back(info);
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "ListFaults returned %zu faults (muted=%u, clusters=%u)", response->faults.size(),
               response->muted_count, response->cluster_count);
}

void FaultManagerNode::handle_clear_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->message = validation_error;
    return;
  }

  // Storage first: a clear that the storage refuses must not change the correlation state.
  bool cleared = false;
  try {
    cleared = storage_->clear_fault(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->message = storage_unavailable(e);
    return;
  }

  // Then the correlation engine (to get the auto-clear list).
  // `skip_correlation_auto_clear` lets the caller opt out of cascade-clearing
  // correlated symptom fault codes. Per-entity DELETE routes set it to true
  // so they cannot reach across entity boundaries via the correlation graph.
  std::vector<std::string> auto_cleared_codes;
  if (correlation_engine_ && !request->skip_correlation_auto_clear) {
    auto clear_result = correlation_engine_->process_clear(request->fault_code);
    auto_cleared_codes = clear_result.auto_cleared_codes;
  }

  response->success = cleared;
  if (cleared) {
    // Evict cooldown tracking for cleared fault and auto-cleared symptoms
    {
      std::lock_guard<std::mutex> lock(last_capture_mutex_);
      last_capture_times_.erase(request->fault_code);
      for (const auto & symptom_code : auto_cleared_codes) {
        last_capture_times_.erase(symptom_code);
      }
    }

    // Auto-clear correlated symptoms
    for (const auto & symptom_code : auto_cleared_codes) {
      try {
        storage_->clear_fault(symptom_code);
      } catch (const FaultStorage::IgnorableConnectionException & e) {
        RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
        response->message = storage_unavailable(e);
        return;
      }
      if (audit_log_) {
        std::optional<fd::FaultRecord> symptom;
        try {
          symptom = storage_->get_fault(symptom_code);
        } catch (const FaultStorage::IgnorableConnectionException & e) {
          RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
          response->message = storage_unavailable(e);
          return;
        }
        if (symptom) {
          audit_transition(kTransitionCleared, *symptom, "clear_service", get_wall_clock_ns());
        }
      }
      RCLCPP_DEBUG(get_logger(), "Auto-cleared symptom: %s (root cause: %s)", symptom_code.c_str(),
                   request->fault_code.c_str());
      // Also cleanup rosbag for auto-cleared faults
      if (rosbag_capture_) {
        rosbag_capture_->on_fault_cleared(symptom_code);
      }
    }

    response->auto_cleared_codes = auto_cleared_codes;
    if (auto_cleared_codes.empty()) {
      response->message = "Fault cleared: " + request->fault_code;
    } else {
      response->message = "Fault cleared: " + request->fault_code + " (auto-cleared " +
                          std::to_string(auto_cleared_codes.size()) + " symptoms)";
    }
    RCLCPP_INFO(get_logger(), "Fault cleared: %s (auto-cleared %zu symptoms)", request->fault_code.c_str(),
                auto_cleared_codes.size());

    // Cleanup rosbag for the main fault (auto_cleanup handled inside RosbagCapture)
    if (rosbag_capture_) {
      rosbag_capture_->on_fault_cleared(request->fault_code);
    }

    // Publish EVENT_CLEARED - get the cleared fault to include in event
    std::optional<fd::FaultRecord> fault;
    try {
      fault = storage_->get_fault(request->fault_code);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      response->message = storage_unavailable(e);
      return;
    }
    if (fault) {
      publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CLEARED, *fault, auto_cleared_codes);
      audit_transition(kTransitionCleared, *fault, "clear_service", get_wall_clock_ns());
    }
  } else {
    response->message = "Fault not found: " + request->fault_code;
    RCLCPP_WARN(get_logger(), "Attempted to clear non-existent fault: %s", request->fault_code.c_str());
  }
}

bool FaultManagerNode::is_valid_severity(uint8_t severity) {
  return severity <= fd::severity::kCritical;
}

std::string FaultManagerNode::extract_topic_name(const std::string & topic_path) {
  auto pos = topic_path.rfind('/');
  if (pos != std::string::npos && pos < topic_path.length() - 1) {
    return topic_path.substr(pos + 1);
  }
  return topic_path;
}

void FaultManagerNode::handle_get_fault(const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                                        const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response) {
  RCLCPP_DEBUG(get_logger(), "GetFault request for: %s", request->fault_code.c_str());

  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->error_message = validation_error;
    return;
  }

  // Get fault from storage
  std::optional<fd::FaultRecord> fault;
  try {
    fault = storage_->get_fault(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  response->success = true;
  response->fault = ros2::to_msg(*fault);

  // Populate extended_data_records with timestamps
  ros2_medkit_msgs::msg::ExtendedDataRecords extended_records;
  extended_records.first_occurrence_ns = fault->first_occurred_ns;
  extended_records.last_occurrence_ns = fault->last_occurred_ns;
  response->environment_data.extended_data_records = extended_records;

  // Get freeze frame snapshots from storage
  std::vector<SnapshotData> stored_snapshots = {};
  try {
    stored_snapshots = storage_->get_snapshots(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }
  for (const auto & stored_snapshot : stored_snapshots) {
    ros2_medkit_msgs::msg::Snapshot snapshot;
    snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_FREEZE_FRAME;
    snapshot.name = extract_topic_name(stored_snapshot.topic);
    snapshot.data = stored_snapshot.data;
    snapshot.topic = stored_snapshot.topic;
    snapshot.message_type = stored_snapshot.message_type;
    snapshot.captured_at_ns = stored_snapshot.captured_at_ns;
    // Rosbag fields left empty for freeze_frame type
    response->environment_data.snapshots.push_back(snapshot);
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
      frame = storage_->get_freeze_frame(request->fault_code);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      response->success = false;
      response->error_message = storage_unavailable(e);
      return;
    }
    if (frame) {
      ros2_medkit_msgs::msg::Snapshot snapshot;
      snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_FREEZE_FRAME;
      snapshot.name = "freeze_frame";
      snapshot.data = frame->data;
      // topic/message_type left empty: the frame aggregates values from multiple topics
      snapshot.captured_at_ns = frame->captured_at_ns;
      response->environment_data.snapshots.push_back(snapshot);
    }
  }

  // One entry per recording, newest first. A fault that keeps re-confirming leaves a
  // trail of black boxes, and every one of them needs its own addressable id here -
  // this list is the only place the fault itself advertises them.
  try {
    for (const auto & rosbag_info : storage_->get_rosbag_files(request->fault_code)) {
      ros2_medkit_msgs::msg::Snapshot rosbag_snapshot;
      rosbag_snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_ROSBAG;
      rosbag_snapshot.name = "rosbag_" + rosbag_info.recording_id;
      // The RECORDING, not the fault code: several recordings of one fault would
      // otherwise all carry the same id and collapse into one download.
      rosbag_snapshot.bulk_data_id = rosbag_info.recording_id;
      rosbag_snapshot.size_bytes = rosbag_info.size_bytes;
      rosbag_snapshot.duration_sec = rosbag_info.duration_sec;
      rosbag_snapshot.format = rosbag_info.format;
      rosbag_snapshot.captured_at_ns = rosbag_info.created_at_ns;
      // Freeze frame fields left empty for rosbag type
      response->environment_data.snapshots.push_back(rosbag_snapshot);
    }
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "GetFault returned fault '%s' with %zu snapshots", request->fault_code.c_str(),
               response->environment_data.snapshots.size());
}

void FaultManagerNode::publish_fault_event(const std::string & event_type, const fd::FaultRecord & fault,
                                           const std::vector<std::string> & auto_cleared_codes) {
  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = event_type;
  event.fault = ros2::to_msg(fault);
  event.timestamp = get_wall_clock_time();
  event.auto_cleared_codes = auto_cleared_codes;

  event_publisher_->publish(event);

  RCLCPP_DEBUG(get_logger(), "Published fault event: %s for fault_code=%s", event_type.c_str(),
               fault.fault_code.c_str());
}

core::FaultManagerConfig FaultManagerNode::declare_config() {
  core::FaultManagerConfig c;
  c.storage_type = declare_parameter<std::string>("storage_type", c.storage_type);
  c.database_path = declare_parameter<std::string>("database_path", c.database_path);
  c.database_url = declare_parameter<std::string>("database_url", c.database_url);
  c.confirmation_threshold = declare_parameter<int64_t>("confirmation_threshold", c.confirmation_threshold);
  c.healing_enabled = declare_parameter<bool>("healing_enabled", c.healing_enabled);
  c.healing_threshold = declare_parameter<int64_t>("healing_threshold", c.healing_threshold);
  c.auto_confirm_after_sec = declare_parameter<double>("auto_confirm_after_sec", c.auto_confirm_after_sec);
  c.snapshots_recapture_cooldown_sec =
      declare_parameter<double>("snapshots.recapture_cooldown_sec", c.snapshots_recapture_cooldown_sec);
  c.snapshots_capture_pool_size =
      declare_parameter<int64_t>("snapshots.capture_pool_size", c.snapshots_capture_pool_size);
  c.snapshots_capture_queue_depth =
      declare_parameter<int64_t>("snapshots.capture_queue_depth", c.snapshots_capture_queue_depth);
  c.snapshots_capture_queue_full_policy =
      declare_parameter<std::string>("snapshots.capture_queue_full_policy", c.snapshots_capture_queue_full_policy);
  c.snapshots_retain_on_clear = declare_parameter<bool>("snapshots.retain_on_clear", c.snapshots_retain_on_clear);
  c.snapshots_max_per_fault = declare_parameter<int64_t>("snapshots.max_per_fault", c.snapshots_max_per_fault);
  c.near_miss_max_per_fault = declare_parameter<int64_t>("near_miss.max_per_fault", c.near_miss_max_per_fault);
  c.entity_thresholds_config_file =
      declare_parameter<std::string>("entity_thresholds.config_file", c.entity_thresholds_config_file);
  c.fault_thresholds_config_file =
      declare_parameter<std::string>("fault_thresholds.config_file", c.fault_thresholds_config_file);
  c.audit_log_enabled = declare_parameter<bool>("audit_log.enabled", c.audit_log_enabled);
  c.audit_log_transitions = declare_parameter<std::string>("audit_log.transitions", c.audit_log_transitions);
  c.audit_log_retention_max_records =
      declare_parameter<int64_t>("audit_log.retention_max_records", c.audit_log_retention_max_records);
  c.audit_log_fail_closed = declare_parameter<bool>("audit_log.fail_closed", c.audit_log_fail_closed);
  c.audit_log_database_path = declare_parameter<std::string>("audit_log.database_path", c.audit_log_database_path);
  c.correlation_config_file = declare_parameter<std::string>("correlation.config_file", c.correlation_config_file);

  c.snapshots_enabled = declare_parameter<bool>("snapshots.enabled", c.snapshots_enabled);
  c.snapshots_background_capture =
      declare_parameter<bool>("snapshots.background_capture", c.snapshots_background_capture);
  c.snapshots_timeout_sec = declare_parameter<double>("snapshots.timeout_sec", c.snapshots_timeout_sec);
  c.snapshots_max_message_size = declare_parameter<int64_t>("snapshots.max_message_size", c.snapshots_max_message_size);
  c.snapshots_default_topics =
      declare_parameter<std::vector<std::string>>("snapshots.default_topics", c.snapshots_default_topics);
  c.snapshots_entity_default = declare_parameter<bool>("snapshots.entity_default", c.snapshots_entity_default);
  c.snapshots_config_file = declare_parameter<std::string>("snapshots.config_file", c.snapshots_config_file);

  // The rosbag parameters are declared only when rosbag capture is on.
  c.snapshots_rosbag_enabled = declare_parameter<bool>("snapshots.rosbag.enabled", c.snapshots_rosbag_enabled);
  if (c.snapshots_rosbag_enabled) {
    c.snapshots_rosbag_duration_sec =
        declare_parameter<double>("snapshots.rosbag.duration_sec", c.snapshots_rosbag_duration_sec);
    c.snapshots_rosbag_duration_after_sec =
        declare_parameter<double>("snapshots.rosbag.duration_after_sec", c.snapshots_rosbag_duration_after_sec);
    c.snapshots_rosbag_topics = declare_parameter<std::string>("snapshots.rosbag.topics", c.snapshots_rosbag_topics);
    c.snapshots_rosbag_include_topics = declare_parameter<std::vector<std::string>>("snapshots.rosbag.include_topics",
                                                                                    c.snapshots_rosbag_include_topics);
    c.snapshots_rosbag_exclude_topics = declare_parameter<std::vector<std::string>>("snapshots.rosbag.exclude_topics",
                                                                                    c.snapshots_rosbag_exclude_topics);
    c.snapshots_rosbag_exclude_sensor_topics =
        declare_parameter<bool>("snapshots.rosbag.exclude_sensor_topics", c.snapshots_rosbag_exclude_sensor_topics);
    c.snapshots_rosbag_qos_match = declare_parameter<bool>("snapshots.rosbag.qos_match", c.snapshots_rosbag_qos_match);
    c.snapshots_rosbag_lazy_start =
        declare_parameter<bool>("snapshots.rosbag.lazy_start", c.snapshots_rosbag_lazy_start);
    c.snapshots_rosbag_format = declare_parameter<std::string>("snapshots.rosbag.format", c.snapshots_rosbag_format);
    c.snapshots_rosbag_storage_path =
        declare_parameter<std::string>("snapshots.rosbag.storage_path", c.snapshots_rosbag_storage_path);
    c.snapshots_rosbag_max_bag_size_mb =
        declare_parameter<int64_t>("snapshots.rosbag.max_bag_size_mb", c.snapshots_rosbag_max_bag_size_mb);
    c.snapshots_rosbag_max_total_storage_mb =
        declare_parameter<int64_t>("snapshots.rosbag.max_total_storage_mb", c.snapshots_rosbag_max_total_storage_mb);
    c.snapshots_rosbag_max_buffer_mb =
        declare_parameter<int64_t>("snapshots.rosbag.max_buffer_mb", c.snapshots_rosbag_max_buffer_mb);
    c.snapshots_rosbag_max_bags_per_fault =
        declare_parameter<int64_t>("snapshots.rosbag.max_bags_per_fault", c.snapshots_rosbag_max_bags_per_fault);
    c.snapshots_rosbag_auto_cleanup =
        declare_parameter<bool>("snapshots.rosbag.auto_cleanup", c.snapshots_rosbag_auto_cleanup);
  }
  return c;
}

SnapshotConfig FaultManagerNode::create_snapshot_config() {
  SnapshotConfig config;

  config.enabled = config_.snapshots_enabled;
  config.background_capture = config_.snapshots_background_capture;
  config.timeout_sec = config_.snapshots_timeout_sec;
  config.max_message_size = static_cast<size_t>(config_.snapshots_max_message_size);

  config.default_topics = config_.snapshots_default_topics;

  config.entity_default = config_.snapshots_entity_default;

  if (!config_.snapshots_config_file.empty()) {
    load_snapshot_config_from_yaml(config_.snapshots_config_file, config);
  }

  config.rosbag.enabled = config_.snapshots_rosbag_enabled;
  if (config.rosbag.enabled) {
    config.rosbag.duration_sec = config_.snapshots_rosbag_duration_sec;
    config.rosbag.duration_after_sec = config_.snapshots_rosbag_duration_after_sec;
    config.rosbag.topics = config_.snapshots_rosbag_topics;
    config.rosbag.include_topics = config_.snapshots_rosbag_include_topics;
    config.rosbag.exclude_topics = config_.snapshots_rosbag_exclude_topics;
    config.rosbag.exclude_sensor_topics = config_.snapshots_rosbag_exclude_sensor_topics;
    config.rosbag.qos_match = config_.snapshots_rosbag_qos_match;
    config.rosbag.lazy_start = config_.snapshots_rosbag_lazy_start;
    config.rosbag.format = config_.snapshots_rosbag_format;
    config.rosbag.storage_path = config_.snapshots_rosbag_storage_path;
    config.rosbag.max_bag_size_mb = static_cast<size_t>(config_.snapshots_rosbag_max_bag_size_mb);
    config.rosbag.max_total_storage_mb = static_cast<size_t>(config_.snapshots_rosbag_max_total_storage_mb);
    config.rosbag.max_buffer_mb = static_cast<size_t>(config_.snapshots_rosbag_max_buffer_mb);
    config.rosbag.max_bags_per_fault = static_cast<size_t>(config_.snapshots_rosbag_max_bags_per_fault);
    config.rosbag.auto_cleanup = config_.snapshots_rosbag_auto_cleanup;

    RCLCPP_INFO(get_logger(),
                "Rosbag capture enabled (duration=%.1fs+%.1fs, topics=%s, qos_match=%s, lazy=%s, format=%s, "
                "max_buffer=%zuMB, max_bag=%zuMB, max_total=%zuMB)",
                config.rosbag.duration_sec, config.rosbag.duration_after_sec, config.rosbag.topics.c_str(),
                config.rosbag.qos_match ? "true" : "false", config.rosbag.lazy_start ? "true" : "false",
                config.rosbag.format.c_str(), config.rosbag.max_buffer_mb, config.rosbag.max_bag_size_mb,
                config.rosbag.max_total_storage_mb);
  }

  if (config.enabled) {
    RCLCPP_INFO(get_logger(),
                "Snapshot capture enabled (background=%s, timeout=%.1fs, max_size=%zu, "
                "fault_specific=%zu, patterns=%zu, default_topics=%zu)",
                config.background_capture ? "true" : "false", config.timeout_sec, config.max_message_size,
                config.fault_specific.size(), config.patterns.size(), config.default_topics.size());
  } else {
    RCLCPP_INFO(get_logger(), "Snapshot capture disabled");
  }

  return config;
}

void FaultManagerNode::load_snapshot_config_from_yaml(const std::string & config_file, SnapshotConfig & config) {
  if (!std::filesystem::exists(config_file)) {
    RCLCPP_ERROR(get_logger(), "Snapshot config file not found: %s", config_file.c_str());
    return;
  }

  try {
    YAML::Node yaml_config = YAML::LoadFile(config_file);

    // Load fault_specific: map<fault_code, vector<topics>>
    if (yaml_config["fault_specific"]) {
      for (const auto & item : yaml_config["fault_specific"]) {
        std::string fault_code = item.first.as<std::string>();
        std::vector<std::string> topics;
        for (const auto & topic : item.second) {
          topics.push_back(topic.as<std::string>());
        }
        config.fault_specific[fault_code] = topics;
        RCLCPP_DEBUG(get_logger(), "Loaded fault_specific[%s] with %zu topics", fault_code.c_str(), topics.size());
      }
    }

    // Load patterns: map<regex_pattern, vector<topics>>
    if (yaml_config["patterns"]) {
      for (const auto & item : yaml_config["patterns"]) {
        std::string pattern = item.first.as<std::string>();
        std::vector<std::string> topics;
        for (const auto & topic : item.second) {
          topics.push_back(topic.as<std::string>());
        }
        config.patterns[pattern] = topics;
        RCLCPP_DEBUG(get_logger(), "Loaded pattern[%s] with %zu topics", pattern.c_str(), topics.size());
      }
    }

    // Optionally override default_topics from YAML (if specified and not already set via parameter)
    if (yaml_config["default_topics"] && config.default_topics.empty()) {
      for (const auto & topic : yaml_config["default_topics"]) {
        config.default_topics.push_back(topic.as<std::string>());
      }
      RCLCPP_DEBUG(get_logger(), "Loaded %zu default_topics from config file", config.default_topics.size());
    }

    RCLCPP_INFO(get_logger(), "Loaded snapshot config from %s (fault_specific=%zu, patterns=%zu)", config_file.c_str(),
                config.fault_specific.size(), config.patterns.size());

  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse snapshot config file %s: %s", config_file.c_str(), e.what());
  }
}

void FaultManagerNode::handle_get_snapshots(
    const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::GetSnapshots::Response> & response) {
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->error_message = validation_error;
    return;
  }

  // Check if fault exists
  std::optional<fd::FaultRecord> fault;
  try {
    fault = storage_->get_fault(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  // Get snapshots from storage, newest capture set first.
  std::vector<SnapshotData> snapshots = {};
  try {
    snapshots = storage_->get_snapshots(request->fault_code, request->topic);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }

  // One capture set, not a blend of several. The response is a topic -> value map,
  // so folding every retained set into it makes the last row per topic win: with
  // several sets kept, some topics would come from the newest reading and some from
  // an older one, under a single captured_at. Keeping only the newest set means the
  // values shown were all sampled at the same moment, which is what a freeze frame
  // claims to be.
  if (!snapshots.empty()) {
    const int64_t newest_capture = snapshots.front().capture_id;
    snapshots.erase(std::remove_if(snapshots.begin(), snapshots.end(),
                                   [newest_capture](const SnapshotData & s) {
                                     return s.capture_id != newest_capture;
                                   }),
                    snapshots.end());
  }

  // Build JSON response
  nlohmann::json result;
  result["fault_code"] = request->fault_code;

  // Find the latest capture time (only include if snapshots exist)
  if (!snapshots.empty()) {
    int64_t latest_captured_at = 0;
    for (const auto & snapshot : snapshots) {
      if (snapshot.captured_at_ns > latest_captured_at) {
        latest_captured_at = snapshot.captured_at_ns;
      }
    }
    result["captured_at"] = static_cast<double>(latest_captured_at) / 1e9;
  }

  // Build topics object. One topic can carry several snapshots - re-confirmations within a cycle,
  // and every earlier occurrence once snapshots.retain_on_clear keeps them - and only one of them
  // fits the topic's key. Keep the newest, tracked explicitly: the backends return these in
  // opposite orders, so relying on the last write to win serves the oldest value from SQLite and
  // the newest from memory, both under the latest captured_at.
  nlohmann::json topics_json = nlohmann::json::object();
  std::map<std::string, int64_t> newest_per_topic;
  for (const auto & snapshot : snapshots) {
    auto seen = newest_per_topic.find(snapshot.topic);
    if (seen != newest_per_topic.end() && seen->second >= snapshot.captured_at_ns) {
      continue;
    }
    newest_per_topic[snapshot.topic] = snapshot.captured_at_ns;

    nlohmann::json topic_entry;
    topic_entry["message_type"] = snapshot.message_type;

    // Parse the stored JSON data
    try {
      topic_entry["data"] = nlohmann::json::parse(snapshot.data);
    } catch (const nlohmann::json::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to parse snapshot data for topic '%s': %s", snapshot.topic.c_str(), e.what());
      topic_entry["data"] = snapshot.data;  // Store as raw string if parsing fails
    }

    topics_json[snapshot.topic] = topic_entry;
  }
  result["topics"] = topics_json;

  // Include rosbag info if available
  std::optional<RosbagFileInfo> rosbag_info;
  try {
    rosbag_info = storage_->get_rosbag_file(request->fault_code);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }
  if (rosbag_info) {
    nlohmann::json rosbag_json;
    rosbag_json["available"] = true;
    rosbag_json["duration_sec"] = rosbag_info->duration_sec;
    rosbag_json["size_bytes"] = rosbag_info->size_bytes;
    rosbag_json["format"] = rosbag_info->format;
    rosbag_json["download_url"] = "/api/v1/faults/" + request->fault_code + "/snapshots/bag";
    result["rosbag"] = rosbag_json;
  } else {
    result["rosbag"] = {{"available", false}};
  }

  response->success = true;
  response->data = result.dump();

  RCLCPP_DEBUG(get_logger(), "GetSnapshots returned %zu topics for fault '%s' (rosbag=%s)", snapshots.size(),
               request->fault_code.c_str(), rosbag_info ? "available" : "not available");
}

void FaultManagerNode::handle_get_rosbag(const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Request> & request,
                                         const std::shared_ptr<ros2_medkit_msgs::srv::GetRosbag::Response> & response) {
  // Two ways in. recording_id addresses one specific bag and is tried first;
  // fault_code keeps working and means "the newest recording of this fault", which
  // is what it has always meant back when a fault could only have one.
  //
  // A caller that cannot tell the two apart - the gateway, holding one URL segment
  // that may be either - sets both to the same string and relies on this fallthrough.
  // An id that names no recording is therefore not an answer yet, only a miss.
  std::optional<RosbagFileInfo> rosbag_info;
  std::vector<std::string> attached_codes;
  std::string subject;

  if (!request->recording_id.empty()) {
    // Validation stays a hard failure: a traversal-shaped id is malformed on
    // either reading, and falling back would quietly re-admit it as a fault code.
    std::string validation_error = validate_recording_id(request->recording_id);
    if (!validation_error.empty()) {
      response->success = false;
      response->error_message = validation_error;
      return;
    }

    std::vector<RosbagFileInfo> rows = {};
    try {
      rows = storage_->get_rosbag_files_by_recording(request->recording_id);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      response->success = false;
      response->error_message = storage_unavailable(e);
      return;
    }
    if (!rows.empty()) {
      subject = "recording " + request->recording_id;
      rosbag_info = rows.front();
      for (const auto & row : rows) {
        attached_codes.push_back(row.fault_code);
      }
    } else if (request->fault_code.empty()) {
      response->success = false;
      response->error_message = "No rosbag file available for recording: " + request->recording_id;
      return;
    }
  }

  if (!rosbag_info) {
    std::string validation_error = validate_fault_code(request->fault_code);
    if (!validation_error.empty()) {
      response->success = false;
      response->error_message = validation_error;
      return;
    }
    subject = "fault " + request->fault_code;

    // Check if fault exists
    std::optional<fd::FaultRecord> fault;
    try {
      fault = storage_->get_fault(request->fault_code);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      response->success = false;
      response->error_message = storage_unavailable(e);
      return;
    }
    if (!fault) {
      response->success = false;
      response->error_message = "Fault not found: " + request->fault_code;
      return;
    }

    try {
      rosbag_info = storage_->get_rosbag_file(request->fault_code);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      response->success = false;
      response->error_message = storage_unavailable(e);
      return;
    }
    if (!rosbag_info) {
      response->success = false;
      response->error_message = "No rosbag file available for fault: " + request->fault_code;
      return;
    }
    // Report every fault the recording covers, not only the one asked about: the
    // caller authorizes the download against this set.
    try {
      for (const auto & row : storage_->get_rosbag_files_by_recording(rosbag_info->recording_id)) {
        attached_codes.push_back(row.fault_code);
      }
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      response->success = false;
      response->error_message = storage_unavailable(e);
      return;
    }
    if (attached_codes.empty()) {
      attached_codes.push_back(request->fault_code);
    }
  }

  // Check if file exists
  if (!std::filesystem::exists(rosbag_info->file_path)) {
    response->success = false;
    response->error_message = "Rosbag file not found on disk: " + rosbag_info->file_path;
    // Clean up the stale record BY RECORDING. Deleting by fault code would take the
    // fault's other, healthy recordings with it because one of them vanished.
    try {
      storage_->delete_rosbag_recording(rosbag_info->recording_id);
    } catch (const FaultStorage::IgnorableConnectionException & e) {
      RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
      return;
    }
    return;
  }

  response->success = true;
  response->file_path = rosbag_info->file_path;
  response->recording_id = rosbag_info->recording_id;
  response->fault_codes = attached_codes;
  response->format = rosbag_info->format;
  response->duration_sec = rosbag_info->duration_sec;
  response->size_bytes = rosbag_info->size_bytes;

  RCLCPP_DEBUG(get_logger(), "GetRosbag returned file '%s' for %s", rosbag_info->file_path.c_str(), subject.c_str());
}

void FaultManagerNode::handle_list_rosbags(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListRosbags::Response> & response) {
  RCLCPP_DEBUG(get_logger(), "ListRosbags request for entity: %s", request->entity_fqn.c_str());

  if (request->entity_fqn.empty()) {
    response->success = false;
    response->error_message = "entity_fqn cannot be empty";
    return;
  }

  // Use batch storage API to get all rosbags for this entity
  std::vector<RosbagFileInfo> rosbags = {};
  try {
    rosbags = storage_->list_rosbags_for_entity(request->entity_fqn);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }

  std::set<std::string> reaped;  // one cleanup per recording, not per row
  for (const auto & info : rosbags) {
    // Skip rosbags whose files no longer exist on disk
    if (!std::filesystem::exists(info.file_path)) {
      // BY RECORDING: the bytes are gone for every fault that referenced them, so
      // leaving the sibling rows would keep charging the quota for nothing. Deleting
      // by fault code would instead take that fault's healthy recordings with it.
      if (reaped.insert(info.recording_id).second) {
        try {
          storage_->delete_rosbag_recording(info.recording_id);
        } catch (const FaultStorage::IgnorableConnectionException & e) {
          RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
          response->success = false;
          response->error_message = storage_unavailable(e);
          return;
        }
      }
      continue;
    }

    response->fault_codes.push_back(info.fault_code);
    response->recording_ids.push_back(info.recording_id);
    response->file_paths.push_back(info.file_path);
    response->formats.push_back(info.format);
    response->durations_sec.push_back(info.duration_sec);
    response->sizes_bytes.push_back(info.size_bytes);
    response->created_at_ns.push_back(info.created_at_ns);
  }

  response->success = true;
  RCLCPP_DEBUG(get_logger(), "ListRosbags returned %zu rosbags for entity '%s'", response->fault_codes.size(),
               request->entity_fqn.c_str());
}

void FaultManagerNode::handle_list_faults_for_entity(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> & response) {
  RCLCPP_DEBUG(get_logger(), "ListFaultsForEntity request for entity: %s", request->entity_id.c_str());

  // Validate entity_id is not empty
  if (request->entity_id.empty()) {
    response->success = false;
    response->error_message = "entity_id cannot be empty";
    return;
  }

  // Get all faults from storage
  std::vector<fd::FaultRecord> all_faults = {};
  try {
    all_faults = storage_->get_all_faults();
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }

  // Filter faults that have this entity in their reporting_sources
  for (const auto & fault : all_faults) {
    if (matches_entity(fault.reporting_sources, request->entity_id)) {
      response->faults.push_back(ros2::to_msg(fault));
    }
  }

  response->success = true;
  RCLCPP_DEBUG(get_logger(), "ListFaultsForEntity returned %zu faults for entity '%s'", response->faults.size(),
               request->entity_id.c_str());
}

bool FaultManagerNode::matches_entity(const std::vector<std::string> & reporting_sources,
                                      const std::string & entity_id) {
  for (const auto & source : reporting_sources) {
    // Exact match
    if (source == entity_id) {
      return true;
    }

    // FQN suffix match: source ends with "/" + entity_id
    // e.g., source="/powertrain/motor_controller" matches entity_id="motor_controller"
    std::string suffix = "/" + entity_id;
    if (source.size() > suffix.size() && source.substr(source.size() - suffix.size()) == suffix) {
      return true;
    }
  }
  return false;
}

DebounceConfig FaultManagerNode::resolve_config(const std::string & source_id, const std::string & fault_code) const {
  DebounceConfig config = global_config_;
  if (threshold_resolver_) {
    config = threshold_resolver_->resolve(source_id, global_config_);
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

void FaultManagerNode::warn_on_conflicting_debounce_policy(const std::string & fault_code,
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
  RCLCPP_WARN(get_logger(),
              "Fault code '%s' is debounced two ways: '%s' resolves confirmation=%d healing_enabled=%s healing=%d, "
              "'%s' resolves confirmation=%d healing_enabled=%s healing=%d. The debounce counter belongs to the fault "
              "code, so whichever source reports decides the transition and the other policy is bypassed. Pin "
              "confirmation_threshold, healing_enabled and healing_threshold for this code in "
              "fault_thresholds.config_file.",
              fault_code.c_str(), it->second.source_id.c_str(), it->second.config.confirmation_threshold,
              it->second.config.healing_enabled ? "true" : "false", it->second.config.healing_threshold,
              source_id.c_str(), resolved.confirmation_threshold, resolved.healing_enabled ? "true" : "false",
              resolved.healing_threshold);
}

}  // namespace ros2_medkit_fault_manager
