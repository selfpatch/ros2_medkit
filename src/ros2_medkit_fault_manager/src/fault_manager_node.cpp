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

using core::storage_unavailable;
using core::validate_fault_code;

/// Validate a recording id (the bag directory basename, `fault_<CODE>_<millis>`).
///
/// Same alphabet and the same no-`..` rule as a fault code, because the id ends up
/// as a path segment and a filesystem lookup. The LENGTH bound is different: a
/// recording id is `fault_` + a fault code + `_` + a millisecond stamp, so a
/// maximal fault code produces ~148 characters and would be rejected by the
/// fault-code validator it is derived from.
std::string validate_recording_id(const std::string & recording_id) {
  constexpr size_t kMaxRecordingIdLength = core::kMaxFaultCodeLength + 32;

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

  RCLCPP_INFO(get_logger(), "Capture concurrency: pool_size=%d, queue_depth=%d, policy=%s", capture_pool_size_,
              capture_queue_depth_,
              capture_queue_full_policy_ == QueueFullPolicy::kRejectNewest ? "reject_newest" : "drop_oldest");

  auto parts = core::build_core_parts(config_, log_sink_);

  // Create event publisher for SSE streaming
  event_publisher_ = create_publisher<ros2_medkit_msgs::msg::FaultEvent>("~/events", rclcpp::QoS(100).reliable());
  event_sink_ = std::make_unique<ros2::RosEventSink>(event_publisher_);
  capture_hooks_ = std::make_unique<ros2::RosCaptureHooks>(
      [this](const std::string & code) {
        return enqueue_capture(code);
      },
      [this](const std::string & code) {
        if (rosbag_capture_) {
          rosbag_capture_->on_fault_prefailed(code);
        }
      },
      [this](const std::string & code) {
        if (rosbag_capture_) {
          rosbag_capture_->on_fault_cleared(code);
        }
      });
  core_ = std::make_unique<core::FaultManagerCore>(config_, std::move(parts), *event_sink_, *capture_hooks_,
                                                   wall_clock_, log_sink_);
  storage_ = &core_->storage();

  // Runs after the override files are loaded, so a code with its own healing_enabled: true keeps
  // its HEALED status. Each flipped fault is audited, or the audit log's verify() would not see it.
  core::reclassify_stale_healed(*storage_, core_->fault_code_resolver(), core_->global_debounce(),
                                core_->audit_log() != nullptr, log_sink_, config_.logger_name,
                                [this](const fd::FaultRecord & fault, int64_t reclassified_at_ns) {
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
    snapshot_capture_ = std::make_shared<SnapshotCapture>(this, storage_, snapshot_config);
  }

  // Initialize rosbag capture if enabled
  if (snapshot_config.rosbag.enabled) {
    rosbag_capture_ = std::make_shared<RosbagCapture>(this, storage_, snapshot_config.rosbag, snapshot_config);

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
        capture_queue_full_policy_, log_sink_,
        [snap, bag, rosbag_mutex](const std::string & fault_code) {
          if (snap) {
            snap->capture(fault_code);
          }
          if (bag) {
            std::lock_guard<std::mutex> bag_lock(*rosbag_mutex);
            bag->on_fault_confirmed(fault_code);
          }
        },
        config_.logger_name);
  }

  // Both jobs run inside the core; the timers only deliver the wall clock.
  if (core_->correlation()) {
    const auto cleanup_interval_ms = static_cast<int64_t>(config_.correlation_cleanup_interval_sec * 1000);
    correlation_cleanup_timer_ = create_wall_timer(std::chrono::milliseconds(cleanup_interval_ms), [this]() {
      core_->tick(get_wall_clock_ns());
    });
  }

  if (auto_confirm_after_sec_ > 0.0) {
    auto_confirm_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      core_->tick(get_wall_clock_ns());
    });
    RCLCPP_INFO(get_logger(),
                "FaultManager node started (storage=%s, confirmation_threshold=%d, "
                "healing=%s, healing_threshold=%d, auto_confirm_after=%.1fs)",
                storage_type_.c_str(), core_->global_debounce().confirmation_threshold,
                core_->global_debounce().healing_enabled ? "enabled" : "disabled",
                core_->global_debounce().healing_threshold, auto_confirm_after_sec_);
  } else {
    RCLCPP_INFO(get_logger(),
                "FaultManager node started (storage=%s, confirmation_threshold=%d, healing=%s, "
                "healing_threshold=%d)",
                storage_type_.c_str(), core_->global_debounce().confirmation_threshold,
                core_->global_debounce().healing_enabled ? "enabled" : "disabled",
                core_->global_debounce().healing_threshold);
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
  if (auto * audit = core_ ? core_->audit_log() : nullptr) {
    try {
      AuditEvent marker;
      marker.fault_code = "__audit__";
      marker.transition = kTransitionLoggingDeactivated;
      marker.status = "INACTIVE";
      marker.source_id = "fault_manager";
      marker.description = "audit logging deactivated";
      marker.occurred_at_ns = get_wall_clock_ns();
      audit->append(marker);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to append audit 'logging_deactivated' marker: %s", e.what());
    }
  }
}

core::CaptureResult FaultManagerNode::enqueue_capture(const std::string & fault_code) {
  if (!capture_pool_) {
    return {};
  }
  const EnqueueOutcome outcome = capture_pool_->enqueue(fault_code);
  // RCLCPP_WARN_THROTTLE needs a non-const Clock lvalue (Humble/Lyrical compat); mirror
  // rosbag_capture.cpp's local-copy pattern. The counter is cast to unsigned long long for %llu
  // to avoid -Wuseless-cast (uint64_t == unsigned long on LP64).
  rclcpp::Clock throttle_clock(*get_clock());
  switch (outcome.result) {
    case EnqueueResult::kAccepted:
      return {core::CaptureEnqueue::kAccepted, {}};
    case EnqueueResult::kEvictedOldest:
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock, 2000,
                           "Capture queue full (drop_oldest): evicted pending '%s' for '%s' "
                           "(pool=%d, queue=%d, total_dropped=%llu)",
                           outcome.evicted_code ? outcome.evicted_code->c_str() : "?", fault_code.c_str(),
                           capture_pool_size_, capture_queue_depth_,
                           static_cast<unsigned long long>(capture_pool_->dropped_captures()));
      return {core::CaptureEnqueue::kEvictedOldest, outcome.evicted_code.value_or(std::string{})};
    case EnqueueResult::kDroppedNewest:
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock, 2000,
                           "Capture queue full (reject_newest): dropped capture for '%s' "
                           "(pool=%d, queue=%d, total_dropped=%llu)",
                           fault_code.c_str(), capture_pool_size_, capture_queue_depth_,
                           static_cast<unsigned long long>(capture_pool_->dropped_captures()));
      return {core::CaptureEnqueue::kDroppedNewest, {}};
    case EnqueueResult::kRejectedShuttingDown:
      RCLCPP_DEBUG(get_logger(), "Capture pool shutting down; skipped capture for '%s'", fault_code.c_str());
      return {core::CaptureEnqueue::kShuttingDown, {}};
  }
  return {};
}

void FaultManagerNode::handle_report_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response) {
  fd::FaultReport report;
  report.fault_code = request->fault_code;
  report.kind = static_cast<fd::ReportKind>(request->event_type);
  report.severity = request->severity;
  report.description = request->description;
  report.source_id = request->source_id;
  response->accepted = core_->report(report).accepted;
}

void FaultManagerNode::handle_list_faults(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> & response) {
  core::ListQuery query;
  query.filter_by_severity = request->filter_by_severity;
  query.severity = request->severity;
  query.statuses = request->statuses;
  query.include_muted = request->include_muted;
  query.include_clusters = request->include_clusters;
  const auto outcome = core_->list(query);

  response->faults.reserve(outcome.faults.size());
  for (const auto & record : outcome.faults) {
    response->faults.push_back(ros2::to_msg(record));
  }
  response->muted_count = outcome.muted_count;
  response->cluster_count = outcome.cluster_count;
  response->muted_faults.reserve(outcome.muted_faults.size());
  for (const auto & muted : outcome.muted_faults) {
    ros2_medkit_msgs::msg::MutedFaultInfo info;
    info.fault_code = muted.fault_code;
    info.root_cause_code = muted.root_cause_code;
    info.rule_id = muted.rule_id;
    info.delay_ms = muted.delay_ms;
    response->muted_faults.push_back(info);
  }
  response->clusters.reserve(outcome.clusters.size());
  for (const auto & cluster : outcome.clusters) {
    ros2_medkit_msgs::msg::ClusterInfo info;
    info.cluster_id = cluster.cluster_id;
    info.rule_id = cluster.rule_id;
    info.rule_name = cluster.rule_name;
    info.label = cluster.label;
    info.representative_code = cluster.representative_code;
    info.representative_severity = cluster.representative_severity;
    info.fault_codes = cluster.fault_codes;
    info.count = cluster.count;
    info.first_at = ros2::ns_to_time_msg(cluster.first_at_ns);
    info.last_at = ros2::ns_to_time_msg(cluster.last_at_ns);
    response->clusters.push_back(info);
  }
}

void FaultManagerNode::handle_clear_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
  auto outcome = core_->clear(request->fault_code, request->skip_correlation_auto_clear);
  response->success = outcome.success;
  response->message = std::move(outcome.message);
  response->auto_cleared_codes = std::move(outcome.auto_cleared_codes);
}

void FaultManagerNode::handle_get_fault(const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> & request,
                                        const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> & response) {
  const auto detail = core_->get(request->fault_code);
  response->success = detail.success;
  response->error_message = detail.error_message;
  if (detail.fault.fault_code.empty()) {
    return;
  }
  response->fault = ros2::to_msg(detail.fault);
  response->environment_data.extended_data_records.first_occurrence_ns = detail.first_occurrence_ns;
  response->environment_data.extended_data_records.last_occurrence_ns = detail.last_occurrence_ns;
  for (const auto & entry : detail.snapshots) {
    ros2_medkit_msgs::msg::Snapshot snapshot;
    snapshot.type = entry.type;
    snapshot.name = entry.name;
    snapshot.data = entry.data;
    snapshot.topic = entry.topic;
    snapshot.message_type = entry.message_type;
    snapshot.bulk_data_id = entry.bulk_data_id;
    snapshot.size_bytes = entry.size_bytes;
    snapshot.duration_sec = entry.duration_sec;
    snapshot.format = entry.format;
    snapshot.captured_at_ns = entry.captured_at_ns;
    response->environment_data.snapshots.push_back(snapshot);
  }
}

void FaultManagerNode::publish_fault_event(const std::string & event_type, const fd::FaultRecord & fault) {
  fd::FaultEventRecord event;
  event.event_type = event_type;
  event.fault = fault;
  event.timestamp_ns = wall_clock_.now_ns();
  event_sink_->publish(event);

  RCLCPP_DEBUG(get_logger(), "Published fault event: %s for fault_code=%s", event_type.c_str(),
               fault.fault_code.c_str());
}

void FaultManagerNode::audit_transition(const char * transition, const fd::FaultRecord & fault,
                                        const std::string & source_id, int64_t occurred_at_ns) {
  core_->record_transition(transition, fault, source_id, occurred_at_ns);
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
  c.correlation_cleanup_interval_sec =
      declare_parameter<double>("correlation.cleanup_interval_sec", c.correlation_cleanup_interval_sec);

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

  std::vector<fd::FaultRecord> faults = {};
  try {
    faults = core_->list_for_entity(request->entity_id);
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    RCLCPP_WARN(get_logger(), "Failed to connect with the fault storage server: %s", e.what());
    response->success = false;
    response->error_message = storage_unavailable(e);
    return;
  }
  for (const auto & fault : faults) {
    response->faults.push_back(ros2::to_msg(fault));
  }

  response->success = true;
  RCLCPP_DEBUG(get_logger(), "ListFaultsForEntity returned %zu faults for entity '%s'", response->faults.size(),
               request->entity_id.c_str());
}

}  // namespace ros2_medkit_fault_manager
