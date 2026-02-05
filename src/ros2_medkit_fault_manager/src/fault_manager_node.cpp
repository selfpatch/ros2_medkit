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

#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_fault_manager/time_utils.hpp"
#include "ros2_medkit_msgs/msg/cluster_info.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/extended_data_records.hpp"
#include "ros2_medkit_msgs/msg/muted_fault_info.hpp"
#include "ros2_medkit_msgs/msg/snapshot.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// Maximum allowed length for fault_code
constexpr size_t kMaxFaultCodeLength = 128;

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

}  // namespace

FaultManagerNode::FaultManagerNode(const rclcpp::NodeOptions & options) : Node("fault_manager", options) {
  // Declare and get parameters
  storage_type_ = declare_parameter<std::string>("storage_type", "sqlite");
  database_path_ = declare_parameter<std::string>("database_path", "/var/lib/ros2_medkit/faults.db");

  auto confirmation_threshold_param = declare_parameter<int>("confirmation_threshold", -1);
  if (confirmation_threshold_param > 0) {
    RCLCPP_WARN(get_logger(),
                "confirmation_threshold should be <= 0 (0 or -1 = immediate confirmation), got %d. Using %d.",
                static_cast<int>(confirmation_threshold_param), static_cast<int>(-confirmation_threshold_param));
    confirmation_threshold_param = -confirmation_threshold_param;
  }
  confirmation_threshold_ = static_cast<int32_t>(confirmation_threshold_param);

  // Healing parameters
  healing_enabled_ = declare_parameter<bool>("healing_enabled", false);
  auto healing_threshold_param = declare_parameter<int>("healing_threshold", 3);
  if (healing_threshold_param < 0) {
    RCLCPP_WARN(get_logger(), "healing_threshold should be >= 0, got %d. Using %d.",
                static_cast<int>(healing_threshold_param), static_cast<int>(-healing_threshold_param));
    healing_threshold_param = -healing_threshold_param;
  }
  healing_threshold_ = static_cast<int32_t>(healing_threshold_param);

  // Time-based auto-confirmation parameter
  auto_confirm_after_sec_ = declare_parameter<double>("auto_confirm_after_sec", 0.0);
  if (auto_confirm_after_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "auto_confirm_after_sec should be >= 0, got %.2f. Disabling.", auto_confirm_after_sec_);
    auto_confirm_after_sec_ = 0.0;
  }

  // Create storage backend
  storage_ = create_storage();

  // Create event publisher for SSE streaming
  event_publisher_ = create_publisher<ros2_medkit_msgs::msg::FaultEvent>("~/events", rclcpp::QoS(100).reliable());

  // Configure debounce settings
  DebounceConfig config;
  config.confirmation_threshold = confirmation_threshold_;
  config.healing_enabled = healing_enabled_;
  config.healing_threshold = healing_threshold_;
  config.auto_confirm_after_sec = auto_confirm_after_sec_;
  storage_->set_debounce_config(config);

  // Create service servers
  report_fault_srv_ = create_service<ros2_medkit_msgs::srv::ReportFault>(
      "~/report_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> & request,
                               const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> & response) {
        handle_report_fault(request, response);
      });

  get_faults_srv_ = create_service<ros2_medkit_msgs::srv::GetFaults>(
      "~/get_faults", [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Request> & request,
                             const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Response> & response) {
        handle_get_faults(request, response);
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

  // Initialize snapshot capture
  auto snapshot_config = create_snapshot_config();
  if (snapshot_config.enabled) {
    snapshot_capture_ = std::make_unique<SnapshotCapture>(this, storage_.get(), snapshot_config);
  }

  // Initialize rosbag capture if enabled
  if (snapshot_config.rosbag.enabled) {
    rosbag_capture_ = std::make_unique<RosbagCapture>(this, storage_.get(), snapshot_config.rosbag, snapshot_config);
  }

  // Initialize correlation engine (nullptr if disabled or not configured)
  correlation_engine_ = create_correlation_engine();

  // Create correlation cleanup timer if correlation is enabled
  if (correlation_engine_) {
    auto cleanup_interval_sec = declare_parameter<double>("correlation.cleanup_interval_sec", 5.0);
    if (cleanup_interval_sec <= 0.0) {
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
      size_t confirmed = storage_->check_time_based_confirmation(get_wall_clock_time());
      if (confirmed > 0) {
        RCLCPP_INFO(get_logger(), "Auto-confirmed %zu PREFAILED fault(s) due to time threshold", confirmed);
      }
    });
    RCLCPP_INFO(get_logger(),
                "FaultManager node started (storage=%s, confirmation_threshold=%d, "
                "healing=%s, auto_confirm_after=%.1fs)",
                storage_type_.c_str(), confirmation_threshold_, healing_enabled_ ? "enabled" : "disabled",
                auto_confirm_after_sec_);
  } else {
    RCLCPP_INFO(get_logger(), "FaultManager node started (storage=%s, confirmation_threshold=%d, healing=%s)",
                storage_type_.c_str(), confirmation_threshold_, healing_enabled_ ? "enabled" : "disabled");
  }
}

std::unique_ptr<FaultStorage> FaultManagerNode::create_storage() {
  if (storage_type_ == "memory") {
    RCLCPP_INFO(get_logger(), "Using in-memory fault storage");
    return std::make_unique<InMemoryFaultStorage>();
  }

  if (storage_type_ == "sqlite") {
    // Create parent directory if it doesn't exist (except for :memory:)
    if (database_path_ != ":memory:") {
      std::filesystem::path db_path(database_path_);
      auto parent_dir = db_path.parent_path();
      std::string parent_dir_str = parent_dir.string();
      if (!parent_dir_str.empty() && !std::filesystem::exists(parent_dir)) {
        try {
          std::filesystem::create_directories(parent_dir);
          RCLCPP_INFO(get_logger(), "Created database directory: %s", parent_dir_str.c_str());
        } catch (const std::filesystem::filesystem_error & e) {
          RCLCPP_ERROR(get_logger(), "Failed to create database directory for fault manager storage at '%s': %s",
                       parent_dir_str.c_str(), e.what());
          throw;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Using SQLite fault storage: %s", database_path_.c_str());
    return std::make_unique<SqliteFaultStorage>(database_path_);
  }

  RCLCPP_ERROR(get_logger(), "Unknown storage_type '%s', falling back to in-memory", storage_type_.c_str());
  return std::make_unique<InMemoryFaultStorage>();
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
  auto fault_before = storage_->get_fault(request->fault_code);
  std::string status_before = fault_before ? fault_before->status : "";

  // Report the fault event (use wall clock time, not sim time, for proper timestamps)
  bool is_new = storage_->report_fault_event(request->fault_code, request->event_type, request->severity,
                                             request->description, request->source_id, get_wall_clock_time());

  response->accepted = true;

  // Get updated fault state to publish event
  auto fault_after = storage_->get_fault(request->fault_code);
  if (fault_after) {
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
    if (is_new && fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
      // New fault immediately confirmed (e.g., CRITICAL severity or threshold=-1)
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && status_before != ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED &&
               fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
      // Existing fault transitioned to CONFIRMED
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED, *fault_after);
      }
      just_confirmed = true;
    } else if (!is_new && fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
      // Fault was already CONFIRMED, data updated (occurrence_count, sources, etc.)
      if (!should_mute) {
        publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_UPDATED, *fault_after);
      }
    }
    // Note: PREFAILED/PREPASSED status changes don't emit events (debounce in progress)

    // Capture snapshots when fault is confirmed (even if muted)
    if (just_confirmed && snapshot_capture_) {
      snapshot_capture_->capture(request->fault_code);
    }

    // Trigger rosbag capture when fault is confirmed (even if muted)
    if (just_confirmed && rosbag_capture_) {
      rosbag_capture_->on_fault_confirmed(request->fault_code);
    }

    // Handle PREFAILED state for lazy_start rosbag capture
    bool just_prefailed = (is_new && fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED) ||
                          (!is_new && status_before != ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED &&
                           fault_after->status == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
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

void FaultManagerNode::handle_get_faults(const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Request> & request,
                                         const std::shared_ptr<ros2_medkit_msgs::srv::GetFaults::Response> & response) {
  response->faults = storage_->get_faults(request->filter_by_severity, request->severity, request->statuses);

  // Include correlation data if engine is enabled
  if (correlation_engine_) {
    // Always include counts
    response->muted_count = correlation_engine_->get_muted_count();
    response->cluster_count = correlation_engine_->get_cluster_count();

    // Include muted faults details if requested
    if (request->include_muted) {
      auto muted_faults = correlation_engine_->get_muted_faults();
      response->muted_faults.reserve(muted_faults.size());
      for (const auto & muted : muted_faults) {
        ros2_medkit_msgs::msg::MutedFaultInfo info;
        info.fault_code = muted.fault_code;
        info.root_cause_code = muted.root_cause_code;
        info.rule_id = muted.rule_id;
        info.delay_ms = muted.delay_ms;
        response->muted_faults.push_back(info);
      }
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

  RCLCPP_DEBUG(get_logger(), "GetFaults returned %zu faults (muted=%u, clusters=%u)", response->faults.size(),
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

  // Process through correlation engine first (to get auto-clear list)
  std::vector<std::string> auto_cleared_codes;
  if (correlation_engine_) {
    auto clear_result = correlation_engine_->process_clear(request->fault_code);
    auto_cleared_codes = clear_result.auto_cleared_codes;
  }

  bool cleared = storage_->clear_fault(request->fault_code);

  response->success = cleared;
  if (cleared) {
    // Auto-clear correlated symptoms
    for (const auto & symptom_code : auto_cleared_codes) {
      storage_->clear_fault(symptom_code);
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
    auto fault = storage_->get_fault(request->fault_code);
    if (fault) {
      publish_fault_event(ros2_medkit_msgs::msg::FaultEvent::EVENT_CLEARED, *fault, auto_cleared_codes);
    }
  } else {
    response->message = "Fault not found: " + request->fault_code;
    RCLCPP_WARN(get_logger(), "Attempted to clear non-existent fault: %s", request->fault_code.c_str());
  }
}

bool FaultManagerNode::is_valid_severity(uint8_t severity) {
  return severity <= ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
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
  auto fault = storage_->get_fault(request->fault_code);
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  response->success = true;
  response->fault = *fault;

  // Populate extended_data_records with timestamps
  ros2_medkit_msgs::msg::ExtendedDataRecords extended_records;
  extended_records.first_occurence_ns = rclcpp::Time(fault->first_occurred).nanoseconds();
  extended_records.last_occurence_ns = rclcpp::Time(fault->last_occurred).nanoseconds();
  response->environment_data.extended_data_records = extended_records;

  // Get freeze frame snapshots from storage
  auto stored_snapshots = storage_->get_snapshots(request->fault_code);
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

  // Get rosbag info if available
  auto rosbag_info = storage_->get_rosbag_file(request->fault_code);
  if (rosbag_info) {
    ros2_medkit_msgs::msg::Snapshot rosbag_snapshot;
    rosbag_snapshot.type = ros2_medkit_msgs::msg::Snapshot::TYPE_ROSBAG;
    rosbag_snapshot.name = "rosbag_" + request->fault_code;
    rosbag_snapshot.bulk_data_id = rosbag_info->bulk_data_id;
    rosbag_snapshot.size_bytes = rosbag_info->size_bytes;
    rosbag_snapshot.duration_sec = rosbag_info->duration_sec;
    rosbag_snapshot.format = rosbag_info->format;
    rosbag_snapshot.captured_at_ns = rosbag_info->created_at_ns;
    // Freeze frame fields left empty for rosbag type
    response->environment_data.snapshots.push_back(rosbag_snapshot);
  }

  RCLCPP_DEBUG(get_logger(), "GetFault returned fault '%s' with %zu snapshots", request->fault_code.c_str(),
               response->environment_data.snapshots.size());
}

void FaultManagerNode::publish_fault_event(const std::string & event_type, const ros2_medkit_msgs::msg::Fault & fault,
                                           const std::vector<std::string> & auto_cleared_codes) {
  ros2_medkit_msgs::msg::FaultEvent event;
  event.event_type = event_type;
  event.fault = fault;
  event.timestamp = get_wall_clock_time();
  event.auto_cleared_codes = auto_cleared_codes;

  event_publisher_->publish(event);

  RCLCPP_DEBUG(get_logger(), "Published fault event: %s for fault_code=%s", event_type.c_str(),
               fault.fault_code.c_str());
}

SnapshotConfig FaultManagerNode::create_snapshot_config() {
  SnapshotConfig config;

  // Declare snapshot parameters
  config.enabled = declare_parameter<bool>("snapshots.enabled", true);
  config.background_capture = declare_parameter<bool>("snapshots.background_capture", false);

  // Validate timeout_sec (must be positive)
  config.timeout_sec = declare_parameter<double>("snapshots.timeout_sec", 1.0);
  if (config.timeout_sec <= 0.0) {
    RCLCPP_WARN(get_logger(), "snapshots.timeout_sec must be positive, got %.2f. Using default 1.0s",
                config.timeout_sec);
    config.timeout_sec = 1.0;
  }

  // Validate max_message_size (must be positive before casting to size_t)
  auto max_message_size_param = declare_parameter<int>("snapshots.max_message_size", 65536);
  if (max_message_size_param <= 0) {
    RCLCPP_WARN(get_logger(), "snapshots.max_message_size must be positive, got %ld. Using default 65536",
                static_cast<long>(max_message_size_param));
    max_message_size_param = 65536;
  }
  config.max_message_size = static_cast<size_t>(max_message_size_param);

  // Default topics (catch-all)
  config.default_topics =
      declare_parameter<std::vector<std::string>>("snapshots.default_topics", std::vector<std::string>{});

  // Load fault_specific and patterns from YAML config file if provided
  auto config_file = declare_parameter<std::string>("snapshots.config_file", "");
  if (!config_file.empty()) {
    load_snapshot_config_from_yaml(config_file, config);
  }

  // Rosbag configuration (opt-in)
  config.rosbag.enabled = declare_parameter<bool>("snapshots.rosbag.enabled", false);
  if (config.rosbag.enabled) {
    config.rosbag.duration_sec = declare_parameter<double>("snapshots.rosbag.duration_sec", 5.0);
    if (config.rosbag.duration_sec <= 0.0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.duration_sec must be positive, got %.2f. Using default 5.0s",
                  config.rosbag.duration_sec);
      config.rosbag.duration_sec = 5.0;
    }

    config.rosbag.duration_after_sec = declare_parameter<double>("snapshots.rosbag.duration_after_sec", 1.0);
    if (config.rosbag.duration_after_sec < 0.0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.duration_after_sec must be non-negative, got %.2f. Using 0.0s",
                  config.rosbag.duration_after_sec);
      config.rosbag.duration_after_sec = 0.0;
    }

    config.rosbag.topics = declare_parameter<std::string>("snapshots.rosbag.topics", "config");
    config.rosbag.include_topics =
        declare_parameter<std::vector<std::string>>("snapshots.rosbag.include_topics", std::vector<std::string>{});
    config.rosbag.exclude_topics =
        declare_parameter<std::vector<std::string>>("snapshots.rosbag.exclude_topics", std::vector<std::string>{});

    config.rosbag.lazy_start = declare_parameter<bool>("snapshots.rosbag.lazy_start", false);
    config.rosbag.format = declare_parameter<std::string>("snapshots.rosbag.format", "sqlite3");
    config.rosbag.storage_path = declare_parameter<std::string>("snapshots.rosbag.storage_path", "");

    int64_t max_bag_size = declare_parameter<int64_t>("snapshots.rosbag.max_bag_size_mb", 50);
    if (max_bag_size <= 0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.max_bag_size_mb must be positive. Using 50MB");
      max_bag_size = 50;
    }
    config.rosbag.max_bag_size_mb = static_cast<size_t>(max_bag_size);

    int64_t max_total_storage = declare_parameter<int64_t>("snapshots.rosbag.max_total_storage_mb", 500);
    if (max_total_storage <= 0) {
      RCLCPP_WARN(get_logger(), "snapshots.rosbag.max_total_storage_mb must be positive. Using 500MB");
      max_total_storage = 500;
    }
    config.rosbag.max_total_storage_mb = static_cast<size_t>(max_total_storage);

    config.rosbag.auto_cleanup = declare_parameter<bool>("snapshots.rosbag.auto_cleanup", true);

    RCLCPP_INFO(get_logger(),
                "Rosbag capture enabled (duration=%.1fs+%.1fs, topics=%s, lazy=%s, format=%s, "
                "max_bag=%zuMB, max_total=%zuMB)",
                config.rosbag.duration_sec, config.rosbag.duration_after_sec, config.rosbag.topics.c_str(),
                config.rosbag.lazy_start ? "true" : "false", config.rosbag.format.c_str(),
                config.rosbag.max_bag_size_mb, config.rosbag.max_total_storage_mb);
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

std::unique_ptr<correlation::CorrelationEngine> FaultManagerNode::create_correlation_engine() {
  // Get correlation config file path from parameter
  auto config_file = declare_parameter<std::string>("correlation.config_file", "");

  if (config_file.empty()) {
    RCLCPP_DEBUG(get_logger(), "Correlation disabled: no config_file specified");
    return nullptr;
  }

  if (!std::filesystem::exists(config_file)) {
    RCLCPP_ERROR(get_logger(), "Correlation config file not found: %s", config_file.c_str());
    return nullptr;
  }

  try {
    auto config = correlation::parse_config_file(config_file);

    if (!config.enabled) {
      RCLCPP_INFO(get_logger(), "Correlation explicitly disabled in config");
      return nullptr;
    }

    // Validate the config
    auto validation = correlation::validate_config(config);
    for (const auto & warning : validation.warnings) {
      RCLCPP_WARN(get_logger(), "Correlation config: %s", warning.c_str());
    }
    if (!validation.valid) {
      for (const auto & error : validation.errors) {
        RCLCPP_ERROR(get_logger(), "Correlation config error: %s", error.c_str());
      }
      return nullptr;
    }

    RCLCPP_INFO(get_logger(), "Correlation engine enabled (default_window=%ums, patterns=%zu, rules=%zu)",
                config.default_window_ms, config.patterns.size(), config.rules.size());

    return std::make_unique<correlation::CorrelationEngine>(config);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load correlation config: %s", e.what());
    return nullptr;
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
  auto fault = storage_->get_fault(request->fault_code);
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  // Get snapshots from storage
  auto snapshots = storage_->get_snapshots(request->fault_code, request->topic);

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

  // Build topics object
  nlohmann::json topics_json = nlohmann::json::object();
  for (const auto & snapshot : snapshots) {
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
  auto rosbag_info = storage_->get_rosbag_file(request->fault_code);
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
  // Validate fault_code
  std::string validation_error = validate_fault_code(request->fault_code);
  if (!validation_error.empty()) {
    response->success = false;
    response->error_message = validation_error;
    return;
  }

  // Check if fault exists
  auto fault = storage_->get_fault(request->fault_code);
  if (!fault) {
    response->success = false;
    response->error_message = "Fault not found: " + request->fault_code;
    return;
  }

  // Get rosbag info from storage
  auto rosbag_info = storage_->get_rosbag_file(request->fault_code);
  if (!rosbag_info) {
    response->success = false;
    response->error_message = "No rosbag file available for fault: " + request->fault_code;
    return;
  }

  // Check if file exists
  if (!std::filesystem::exists(rosbag_info->file_path)) {
    response->success = false;
    response->error_message = "Rosbag file not found on disk: " + rosbag_info->file_path;
    // Clean up the stale record
    storage_->delete_rosbag_file(request->fault_code);
    return;
  }

  response->success = true;
  response->file_path = rosbag_info->file_path;
  response->format = rosbag_info->format;
  response->duration_sec = rosbag_info->duration_sec;
  response->size_bytes = rosbag_info->size_bytes;

  RCLCPP_DEBUG(get_logger(), "GetRosbag returned file '%s' for fault '%s'", rosbag_info->file_path.c_str(),
               request->fault_code.c_str());
}

}  // namespace ros2_medkit_fault_manager
