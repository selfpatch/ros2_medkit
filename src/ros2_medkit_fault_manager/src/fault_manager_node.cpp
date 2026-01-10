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

#include <filesystem>

#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"

namespace ros2_medkit_fault_manager {

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

  clear_fault_srv_ = create_service<ros2_medkit_msgs::srv::ClearFault>(
      "~/clear_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
                              const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
        handle_clear_fault(request, response);
      });

  // Create auto-confirmation timer if enabled
  if (auto_confirm_after_sec_ > 0.0) {
    auto_confirm_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      size_t confirmed = storage_->check_time_based_confirmation(now());
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
  if (request->fault_code.empty()) {
    response->accepted = false;
    RCLCPP_WARN(get_logger(), "ReportFault rejected: fault_code cannot be empty");
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

  // Report the fault event
  bool is_new = storage_->report_fault_event(request->fault_code, request->event_type, request->severity,
                                             request->description, request->source_id, now());

  response->accepted = true;

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

  RCLCPP_DEBUG(get_logger(), "GetFaults returned %zu faults", response->faults.size());
}

void FaultManagerNode::handle_clear_fault(
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & request,
    const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & response) {
  // Validate fault_code
  if (request->fault_code.empty()) {
    response->success = false;
    response->message = "fault_code cannot be empty";
    return;
  }

  bool cleared = storage_->clear_fault(request->fault_code);

  response->success = cleared;
  if (cleared) {
    response->message = "Fault cleared: " + request->fault_code;
    RCLCPP_INFO(get_logger(), "Fault cleared: %s", request->fault_code.c_str());
  } else {
    response->message = "Fault not found: " + request->fault_code;
    RCLCPP_WARN(get_logger(), "Attempted to clear non-existent fault: %s", request->fault_code.c_str());
  }
}

bool FaultManagerNode::is_valid_severity(uint8_t severity) {
  return severity <= ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
}

}  // namespace ros2_medkit_fault_manager
