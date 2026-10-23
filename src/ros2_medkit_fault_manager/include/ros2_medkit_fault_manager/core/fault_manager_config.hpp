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

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/core/log_sink.hpp"

namespace ros2_medkit_fault_manager::core {

/// Period of the time-based auto-confirmation job.
inline constexpr double kAutoConfirmPeriodSec = 1.0;

/// Bounds of `snapshots.capture_pool_size` and `snapshots.capture_queue_depth`. A worker is an OS
/// thread running a multi-second capture; a queue entry is a fault code.
inline constexpr int64_t kMaxCapturePoolSize = 64;
inline constexpr int64_t kMaxCaptureQueueDepth = 65536;

/// One line `FaultManagerConfig::validate()` wants logged.
struct ConfigNotice {
  LogLevel level;
  std::string logger;
  std::string text;
};

/// Every value the fault manager reads at startup, named after its parameter with dots replaced
/// by `_`. Defaults equal the parameter defaults. Values are held as read; `validate()` applies
/// the clamps and fallbacks.
struct FaultManagerConfig {
  /// Logger name of the notices and of the lines `build_core_parts` logs. Not a parameter.
  std::string logger_name{"fault_manager"};

  std::string storage_type{"sqlite"};
  std::string database_path{"/var/lib/ros2_medkit/faults.db"};
  std::string database_url;

  int64_t confirmation_threshold{-1};
  bool healing_enabled{false};
  int64_t healing_threshold{3};
  double auto_confirm_after_sec{0.0};

  double snapshots_recapture_cooldown_sec{60.0};
  int64_t snapshots_capture_pool_size{2};
  int64_t snapshots_capture_queue_depth{16};
  std::string snapshots_capture_queue_full_policy{"reject_newest"};
  bool snapshots_retain_on_clear{false};
  int64_t snapshots_max_per_fault{10};
  int64_t near_miss_max_per_fault{200};

  std::string entity_thresholds_config_file;
  std::string fault_thresholds_config_file;

  bool audit_log_enabled{false};
  std::string audit_log_transitions{"all"};
  int64_t audit_log_retention_max_records{0};
  bool audit_log_fail_closed{false};
  std::string audit_log_database_path;

  std::string correlation_config_file;
  double correlation_cleanup_interval_sec{5.0};

  bool snapshots_enabled{true};
  bool snapshots_background_capture{false};
  double snapshots_timeout_sec{1.0};
  int64_t snapshots_max_message_size{65536};
  std::vector<std::string> snapshots_default_topics;
  bool snapshots_entity_default{true};
  std::string snapshots_config_file;

  /// The `snapshots.rosbag.*` values below are read and checked only when this is true.
  bool snapshots_rosbag_enabled{false};
  double snapshots_rosbag_duration_sec{5.0};
  double snapshots_rosbag_duration_after_sec{1.0};
  std::string snapshots_rosbag_topics{"entity"};
  std::vector<std::string> snapshots_rosbag_include_topics;
  std::vector<std::string> snapshots_rosbag_exclude_topics;
  bool snapshots_rosbag_exclude_sensor_topics{true};
  bool snapshots_rosbag_qos_match{true};
  bool snapshots_rosbag_lazy_start{false};
  std::string snapshots_rosbag_format{"mcap"};
  std::string snapshots_rosbag_storage_path;
  int64_t snapshots_rosbag_max_bag_size_mb{50};
  int64_t snapshots_rosbag_max_total_storage_mb{500};
  int64_t snapshots_rosbag_max_buffer_mb{256};
  int64_t snapshots_rosbag_max_bags_per_fault{1};
  bool snapshots_rosbag_auto_cleanup{true};

  /// Applies the clamps, sign flips and fallbacks in place and returns the lines to log, in order.
  std::vector<ConfigNotice> validate();
};

}  // namespace ros2_medkit_fault_manager::core
