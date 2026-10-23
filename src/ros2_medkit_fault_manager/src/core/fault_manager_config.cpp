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

#include "ros2_medkit_fault_manager/core/fault_manager_config.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <utility>

namespace ros2_medkit_fault_manager::core {

namespace {

/// Upper bound of auto_confirm_after_sec: the storage backends evaluate the window as
/// std::llround(sec * 1e9), which is undefined past the int64 range.
constexpr double kMaxAutoConfirmSec = 9.0e9;

/// Largest magnitude of the debounce thresholds, which the node narrows to int32.
constexpr int64_t kMaxThresholdMagnitude = INT32_MAX;

/// Upper bound of correlation.cleanup_interval_sec: the period is kept in nanoseconds.
constexpr double kMaxCleanupIntervalSec = 9.0e9;

constexpr int64_t kDefaultMaxNearMissesPerFault = 200;

/// Collects what `validate()` logs.
class CollectSink : public LogSink {
 public:
  void log(LogLevel level, const std::string & logger, const std::string & message) override {
    notices.push_back({level, logger, message});
  }
  std::vector<ConfigNotice> notices;
};

}  // namespace

// Every double is range-checked as a positive predicate that is then negated: every comparison
// against NaN is false, so "below or above" accepts NaN. The named predicate keeps
// readability-simplify-boolean-expr from rewriting it back.
std::vector<ConfigNotice> FaultManagerConfig::validate() {
  CollectSink sink;

  // The thresholds are narrowed to int32 and negated below, so the range is symmetric.
  const bool confirmation_in_range =
      confirmation_threshold >= -kMaxThresholdMagnitude && confirmation_threshold <= kMaxThresholdMagnitude;
  if (!confirmation_in_range) {
    const int64_t clamped = confirmation_threshold < 0 ? -kMaxThresholdMagnitude : kMaxThresholdMagnitude;
    logf(sink, LogLevel::kWarn, logger_name,
         "confirmation_threshold must be in [%" PRId64 ", %" PRId64 "], got %" PRId64 ". Clamping to %" PRId64,
         -kMaxThresholdMagnitude, kMaxThresholdMagnitude, confirmation_threshold, clamped);
    confirmation_threshold = clamped;
  }

  const bool healing_in_range =
      healing_threshold >= -kMaxThresholdMagnitude && healing_threshold <= kMaxThresholdMagnitude;
  if (!healing_in_range) {
    const int64_t clamped = healing_threshold < 0 ? -kMaxThresholdMagnitude : kMaxThresholdMagnitude;
    logf(sink, LogLevel::kWarn, logger_name,
         "healing_threshold must be in [%" PRId64 ", %" PRId64 "], got %" PRId64 ". Clamping to %" PRId64,
         -kMaxThresholdMagnitude, kMaxThresholdMagnitude, healing_threshold, clamped);
    healing_threshold = clamped;
  }

  if (confirmation_threshold > 0) {
    logf(sink, LogLevel::kWarn, logger_name,
         "confirmation_threshold should be < 0 (-1 = immediate confirmation), got %d. Using %d.",
         static_cast<int>(confirmation_threshold), static_cast<int>(-confirmation_threshold));
    confirmation_threshold = -confirmation_threshold;
  }

  if (healing_threshold < 0) {
    logf(sink, LogLevel::kWarn, logger_name, "healing_threshold should be >= 0, got %d. Using %d.",
         static_cast<int>(healing_threshold), static_cast<int>(-healing_threshold));
    healing_threshold = -healing_threshold;
  }

  const bool auto_confirm_in_range = std::isfinite(auto_confirm_after_sec) && auto_confirm_after_sec >= 0.0 &&
                                     auto_confirm_after_sec <= kMaxAutoConfirmSec;
  if (!auto_confirm_in_range) {
    logf(sink, LogLevel::kWarn, logger_name,
         "auto_confirm_after_sec must be a finite value in [0, %.1e], got %.2f. Disabling.", kMaxAutoConfirmSec,
         auto_confirm_after_sec);
    auto_confirm_after_sec = 0.0;
  }

  const bool cooldown_in_range =
      std::isfinite(snapshots_recapture_cooldown_sec) && snapshots_recapture_cooldown_sec >= 0.0;
  if (!cooldown_in_range) {
    logf(sink, LogLevel::kWarn, logger_name, "snapshots.recapture_cooldown_sec should be >= 0, got %.2f. Disabling.",
         snapshots_recapture_cooldown_sec);
    snapshots_recapture_cooldown_sec = 0.0;
  }

  // The node narrows these to int, so a value outside the range is rejected before it can wrap.
  const bool pool_in_range = snapshots_capture_pool_size >= 1 && snapshots_capture_pool_size <= kMaxCapturePoolSize;
  if (!pool_in_range) {
    const int64_t clamped = snapshots_capture_pool_size < 1 ? 1 : kMaxCapturePoolSize;
    logf(sink, LogLevel::kWarn, logger_name,
         "snapshots.capture_pool_size must be in [1, %" PRId64 "], got %" PRId64 ". Clamping to %" PRId64,
         kMaxCapturePoolSize, snapshots_capture_pool_size, clamped);
    snapshots_capture_pool_size = clamped;
  }

  const bool depth_in_range =
      snapshots_capture_queue_depth >= 1 && snapshots_capture_queue_depth <= kMaxCaptureQueueDepth;
  if (!depth_in_range) {
    const int64_t clamped = snapshots_capture_queue_depth < 1 ? 1 : kMaxCaptureQueueDepth;
    logf(sink, LogLevel::kWarn, logger_name,
         "snapshots.capture_queue_depth must be in [1, %" PRId64 "], got %" PRId64 ". Clamping to %" PRId64,
         kMaxCaptureQueueDepth, snapshots_capture_queue_depth, clamped);
    snapshots_capture_queue_depth = clamped;
  }

  if (snapshots_capture_queue_full_policy != "reject_newest" && snapshots_capture_queue_full_policy != "drop_oldest") {
    logf(sink, LogLevel::kWarn, logger_name,
         "snapshots.capture_queue_full_policy '%s' is invalid (expected 'reject_newest' or "
         "'drop_oldest'). Using 'reject_newest'",
         snapshots_capture_queue_full_policy.c_str());
    snapshots_capture_queue_full_policy = "reject_newest";
  }

  if (snapshots_max_per_fault < 0) {
    logf(sink, LogLevel::kWarn, logger_name, "snapshots.max_per_fault should be >= 0, got %ld. Disabling limit.",
         static_cast<long>(snapshots_max_per_fault));
    snapshots_max_per_fault = 0;
  }

  // A negative value cast to size_t would become SIZE_MAX and remove the bound.
  if (near_miss_max_per_fault < 0) {
    logf(sink, LogLevel::kWarn, logger_name, "near_miss.max_per_fault must be >= 0, got %ld. Using default %ld.",
         static_cast<long>(near_miss_max_per_fault), static_cast<long>(kDefaultMaxNearMissesPerFault));
    near_miss_max_per_fault = kDefaultMaxNearMissesPerFault;
  }

  if (audit_log_transitions != "all" && audit_log_transitions != "confirmed_only") {
    logf(sink, LogLevel::kWarn, logger_name, "audit_log.transitions '%s' invalid, using 'all'",
         audit_log_transitions.c_str());
    audit_log_transitions = "all";
  }

  if (audit_log_retention_max_records < 0) {
    logf(sink, LogLevel::kWarn, logger_name, "audit_log.retention_max_records < 0, disabling rotation");
    audit_log_retention_max_records = 0;
  }

  const bool timeout_in_range = std::isfinite(snapshots_timeout_sec) && snapshots_timeout_sec > 0.0;
  if (!timeout_in_range) {
    logf(sink, LogLevel::kWarn, logger_name, "snapshots.timeout_sec must be positive, got %.2f. Using default 1.0s",
         snapshots_timeout_sec);
    snapshots_timeout_sec = 1.0;
  }

  // Empty entries are dropped: launch files cannot express an empty string array, so [''] is the
  // conventional "no defaults" and must not shadow the entity-default fallback.
  snapshots_default_topics.erase(
      std::remove(snapshots_default_topics.begin(), snapshots_default_topics.end(), std::string{}),
      snapshots_default_topics.end());

  if (snapshots_max_message_size <= 0) {
    logf(sink, LogLevel::kWarn, logger_name,
         "snapshots.max_message_size must be positive, got %lld. Using default 65536",
         static_cast<long long>(snapshots_max_message_size));
    snapshots_max_message_size = 65536;
  }

  const bool cleanup_in_range = std::isfinite(correlation_cleanup_interval_sec) &&
                                correlation_cleanup_interval_sec > 0.0 &&
                                correlation_cleanup_interval_sec <= kMaxCleanupIntervalSec;
  if (!cleanup_in_range) {
    logf(sink, LogLevel::kWarn, logger_name,
         "correlation.cleanup_interval_sec must be positive, got %.2f. Using default 5.0s",
         correlation_cleanup_interval_sec);
    correlation_cleanup_interval_sec = 5.0;
  }

  if (snapshots_rosbag_enabled) {
    const bool duration_in_range = std::isfinite(snapshots_rosbag_duration_sec) && snapshots_rosbag_duration_sec > 0.0;
    if (!duration_in_range) {
      logf(sink, LogLevel::kWarn, logger_name,
           "snapshots.rosbag.duration_sec must be positive, got %.2f. Using default 5.0s",
           snapshots_rosbag_duration_sec);
      snapshots_rosbag_duration_sec = 5.0;
    }

    const bool after_in_range =
        std::isfinite(snapshots_rosbag_duration_after_sec) && snapshots_rosbag_duration_after_sec >= 0.0;
    if (!after_in_range) {
      logf(sink, LogLevel::kWarn, logger_name,
           "snapshots.rosbag.duration_after_sec must be non-negative, got %.2f. Using 0.0s",
           snapshots_rosbag_duration_after_sec);
      snapshots_rosbag_duration_after_sec = 0.0;
    }

    if (snapshots_rosbag_max_bag_size_mb <= 0) {
      logf(sink, LogLevel::kWarn, logger_name, "snapshots.rosbag.max_bag_size_mb must be positive. Using 50MB");
      snapshots_rosbag_max_bag_size_mb = 50;
    }

    if (snapshots_rosbag_max_total_storage_mb <= 0) {
      logf(sink, LogLevel::kWarn, logger_name, "snapshots.rosbag.max_total_storage_mb must be positive. Using 500MB");
      snapshots_rosbag_max_total_storage_mb = 500;
    }

    if (snapshots_rosbag_max_buffer_mb <= 0) {
      logf(sink, LogLevel::kWarn, logger_name, "snapshots.rosbag.max_buffer_mb must be positive. Using 256MB");
      snapshots_rosbag_max_buffer_mb = 256;
    }

    if (snapshots_rosbag_max_bags_per_fault < 0) {
      logf(sink, LogLevel::kWarn, logger_name,
           "snapshots.rosbag.max_bags_per_fault must be >= 0. Treating as unlimited");
      snapshots_rosbag_max_bags_per_fault = 0;
    }
  }

  return std::move(sink.notices);
}

}  // namespace ros2_medkit_fault_manager::core
