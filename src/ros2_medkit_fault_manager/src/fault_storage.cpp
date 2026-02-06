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

#include "ros2_medkit_fault_manager/fault_storage.hpp"

#include <algorithm>
#include <filesystem>

namespace ros2_medkit_fault_manager {

ros2_medkit_msgs::msg::Fault FaultState::to_msg() const {
  ros2_medkit_msgs::msg::Fault msg;
  msg.fault_code = fault_code;
  msg.severity = severity;
  msg.description = description;
  msg.first_occurred = first_occurred;
  msg.last_occurred = last_occurred;
  msg.occurrence_count = occurrence_count;
  msg.status = status;

  // Convert set to vector
  msg.reporting_sources.reserve(reporting_sources.size());
  for (const auto & source : reporting_sources) {
    msg.reporting_sources.push_back(source);
  }

  return msg;
}

void InMemoryFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig InMemoryFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void InMemoryFaultStorage::update_status(FaultState & state) {
  // Note: CLEARED faults are handled in report_fault_event() before this is called

  if (state.debounce_counter <= config_.confirmation_threshold) {
    state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  } else if (config_.healing_enabled && state.debounce_counter >= config_.healing_threshold) {
    state.status = ros2_medkit_msgs::msg::Fault::STATUS_HEALED;
  } else if (state.debounce_counter < 0) {
    state.status = ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED;
  } else if (state.debounce_counter > 0) {
    state.status = ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED;
  }
  // Note: debounce_counter == 0 keeps current status (hysteresis behavior).
  // This avoids rapid status flapping at the zero crossing boundary.
}

bool InMemoryFaultStorage::report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                              const std::string & description, const std::string & source_id,
                                              const rclcpp::Time & timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool is_failed = (event_type == EventType::EVENT_FAILED);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    // New fault - only create entry for FAILED events
    if (!is_failed) {
      return false;  // PASSED event for non-existent fault is ignored
    }

    FaultState state;
    state.fault_code = fault_code;
    state.severity = severity;
    state.description = description;
    state.first_occurred = timestamp;
    state.last_occurred = timestamp;
    state.last_failed_time = timestamp;
    state.occurrence_count = 1;
    state.debounce_counter = -1;  // First FAILED event
    state.reporting_sources.insert(source_id);

    // CRITICAL severity bypasses debounce and confirms immediately
    if (config_.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    } else {
      // Set status based on debounce counter vs threshold
      update_status(state);
    }

    faults_.emplace(fault_code, std::move(state));
    return true;
  }

  // Existing fault - update
  auto & state = it->second;

  // CLEARED faults can be reactivated by FAILED events
  if (state.status == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
    if (!is_failed) {
      // PASSED events for CLEARED faults are ignored
      return false;
    }
    // FAILED event reactivates the fault - reset debounce counter
    state.debounce_counter = -1;
    state.last_failed_time = timestamp;
    state.last_occurred = timestamp;
    state.reporting_sources.insert(source_id);
    if (state.occurrence_count < std::numeric_limits<uint32_t>::max()) {
      ++state.occurrence_count;
    }
    if (severity > state.severity) {
      state.severity = severity;
    }
    if (!description.empty()) {
      state.description = description;
    }
    // Check for immediate CRITICAL confirmation
    if (config_.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    } else {
      update_status(state);
    }
    return true;  // Reactivation treated as new occurrence for event publishing
  }

  state.last_occurred = timestamp;

  if (is_failed) {
    state.last_failed_time = timestamp;

    // Increment occurrence_count with saturation
    if (state.occurrence_count < std::numeric_limits<uint32_t>::max()) {
      ++state.occurrence_count;
    }

    // Decrement debounce counter (towards confirmation) with saturation
    if (state.debounce_counter > std::numeric_limits<int32_t>::min()) {
      --state.debounce_counter;
    }

    // Add source if not already present
    state.reporting_sources.insert(source_id);

    // Update severity if higher
    if (severity > state.severity) {
      state.severity = severity;
    }

    // Update description if provided
    if (!description.empty()) {
      state.description = description;
    }

    // Check for immediate confirmation of CRITICAL
    if (config_.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
      return false;
    }
  } else {
    // PASSED event
    state.last_passed_time = timestamp;

    // Increment debounce counter (towards healing) with saturation
    if (state.debounce_counter < std::numeric_limits<int32_t>::max()) {
      ++state.debounce_counter;
    }
  }

  // Update status based on debounce counter
  update_status(state);

  return false;
}

std::vector<ros2_medkit_msgs::msg::Fault>
InMemoryFaultStorage::list_faults(bool filter_by_severity, uint8_t severity,
                                  const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Determine which statuses to include
  std::set<std::string> status_filter;
  if (statuses.empty()) {
    // Default: only CONFIRMED faults (excludes PREFAILED, CLEARED, HEALED)
    status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  } else {
    for (const auto & s : statuses) {
      // Only add valid statuses (invalid ones are silently ignored)
      if (s == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED || s == ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED || s == ros2_medkit_msgs::msg::Fault::STATUS_HEALED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
        status_filter.insert(s);
      }
    }
    // If all provided statuses were invalid, default to CONFIRMED for consistency
    if (status_filter.empty()) {
      status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
    }
  }

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  result.reserve(faults_.size());

  for (const auto & [code, state] : faults_) {
    // Filter by status
    if (status_filter.find(state.status) == status_filter.end()) {
      continue;
    }

    // Filter by severity if requested
    if (filter_by_severity && state.severity != severity) {
      continue;
    }

    result.push_back(state.to_msg());
  }

  return result;
}

std::optional<ros2_medkit_msgs::msg::Fault> InMemoryFaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    return std::nullopt;
  }

  return it->second.to_msg();
}

bool InMemoryFaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    return false;
  }

  // Delete associated snapshots when fault is cleared
  snapshots_.erase(std::remove_if(snapshots_.begin(), snapshots_.end(),
                                  [&fault_code](const SnapshotData & s) {
                                    return s.fault_code == fault_code;
                                  }),
                   snapshots_.end());

  it->second.status = ros2_medkit_msgs::msg::Fault::STATUS_CLEARED;
  return true;
}

size_t InMemoryFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.size();
}

bool InMemoryFaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.find(fault_code) != faults_.end();
}

size_t InMemoryFaultStorage::check_time_based_confirmation(const rclcpp::Time & current_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (config_.auto_confirm_after_sec <= 0.0) {
    return 0;  // Time-based confirmation disabled
  }

  size_t confirmed_count = 0;
  const double threshold_ns = config_.auto_confirm_after_sec * 1e9;

  for (auto & [code, state] : faults_) {
    if (state.status == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED) {
      const int64_t age_ns = (current_time - state.last_failed_time).nanoseconds();
      if (static_cast<double>(age_ns) >= threshold_ns) {
        state.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
        ++confirmed_count;
      }
    }
  }

  return confirmed_count;
}

void InMemoryFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshots_.push_back(snapshot);
}

std::vector<SnapshotData> InMemoryFaultStorage::get_snapshots(const std::string & fault_code,
                                                              const std::string & topic_filter) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<SnapshotData> result;
  for (const auto & snapshot : snapshots_) {
    if (snapshot.fault_code == fault_code) {
      if (topic_filter.empty() || snapshot.topic == topic_filter) {
        result.push_back(snapshot);
      }
    }
  }
  return result;
}

void InMemoryFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Delete existing bag file if present (prevent orphaned files on re-confirm)
  auto it = rosbag_files_.find(info.fault_code);
  if (it != rosbag_files_.end()) {
    if (it->second.file_path != info.file_path) {
      std::error_code ec;
      std::filesystem::remove_all(it->second.file_path, ec);
      // Ignore errors - file may already be deleted
    }
  }

  rosbag_files_[info.fault_code] = info;
}

std::optional<RosbagFileInfo> InMemoryFaultStorage::get_rosbag_file(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = rosbag_files_.find(fault_code);
  if (it == rosbag_files_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool InMemoryFaultStorage::delete_rosbag_file(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = rosbag_files_.find(fault_code);
  if (it == rosbag_files_.end()) {
    return false;
  }

  // Try to delete the actual file
  std::error_code ec;
  std::filesystem::remove_all(it->second.file_path, ec);
  // Ignore errors - file may already be deleted

  rosbag_files_.erase(it);
  return true;
}

size_t InMemoryFaultStorage::get_total_rosbag_storage_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);

  size_t total = 0;
  for (const auto & [code, info] : rosbag_files_) {
    total += info.size_bytes;
  }
  return total;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;
  result.reserve(rosbag_files_.size());
  for (const auto & [code, info] : rosbag_files_) {
    result.push_back(info);
  }

  // Sort by creation time (oldest first)
  std::sort(result.begin(), result.end(), [](const RosbagFileInfo & a, const RosbagFileInfo & b) {
    return a.created_at_ns < b.created_at_ns;
  });

  return result;
}

std::vector<RosbagFileInfo> InMemoryFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<RosbagFileInfo> result;

  for (const auto & [fault_code, rosbag_info] : rosbag_files_) {
    // Check if any of the fault's reporting sources contain this entity
    auto fault_it = faults_.find(fault_code);
    if (fault_it != faults_.end()) {
      const auto & fault_state = fault_it->second;
      if (fault_state.reporting_sources.find(entity_fqn) != fault_state.reporting_sources.end()) {
        result.push_back(rosbag_info);
      }
    }
  }

  return result;
}

std::vector<ros2_medkit_msgs::msg::Fault> InMemoryFaultStorage::get_all_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  result.reserve(faults_.size());

  for (const auto & [code, state] : faults_) {
    result.push_back(state.to_msg());
  }

  return result;
}

}  // namespace ros2_medkit_fault_manager
