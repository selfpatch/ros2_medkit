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

bool FaultStorage::report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                                const std::string & source_id, const rclcpp::Time & timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    // New fault
    FaultState state;
    state.fault_code = fault_code;
    state.severity = severity;
    state.description = description;
    state.first_occurred = timestamp;
    state.last_occurred = timestamp;
    state.occurrence_count = 1;
    state.status = ros2_medkit_msgs::msg::Fault::STATUS_PENDING;
    state.reporting_sources.insert(source_id);

    faults_.emplace(fault_code, std::move(state));
    return true;
  }

  // Existing fault - update
  auto & state = it->second;
  state.last_occurred = timestamp;

  // Increment occurrence_count with saturation
  if (state.occurrence_count < std::numeric_limits<uint32_t>::max()) {
    ++state.occurrence_count;
  }

  // Add source if not already present
  state.reporting_sources.insert(source_id);

  // Update severity if higher (severity constants are ordered: INFO=0 < WARN=1 < ERROR=2 < CRITICAL=3)
  if (severity > state.severity) {
    state.severity = severity;
  }

  // Update description if provided
  if (!description.empty()) {
    state.description = description;
  }

  return false;
}

std::vector<ros2_medkit_msgs::msg::Fault> FaultStorage::get_faults(bool filter_by_severity, uint8_t severity,
                                                                   const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Determine which statuses to include
  std::set<std::string> status_filter;
  if (statuses.empty()) {
    // Default: CONFIRMED only
    status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  } else {
    for (const auto & s : statuses) {
      // Only add valid statuses (invalid ones are silently ignored)
      if (s == ros2_medkit_msgs::msg::Fault::STATUS_PENDING || s == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED ||
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

std::optional<ros2_medkit_msgs::msg::Fault> FaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    return std::nullopt;
  }

  return it->second.to_msg();
}

bool FaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = faults_.find(fault_code);
  if (it == faults_.end()) {
    return false;
  }

  it->second.status = ros2_medkit_msgs::msg::Fault::STATUS_CLEARED;
  return true;
}

size_t FaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.size();
}

bool FaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_.find(fault_code) != faults_.end();
}

}  // namespace ros2_medkit_fault_manager
