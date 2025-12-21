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

#include "ros2_medkit_fault_reporter/local_filter.hpp"

#include <algorithm>

namespace ros2_medkit_fault_reporter {

LocalFilter::LocalFilter(const FilterConfig & config) : config_(config) {
  validate_config();
}

void LocalFilter::validate_config() {
  // Ensure threshold is at least 1
  if (config_.default_threshold < 1) {
    config_.default_threshold = 1;
  }
  // Ensure window is positive
  if (config_.default_window_sec <= 0.0) {
    config_.default_window_sec = 1.0;
  }
}

bool LocalFilter::should_forward(const std::string & fault_code, uint8_t severity) {
  std::lock_guard<std::mutex> lock(mutex_);

  // If filtering is disabled, always forward
  if (!config_.enabled) {
    return true;
  }

  // High severity faults bypass filtering
  if (severity >= config_.bypass_severity) {
    return true;
  }

  auto now = std::chrono::steady_clock::now();
  auto & tracker = trackers_[fault_code];

  // Remove expired timestamps
  cleanup_expired(tracker, now);

  // Add current timestamp
  tracker.timestamps.push_back(now);

  // Check if threshold is met
  return static_cast<int>(tracker.timestamps.size()) >= config_.default_threshold;
}

void LocalFilter::reset(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);
  trackers_.erase(fault_code);
}

void LocalFilter::reset_all() {
  std::lock_guard<std::mutex> lock(mutex_);
  trackers_.clear();
}

void LocalFilter::set_config(const FilterConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  validate_config();
  trackers_.clear();
}

void LocalFilter::cleanup_expired(FaultTracker & tracker, std::chrono::steady_clock::time_point now) {
  auto window = std::chrono::duration<double>(config_.default_window_sec);
  auto cutoff = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(window);

  // Fast path: if nothing has expired, avoid any work
  if (tracker.timestamps.empty() || tracker.timestamps.front() >= cutoff) {
    return;
  }

  // Timestamps are sorted (pushed in chronological order), use lower_bound for efficiency
  auto it = std::lower_bound(tracker.timestamps.begin(), tracker.timestamps.end(), cutoff);
  tracker.timestamps.erase(tracker.timestamps.begin(), it);
}

}  // namespace ros2_medkit_fault_reporter
