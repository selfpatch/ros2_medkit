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

#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_fault_reporter {

/// Configuration for local fault filtering
struct FilterConfig {
  bool enabled{true};
  int default_threshold{3};
  double default_window_sec{10.0};
  uint8_t bypass_severity{2};  // SEVERITY_ERROR and above bypass filtering
};

/// Local filter for fault reports
///
/// Tracks fault occurrences per fault_code and determines whether a report
/// should be forwarded to the central FaultManager. Implements threshold-based
/// filtering within a sliding time window.
///
/// Thread-safe: All public methods can be called from multiple threads.
class LocalFilter {
 public:
  explicit LocalFilter(const FilterConfig & config = FilterConfig{});

  /// Check if a fault report should be forwarded
  ///
  /// @param fault_code The fault identifier
  /// @param severity The fault severity level
  /// @return true if the report should be sent to FaultManager
  bool should_forward(const std::string & fault_code, uint8_t severity);

  /// Reset tracking for a specific fault code
  void reset(const std::string & fault_code);

  /// Reset all tracking state
  void reset_all();

  /// Get the current configuration
  const FilterConfig & config() const {
    return config_;
  }

  /// Update configuration (clears existing state)
  void set_config(const FilterConfig & config);

 private:
  struct FaultTracker {
    std::vector<std::chrono::steady_clock::time_point> timestamps;
  };

  /// Remove expired timestamps from tracker
  void cleanup_expired(FaultTracker & tracker, std::chrono::steady_clock::time_point now);

  FilterConfig config_;
  std::unordered_map<std::string, FaultTracker> trackers_;
  mutable std::mutex mutex_;
};

}  // namespace ros2_medkit_fault_reporter
