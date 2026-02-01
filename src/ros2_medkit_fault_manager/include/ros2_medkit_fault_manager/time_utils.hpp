// Copyright 2026 mfaferek93
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

#include "rclcpp/rclcpp.hpp"

namespace ros2_medkit_fault_manager {

/// Get current wall clock time in nanoseconds.
/// This is not affected by use_sim_time parameter.
inline int64_t get_wall_clock_ns() {
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

/// Get current wall clock time as rclcpp::Time.
/// This is not affected by use_sim_time parameter.
inline rclcpp::Time get_wall_clock_time() {
  return rclcpp::Time(get_wall_clock_ns(), RCL_SYSTEM_TIME);
}

}  // namespace ros2_medkit_fault_manager
