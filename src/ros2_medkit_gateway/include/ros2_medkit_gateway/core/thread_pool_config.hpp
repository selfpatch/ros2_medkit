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

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace ros2_medkit_gateway {

// Resolve a ROS-parameter thread-count into a usable pool size (issue #440).
//
// Both the rclcpp executor and the cpp-httplib request pool are sized from an
// int64 ROS parameter that an operator may mis-set. This clamps the value to a
// closed [min_threads, max_threads] range so a typo can neither drop the pool to
// zero (which would queue requests forever / mean "all cores") nor spawn a
// pathological thread count. The range is two-sided to match the established
// clamp pattern used by the subscription_executor.* / data_provider.* knobs.
//
// Pre-condition: 1 <= min_threads <= max_threads.
inline std::size_t clamp_thread_count(int64_t requested, int64_t min_threads, int64_t max_threads) {
  return static_cast<std::size_t>(std::clamp<int64_t>(requested, min_threads, max_threads));
}

}  // namespace ros2_medkit_gateway
