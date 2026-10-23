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

#include <chrono>
#include <cstdint>

#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"

namespace ros2_medkit_fault_manager::ros2 {

/// System wall clock. Not affected by use_sim_time.
class SystemWallClock : public core::WallClock {
 public:
  int64_t now_ns() const override {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
  }
};

}  // namespace ros2_medkit_fault_manager::ros2
