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

// Include all discovery models from the new modular structure
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"

#include <chrono>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief Cache for discovered entities
 */
struct EntityCache {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
  std::chrono::system_clock::time_point last_update;
};

}  // namespace ros2_medkit_gateway
