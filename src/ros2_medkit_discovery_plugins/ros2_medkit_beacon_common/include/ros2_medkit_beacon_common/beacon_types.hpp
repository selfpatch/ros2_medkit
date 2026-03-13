// Copyright 2026 selfpatch GmbH
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
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_beacon {

struct BeaconHint {
  std::string entity_id;  // required

  // Identity
  std::string stable_id;
  std::string display_name;

  // Topology
  std::vector<std::string> function_ids;
  std::vector<std::string> depends_on;
  std::string component_id;

  // Transport
  std::string transport_type;
  std::string negotiated_format;

  // Process diagnostics
  uint32_t process_id{0};  // 0 = not provided
  std::string process_name;
  std::string hostname;

  // Freeform
  std::unordered_map<std::string, std::string> metadata;

  // Timing (set by plugin, not from message directly)
  std::chrono::steady_clock::time_point received_at;
};

}  // namespace ros2_medkit_beacon
