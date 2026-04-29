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

#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// Outcome of a fault-management operation that returns JSON. `data` carries
/// the response body the handler will serve on success; remains empty on
/// errors. The richer `FaultWithEnvResult`, which still exposes raw message
/// types, lives alongside the FaultManager facade until the transport
/// extraction lands.
struct FaultResult {
  bool success;
  json data;
  std::string error_message;
};

/// Neutral outcome of `get_fault_with_env`. `data` carries
/// `{ "fault": {...}, "environment_data": {...} }` already converted to JSON
/// by the transport. The legacy `FaultWithEnvResult` (which exposes raw
/// message types) still lives next to the FaultManager facade until a later
/// phase reconciles the two; the transport port returns this neutral form so
/// the interface compiles in the ROS-free build layer.
struct FaultWithEnvJsonResult {
  bool success;
  std::string error_message;
  json data;
};

}  // namespace ros2_medkit_gateway
