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

#include "ros2_medkit_fault_detection/fault_types.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway::ros2::conversions {

/// Message to neutral record; times become nanoseconds since the epoch.
ros2_medkit::fault_detection::FaultRecord from_msg(const ros2_medkit_msgs::msg::Fault & msg);
ros2_medkit::fault_detection::FaultEventRecord from_msg(const ros2_medkit_msgs::msg::FaultEvent & msg);

/// Fault message to JSON through fault_record_to_json.
nlohmann::json fault_to_json(const ros2_medkit_msgs::msg::Fault & fault);

/// Convert a ros2_medkit_msgs::msg::EnvironmentData to JSON.
///
/// Produces:
///   {
///     "extended_data_records": { "first_occurrence": "...", "last_occurrence": "..." },
///     "snapshots": [
///       { "type": "freeze_frame", "name": "...", "data": <primary>, "x-medkit": {...} },
///       { "type": "rosbag", "name": "...", "fault_code": "...", "size_bytes": ...,
///         "duration_sec": ..., "format": "..." }
///     ]
///   }
///
/// `bulk_data_uri` is intentionally NOT populated for rosbag snapshots; the
/// per-request URL depends on the entity_path that only the handler knows.
/// The handler post-processes rosbag snapshot entries to add it.
nlohmann::json environment_data_to_json(const ros2_medkit_msgs::msg::EnvironmentData & env_data);

}  // namespace ros2_medkit_gateway::ros2::conversions
