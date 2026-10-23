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

#include "ros2_medkit_fault_detection/fault_types.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_fault_manager::ros2 {

/// Message time exactly as rclcpp::Time(ns, RCL_SYSTEM_TIME) builds it.
builtin_interfaces::msg::Time ns_to_time_msg(int64_t ns);

ros2_medkit_msgs::msg::Fault to_msg(const ros2_medkit::fault_detection::FaultRecord & record);
ros2_medkit::fault_detection::FaultRecord from_msg(const ros2_medkit_msgs::msg::Fault & msg);
ros2_medkit_msgs::msg::FaultEvent to_msg(const ros2_medkit::fault_detection::FaultEventRecord & record);

}  // namespace ros2_medkit_fault_manager::ros2
