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

#include "ros2_medkit_fault_manager/ros2/fault_msg_conversions.hpp"

namespace ros2_medkit_fault_manager::ros2 {

namespace fd = ros2_medkit::fault_detection;

builtin_interfaces::msg::Time ns_to_time_msg(int64_t ns) {
  constexpr int64_t kNsPerSec = 1000000000LL;
  int64_t sec = ns / kNsPerSec;
  int64_t rem = ns % kNsPerSec;
  if (rem < 0) {
    --sec;
    rem += kNsPerSec;
  }
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(rem);
  return t;
}

ros2_medkit_msgs::msg::Fault to_msg(const fd::FaultRecord & record) {
  ros2_medkit_msgs::msg::Fault msg;
  msg.fault_code = record.fault_code;
  msg.severity = record.severity;
  msg.description = record.description;
  msg.first_occurred = ns_to_time_msg(record.first_occurred_ns);
  msg.last_occurred = ns_to_time_msg(record.last_occurred_ns);
  msg.last_passed = ns_to_time_msg(record.last_passed_ns);
  msg.occurrence_count = record.occurrence_count;
  msg.status = record.status;
  msg.reporting_sources = record.reporting_sources;
  return msg;
}

ros2_medkit_msgs::msg::FaultEvent to_msg(const fd::FaultEventRecord & record) {
  ros2_medkit_msgs::msg::FaultEvent msg;
  msg.event_type = record.event_type;
  msg.fault = to_msg(record.fault);
  msg.timestamp = ns_to_time_msg(record.timestamp_ns);
  msg.auto_cleared_codes = record.auto_cleared_codes;
  return msg;
}

}  // namespace ros2_medkit_fault_manager::ros2
