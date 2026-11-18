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

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_fault_manager::ros2 {

/// Publishes core events as `FaultEvent` on the given publisher.
class RosEventSink : public core::EventSink {
 public:
  explicit RosEventSink(rclcpp::Publisher<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr publisher);

  void publish(const ros2_medkit::fault_detection::FaultEventRecord & event) override;

 private:
  rclcpp::Publisher<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr publisher_;
};

}  // namespace ros2_medkit_fault_manager::ros2
