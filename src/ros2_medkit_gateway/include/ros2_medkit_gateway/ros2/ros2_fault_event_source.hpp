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

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

/// Fault events from the fault manager's events topic.
class Ros2FaultEventSource final : public FaultEventSource {
 public:
  /// @param node Node the subscription lives on; must outlive this object.
  /// @param topic Fully qualified events topic.
  Ros2FaultEventSource(rclcpp::Node & node, std::string topic);
  ~Ros2FaultEventSource() override;
  Ros2FaultEventSource(const Ros2FaultEventSource &) = delete;
  Ros2FaultEventSource & operator=(const Ros2FaultEventSource &) = delete;
  Ros2FaultEventSource(Ros2FaultEventSource &&) = delete;
  Ros2FaultEventSource & operator=(Ros2FaultEventSource &&) = delete;

  void subscribe(Callback callback) override;
  void unsubscribe() override;

 private:
  void on_message(const ros2_medkit_msgs::msg::FaultEvent & msg);

  rclcpp::Node & node_;
  std::string topic_;
  /// Held while a callback runs, so unsubscribe() waits for it.
  std::mutex callback_mutex_;
  Callback callback_;
  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr subscription_;
};

}  // namespace ros2_medkit_gateway
