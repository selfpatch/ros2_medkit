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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

/// Fault events from the fault manager's events topic. The ROS subscription is
/// created on the first subscribe() and kept for the source's life.
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

  [[nodiscard]] FaultEventSubscription subscribe(Callback callback) override;

 private:
  /// One subscription's callback. `mutex` is held while it runs, so removal waits for it.
  struct Subscriber {
    std::mutex mutex;
    Callback callback;  ///< Null once removed. Guarded by `mutex`.
  };

  void on_message(const ros2_medkit_msgs::msg::FaultEvent & msg);
  void remove(const std::shared_ptr<Subscriber> & subscriber);

  rclcpp::Node & node_;
  std::string topic_;
  /// Guards subscribers_ and subscription_. Never held while a callback runs.
  std::mutex subscribers_mutex_;
  std::vector<std::shared_ptr<Subscriber>> subscribers_;
  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr subscription_;
};

}  // namespace ros2_medkit_gateway
