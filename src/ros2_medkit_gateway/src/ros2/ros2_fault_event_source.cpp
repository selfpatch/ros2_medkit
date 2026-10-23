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

#include "ros2_medkit_gateway/ros2/ros2_fault_event_source.hpp"

#include <utility>

#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"

namespace ros2_medkit_gateway {

Ros2FaultEventSource::Ros2FaultEventSource(rclcpp::Node & node, std::string topic)
  : node_(node), topic_(std::move(topic)) {
}

Ros2FaultEventSource::~Ros2FaultEventSource() {
  unsubscribe();
}

void Ros2FaultEventSource::subscribe(Callback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callback_ = std::move(callback);
  if (!subscription_) {
    subscription_ = node_.create_subscription<ros2_medkit_msgs::msg::FaultEvent>(
        topic_, rclcpp::QoS(100).reliable(), [this](const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
          on_message(*msg);
        });
  }
}

void Ros2FaultEventSource::unsubscribe() {
  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr dropped;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = nullptr;
    dropped = std::move(subscription_);
  }
  // Released outside the lock: the last reference may be dropped here.
  dropped.reset();
}

void Ros2FaultEventSource::on_message(const ros2_medkit_msgs::msg::FaultEvent & msg) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (callback_) {
    callback_(ros2::conversions::from_msg(msg));
  }
}

}  // namespace ros2_medkit_gateway
