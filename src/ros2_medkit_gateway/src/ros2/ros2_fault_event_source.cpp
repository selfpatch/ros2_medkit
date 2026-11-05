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

#include <algorithm>
#include <utility>

#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"

namespace ros2_medkit_gateway {

Ros2FaultEventSource::Ros2FaultEventSource(rclcpp::Node & node, std::string topic)
  : node_(node), topic_(std::move(topic)) {
}

Ros2FaultEventSource::~Ros2FaultEventSource() {
  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr dropped;
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.clear();
    dropped = std::move(subscription_);
  }
  // Released outside the lock: the last reference may be dropped here.
  dropped.reset();
}

FaultEventSubscription Ros2FaultEventSource::subscribe(Callback callback) {
  auto subscriber = std::make_shared<Subscriber>();
  subscriber->callback = std::move(callback);
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    if (!subscription_) {
      subscription_ = node_.create_subscription<ros2_medkit_msgs::msg::FaultEvent>(
          topic_, rclcpp::QoS(100).reliable(), [this](const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
            on_message(*msg);
          });
    }
    // Registered after the ROS subscription exists, so a failed create leaves no callback behind.
    subscribers_.push_back(subscriber);
  }
  return FaultEventSubscription([this, subscriber]() {
    remove(subscriber);
  });
}

void Ros2FaultEventSource::remove(const std::shared_ptr<Subscriber> & subscriber) {
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.erase(std::remove(subscribers_.begin(), subscribers_.end(), subscriber), subscribers_.end());
  }
  // Waits for a running callback. A delivery that copied the list before the
  // erase finds the callback null.
  std::lock_guard<std::mutex> lock(subscriber->mutex);
  subscriber->callback = nullptr;
}

void Ros2FaultEventSource::on_message(const ros2_medkit_msgs::msg::FaultEvent & msg) {
  std::vector<std::shared_ptr<Subscriber>> targets;
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    targets = subscribers_;
  }
  if (targets.empty()) {
    return;
  }
  const auto record = ros2::conversions::from_msg(msg);
  for (const auto & subscriber : targets) {
    std::lock_guard<std::mutex> lock(subscriber->mutex);
    if (subscriber->callback) {
      subscriber->callback(record);
    }
  }
}

}  // namespace ros2_medkit_gateway
