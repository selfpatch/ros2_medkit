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

#include "ros2_medkit_gateway/ros2/transports/ros2_topic_subscription_transport.hpp"

#include <stdexcept>
#include <utility>

namespace ros2_medkit_gateway::ros2 {

Ros2TopicSubscriptionTransport::Ros2TopicSubscriptionTransport(TriggerTopicSubscriber * subscriber)
  : subscriber_(subscriber) {
  if (!subscriber_) {
    throw std::invalid_argument("Ros2TopicSubscriptionTransport requires a valid subscriber pointer");
  }
}

std::unique_ptr<TopicSubscriptionHandle> Ros2TopicSubscriptionTransport::subscribe(const std::string & topic_path,
                                                                                   const std::string & msg_type,
                                                                                   SampleCallback callback) {
  auto key = allocate_key();
  subscriber_->subscribe(topic_path, msg_type, key, std::move(callback));
  return std::make_unique<Handle>(subscriber_, key);
}

std::string Ros2TopicSubscriptionTransport::allocate_key() {
  // Monotonic synthetic id; the underlying subscriber treats each key as
  // an opaque identifier so collisions between adapter instances are
  // structurally impossible (each adapter wraps one subscriber).
  uint64_t id = next_key_.fetch_add(1, std::memory_order_relaxed);
  return "trig_handle_" + std::to_string(id);
}

Ros2TopicSubscriptionTransport::Handle::Handle(TriggerTopicSubscriber * subscriber, std::string key)
  : subscriber_(subscriber), key_(std::move(key)) {
}

Ros2TopicSubscriptionTransport::Handle::~Handle() {
  // Subscriber outlives the handle by construction (GatewayNode tear-down
  // order); calling unsubscribe() here drains in-flight rclcpp callbacks
  // before the user callback is released.
  subscriber_->unsubscribe(key_);
}

}  // namespace ros2_medkit_gateway::ros2
