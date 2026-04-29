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

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "ros2_medkit_gateway/core/transports/topic_subscription_transport.hpp"
#include "ros2_medkit_gateway/ros2/trigger_topic_subscriber.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing TopicSubscriptionTransport by wrapping
 *        TriggerTopicSubscriber.
 *
 * Each subscribe() allocates a unique handle key, registers the user's
 * SampleCallback against the underlying TriggerTopicSubscriber, and returns
 * a handle whose destructor unsubscribes that key. The trigger manager
 * stores these handles inside its tracking map; dropping a trigger drops
 * its handle which deterministically tears down the underlying
 * rclcpp::GenericSubscription via TriggerTopicSubscriber::unsubscribe().
 *
 * The adapter is non-owning of the TriggerTopicSubscriber: the subscriber
 * is owned by GatewayNode and outlives the adapter (via the trigger
 * subsystem's destruction order).
 */
class Ros2TopicSubscriptionTransport : public TopicSubscriptionTransport {
 public:
  /**
   * @param subscriber Non-owning pointer to the underlying
   *                   TriggerTopicSubscriber. Must outlive every handle
   *                   returned by subscribe().
   */
  explicit Ros2TopicSubscriptionTransport(TriggerTopicSubscriber * subscriber);

  ~Ros2TopicSubscriptionTransport() override = default;

  Ros2TopicSubscriptionTransport(const Ros2TopicSubscriptionTransport &) = delete;
  Ros2TopicSubscriptionTransport & operator=(const Ros2TopicSubscriptionTransport &) = delete;
  Ros2TopicSubscriptionTransport(Ros2TopicSubscriptionTransport &&) = delete;
  Ros2TopicSubscriptionTransport & operator=(Ros2TopicSubscriptionTransport &&) = delete;

  std::unique_ptr<TopicSubscriptionHandle> subscribe(const std::string & topic_path, const std::string & msg_type,
                                                     SampleCallback callback) override;

 private:
  /// Concrete handle: holds the subscriber pointer + handle key. The dtor
  /// unsubscribes the handle key, satisfying the subscription-destructor
  /// pattern.
  class Handle : public TopicSubscriptionHandle {
   public:
    Handle(TriggerTopicSubscriber * subscriber, std::string key);
    ~Handle() override;

    Handle(const Handle &) = delete;
    Handle & operator=(const Handle &) = delete;
    Handle(Handle &&) = delete;
    Handle & operator=(Handle &&) = delete;

   private:
    TriggerTopicSubscriber * subscriber_;
    std::string key_;
  };

  std::string allocate_key();

  TriggerTopicSubscriber * subscriber_;
  std::atomic<uint64_t> next_key_{1};
};

}  // namespace ros2_medkit_gateway::ros2
