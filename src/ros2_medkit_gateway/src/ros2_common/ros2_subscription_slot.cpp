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

#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>

namespace ros2_medkit_gateway::ros2_common {

Ros2SubscriptionSlot::Ros2SubscriptionSlot(Ros2SubscriptionExecutor & exec, std::string topic, std::string type_name,
                                           rclcpp::SubscriptionBase::SharedPtr sub)
  : exec_{exec}, topic_{std::move(topic)}, type_name_{std::move(type_name)}, sub_{std::move(sub)} {
}

Ros2SubscriptionSlot::~Ros2SubscriptionSlot() {
  if (!sub_) {
    return;
  }

  if (exec_.is_shutting_down()) {
    // Fast path: executor teardown will call rcl_node_fini on the subscription
    // node which cleans up the rcl handle atomically. Releasing the
    // SharedPtr on the current thread is safe in this window.
    sub_.reset();
    return;
  }

  // Normal path: post destroy to the worker so we do not race with
  // concurrent rcl mutations. Bounded deadline to avoid hanging teardown.
  auto result = exec_.run_sync<void>(std::function<void()>([this] {
                                       sub_.reset();
                                     }),
                                     kDestroyDeadline);
  if (!result) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_slot"),
                 "Failed to destroy subscription for topic '%s' via worker (%s). "
                 "Releasing on current thread - accept potential race over hang.",
                 topic_.c_str(), result.error().c_str());
    sub_.reset();
  }
}

tl::expected<std::unique_ptr<Ros2SubscriptionSlot>, std::string>
Ros2SubscriptionSlot::create_generic(Ros2SubscriptionExecutor & exec, const std::string & topic,
                                     const std::string & type_name, const rclcpp::QoS & qos,
                                     std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)> cb) {
  if (exec.is_shutting_down()) {
    return tl::unexpected(std::string{"executor shutting down"});
  }
  rclcpp::Node * node = exec.node();
  if (node == nullptr) {
    return tl::unexpected(std::string{"executor has no subscription node"});
  }

  auto result = exec.run_sync<rclcpp::SubscriptionBase::SharedPtr>(std::function<rclcpp::SubscriptionBase::SharedPtr()>(
      [node, topic, type_name, qos, cb_owned = std::move(cb)]() -> rclcpp::SubscriptionBase::SharedPtr {
        return node->create_generic_subscription(topic, type_name, qos, cb_owned);
      }));
  if (!result) {
    return tl::unexpected(result.error());
  }
  return std::unique_ptr<Ros2SubscriptionSlot>(new Ros2SubscriptionSlot(exec, topic, type_name, std::move(*result)));
}

}  // namespace ros2_medkit_gateway::ros2_common
