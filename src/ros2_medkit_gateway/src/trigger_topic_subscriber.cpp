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

#include "ros2_medkit_gateway/trigger_topic_subscriber.hpp"

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_gateway {

TriggerTopicSubscriber::TriggerTopicSubscriber(rclcpp::Node * node, ResourceChangeNotifier & notifier)
  : node_(node), notifier_(notifier), serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>()) {
  if (!node_) {
    throw std::invalid_argument("TriggerTopicSubscriber requires a valid node pointer");
  }

  // Create a wall timer that retries pending subscriptions every kRetryIntervalSec seconds.
  retry_timer_ = node_->create_wall_timer(std::chrono::seconds(kRetryIntervalSec), [this]() {
    retry_pending_subscriptions();
  });

  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber initialized");
}

TriggerTopicSubscriber::~TriggerTopicSubscriber() {
  shutdown();
}

void TriggerTopicSubscriber::subscribe(const std::string & topic_name, const std::string & resource_path,
                                       const std::string & entity_id) {
  if (shutdown_flag_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // If already subscribed, add entity_id and increment ref count
  auto it = subscriptions_.find(topic_name);
  if (it != subscriptions_.end()) {
    it->second.ref_count++;
    it->second.entity_ids.insert(entity_id);
    RCLCPP_DEBUG(node_->get_logger(),
                 "TriggerTopicSubscriber: incremented ref_count for '%s' to %d, "
                 "entity_ids count=%zu",
                 topic_name.c_str(), it->second.ref_count, it->second.entity_ids.size());
    return;
  }

  // If already pending, add entity_id to the pending entry
  auto pit = pending_.find(topic_name);
  if (pit != pending_.end()) {
    pit->second.entity_ids.insert(entity_id);
    RCLCPP_DEBUG(node_->get_logger(), "TriggerTopicSubscriber: added entity '%s' to pending subscription for '%s'",
                 entity_id.c_str(), topic_name.c_str());
    return;
  }

  // Determine topic type from the ROS 2 graph
  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto type_it = topic_names_and_types.find(topic_name);
  if (type_it == topic_names_and_types.end() || type_it->second.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "TriggerTopicSubscriber: cannot determine type for topic '%s' "
                "(no publishers yet), queuing for retry",
                topic_name.c_str());
    auto & pending = pending_[topic_name];
    pending.resource_path = resource_path;
    pending.entity_ids.insert(entity_id);
    pending.created_at = std::chrono::steady_clock::now();
    return;
  }

  const std::string & msg_type = type_it->second[0];
  create_subscription_internal(topic_name, msg_type, resource_path, {entity_id});
}

void TriggerTopicSubscriber::create_subscription_internal(const std::string & topic_name, const std::string & msg_type,
                                                          const std::string & resource_path,
                                                          const std::unordered_set<std::string> & entity_ids) {
  // Create a GenericSubscription that deserializes and forwards to notifier.
  // Use SensorDataQoS (best effort) as default - matches NativeTopicSampler pattern.
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  // Capture topic_name and msg_type by value for the callback.
  // Entity IDs are read from subscriptions_ under lock in the callback.
  // NOLINTNEXTLINE(performance-unnecessary-value-param) - GenericSubscription requires value type in callback
  auto callback = [this, topic_name, msg_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
    if (shutdown_flag_.load()) {
      return;
    }

    try {
      // Deserialize CDR data to JSON
      nlohmann::json data_json = serializer_->deserialize(msg_type, *msg);

      // Read current entity_ids and resource_path under lock
      std::unordered_set<std::string> current_entity_ids;
      std::string current_resource_path;
      {
        std::lock_guard<std::mutex> cb_lock(mutex_);
        auto sub_it = subscriptions_.find(topic_name);
        if (sub_it == subscriptions_.end()) {
          return;
        }
        current_entity_ids = sub_it->second.entity_ids;
        current_resource_path = sub_it->second.resource_path;
      }

      // Emit one notification per entity_id, using resource_path (not topic_name)
      for (const auto & eid : current_entity_ids) {
        notifier_.notify("data", eid, current_resource_path, data_json, ChangeType::UPDATED);
      }

    } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "TriggerTopicSubscriber: unknown type '%s' for topic '%s': %s", msg_type.c_str(),
                           topic_name.c_str(), e.what());
    } catch (const ros2_medkit_serialization::SerializationError & e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "TriggerTopicSubscriber: deserialization failed for '%s': %s", topic_name.c_str(), e.what());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "TriggerTopicSubscriber: error processing message from '%s': %s", topic_name.c_str(),
                           e.what());
    }
  };

  try {
    auto subscription = node_->create_generic_subscription(topic_name, msg_type, qos, callback);

    SubscriptionEntry entry;
    entry.subscription = std::move(subscription);
    entry.ref_count = static_cast<int>(entity_ids.size());
    entry.resource_path = resource_path;
    entry.entity_ids = entity_ids;
    subscriptions_[topic_name] = std::move(entry);

    RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: subscribed to '%s' (type=%s, entities=%zu, resource=%s)",
                topic_name.c_str(), msg_type.c_str(), entity_ids.size(), resource_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "TriggerTopicSubscriber: failed to create subscription for '%s': %s",
                 topic_name.c_str(), e.what());
  }
}

void TriggerTopicSubscriber::unsubscribe(const std::string & topic_name, const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check pending subscriptions first
  auto pit = pending_.find(topic_name);
  if (pit != pending_.end()) {
    pit->second.entity_ids.erase(entity_id);
    if (pit->second.entity_ids.empty()) {
      RCLCPP_INFO(node_->get_logger(),
                  "TriggerTopicSubscriber: removed pending subscription for '%s' (no entities remaining)",
                  topic_name.c_str());
      pending_.erase(pit);
    }
    return;
  }

  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    return;
  }

  it->second.ref_count--;
  it->second.entity_ids.erase(entity_id);

  if (it->second.ref_count <= 0) {
    RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: unsubscribed from '%s' (ref_count reached 0)",
                topic_name.c_str());
    subscriptions_.erase(it);
  } else {
    RCLCPP_DEBUG(node_->get_logger(),
                 "TriggerTopicSubscriber: decremented ref_count for '%s' to %d, "
                 "entity_ids count=%zu",
                 topic_name.c_str(), it->second.ref_count, it->second.entity_ids.size());
  }
}

void TriggerTopicSubscriber::shutdown() {
  if (shutdown_flag_.exchange(true)) {
    return;  // Already shut down
  }

  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();
  subscriptions_.clear();
  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: shutdown complete");
}

void TriggerTopicSubscriber::retry_pending_subscriptions() {
  if (shutdown_flag_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (pending_.empty()) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  std::vector<std::string> resolved;
  std::vector<std::string> expired;

  // Query the ROS 2 graph once for all pending topics
  auto topic_names_and_types = node_->get_topic_names_and_types();

  for (auto & [topic_name, pending] : pending_) {
    // Check timeout
    if (now - pending.created_at > std::chrono::seconds(kPendingTimeoutSec)) {
      RCLCPP_ERROR(node_->get_logger(), "TriggerTopicSubscriber: topic '%s' type not available after %ds, giving up",
                   topic_name.c_str(), kPendingTimeoutSec);
      expired.push_back(topic_name);
      continue;
    }

    // Try to resolve topic type
    auto type_it = topic_names_and_types.find(topic_name);
    if (type_it != topic_names_and_types.end() && !type_it->second.empty()) {
      // Topic type is now available - create the subscription
      create_subscription_internal(topic_name, type_it->second[0], pending.resource_path, pending.entity_ids);
      resolved.push_back(topic_name);
    }
  }

  for (const auto & name : resolved) {
    RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: resolved pending subscription for '%s'", name.c_str());
    pending_.erase(name);
  }
  for (const auto & name : expired) {
    pending_.erase(name);
  }
}

}  // namespace ros2_medkit_gateway
