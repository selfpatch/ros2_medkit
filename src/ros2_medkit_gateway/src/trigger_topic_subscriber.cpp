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

  // Determine topic type from the ROS 2 graph
  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto type_it = topic_names_and_types.find(topic_name);
  if (type_it == topic_names_and_types.end() || type_it->second.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "TriggerTopicSubscriber: cannot determine type for topic '%s' "
                "(no publishers yet), skipping subscription",
                topic_name.c_str());
    return;
  }

  const std::string & msg_type = type_it->second[0];

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
    entry.ref_count = 1;
    entry.resource_path = resource_path;
    entry.entity_ids.insert(entity_id);
    subscriptions_[topic_name] = std::move(entry);

    RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: subscribed to '%s' (type=%s, entity=%s, resource=%s)",
                topic_name.c_str(), msg_type.c_str(), entity_id.c_str(), resource_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "TriggerTopicSubscriber: failed to create subscription for '%s': %s",
                 topic_name.c_str(), e.what());
  }
}

void TriggerTopicSubscriber::unsubscribe(const std::string & topic_name, const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(mutex_);

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

  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: shutdown complete");
}

}  // namespace ros2_medkit_gateway
