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
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/resource_change_notifier.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Manages persistent ROS 2 topic subscriptions for data triggers.
 *
 * When a trigger observes a "data" resource, TriggerTopicSubscriber creates a
 * GenericSubscription for the corresponding topic, deserializes incoming
 * messages to JSON, and forwards them to the ResourceChangeNotifier for
 * trigger evaluation.
 *
 * Subscriptions are ref-counted: multiple triggers on the same topic share a
 * single ROS 2 subscription. When the last trigger on a topic is removed, the
 * subscription is destroyed.
 */
class TriggerTopicSubscriber {
 public:
  /**
   * @brief Construct the subscriber.
   * @param node Raw pointer to the ROS 2 node for creating subscriptions (caller manages lifetime)
   * @param notifier ResourceChangeNotifier to forward data events to
   */
  TriggerTopicSubscriber(rclcpp::Node * node, ResourceChangeNotifier & notifier);

  ~TriggerTopicSubscriber();

  // Non-copyable, non-movable
  TriggerTopicSubscriber(const TriggerTopicSubscriber &) = delete;
  TriggerTopicSubscriber & operator=(const TriggerTopicSubscriber &) = delete;
  TriggerTopicSubscriber(TriggerTopicSubscriber &&) = delete;
  TriggerTopicSubscriber & operator=(TriggerTopicSubscriber &&) = delete;

  /**
   * @brief Subscribe to a topic for data trigger monitoring.
   *
   * If the topic is already subscribed, adds the entity_id to the set and
   * increments the ref count. Otherwise, creates a GenericSubscription that
   * deserializes messages to JSON and forwards them to the notifier.
   *
   * If the topic type cannot be determined (e.g., no publishers yet),
   * logs a warning and skips subscription creation.
   *
   * @param topic_name ROS 2 topic name (e.g., "/sensor/temperature")
   * @param resource_path Resource path for notification matching (e.g., "/temperature")
   * @param entity_id Entity ID for the notification (e.g., "temp_sensor")
   */
  void subscribe(const std::string & topic_name, const std::string & resource_path, const std::string & entity_id);

  /**
   * @brief Unsubscribe an entity from a topic.
   *
   * Decrements the ref count and removes the entity_id from the set.
   * If the ref count reaches zero, the ROS 2 subscription is destroyed.
   *
   * @param topic_name ROS 2 topic name to unsubscribe from
   * @param entity_id Entity ID to remove from the subscription
   */
  void unsubscribe(const std::string & topic_name, const std::string & entity_id);

  /**
   * @brief Shutdown all subscriptions.
   *
   * Clears all active subscriptions. Safe to call multiple times.
   */
  void shutdown();

 private:
  /// Per-topic subscription state with ref counting and multi-entity support.
  struct SubscriptionEntry {
    rclcpp::GenericSubscription::SharedPtr subscription;
    int ref_count = 0;
    std::string resource_path;                   ///< Resource path for notifications (e.g., "/temperature")
    std::unordered_set<std::string> entity_ids;  ///< All entities watching this topic
  };

  rclcpp::Node * node_;
  ResourceChangeNotifier & notifier_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  std::mutex mutex_;
  std::unordered_map<std::string, SubscriptionEntry> subscriptions_;
  std::atomic<bool> shutdown_flag_{false};
};

}  // namespace ros2_medkit_gateway
