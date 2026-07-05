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
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief rclcpp-coupled topic subscription executor for the trigger subsystem.
 *
 * Each trigger that observes a "data" resource registers exactly one
 * subscription here, identified by a caller-provided handle key. The
 * subscriber owns the rclcpp::GenericSubscription, performs the CDR -> JSON
 * deserialization, and delivers each sample to the handle's callback. When
 * the topic's type cannot be resolved at subscribe time (publisher not yet
 * up), the entry is queued as "pending" and retried every 5 seconds. Pending
 * entries time out after 60 seconds.
 *
 * Threading: subscribe()/unsubscribe()/shutdown() are mutex-guarded.
 * Per-handle callbacks are invoked from rclcpp executor threads. The
 * subscription-destructor pattern (callbacks cannot fire on a partially
 * destroyed subscriber) is enforced by the shutdown_flag_ guard followed by
 * subscription map clearing inside shutdown().
 *
 * The lifetime of an individual subscription is tied to the corresponding
 * handle key: unsubscribe(handle_key) atomically removes both the user
 * callback and the underlying rclcpp::Subscription, so any in-flight
 * dispatch sees the empty map and short-circuits.
 *
 * The retry callback (set via set_retry_callback) is fired on every retry
 * tick. The trigger manager wires this to retry_unresolved_triggers() so
 * that triggers whose resource_path was not yet resolvable to a topic at
 * creation time get a chance to convert into a real subscription.
 */
class TriggerTopicSubscriber {
 public:
  /// Callback delivering one deserialized sample as JSON. Invoked on the
  /// rclcpp executor thread.
  using SampleCallback = std::function<void(const nlohmann::json & sample)>;

  /**
   * @brief Construct the subscriber.
   * @param node Non-owning ROS node used to create rclcpp::GenericSubscription
   *             and the retry wall timer.
   */
  explicit TriggerTopicSubscriber(rclcpp::Node * node);

  ~TriggerTopicSubscriber();

  TriggerTopicSubscriber(const TriggerTopicSubscriber &) = delete;
  TriggerTopicSubscriber & operator=(const TriggerTopicSubscriber &) = delete;
  TriggerTopicSubscriber(TriggerTopicSubscriber &&) = delete;
  TriggerTopicSubscriber & operator=(TriggerTopicSubscriber &&) = delete;

  /**
   * @brief Subscribe to a topic under a unique handle key.
   *
   * Each call yields one rclcpp::GenericSubscription dedicated to the
   * caller's handle. If @p msg_type is empty the type is resolved from the
   * ROS graph; if no publisher is yet advertising the topic, the entry is
   * queued for retry.
   *
   * Idempotent: a second call with an already-registered key replaces both
   * the type and the callback for that key.
   *
   * @param topic_name Fully qualified ROS 2 topic name (e.g. "/sensor/temp").
   * @param msg_type   Fully qualified message type, or empty for auto-resolve.
   * @param handle_key Caller-allocated unique key. The transport adapter uses
   *                   this to disambiguate per-trigger subscriptions.
   * @param callback   Invoked for every successfully deserialized sample.
   */
  void subscribe(const std::string & topic_name, const std::string & msg_type, const std::string & handle_key,
                 SampleCallback callback);

  /// Drop the subscription for @p handle_key. Both the user callback and the
  /// rclcpp::GenericSubscription are released; in-flight dispatch is
  /// guaranteed not to fire after this returns.
  void unsubscribe(const std::string & handle_key);

  /// Shut down all subscriptions. Idempotent.
  void shutdown();

  /// Callback invoked on every retry tick (alongside pending-subscription
  /// resolution). The trigger manager wires this to retry_unresolved_triggers().
  using RetryCallback = std::function<void()>;
  void set_retry_callback(RetryCallback cb);

 private:
  /// Per-handle live subscription state.
  struct LiveEntry {
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::string topic_name;
    std::string msg_type;
    SampleCallback callback;
  };

  /// Per-handle pending entry awaiting topic-type resolution.
  struct PendingEntry {
    std::string topic_name;
    SampleCallback callback;
    std::chrono::steady_clock::time_point created_at;
  };

  /// Timeout for pending subscriptions before giving up (seconds).
  static constexpr int kPendingTimeoutSec = 60;

  /// Interval between retry attempts for pending subscriptions (seconds).
  static constexpr int kRetryIntervalSec = 5;

  /// Create a GenericSubscription for @p handle_key with the resolved type.
  /// Caller must hold mutex_.
  void create_subscription_internal(const std::string & handle_key, const std::string & topic_name,
                                    const std::string & msg_type, SampleCallback callback);

  /// Retry callback for pending subscriptions. Called by retry_timer_.
  void retry_pending_subscriptions();

  rclcpp::Node * node_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  std::mutex mutex_;
  std::unordered_map<std::string, LiveEntry> live_;
  std::unordered_map<std::string, PendingEntry> pending_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  RetryCallback retry_callback_;
  std::atomic<bool> shutdown_flag_{false};
};

}  // namespace ros2_medkit_gateway
