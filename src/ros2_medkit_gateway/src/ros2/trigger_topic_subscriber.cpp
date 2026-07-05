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

#include "ros2_medkit_gateway/ros2/trigger_topic_subscriber.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_gateway {

TriggerTopicSubscriber::TriggerTopicSubscriber(rclcpp::Node * node)
  : node_(node), serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>()) {
  if (!node_) {
    throw std::invalid_argument("TriggerTopicSubscriber requires a valid node pointer");
  }

  retry_timer_ = node_->create_wall_timer(std::chrono::seconds(kRetryIntervalSec), [this]() {
    retry_pending_subscriptions();
  });

  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber initialized");
}

TriggerTopicSubscriber::~TriggerTopicSubscriber() {
  shutdown();
}

void TriggerTopicSubscriber::subscribe(const std::string & topic_name, const std::string & msg_type,
                                       const std::string & handle_key, SampleCallback callback) {
  if (shutdown_flag_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Idempotent: drop any prior registration for this handle so the new one
  // takes over cleanly (both live and pending paths). Retire the prior live
  // subscription rather than destroying it inline - see unsubscribe().
  if (auto it = live_.find(handle_key); it != live_.end()) {
    retire_locked(it->second);
    live_.erase(it);
  }
  pending_.erase(handle_key);

  std::string resolved_type = msg_type;
  if (resolved_type.empty()) {
    auto topic_names_and_types = node_->get_topic_names_and_types();
    auto it = topic_names_and_types.find(topic_name);
    if (it != topic_names_and_types.end() && !it->second.empty()) {
      resolved_type = it->second[0];
    }
  }

  if (resolved_type.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "TriggerTopicSubscriber: cannot determine type for topic '%s' "
                "(no publishers yet), queuing handle '%s' for retry",
                topic_name.c_str(), handle_key.c_str());
    PendingEntry entry;
    entry.topic_name = topic_name;
    entry.callback = std::move(callback);
    entry.created_at = std::chrono::steady_clock::now();
    pending_[handle_key] = std::move(entry);
    return;
  }

  create_subscription_internal(handle_key, topic_name, resolved_type, std::move(callback));
}

void TriggerTopicSubscriber::create_subscription_internal(const std::string & handle_key,
                                                          const std::string & topic_name, const std::string & msg_type,
                                                          SampleCallback callback) {
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  // Snapshot the handle key + topic + type by value so the callback never
  // dereferences any expired strings; the user callback is looked up under
  // mutex_ to avoid firing after unsubscribe()/shutdown() returns.
  // NOLINTNEXTLINE(performance-unnecessary-value-param) - GenericSubscription requires value type in callback
  auto cb = [this, handle_key, topic_name, msg_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
    if (shutdown_flag_.load()) {
      return;
    }

    SampleCallback user_cb;
    {
      std::lock_guard<std::mutex> cb_lock(mutex_);
      auto it = live_.find(handle_key);
      if (it == live_.end()) {
        // Unsubscribed or retired: stop delivering. A retired subscription's
        // callback always returns here, before the deserialize section below;
        // the subscription (and this closure) is kept alive in retired_ and is
        // never destroyed while the executor spins, so nothing frees the
        // captured strings/type support out from under an in-flight dispatch.
        return;
      }
      user_cb = it->second.callback;
    }
    if (!user_cb) {
      return;
    }

    try {
      nlohmann::json data_json = serializer_->deserialize(msg_type, *msg);
      user_cb(data_json);
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
    auto subscription = node_->create_generic_subscription(topic_name, msg_type, qos, cb);

    LiveEntry entry;
    entry.subscription = std::move(subscription);
    entry.topic_name = topic_name;
    entry.msg_type = msg_type;
    entry.callback = std::move(callback);
    live_[handle_key] = std::move(entry);

    RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: subscribed handle '%s' to '%s' (type=%s)",
                handle_key.c_str(), topic_name.c_str(), msg_type.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "TriggerTopicSubscriber: failed to create subscription for '%s' (handle '%s'): %s", topic_name.c_str(),
                 handle_key.c_str(), e.what());
    // Propagate. The transport adapter wraps each subscribe in a try/catch
    // and returns nullptr so the trigger manager rejects the trigger
    // creation with a 5xx instead of returning 201 for a dead handle.
    throw;
  }
}

void TriggerTopicSubscriber::unsubscribe(const std::string & handle_key) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (pending_.erase(handle_key) > 0) {
    RCLCPP_DEBUG(node_->get_logger(), "TriggerTopicSubscriber: removed pending handle '%s'", handle_key.c_str());
    return;
  }

  auto it = live_.find(handle_key);
  if (it == live_.end()) {
    return;
  }

  // Stop delivering immediately by removing the entry from live_ (every future
  // dispatch early-returns at the lookup in the callback), but do NOT destroy
  // the GenericSubscription here. Under a MultiThreadedExecutor a callback for
  // this handle may be running on another worker thread, mid deserialize, still
  // reading the closure's captured strings and the dlopened type support.
  // Worse, destroying the subscription while the executor spins hands the last
  // reference to an executor worker (via its wait-result), which then frees the
  // closure on its own thread with no happens-before to that in-flight read -
  // the data race this class previously shipped. Retire the subscription to a
  // graveyard instead; it is destroyed at shutdown(), after the executor has
  // stopped. See retired_.
  retire_locked(it->second);
  live_.erase(it);

  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: unsubscribed handle '%s'", handle_key.c_str());
}

void TriggerTopicSubscriber::shutdown() {
  if (shutdown_flag_.exchange(true)) {
    return;
  }

  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }

  std::vector<rclcpp::GenericSubscription::SharedPtr> doomed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
    // Retire every live subscription, then take ownership of the whole
    // graveyard so it is destroyed below.
    for (auto & [_, entry] : live_) {
      retire_locked(entry);
    }
    live_.clear();
    doomed.swap(retired_);
  }

  // Destroy every retired subscription here. shutdown() runs in the gateway's
  // teardown path AFTER executor.spin() has returned and its worker threads
  // have joined (see the gateway main teardown order), so no callback is in
  // flight and no executor worker holds a wait-result reference - this is the
  // one point where freeing the closures + unloading their type support cannot
  // race a dispatch. Done outside mutex_ for clarity; there is no contender
  // left at this stage.
  doomed.clear();

  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: shutdown complete");
}

void TriggerTopicSubscriber::retire_locked(LiveEntry & entry) {
  entry.callback = nullptr;
  if (entry.subscription) {
    retired_.push_back(std::move(entry.subscription));
  }
}

void TriggerTopicSubscriber::set_retry_callback(RetryCallback cb) {
  retry_callback_ = std::move(cb);
}

void TriggerTopicSubscriber::retry_pending_subscriptions() {
  // Run the manager-side retry hook first so that any newly resolvable
  // unresolved triggers get a chance to call subscribe() with a real
  // topic name before the per-handle retry below.
  if (retry_callback_) {
    retry_callback_();
  }

  if (shutdown_flag_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_.empty()) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  std::vector<std::string> resolved_keys;
  std::vector<std::string> expired_keys;

  auto topic_names_and_types = node_->get_topic_names_and_types();

  for (auto & [handle_key, entry] : pending_) {
    if (now - entry.created_at > std::chrono::seconds(kPendingTimeoutSec)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "TriggerTopicSubscriber: handle '%s' (topic '%s') type not available after %ds, giving up",
                   handle_key.c_str(), entry.topic_name.c_str(), kPendingTimeoutSec);
      expired_keys.push_back(handle_key);
      continue;
    }

    auto type_it = topic_names_and_types.find(entry.topic_name);
    if (type_it != topic_names_and_types.end() && !type_it->second.empty()) {
      create_subscription_internal(handle_key, entry.topic_name, type_it->second[0], std::move(entry.callback));
      resolved_keys.push_back(handle_key);
    }
  }

  for (const auto & key : resolved_keys) {
    RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: resolved pending handle '%s'", key.c_str());
    pending_.erase(key);
  }
  for (const auto & key : expired_keys) {
    pending_.erase(key);
  }
}

}  // namespace ros2_medkit_gateway
