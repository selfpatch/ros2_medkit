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

#include <rclcpp/qos.hpp>
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

void TriggerTopicSubscriber::set_subscription_executor(ros2_common::Ros2SubscriptionExecutor * exec) {
  // Release so a subscribe() on an httplib thread that observes this executor
  // also observes everything the setter's caller published before it.
  sub_exec_.store(exec, std::memory_order_release);
}

std::string TriggerTopicSubscriber::resolve_topic_type(const std::string & topic_name) const {
  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic_name);
  if (it != topic_names_and_types.end() && !it->second.empty()) {
    return it->second[0];
  }
  return {};
}

TriggerTopicSubscriber::MessageCallback TriggerTopicSubscriber::make_message_callback(const std::string & handle_key,
                                                                                      const std::string & topic_name,
                                                                                      const std::string & msg_type) {
  // Snapshot the handle key + topic + type by value so the callback never
  // dereferences any expired strings; the user callback is looked up under
  // mutex_ to avoid firing after unsubscribe()/shutdown() drops the entry.
  return [this, handle_key, topic_name, msg_type](const std::shared_ptr<const rclcpp::SerializedMessage> & msg) {
    if (shutdown_flag_.load()) {
      return;
    }

    SampleCallback user_cb;
    {
      std::lock_guard<std::mutex> cb_lock(mutex_);
      auto it = live_.find(handle_key);
      if (it == live_.end()) {
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
}

tl::expected<std::unique_ptr<ros2_common::Ros2SubscriptionSlot>, std::string>
TriggerTopicSubscriber::create_slot_unlocked(const std::string & handle_key, const std::string & topic_name,
                                             const std::string & msg_type) {
  auto * exec = sub_exec_.load(std::memory_order_acquire);
  if (!exec) {
    // Defensive: the callers only reach here once the executor is wired -
    // subscribe() defers to pending while it is unwired, and retry runs off the
    // timer, which fires only after spin (i.e. after wiring). Never create on
    // the racy main-node path; report so the caller leaves the handle pending.
    return tl::unexpected(std::string{"TriggerTopicSubscriber: no subscription executor wired"});
  }
  const rclcpp::QoS qos = rclcpp::SensorDataQoS();
  return ros2_common::Ros2SubscriptionSlot::create_generic(*exec, topic_name, msg_type, qos,
                                                           make_message_callback(handle_key, topic_name, msg_type));
}

void TriggerTopicSubscriber::subscribe(const std::string & topic_name, const std::string & msg_type,
                                       const std::string & handle_key, SampleCallback callback) {
  if (shutdown_flag_.load()) {
    return;
  }

  // If the subscription executor is not wired yet, defer to pending instead of
  // failing. This window is real: persistent triggers are restored in the
  // GatewayNode constructor, and the REST server (also started there) can take a
  // POST /triggers, both before main.cpp wires the executor. The retry timer
  // only fires once the executor is spinning (after wiring), so a deferred
  // handle resolves on its first tick - no error log, no 5xx for a valid
  // request. sub_exec_ is set once and never cleared, so a value seen here stays
  // valid through Phase B below.
  const bool executor_ready = sub_exec_.load(std::memory_order_acquire) != nullptr;

  // Resolve the message type OUTSIDE mutex_. get_topic_names_and_types() takes
  // the rmw graph locks and building the full map is not cheap; holding mutex_
  // across it would stall every sample dispatch on the worker (each takes
  // mutex_) for the query's duration. Phase C re-checks the world under the
  // lock, so a race between this resolve and the commit is harmless. Skip it
  // when the executor is unwired - we would defer to pending regardless, and
  // retry re-resolves then.
  std::string resolved_type = msg_type;
  if (executor_ready && resolved_type.empty()) {
    resolved_type = resolve_topic_type(topic_name);
  }
  // A single const decision, used for both the queue-under-lock branch and the
  // early return below, so `callback` is moved on exactly one of the two
  // mutually exclusive paths (queue vs create).
  const bool should_defer = !executor_ready || resolved_type.empty();

  // Phase A (under lock): drop any prior registration for this handle. The prior
  // slot is moved out and destroyed OUTSIDE the lock - ~Ros2SubscriptionSlot
  // posts a run_sync destroy to the worker, which would deadlock against a
  // callback waiting on mutex_. old_slot is declared in the function scope so it
  // is destroyed only after the lock_guard releases mutex_.
  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> old_slot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = live_.find(handle_key);
    if (it != live_.end()) {
      old_slot = std::move(it->second.slot);
      live_.erase(it);
    }
    pending_.erase(handle_key);
  }
  // Drop the prior slot outside the lock (run_sync destroy on the worker).
  old_slot.reset();

  if (should_defer) {
    // Queue for retry, which creates the slot once the type resolves and the
    // executor is wired. `callback` is consumed on this branch only; the create
    // path below (reached only when !should_defer) never touches it.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      PendingEntry entry;
      entry.topic_name = topic_name;
      entry.msg_type = msg_type;  // caller's original; retry re-resolves when empty
      entry.callback = std::move(callback);
      entry.created_at = std::chrono::steady_clock::now();
      pending_[handle_key] = std::move(entry);
    }
    if (!executor_ready) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "TriggerTopicSubscriber: subscription executor not wired yet, queuing handle '%s' ('%s') for retry",
                   handle_key.c_str(), topic_name.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "TriggerTopicSubscriber: cannot determine type for topic '%s' "
                  "(no publishers yet), queuing handle '%s' for retry",
                  topic_name.c_str(), handle_key.c_str());
    }
    return;
  }

  // Phase B (outside lock): create the slot on the shared executor worker.
  auto slot_or_err = create_slot_unlocked(handle_key, topic_name, resolved_type);
  if (!slot_or_err) {
    RCLCPP_ERROR(node_->get_logger(),
                 "TriggerTopicSubscriber: failed to create subscription for '%s' (handle '%s'): %s", topic_name.c_str(),
                 handle_key.c_str(), slot_or_err.error().c_str());
    // Propagate so the transport returns nullptr and the trigger is rejected.
    throw std::runtime_error(slot_or_err.error());
  }
  auto slot = std::move(*slot_or_err);

  // Populate the entry BEFORE the lock (function scope). If live_'s insert then
  // throws bad_alloc under the lock, entry - which owns the slot - is destroyed
  // during unwind AFTER the lock_guard releases mutex_, so ~Ros2SubscriptionSlot
  // posts its run_sync destroy off the lock, never under it (which would
  // deadlock the worker). Same discipline as old_slot/slots_to_drop.
  LiveEntry entry;
  entry.slot = std::move(slot);
  entry.topic_name = topic_name;
  entry.msg_type = resolved_type;
  entry.callback = std::move(callback);

  // Phase C (under lock): commit unless shutdown began meanwhile. A fresh handle
  // key has no concurrent unsubscribe (the transport Handle does not exist until
  // this returns), so shutdown_flag_ is the only world-change to guard here.
  bool committed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!shutdown_flag_.load()) {
      live_[handle_key] = std::move(entry);
      committed = true;
    }
  }
  if (!committed) {
    // Shutdown raced us between create and insert. `entry` still owns the slot;
    // it is destroyed at function return, outside the lock (run_sync destroy on
    // the worker, which is still alive because shutdown() runs before
    // sub_exec.reset()).
    RCLCPP_INFO(node_->get_logger(),
                "TriggerTopicSubscriber: dropped subscription for handle '%s' (shutdown in progress)",
                handle_key.c_str());
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: subscribed handle '%s' to '%s' (type=%s)",
              handle_key.c_str(), topic_name.c_str(), resolved_type.c_str());
}

void TriggerTopicSubscriber::unsubscribe(const std::string & handle_key) {
  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> slot_to_drop;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pending_.erase(handle_key) > 0) {
      RCLCPP_DEBUG(node_->get_logger(), "TriggerTopicSubscriber: removed pending handle '%s'", handle_key.c_str());
      return;
    }

    auto it = live_.find(handle_key);
    if (it == live_.end()) {
      return;
    }

    // Move the slot out and erase the map entry atomically under the lock so
    // any in-flight dispatch already sees the empty map and short-circuits.
    slot_to_drop = std::move(it->second.slot);
    it->second.callback = nullptr;
    live_.erase(it);
  }
  // Destroy the slot OUTSIDE the lock. ~Ros2SubscriptionSlot posts a destroy
  // task to the executor worker (the same thread that dispatches callbacks),
  // so any in-flight callback for this handle drains before the destroy runs.
  // Doing this under mutex_ would deadlock against such a callback.
  slot_to_drop.reset();

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

  // Move every live slot out under the lock, clear the maps, then destroy the
  // slots OUTSIDE the lock. shutdown_flag_ (set above) makes new dispatches
  // short-circuit; the slot destructors post run_sync destroys to the executor
  // worker, draining any in-flight callback before returning. Destroying slots
  // while holding mutex_ would deadlock against a callback blocked on mutex_.
  std::vector<std::unique_ptr<ros2_common::Ros2SubscriptionSlot>> slots_to_drop;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
    slots_to_drop.reserve(live_.size());
    for (auto & [_, entry] : live_) {
      slots_to_drop.push_back(std::move(entry.slot));
      entry.callback = nullptr;
    }
    live_.clear();
  }
  slots_to_drop.clear();  // slot destructors run here, outside the lock
  RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: shutdown complete");
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

  // The callback is COPIED (not moved) out of pending_ so pending_[handle_key]
  // stays intact as the "still wanted" marker until the slot is committed; if
  // unsubscribe() removes it while we create the slot (outside the lock), the
  // commit is skipped and the slot dropped.
  struct Resolvable {
    std::string handle_key;
    std::string topic_name;
    std::string msg_type;
    SampleCallback callback;
  };

  // Phase A (under lock): expire stale handles and snapshot the rest. NO graph
  // query here - get_topic_names_and_types() takes the rmw graph locks and is
  // not cheap, and holding mutex_ across it would stall every sample dispatch on
  // the worker (each takes mutex_) for the query's duration.
  struct Candidate {
    std::string handle_key;
    std::string topic_name;
    std::string msg_type;  // caller's requested type, empty = auto-resolve
    SampleCallback callback;
  };
  std::vector<Candidate> candidates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.empty()) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    std::vector<std::string> expired_keys;

    for (auto & [handle_key, entry] : pending_) {
      if (now - entry.created_at > std::chrono::seconds(kPendingTimeoutSec)) {
        RCLCPP_ERROR(node_->get_logger(),
                     "TriggerTopicSubscriber: handle '%s' (topic '%s') type not available after %ds, giving up",
                     handle_key.c_str(), entry.topic_name.c_str(), kPendingTimeoutSec);
        expired_keys.push_back(handle_key);
        continue;
      }
      candidates.push_back(Candidate{handle_key, entry.topic_name, entry.msg_type, entry.callback});
    }

    for (const auto & key : expired_keys) {
      pending_.erase(key);
    }
  }

  // Resolve types OUTSIDE the lock: use the caller's explicit type when given,
  // otherwise the current ROS graph. A handle unsubscribed between here and the
  // commit is caught by the Phase C re-check, so racing this query is harmless.
  const auto topic_names_and_types = node_->get_topic_names_and_types();
  std::vector<Resolvable> resolvable;
  for (auto & c : candidates) {
    std::string type = c.msg_type;
    if (type.empty()) {
      auto type_it = topic_names_and_types.find(c.topic_name);
      if (type_it != topic_names_and_types.end() && !type_it->second.empty()) {
        type = type_it->second[0];
      }
    }
    if (!type.empty()) {
      resolvable.push_back(Resolvable{c.handle_key, c.topic_name, std::move(type), std::move(c.callback)});
    }
  }

  // Phase B/C (per handle, outside lock): create the slot, then commit it into
  // live_ iff the handle is still pending (not unsubscribed meanwhile) and
  // shutdown has not begun. pending_[key] is the "still wanted" marker.
  for (auto & r : resolvable) {
    if (shutdown_flag_.load()) {
      break;
    }

    auto slot_or_err = create_slot_unlocked(r.handle_key, r.topic_name, r.msg_type);
    if (!slot_or_err) {
      RCLCPP_ERROR(node_->get_logger(),
                   "TriggerTopicSubscriber: retry failed to create subscription for '%s' (handle '%s'): %s",
                   r.topic_name.c_str(), r.handle_key.c_str(), slot_or_err.error().c_str());
      continue;  // leave it pending; a later tick retries until the timeout
    }
    // Populate the entry BEFORE the lock. If live_'s insert throws bad_alloc
    // under the lock, entry - which owns the slot - is destroyed during unwind
    // AFTER the lock releases, so ~Ros2SubscriptionSlot posts its run_sync
    // destroy off the lock, never under it (which would deadlock the worker).
    LiveEntry entry;
    entry.slot = std::move(*slot_or_err);
    entry.topic_name = r.topic_name;
    entry.msg_type = r.msg_type;
    entry.callback = std::move(r.callback);

    bool committed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto pend_it = pending_.find(r.handle_key);
      if (!shutdown_flag_.load() && pend_it != pending_.end()) {
        // Insert BEFORE erasing pending_: if the insert throws, the handle
        // stays pending and a later tick retries it, rather than being lost.
        live_[r.handle_key] = std::move(entry);
        pending_.erase(pend_it);
        committed = true;
      }
    }
    if (committed) {
      RCLCPP_INFO(node_->get_logger(), "TriggerTopicSubscriber: resolved pending handle '%s' -> '%s' (type=%s)",
                  r.handle_key.c_str(), r.topic_name.c_str(), r.msg_type.c_str());
    } else {
      // Unsubscribed or shutdown between snapshot and commit. `entry` still owns
      // the slot; it is destroyed at the end of this loop iteration, outside the
      // lock.
      RCLCPP_DEBUG(node_->get_logger(),
                   "TriggerTopicSubscriber: pending handle '%s' vanished before commit; dropped subscription",
                   r.handle_key.c_str());
    }
  }
}

}  // namespace ros2_medkit_gateway
