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
  : shutdown_flag_{exec.shutdown_flag_ptr()}
  , exec_{&exec}
  , topic_{std::move(topic)}
  , type_name_{std::move(type_name)}
  , sub_{std::move(sub)} {
}

Ros2SubscriptionSlot::~Ros2SubscriptionSlot() noexcept {
  // Wrapped because run_sync allocates a std::function + promise/future +
  // tl::expected; any of those can throw on OOM and would otherwise escape
  // a non-noexcept destructor and trip std::terminate when another exception
  // is in flight. Logging-only handler keeps teardown going.
  try {
    if (!sub_) {
      return;
    }

    // Move ownership OUT of `this` and into a local. The posted task below
    // captures the local by move, so the rcl handle's lifetime no longer
    // depends on `this` still being alive when the worker eventually runs.
    //
    // Why this matters: Ros2SubscriptionExecutor::run_sync posts a wrapper
    // task that captures the lambda we pass in. On deadline timeout, run_sync
    // returns an error but DOES NOT cancel the queued wrapper - the worker
    // will dequeue and execute it later (at latest during executor shutdown
    // queue drain). If we captured `[this]` the queued task would dereference
    // this freed slot: classic UAF. Capturing the shared_ptr by move makes the
    // task self-sufficient.
    auto sub_to_drop = std::move(sub_);

    // Read the shutdown flag through our shared_ptr copy, not through exec_.
    // The executor may have been destroyed ahead of this slot (tests and
    // process teardown both exercise that ordering); the shared flag stays
    // readable because slot and executor co-own it.
    if (shutdown_flag_ && shutdown_flag_->load(std::memory_order_acquire)) {
      // Fast path: executor teardown (if still ongoing) will call rcl_node_fini
      // on the subscription node which cleans up the rcl handle atomically;
      // if the executor is already gone, the node died with it. Releasing the
      // local shared_ptr on the current thread is safe in both sub-cases
      // because rcl handle lifetime is already bound to the node.
      sub_to_drop.reset();
      return;
    }

    // Worker self-destruct fast path: if a callback running ON the worker
    // (e.g. a graph callback evicting a pool entry whose slot it owns) drops
    // its own slot, posting to run_sync and waiting deadlocks - the worker
    // is the one that has to drain the queue. Detect "I am the worker" via
    // thread id and drop the subscription inline, which is single-writer-safe
    // because we ARE the single writer.
    if (exec_ && std::this_thread::get_id() == exec_->worker_thread_id()) {
      sub_to_drop.reset();
      return;
    }

    // Normal path: post destroy to the worker so we do not race with
    // concurrent rcl mutations. Bounded deadline to avoid hanging teardown.
    // The wrapper owns sub_to_drop by move; even if run_sync's deadline fires
    // before the worker drains the task, the destructor returns safely while
    // the queued task still holds the last reference and releases it when it
    // finally runs (normal path or shutdown queue drain).
    auto result = exec_->run_sync<void>(std::function<void()>([sub = std::move(sub_to_drop)]() mutable {
                                          sub.reset();
                                        }),
                                        kDestroyDeadline);
    if (!result) {
      RCLCPP_WARN(rclcpp::get_logger("ros2_subscription_slot"),
                  "Subscription destroy for topic '%s' did not complete within deadline (%s); "
                  "rcl handle release deferred to executor shutdown drain.",
                  topic_.c_str(), result.error().c_str());
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_slot"),
                 "Exception during ~Ros2SubscriptionSlot for topic '%s': %s", topic_.c_str(), ex.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_slot"), "Unknown exception during ~Ros2SubscriptionSlot");
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
