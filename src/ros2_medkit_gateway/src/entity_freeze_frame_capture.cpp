// Copyright 2026 mfaferek93
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

#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Bound on waiting for the events subscription to match a publisher before
/// the standing-fault snapshot; past it the catch-up proceeds best-effort.
constexpr std::chrono::seconds kEventsMatchTimeout{10};

}  // namespace

namespace ros2_capture_detail {

SlotFaultEvents::SlotFaultEvents(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec)
  : exec_(exec), logger_(node->get_logger()), topic_(build_fault_manager_events_topic(node)) {
}

SlotFaultEvents::~SlotFaultEvents() {
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.clear();
  }
  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> dropped;
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    dropped = std::move(slot_);
  }
  // Not a synchronous barrier: the slot posts an async destroy with a bounded
  // deadline, so use-after-free safety also relies on the owner tearing down
  // the subscription executor (joining its worker) before this object's node
  // dies - keep that ordering in main.cpp / gateway_node shutdown. A late
  // callback before that teardown finds no subscriber and drops its event.
  dropped.reset();
}

FaultEventSubscription SlotFaultEvents::subscribe(Callback callback) {
  {
    // Held across the create, so concurrent first subscribers make one slot.
    std::lock_guard<std::mutex> lock(slot_mutex_);
    if (!slot_) {
      // The topic comes from the gateway node (it owns fault_manager.namespace);
      // the subscription itself is created on the executor's dedicated _sub node so
      // it never races rcl's hash-map on the main node (issue #375).
      auto slot = ros2_common::Ros2SubscriptionSlot::create_typed<ros2_medkit_msgs::msg::FaultEvent>(
          exec_, topic_, rclcpp::QoS(100).reliable(),
          [this](const std::shared_ptr<const ros2_medkit_msgs::msg::FaultEvent> & msg) {
            on_message(*msg);
          });
      if (!slot) {
        RCLCPP_ERROR(logger_, "EntityFreezeFrameCapture: failed to subscribe to %s: %s", topic_.c_str(),
                     slot.error().c_str());
        return FaultEventSubscription{};
      }
      slot_ = std::move(*slot);
    }
  }
  auto subscriber = std::make_shared<Subscriber>();
  subscriber->callback = std::move(callback);
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.push_back(subscriber);
  }
  return FaultEventSubscription([this, subscriber]() {
    remove(subscriber);
  });
}

void SlotFaultEvents::remove(const std::shared_ptr<Subscriber> & subscriber) {
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.erase(std::remove(subscribers_.begin(), subscribers_.end(), subscriber), subscribers_.end());
  }
  // Waits for a running callback. A delivery that copied the list before the
  // erase finds the callback null.
  std::lock_guard<std::mutex> lock(subscriber->mutex);
  subscriber->callback = nullptr;
}

void SlotFaultEvents::on_message(const ros2_medkit_msgs::msg::FaultEvent & msg) {
  std::vector<std::shared_ptr<Subscriber>> targets;
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    targets = subscribers_;
  }
  if (targets.empty()) {
    return;
  }
  const auto record = ros2::conversions::from_msg(msg);
  for (const auto & subscriber : targets) {
    std::lock_guard<std::mutex> lock(subscriber->mutex);
    if (subscriber->callback) {
      subscriber->callback(record);
    }
  }
}

void SlotFaultEvents::wait_for_publisher(const std::function<bool()> & should_abort) const {
  const auto deadline = std::chrono::steady_clock::now() + kEventsMatchTimeout;
  while (std::chrono::steady_clock::now() < deadline && !should_abort()) {
    try {
      {
        std::lock_guard<std::mutex> lock(slot_mutex_);
        if (!slot_) {
          return;
        }
        if (slot_->publisher_count() > 0) {
          return;
        }
      }
    } catch (const std::exception &) {
      return;  // graph query failed; proceed best-effort
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace ros2_capture_detail

EntityFreezeFrameCapture::EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                                                   DataProviderResolver resolver, RouteDataFetcher route_fetcher,
                                                   size_t max_faults, StandingFaultLister standing_lister)
  : logger_(node->get_logger())
  , events_(node, exec)
  , capture_(std::make_unique<FreezeFrameCapture>(
        events_, FreezeFrameCaptureConfig{true, max_faults},
        FreezeFrameCapture::Hooks{PluginDataReaders{std::move(resolver), std::move(route_fetcher)},
                                  std::move(standing_lister),
                                  [&events = events_](const std::function<bool()> & should_abort) {
                                    events.wait_for_publisher(should_abort);
                                  }})) {
  RCLCPP_INFO(logger_, "EntityFreezeFrameCapture initialized, subscribed to %s", events_.topic().c_str());
}

EntityFreezeFrameCapture::~EntityFreezeFrameCapture() = default;

std::vector<FreezeFrameSource::Frame> EntityFreezeFrameCapture::frames_for(const std::string & fault_code) const {
  return capture_->frames_for(fault_code);
}

}  // namespace ros2_medkit_gateway
