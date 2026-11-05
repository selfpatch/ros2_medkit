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

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_gateway/core/faults/freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

namespace ros2_capture_detail {

/// Fault events from the fault manager's events topic, delivered through a
/// slot on the shared subscription executor (issue #375 invariant: the
/// subscription is created and torn down on that executor's serial worker).
/// Volatile on purpose: a transient_local reader does not match the fault
/// manager's volatile publisher, so upgrading the QoS would break matching
/// against older fault managers. The startup catch-up covers the pre-match window.
/// The slot is created on the first subscribe() and kept for the source's life.
class SlotFaultEvents : public FaultEventSource {
 public:
  SlotFaultEvents(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec);
  ~SlotFaultEvents() override;
  SlotFaultEvents(const SlotFaultEvents &) = delete;
  SlotFaultEvents & operator=(const SlotFaultEvents &) = delete;
  SlotFaultEvents(SlotFaultEvents &&) = delete;
  SlotFaultEvents & operator=(SlotFaultEvents &&) = delete;

  /// Returns an empty handle when the slot cannot be created (logged).
  /// Must not be called from the executor worker: creating the slot waits on it.
  [[nodiscard]] FaultEventSubscription subscribe(Callback callback) override;

  /// Bounded, abortable wait until the subscription has a matched publisher.
  void wait_for_publisher(const std::function<bool()> & should_abort) const;

  const std::string & topic() const {
    return topic_;
  }

 private:
  /// One subscription's callback. `mutex` is held while it runs, so removal waits for it.
  struct Subscriber {
    std::mutex mutex;
    Callback callback;  ///< Null once removed. Guarded by `mutex`.
  };

  void on_message(const ros2_medkit_msgs::msg::FaultEvent & msg);
  void remove(const std::shared_ptr<Subscriber> & subscriber);

  ros2_common::Ros2SubscriptionExecutor & exec_;
  rclcpp::Logger logger_;
  std::string topic_;
  /// Guards subscribers_. Never held while a callback runs.
  std::mutex subscribers_mutex_;
  std::vector<std::shared_ptr<Subscriber>> subscribers_;
  /// Guards slot_. Held while the slot is created, never on the executor worker.
  mutable std::mutex slot_mutex_;
  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> slot_;
};

}  // namespace ros2_capture_detail

/**
 * @brief The freeze-frame capture of the ROS node.
 *
 * The capture itself is FreezeFrameCapture (gateway_core). This class owns it
 * and the source that feeds it the fault_manager's events through the shared
 * subscription executor, and orders the startup catch-up after the subscription
 * has matched a publisher. The subscription callback only enqueues - plugin
 * calls run on the capture thread so a slow read never stalls the executor worker.
 */
class EntityFreezeFrameCapture : public FreezeFrameSource {
 public:
  using DataProviderResolver = FreezeFrameCapture::DataProviderResolver;
  using RouteDataFetcher = FreezeFrameCapture::RouteDataFetcher;
  using StandingFault = FreezeFrameCapture::StandingFault;
  using StandingFaultLister = FreezeFrameCapture::StandingFaultLister;

  /**
   * @param node ROS 2 node used to resolve the fault-events topic name and logger
   * @param exec shared subscription executor; the fault-events subscription is
   *        created and torn down on its serial worker (issue #375 invariant).
   * @param resolver entity-to-DataProvider resolver (typically wraps PluginManager)
   * @param route_fetcher x-plc-data route fallback for entities whose plugin
   *        has no DataProvider (the commercial PLC bridges); may be null
   * @param max_faults retained-frame bound; oldest fault's frames evicted past it
   * @param standing_lister startup catch-up; may be null
   */
  EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                           DataProviderResolver resolver, RouteDataFetcher route_fetcher = nullptr,
                           size_t max_faults = 256, StandingFaultLister standing_lister = nullptr);

  ~EntityFreezeFrameCapture() override;
  EntityFreezeFrameCapture(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture & operator=(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture(EntityFreezeFrameCapture &&) = delete;
  EntityFreezeFrameCapture & operator=(EntityFreezeFrameCapture &&) = delete;

  std::vector<Frame> frames_for(const std::string & fault_code) const override;

  /// See FreezeFrameCapture::standing_faults_from_list_reply.
  static std::optional<std::vector<StandingFault>> standing_faults_from_list_reply(const nlohmann::json & data) {
    return FreezeFrameCapture::standing_faults_from_list_reply(data);
  }

 private:
  rclcpp::Logger logger_;
  /// Declared before capture_: built before the capture and destroyed after it.
  ros2_capture_detail::SlotFaultEvents events_;
  std::unique_ptr<FreezeFrameCapture> capture_;
};

}  // namespace ros2_medkit_gateway
