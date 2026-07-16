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
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Zero-config freeze-frames for plugin-backed entities.
 *
 * Plugin entities (PLC apps bridged by protocol plugins) report faults under
 * their bare SOVD entity id, and their live values are not ROS topics - the
 * fault_manager's snapshot capture can never freeze-frame them. This class
 * closes that gap on the gateway side: it subscribes to fault_manager events
 * and, when a fault confirms with a plugin-owned reporting source, snapshots
 * that entity's current data values (DataProvider::list_data, which serves
 * from the plugin's latest polled values) at fault time.
 *
 * The frames are merged into the fault detail's environment_data.snapshots
 * only when the fault_manager captured no freeze-frame itself - explicit
 * snapshot config always wins (see FaultHandlers::merge_entity_freeze_frames).
 */
class EntityFreezeFrameCapture {
 public:
  /// One captured frame: the entity's data values at fault-confirm time.
  struct Frame {
    std::string entity_id;
    nlohmann::json values;  ///< compact {resource_id: value} dict
    int64_t captured_at_ns{0};
  };

  /// Resolves an entity id to its owning plugin's DataProvider (nullptr when
  /// the entity is not plugin-owned). Called from the subscription callback.
  using DataProviderResolver = std::function<DataProvider *(const std::string & entity_id)>;

  /**
   * @param node ROS 2 node used to resolve the fault-events topic name and logger
   * @param exec shared subscription executor; the fault-events subscription is
   *        created and torn down on its serial worker (issue #375 invariant)
   * @param resolver entity-to-DataProvider resolver (typically wraps PluginManager)
   * @param max_faults retained-frame bound; oldest fault's frames evicted past it
   */
  EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                           DataProviderResolver resolver, size_t max_faults = 256);

  ~EntityFreezeFrameCapture();

  // Non-copyable, non-movable
  EntityFreezeFrameCapture(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture & operator=(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture(EntityFreezeFrameCapture &&) = delete;
  EntityFreezeFrameCapture & operator=(EntityFreezeFrameCapture &&) = delete;

  /// Frames captured for a fault code (empty when none). Thread-safe.
  std::vector<Frame> frames_for(const std::string & fault_code) const;

  /// Build the compact {resource_id: value} dict from a DataProvider::list_data
  /// response. Items without a "value" field map to null; a response without an
  /// "items" array is kept verbatim (plugin-defined shape).
  static nlohmann::json values_from_list_content(const nlohmann::json & content);

 private:
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> subscription_slot_;
  DataProviderResolver resolver_;
  rclcpp::Logger logger_;
  const size_t max_faults_;

  /// Guards frames_ and insertion_order_: subscription callback writes,
  /// HTTP handler threads read.
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::vector<Frame>> frames_;
  std::deque<std::string> insertion_order_;  ///< eviction order (FIFO)
};

}  // namespace ros2_medkit_gateway
