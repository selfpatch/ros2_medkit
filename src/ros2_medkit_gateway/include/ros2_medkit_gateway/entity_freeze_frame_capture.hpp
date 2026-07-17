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
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
 *
 * Retention mirrors the fault_manager's freeze-frame semantics: frames are
 * kept across EVENT_CLEARED (the confirmed-state record stays attached to
 * the cleared fault's detail) and overwritten on every EVENT_CONFIRMED, so
 * a re-occurrence re-samples the plugin at its own confirm time.
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

  /// Fallback for plugins without a DataProvider: fetches the entity's current
  /// values by dispatching the owning plugin's own `x-plc-data` route
  /// in-process (typically wraps PluginManager::fetch_entity_data_via_route).
  /// Returns the parsed route response, nullopt when unavailable.
  using RouteDataFetcher = std::function<std::optional<nlohmann::json>(const std::string & entity_id)>;

  /**
   * @param node ROS 2 node used to resolve the fault-events topic name and logger
   * @param exec shared subscription executor; the fault-events subscription is
   *        created and torn down on its serial worker (issue #375 invariant)
   * @param resolver entity-to-DataProvider resolver (typically wraps PluginManager)
   * @param route_fetcher x-plc-data route fallback for entities whose plugin
   *        has no DataProvider (the commercial PLC bridges); may be null
   * @param max_faults retained-frame bound; oldest fault's frames evicted past it
   */
  EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                           DataProviderResolver resolver, RouteDataFetcher route_fetcher = nullptr,
                           size_t max_faults = 256);

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

  /// True when an x-plc-data route response carries real live values: not
  /// flagged disconnected, with a non-empty items array. A PLC that is down
  /// must not freeze-frame a row of nulls.
  static bool route_content_has_live_data(const nlohmann::json & content);

 private:
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  /// Capture via the plugin's own x-plc-data route (no DataProvider exported).
  /// Returns nullopt when the route yields nothing usable.
  std::optional<Frame> capture_via_route(const std::string & entity_id, const std::string & fault_code);

  /// Log a fallback failure once per fault code (faults re-confirm on every
  /// clear/re-report cycle; one line per code is enough for an operator).
  void log_fallback_failure_once(const std::string & fault_code, const std::string & message);

  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> subscription_slot_;
  DataProviderResolver resolver_;
  RouteDataFetcher route_fetcher_;
  rclcpp::Logger logger_;
  const size_t max_faults_;

  /// Guards frames_, insertion_order_ and fallback_logged_: subscription
  /// callback writes, HTTP handler threads read.
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::vector<Frame>> frames_;
  std::deque<std::string> insertion_order_;          ///< eviction order (FIFO)
  std::unordered_set<std::string> fallback_logged_;  ///< fault codes already warned about
};

}  // namespace ros2_medkit_gateway
