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

#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/faults/trigger_fault_feed.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/ros2/ros2_fault_event_source.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Feeds the fault events of the configured fault manager events topic to
 *        ResourceChangeNotifier for trigger evaluation.
 *
 * The ROS adapter of TriggerFaultFeed: it owns a Ros2FaultEventSource on the
 * events topic and a feed on top of it. The conversion and the entity mapping
 * are the feed's.
 */
class TriggerFaultSubscriber {
 public:
  /**
   * @brief Construct the subscriber.
   * @param node ROS 2 node for creating the subscription (caller manages lifetime)
   * @param notifier ResourceChangeNotifier to forward events to
   */
  TriggerFaultSubscriber(rclcpp::Node * node, ResourceChangeNotifier & notifier);

  /// Callback to resolve ROS node FQN to manifest entity ID.
  /// Returns manifest entity ID, or empty string if not resolvable.
  using NodeToEntityFn = TriggerFaultFeed::NodeToEntityFn;

  /// Set the node-to-entity resolver for manifest entity ID lookup.
  /// Write-once: call it before the ROS executor starts spinning.
  void set_node_to_entity_resolver(NodeToEntityFn resolver);

  ~TriggerFaultSubscriber() = default;
  TriggerFaultSubscriber(const TriggerFaultSubscriber &) = delete;
  TriggerFaultSubscriber & operator=(const TriggerFaultSubscriber &) = delete;
  TriggerFaultSubscriber(TriggerFaultSubscriber &&) = delete;
  TriggerFaultSubscriber & operator=(TriggerFaultSubscriber &&) = delete;

 private:
  Ros2FaultEventSource source_;
  TriggerFaultFeed feed_;  ///< after source_: destroyed first, so it unsubscribes while the source lives
};

}  // namespace ros2_medkit_gateway
