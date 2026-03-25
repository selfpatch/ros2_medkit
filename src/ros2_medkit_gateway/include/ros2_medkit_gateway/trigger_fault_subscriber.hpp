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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/resource_change_notifier.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Subscribes to the configured fault manager events topic and forwards fault events
 *        to ResourceChangeNotifier for trigger evaluation.
 *
 * Follows the same ROS 2 subscription pattern as SSEFaultHandler - subscribes
 * to the fault event topic and converts each FaultEvent into a ResourceChange
 * notification on the "faults" collection.
 *
 * Entity ID is derived from the first reporting source of the fault. The
 * fault_code is used as the resource_path. The full fault JSON (via
 * FaultManager::fault_to_json) is passed as the change value.
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
  using NodeToEntityFn = std::function<std::string(const std::string &)>;

  /// Set the node-to-entity resolver for manifest entity ID lookup.
  void set_node_to_entity_resolver(NodeToEntityFn resolver);

  // Non-copyable, non-movable
  TriggerFaultSubscriber(const TriggerFaultSubscriber &) = delete;
  TriggerFaultSubscriber & operator=(const TriggerFaultSubscriber &) = delete;
  TriggerFaultSubscriber(TriggerFaultSubscriber &&) = delete;
  TriggerFaultSubscriber & operator=(TriggerFaultSubscriber &&) = delete;

 private:
  /// Callback for fault events from ROS 2 topic
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr subscription_;
  ResourceChangeNotifier & notifier_;
  rclcpp::Logger logger_;
  /// Write-once: must be set before ROS executor starts spinning.
  /// After that, only read from subscription callbacks (no mutex needed).
  NodeToEntityFn node_to_entity_resolver_;
};

}  // namespace ros2_medkit_gateway
