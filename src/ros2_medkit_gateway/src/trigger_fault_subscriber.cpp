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

#include "ros2_medkit_gateway/trigger_fault_subscriber.hpp"

#include <utility>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"

namespace ros2_medkit_gateway {

TriggerFaultSubscriber::TriggerFaultSubscriber(rclcpp::Node * node, ResourceChangeNotifier & notifier)
  : source_(*node, build_fault_manager_events_topic(node)), feed_(source_, notifier) {
  RCLCPP_INFO(node->get_logger(), "TriggerFaultSubscriber initialized, subscribed to %s",
              build_fault_manager_events_topic(node).c_str());
}

void TriggerFaultSubscriber::set_node_to_entity_resolver(NodeToEntityFn resolver) {
  feed_.set_node_to_entity_resolver(std::move(resolver));
}

}  // namespace ros2_medkit_gateway
