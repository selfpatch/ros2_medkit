// Copyright 2025 selfpatch
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

#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"

namespace ros2_medkit_gateway {

DiscoveryManager::DiscoveryManager(rclcpp::Node * node)
  : node_(node), runtime_strategy_(std::make_unique<discovery::RuntimeDiscoveryStrategy>(node)) {
}

std::vector<Area> DiscoveryManager::discover_areas() {
  return runtime_strategy_->discover_areas();
}

std::vector<Component> DiscoveryManager::discover_components() {
  return runtime_strategy_->discover_components();
}

std::vector<App> DiscoveryManager::discover_apps() {
  return runtime_strategy_->discover_apps();
}

std::vector<Function> DiscoveryManager::discover_functions() {
  return runtime_strategy_->discover_functions();
}

std::vector<Component> DiscoveryManager::discover_topic_components() {
  return runtime_strategy_->discover_topic_components();
}

std::vector<ServiceInfo> DiscoveryManager::discover_services() {
  return runtime_strategy_->discover_services();
}

std::vector<ActionInfo> DiscoveryManager::discover_actions() {
  return runtime_strategy_->discover_actions();
}

std::optional<ServiceInfo> DiscoveryManager::find_service(const std::string & component_ns,
                                                          const std::string & operation_name) const {
  return runtime_strategy_->find_service(component_ns, operation_name);
}

std::optional<ActionInfo> DiscoveryManager::find_action(const std::string & component_ns,
                                                        const std::string & operation_name) const {
  return runtime_strategy_->find_action(component_ns, operation_name);
}

void DiscoveryManager::set_topic_sampler(NativeTopicSampler * sampler) {
  runtime_strategy_->set_topic_sampler(sampler);
}

void DiscoveryManager::set_type_introspection(TypeIntrospection * introspection) {
  runtime_strategy_->set_type_introspection(introspection);
}

void DiscoveryManager::refresh_topic_map() {
  runtime_strategy_->refresh_topic_map();
}

bool DiscoveryManager::is_topic_map_ready() const {
  return runtime_strategy_->is_topic_map_ready();
}

std::string DiscoveryManager::get_strategy_name() const {
  return runtime_strategy_->get_name();
}

}  // namespace ros2_medkit_gateway
