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

#include "ros2_medkit_gateway/discovery/hybrid_discovery.hpp"

#include <utility>

namespace ros2_medkit_gateway {
namespace discovery {

HybridDiscoveryStrategy::HybridDiscoveryStrategy(rclcpp::Node * node, MergePipeline pipeline)
  : node_(node), pipeline_(std::move(pipeline)) {
  // Initial pipeline execution
  refresh();
}

std::vector<Area> HybridDiscoveryStrategy::discover_areas() {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.areas;
}

std::vector<Component> HybridDiscoveryStrategy::discover_components() {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.components;
}

std::vector<App> HybridDiscoveryStrategy::discover_apps() {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.apps;
}

std::vector<Function> HybridDiscoveryStrategy::discover_functions() {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.functions;
}

void HybridDiscoveryStrategy::refresh() {
  // Execute pipeline WITHOUT lock - safe because:
  // 1. add_layer() is called during initialization (from DiscoveryManager::initialize()
  //    and gateway_node plugin setup) before the EntityCache timer starts
  // 2. refresh() is only called from the ROS executor timer callback (single-threaded)
  //    - concurrent refresh() calls cannot occur under the default executor model
  auto new_result = pipeline_.execute();

  // Swap result under lock to protect concurrent discover_*() readers
  std::lock_guard<std::mutex> lock(mutex_);
  cached_result_ = std::move(new_result);
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "Hybrid discovery refreshed: %zu entities", cached_result_.report.total_entities);
  }
}

MergeReport HybridDiscoveryStrategy::get_merge_report() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.report;
}

LinkingResult HybridDiscoveryStrategy::get_linking_result() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.linking_result;
}

std::vector<std::string> HybridDiscoveryStrategy::get_orphan_nodes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.linking_result.orphan_nodes;
}

void HybridDiscoveryStrategy::add_layer(std::unique_ptr<DiscoveryLayer> layer) {
  pipeline_.add_layer(std::move(layer));
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
