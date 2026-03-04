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
  std::lock_guard<std::mutex> lock(mutex_);
  cached_result_ = pipeline_.execute();
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "Hybrid discovery refreshed: %zu entities", cached_result_.report.total_entities);
  }
}

const MergeReport & HybridDiscoveryStrategy::get_merge_report() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_.report;
}

const LinkingResult & HybridDiscoveryStrategy::get_linking_result() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pipeline_.get_linking_result();
}

std::vector<std::string> HybridDiscoveryStrategy::get_orphan_nodes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pipeline_.get_linking_result().orphan_nodes;
}

void HybridDiscoveryStrategy::add_layer(std::unique_ptr<DiscoveryLayer> layer) {
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_.add_layer(std::move(layer));
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
