// Copyright 2025 mfaferek93
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

#include "ros2_medkit_gateway/discovery_manager.hpp"

#include <algorithm>
#include <set>

namespace ros2_medkit_gateway {

DiscoveryManager::DiscoveryManager(rclcpp::Node * node) : node_(node) {
}

std::vector<Area> DiscoveryManager::discover_areas() {
  // Extract unique areas from namespaces
  std::set<std::string> area_set;

  // Get node graph interface
  auto node_graph = node_->get_node_graph_interface();

  // Iterate through all nodes to find namespaces
  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();

  for (const auto & name_and_ns : names_and_namespaces) {
    std::string ns = name_and_ns.second;
    std::string area = extract_area_from_namespace(ns);
    area_set.insert(area);
  }

  // Convert set to vector of Area structs
  std::vector<Area> areas;
  for (const auto & area_name : area_set) {
    Area area;
    area.id = area_name;
    area.namespace_path = (area_name == "root") ? "/" : "/" + area_name;
    areas.push_back(area);
  }

  return areas;
}

std::vector<Component> DiscoveryManager::discover_components() {
  std::vector<Component> components;

  auto node_graph = node_->get_node_graph_interface();
  auto names_and_namespaces = node_graph->get_node_names_and_namespaces();

  // Build topic map if not yet ready (first call or after manual refresh)
  if (topic_sampler_ && !topic_map_ready_) {
    refresh_topic_map();
  }

  for (const auto & name_and_ns : names_and_namespaces) {
    const auto & name = name_and_ns.first;
    const auto & ns = name_and_ns.second;

    Component comp;
    comp.id = name;
    comp.namespace_path = ns;
    comp.fqn = (ns == "/") ? std::string("/").append(name) : std::string(ns).append("/").append(name);
    comp.area = extract_area_from_namespace(ns);

    // Populate topics from cached map
    if (topic_sampler_) {
      auto it = cached_topic_map_.find(comp.fqn);
      if (it != cached_topic_map_.end()) {
        comp.topics = it->second;
      }
    }

    components.push_back(comp);
  }

  return components;
}

void DiscoveryManager::set_topic_sampler(NativeTopicSampler * sampler) {
  topic_sampler_ = sampler;
}

void DiscoveryManager::refresh_topic_map() {
  if (!topic_sampler_) {
    return;
  }
  cached_topic_map_ = topic_sampler_->build_component_topic_map();
  topic_map_ready_ = true;
  RCLCPP_DEBUG(node_->get_logger(), "Topic map refreshed: %zu components", cached_topic_map_.size());
}

bool DiscoveryManager::is_topic_map_ready() const {
  return topic_map_ready_;
}

std::string DiscoveryManager::extract_area_from_namespace(const std::string & ns) {
  if (ns == "/" || ns.empty()) {
    return "root";
  }

  // Remove leading slash
  std::string cleaned = ns;
  if (cleaned[0] == '/') {
    cleaned = cleaned.substr(1);
  }

  // Get first segment
  auto pos = cleaned.find('/');
  if (pos != std::string::npos) {
    return cleaned.substr(0, pos);
  }

  return cleaned;
}

}  // namespace ros2_medkit_gateway
