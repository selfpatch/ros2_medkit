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

namespace ros2_medkit_gateway {
namespace discovery {

HybridDiscoveryStrategy::HybridDiscoveryStrategy(rclcpp::Node * node, ManifestManager * manifest_manager,
                                                 RuntimeDiscoveryStrategy * runtime_strategy)
  : node_(node), manifest_manager_(manifest_manager), runtime_strategy_(runtime_strategy), linker_(node) {
  // Perform initial linking if manifest is loaded
  if (manifest_manager_ && manifest_manager_->is_manifest_active()) {
    perform_linking();
  }
}

std::vector<Area> HybridDiscoveryStrategy::discover_areas() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!manifest_manager_ || !manifest_manager_->is_manifest_active()) {
    // Fallback to runtime if no manifest
    return runtime_strategy_->discover_areas();
  }

  return manifest_manager_->get_areas();
}

std::vector<Component> HybridDiscoveryStrategy::discover_components() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!manifest_manager_ || !manifest_manager_->is_manifest_active()) {
    return runtime_strategy_->discover_components();
  }

  // Get manifest components
  std::vector<Component> result = manifest_manager_->get_components();

  // Get runtime components for enrichment
  auto runtime_components = runtime_strategy_->discover_components();

  // Build a map of runtime components by FQN for quick lookup
  std::unordered_map<std::string, const Component *> runtime_map;
  for (const auto & comp : runtime_components) {
    runtime_map[comp.fqn] = &comp;
  }

  // Enrich manifest components with runtime data if they match
  for (auto & comp : result) {
    auto it = runtime_map.find(comp.fqn);
    if (it != runtime_map.end()) {
      // Copy runtime data
      comp.topics = it->second->topics;
      comp.services = it->second->services;
      comp.actions = it->second->actions;
    }
  }

  // Handle orphan nodes based on policy
  auto config = manifest_manager_->get_config();
  if (config.unmanifested_nodes == ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN) {
    // Add orphan nodes as components
    std::set<std::string> manifest_fqns;
    for (const auto & comp : result) {
      manifest_fqns.insert(comp.fqn);
    }

    for (const auto & runtime_comp : runtime_components) {
      if (manifest_fqns.find(runtime_comp.fqn) == manifest_fqns.end()) {
        Component orphan = runtime_comp;
        orphan.source = "orphan";
        result.push_back(orphan);
      }
    }
  }

  return result;
}

std::vector<App> HybridDiscoveryStrategy::discover_apps() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!manifest_manager_ || !manifest_manager_->is_manifest_active()) {
    // No apps in runtime-only mode
    return {};
  }

  // Return linked apps from last linking result
  return linking_result_.linked_apps;
}

std::vector<Function> HybridDiscoveryStrategy::discover_functions() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!manifest_manager_ || !manifest_manager_->is_manifest_active()) {
    // No functions in runtime-only mode
    return {};
  }

  return manifest_manager_->get_functions();
}

void HybridDiscoveryStrategy::refresh_linking() {
  std::lock_guard<std::mutex> lock(mutex_);
  perform_linking();
}

void HybridDiscoveryStrategy::perform_linking() {
  if (!manifest_manager_ || !manifest_manager_->is_manifest_active()) {
    log_info("Cannot perform linking: no active manifest");
    return;
  }

  // Get manifest apps
  auto apps = manifest_manager_->get_apps();

  // Get runtime node components (raw nodes, not synthetic groupings)
  // Runtime linking needs individual node FQNs to match against manifest bindings
  auto runtime_components = runtime_strategy_->discover_node_components();

  // Get config for orphan policy
  auto config = manifest_manager_->get_config();

  // Perform linking
  linking_result_ = linker_.link(apps, runtime_components, config);

  log_info("Hybrid linking complete: " + linking_result_.summary());
}

void HybridDiscoveryStrategy::log_info(const std::string & msg) const {
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
