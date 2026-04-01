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

#include "ros2_medkit_gateway/discovery/layers/runtime_layer.hpp"

#include <algorithm>
#include <utility>

namespace ros2_medkit_gateway {
namespace discovery {

namespace {

// Check if a namespace is allowed by the gap-fill config
bool is_namespace_allowed(const std::string & ns, const GapFillConfig & config) {
  // If whitelist is non-empty, namespace must match
  if (!config.namespace_whitelist.empty()) {
    bool found =
        std::any_of(config.namespace_whitelist.begin(), config.namespace_whitelist.end(), [&ns](const std::string & w) {
          return ns == w || ns.find(w + "/") == 0;
        });
    if (!found) {
      return false;
    }
  }
  // Check blacklist
  for (const auto & b : config.namespace_blacklist) {
    if (ns == b || ns.find(b + "/") == 0) {
      return false;
    }
  }
  return true;
}

// Extract namespace from a fully-qualified node name (e.g. "/ns/sub/node" -> "/ns/sub")
std::string namespace_from_fqn(const std::string & fqn) {
  auto pos = fqn.rfind('/');
  if (pos == std::string::npos || pos == 0) {
    return "/";
  }
  return fqn.substr(0, pos);
}

// Filter apps by namespace derived from bound_fqn
size_t filter_apps_by_namespace(std::vector<App> & apps, const GapFillConfig & config) {
  size_t before = apps.size();
  apps.erase(std::remove_if(apps.begin(), apps.end(),
                            [&config](const App & a) {
                              if (!a.bound_fqn.has_value()) {
                                return false;  // Keep unbound apps
                              }
                              return !is_namespace_allowed(namespace_from_fqn(*a.bound_fqn), config);
                            }),
             apps.end());
  return before - apps.size();
}

}  // namespace

RuntimeLayer::RuntimeLayer(RuntimeDiscoveryStrategy * runtime_strategy) : runtime_strategy_(runtime_strategy) {
  policies_ = {{FieldGroup::IDENTITY, MergePolicy::FALLBACK},
               {FieldGroup::HIERARCHY, MergePolicy::FALLBACK},
               {FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE},
               {FieldGroup::STATUS, MergePolicy::AUTHORITATIVE},
               {FieldGroup::METADATA, MergePolicy::ENRICHMENT}};
}

LayerOutput RuntimeLayer::discover() {
  LayerOutput output;
  last_filtered_count_ = 0;
  linking_apps_.clear();
  if (!runtime_strategy_) {
    return output;
  }

  // Areas and Components are never created by runtime discovery.
  // Areas come from manifest only. Components come from HostInfoProvider or manifest.

  // Discover apps once. Always save unfiltered apps for post-merge linking.
  // The linker needs all runtime apps to bind manifest apps to live nodes,
  // regardless of gap-fill settings.
  auto apps = runtime_strategy_->discover_apps();
  linking_apps_ = apps;

  if (gap_fill_config_.allow_heuristic_apps) {
    output.apps = std::move(apps);
    last_filtered_count_ += filter_apps_by_namespace(output.apps, gap_fill_config_);
  }

  if (gap_fill_config_.allow_heuristic_functions) {
    // Use the pre-discovered apps to avoid redundant ROS 2 graph introspection
    output.functions = runtime_strategy_->discover_functions(linking_apps_);
  }

  return output;
}

MergePolicy RuntimeLayer::policy_for(FieldGroup group) const {
  auto it = policies_.find(group);
  if (it != policies_.end()) {
    return it->second;
  }
  return MergePolicy::ENRICHMENT;
}

void RuntimeLayer::set_policy(FieldGroup group, MergePolicy policy) {
  policies_[group] = policy;
}

void RuntimeLayer::set_gap_fill_config(GapFillConfig config) {
  gap_fill_config_ = std::move(config);
}

std::vector<ServiceInfo> RuntimeLayer::discover_services() {
  if (!runtime_strategy_) {
    return {};
  }
  return runtime_strategy_->discover_services();
}

std::vector<ActionInfo> RuntimeLayer::discover_actions() {
  if (!runtime_strategy_) {
    return {};
  }
  return runtime_strategy_->discover_actions();
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
