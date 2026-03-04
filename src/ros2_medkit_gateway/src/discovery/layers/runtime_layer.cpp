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
#include <iterator>
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

// Filter entities with namespace_path by gap-fill config, returns count of removed entities
template <typename Entity>
size_t filter_by_namespace(std::vector<Entity> & entities, const GapFillConfig & config) {
  size_t before = entities.size();
  entities.erase(std::remove_if(entities.begin(), entities.end(),
                                [&config](const Entity & e) {
                                  return !is_namespace_allowed(e.namespace_path, config);
                                }),
                 entities.end());
  return before - entities.size();
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
  if (!runtime_strategy_) {
    return output;
  }

  if (gap_fill_config_.allow_heuristic_areas) {
    output.areas = runtime_strategy_->discover_areas();
    last_filtered_count_ += filter_by_namespace(output.areas, gap_fill_config_);
  }

  if (gap_fill_config_.allow_heuristic_components) {
    output.components = runtime_strategy_->discover_components();

    // Topic components are discovered separately and must be included
    auto topic_components = runtime_strategy_->discover_topic_components();
    output.components.insert(output.components.end(), std::make_move_iterator(topic_components.begin()),
                             std::make_move_iterator(topic_components.end()));

    last_filtered_count_ += filter_by_namespace(output.components, gap_fill_config_);
  }

  if (gap_fill_config_.allow_heuristic_apps) {
    output.apps = runtime_strategy_->discover_apps();
  }

  if (gap_fill_config_.allow_heuristic_functions) {
    output.functions = runtime_strategy_->discover_functions();
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
