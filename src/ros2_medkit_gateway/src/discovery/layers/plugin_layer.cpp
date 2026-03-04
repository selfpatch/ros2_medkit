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

#include "ros2_medkit_gateway/discovery/layers/plugin_layer.hpp"

#include <utility>

namespace ros2_medkit_gateway {
namespace discovery {

PluginLayer::PluginLayer(std::string plugin_name, IntrospectionProvider * provider)
  : name_(std::move(plugin_name)), provider_(provider) {
  policies_ = {{FieldGroup::IDENTITY, MergePolicy::ENRICHMENT},
               {FieldGroup::HIERARCHY, MergePolicy::ENRICHMENT},
               {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT},
               {FieldGroup::STATUS, MergePolicy::ENRICHMENT},
               {FieldGroup::METADATA, MergePolicy::AUTHORITATIVE}};
}

LayerOutput PluginLayer::discover() {
  LayerOutput output;
  if (!provider_) {
    return output;
  }

  // Build input (currently empty - pipeline will provide current entities in a future step)
  IntrospectionInput input;
  auto result = provider_->introspect(input);

  // Map new_entities to LayerOutput (no functions - plugins cannot create functions)
  output.areas = std::move(result.new_entities.areas);
  output.components = std::move(result.new_entities.components);
  output.apps = std::move(result.new_entities.apps);

  // Store metadata and pass through LayerOutput for pipeline consumption
  last_metadata_ = result.metadata;
  output.entity_metadata = result.metadata;

  return output;
}

MergePolicy PluginLayer::policy_for(FieldGroup group) const {
  auto it = policies_.find(group);
  if (it != policies_.end()) {
    return it->second;
  }
  return MergePolicy::ENRICHMENT;
}

void PluginLayer::set_policy(FieldGroup group, MergePolicy policy) {
  policies_[group] = policy;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
