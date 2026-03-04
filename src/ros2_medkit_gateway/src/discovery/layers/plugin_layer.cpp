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

#include <algorithm>
#include <utility>

namespace ros2_medkit_gateway {
namespace discovery {

namespace {

bool is_valid_entity_id(const std::string & id) {
  if (id.empty() || id.size() > 256) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
  });
}

template <typename T>
void validate_entities(std::vector<T> & entities, const std::string & layer_name, const rclcpp::Logger & logger) {
  auto it = std::remove_if(entities.begin(), entities.end(), [&](const T & e) {
    if (!is_valid_entity_id(e.id)) {
      RCLCPP_WARN(logger, "Plugin '%s': dropping entity with invalid ID '%s'", layer_name.c_str(), e.id.c_str());
      return true;
    }
    return false;
  });
  entities.erase(it, entities.end());
}

}  // namespace

PluginLayer::PluginLayer(std::string plugin_name, IntrospectionProvider * provider)
  : name_(std::move(plugin_name)), provider_(provider), logger_(rclcpp::get_logger("plugin_layer." + name_)) {
  policies_ = {{FieldGroup::IDENTITY, MergePolicy::ENRICHMENT},
               {FieldGroup::HIERARCHY, MergePolicy::ENRICHMENT},
               {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT},
               {FieldGroup::STATUS, MergePolicy::ENRICHMENT},
               {FieldGroup::METADATA, MergePolicy::ENRICHMENT}};
}

LayerOutput PluginLayer::discover() {
  LayerOutput output;
  if (!provider_) {
    return output;
  }

  auto result = provider_->introspect(discovery_context_);

  // Map new_entities to LayerOutput (no functions - plugins cannot create functions)
  output.areas = std::move(result.new_entities.areas);
  output.components = std::move(result.new_entities.components);
  output.apps = std::move(result.new_entities.apps);

  // Validate entity IDs from plugin
  validate_entities(output.areas, name_, logger_);
  validate_entities(output.components, name_, logger_);
  validate_entities(output.apps, name_, logger_);

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

void PluginLayer::set_discovery_context(const IntrospectionInput & context) {
  discovery_context_ = context;
}

void PluginLayer::set_policy(FieldGroup group, MergePolicy policy) {
  policies_[group] = policy;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
