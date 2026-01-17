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

#include "ros2_medkit_gateway/discovery/models/app.hpp"

namespace ros2_medkit_gateway {

json App::to_json() const {
  json j = {{"id", id}, {"name", name}, {"type", "App"}, {"source", source}};

  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }
  if (!description.empty()) {
    j["description"] = description;
  }
  if (!tags.empty()) {
    j["tags"] = tags;
  }
  if (!component_id.empty()) {
    j["componentId"] = component_id;
  }
  if (!depends_on.empty()) {
    j["dependsOn"] = depends_on;
  }
  if (ros_binding.has_value() && !ros_binding->is_empty()) {
    j["rosBinding"] = ros_binding->to_json();
  }
  if (bound_fqn.has_value()) {
    j["boundFqn"] = bound_fqn.value();
  }
  j["isOnline"] = is_online;
  if (external) {
    j["external"] = external;
  }

  // Add topics if present
  if (!topics.publishes.empty() || !topics.subscribes.empty()) {
    j["topics"] = topics.to_json();
  }

  // Add operations (combine services and actions)
  json operations = json::array();
  for (const auto & svc : services) {
    operations.push_back(svc.to_json());
  }
  for (const auto & act : actions) {
    operations.push_back(act.to_json());
  }
  if (!operations.empty()) {
    j["operations"] = operations;
  }

  return j;
}

json App::to_entity_reference(const std::string & base_url) const {
  json j = {{"id", id}, {"name", name}, {"href", base_url + "/apps/" + id}};

  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }
  if (!tags.empty()) {
    j["tags"] = tags;
  }

  return j;
}

json App::to_capabilities(const std::string & base_url) const {
  std::string app_base = base_url + "/apps/" + id;

  json j = {{"id", id}, {"name", name}};

  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }

  // Add capability URIs
  if (!topics.publishes.empty() || !topics.subscribes.empty()) {
    j["data"] = app_base + "/data";
  }
  if (!services.empty() || !actions.empty()) {
    j["operations"] = app_base + "/operations";
  }
  // Always include configurations (parameters) for non-external apps
  if (!external) {
    j["configurations"] = app_base + "/configurations";
  }
  // Always include faults
  j["faults"] = app_base + "/faults";

  // Relationships
  if (!component_id.empty()) {
    j["isLocatedOn"] = base_url + "/components/" + component_id;
  }
  if (!depends_on.empty()) {
    j["dependsOn"] = app_base + "/depends-on";
  }

  return j;
}

}  // namespace ros2_medkit_gateway
