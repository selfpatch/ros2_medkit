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

#include "ros2_medkit_gateway/discovery/models/app.hpp"

namespace ros2_medkit_gateway {

json App::to_json() const {
  // SOVD-compliant base fields
  json j = {{"id", id}};

  if (!name.empty()) {
    j["name"] = name;
  }
  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }
  if (!tags.empty()) {
    j["tags"] = tags;
  }

  // ROS 2 extensions in x-medkit (SOVD vendor extension)
  json x_medkit = {{"entityType", "App"}, {"source", source}};
  if (!description.empty()) {
    x_medkit["description"] = description;
  }
  if (!component_id.empty()) {
    x_medkit["componentId"] = component_id;
  }
  if (!depends_on.empty()) {
    x_medkit["dependsOn"] = depends_on;
  }
  if (ros_binding.has_value() && !ros_binding->is_empty()) {
    x_medkit["rosBinding"] = ros_binding->to_json();
  }
  if (bound_fqn.has_value()) {
    x_medkit["boundFqn"] = bound_fqn.value();
  }
  x_medkit["isOnline"] = is_online;
  if (external) {
    x_medkit["external"] = external;
  }
  // Add topics if present
  if (!topics.publishes.empty() || !topics.subscribes.empty()) {
    x_medkit["topics"] = topics.to_json();
  }
  j["x-medkit"] = x_medkit;

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
  // SOVD-compliant EntityReference: id, name, href, [translationId, tags]
  json j = {{"id", id}, {"href", base_url + "/apps/" + id}};

  if (!name.empty()) {
    j["name"] = name;
  }
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

  // SOVD-compliant capabilities response
  json j = {{"id", id}};

  if (!name.empty()) {
    j["name"] = name;
  }
  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }

  // Capabilities as URI references (SOVD compliant)
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

  // Relationships (SOVD standard)
  if (!component_id.empty()) {
    j["is-located-on"] = base_url + "/components/" + component_id;
  }
  if (!depends_on.empty()) {
    j["depends-on"] = app_base + "/depends-on";
  }

  // x-medkit extension for ROS 2 specific info
  json x_medkit = {{"entityType", "App"}, {"source", source}, {"isOnline", is_online}};
  if (bound_fqn.has_value()) {
    x_medkit["boundFqn"] = bound_fqn.value();
  }
  j["x-medkit"] = x_medkit;

  return j;
}

}  // namespace ros2_medkit_gateway
