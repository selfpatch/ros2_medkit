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

#include "ros2_medkit_gateway/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

json Function::to_json() const {
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
  json x_medkit = {{"entityType", "Function"}, {"source", source}};
  if (!description.empty()) {
    x_medkit["description"] = description;
  }
  if (!hosts.empty()) {
    x_medkit["hosts"] = hosts;
  }
  if (!depends_on.empty()) {
    x_medkit["dependsOn"] = depends_on;
  }
  j["x-medkit"] = x_medkit;

  return j;
}

json Function::to_entity_reference(const std::string & base_url) const {
  // SOVD-compliant EntityReference: id, name, href, [translationId, tags]
  json j = {{"id", id}, {"href", base_url + "/functions/" + id}};

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

json Function::to_capabilities(const std::string & base_url) const {
  std::string func_base = base_url + "/functions/" + id;

  // SOVD-compliant capabilities response
  json j = {{"id", id}};

  if (!name.empty()) {
    j["name"] = name;
  }
  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }

  // Function-specific capabilities (SOVD compliant)
  if (!hosts.empty()) {
    j["hosts"] = func_base + "/hosts";
  }
  if (!depends_on.empty()) {
    j["depends-on"] = func_base + "/depends-on";
  }

  // Functions can also have data, operations, faults aggregated from hosted entities
  j["data"] = func_base + "/data";
  j["operations"] = func_base + "/operations";
  j["faults"] = func_base + "/faults";

  // x-medkit extension for ROS 2 specific info
  j["x-medkit"] = {{"entityType", "Function"}, {"source", source}};

  return j;
}

}  // namespace ros2_medkit_gateway
