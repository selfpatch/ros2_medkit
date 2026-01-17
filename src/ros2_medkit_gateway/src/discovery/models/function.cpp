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

#include "ros2_medkit_gateway/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

json Function::to_json() const {
  json j = {{"id", id}, {"name", name}, {"type", "Function"}, {"source", source}};

  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }
  if (!description.empty()) {
    j["description"] = description;
  }
  if (!tags.empty()) {
    j["tags"] = tags;
  }
  if (!hosts.empty()) {
    j["hosts"] = hosts;
  }
  if (!depends_on.empty()) {
    j["dependsOn"] = depends_on;
  }

  return j;
}

json Function::to_entity_reference(const std::string & base_url) const {
  json j = {{"id", id}, {"name", name}, {"href", base_url + "/functions/" + id}};

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

  json j = {{"id", id}, {"name", name}};

  if (!translation_id.empty()) {
    j["translationId"] = translation_id;
  }

  // Function-specific capabilities
  if (!hosts.empty()) {
    j["hosts"] = func_base + "/hosts";
  }
  if (!depends_on.empty()) {
    j["dependsOn"] = func_base + "/depends-on";
  }

  // Functions can also have data, operations, faults aggregated from hosted entities
  j["data"] = func_base + "/data";
  j["operations"] = func_base + "/operations";
  j["faults"] = func_base + "/faults";

  return j;
}

}  // namespace ros2_medkit_gateway
