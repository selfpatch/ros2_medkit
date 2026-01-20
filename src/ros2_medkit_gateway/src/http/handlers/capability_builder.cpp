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

#include "ros2_medkit_gateway/http/handlers/capability_builder.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

std::string CapabilityBuilder::capability_to_name(Capability cap) {
  switch (cap) {
    case Capability::DATA:
      return "data";
    case Capability::OPERATIONS:
      return "operations";
    case Capability::CONFIGURATIONS:
      return "configurations";
    case Capability::FAULTS:
      return "faults";
    case Capability::SUBAREAS:
      return "subareas";
    case Capability::SUBCOMPONENTS:
      return "subcomponents";
    case Capability::RELATED_COMPONENTS:
      return "related-components";
    case Capability::RELATED_APPS:
      return "related-apps";
    case Capability::HOSTS:
      return "hosts";
    case Capability::DEPENDS_ON:
      return "depends-on";
    default:
      return "unknown";
  }
}

std::string CapabilityBuilder::capability_to_path(Capability cap) {
  // Path segments match names for most capabilities
  return capability_to_name(cap);
}

nlohmann::json CapabilityBuilder::build_capabilities(const std::string & entity_type, const std::string & entity_id,
                                                     const std::vector<Capability> & capabilities) {
  nlohmann::json result = nlohmann::json::array();

  for (const auto & cap : capabilities) {
    nlohmann::json cap_obj;
    cap_obj["name"] = capability_to_name(cap);

    // Build href: /api/v1/{entity_type}/{entity_id}/{capability_path}
    std::string href = "/api/v1/";
    href.append(entity_type).append("/").append(entity_id).append("/").append(capability_to_path(cap));
    cap_obj["href"] = href;

    result.push_back(cap_obj);
  }

  return result;
}

LinksBuilder & LinksBuilder::self(const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_["self"] = href;
  return *this;
}

LinksBuilder & LinksBuilder::parent(const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_["parent"] = href;
  return *this;
}

LinksBuilder & LinksBuilder::collection(const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_["collection"] = href;
  return *this;
}

LinksBuilder & LinksBuilder::add(const std::string & rel, const std::string & href) {
  if (!links_.is_object()) {
    links_ = nlohmann::json::object();
  }
  links_[rel] = href;
  return *this;
}

nlohmann::json LinksBuilder::build() const {
  if (links_.is_null()) {
    return nlohmann::json::object();
  }
  return links_;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
