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

#ifndef ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__COMPONENT_HPP_
#define ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__COMPONENT_HPP_

#include "ros2_medkit_gateway/discovery/models/common.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD Component entity - represents a software/hardware component (ROS 2 node)
 *
 * Components are derived from ROS 2 nodes or defined in manifests.
 * They expose operations (services/actions), data (topics), and configurations (parameters).
 */
struct Component {
  std::string id;                     ///< Unique identifier (node name)
  std::string name;                   ///< Human-readable name
  std::string namespace_path;         ///< ROS 2 namespace path
  std::string fqn;                    ///< Fully qualified name (namespace + id)
  std::string type = "Component";     ///< Entity type (always "Component")
  std::string area;                   ///< Parent area ID
  std::string source = "node";        ///< Discovery source: "node", "topic", or "manifest"
  std::string translation_id;         ///< Internationalization key
  std::string description;            ///< Human-readable description
  std::string variant;                ///< Hardware variant identifier
  std::vector<std::string> tags;      ///< Tags for filtering
  std::string parent_component_id;    ///< Parent component ID for sub-components
  std::vector<std::string> depends_on;  ///< Component IDs this component depends on
  std::vector<ServiceInfo> services;  ///< Services exposed by this component
  std::vector<ActionInfo> actions;    ///< Actions exposed by this component
  ComponentTopics topics;             ///< Topics this component publishes/subscribes

  /**
   * @brief Convert to JSON representation
   * @return JSON object with component data
   */
  json to_json() const {
    json j = {{"id", id},         {"namespace", namespace_path}, {"fqn", fqn}, {"type", type}, {"area", area},
              {"source", source}, {"topics", topics.to_json()}};

    // Include optional fields only if set
    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }
    if (!description.empty()) {
      j["description"] = description;
    }
    if (!variant.empty()) {
      j["variant"] = variant;
    }
    if (!tags.empty()) {
      j["tags"] = tags;
    }
    if (!parent_component_id.empty()) {
      j["parentComponentId"] = parent_component_id;
    }
    if (!depends_on.empty()) {
      j["dependsOn"] = depends_on;
    }

    // Add operations array combining services and actions
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

  /**
   * @brief Create SOVD EntityReference format
   * @param base_url Base URL for self links
   * @return JSON object in EntityReference format
   */
  json to_entity_reference(const std::string & base_url) const {
    json j = {{"id", id}, {"type", type}};
    if (!name.empty()) {
      j["name"] = name;
    }
    j["self"] = base_url + "/components/" + id;
    return j;
  }

  /**
   * @brief Create SOVD Entity Capabilities format
   * @param base_url Base URL for capability links
   * @return JSON object listing available sub-resources
   */
  json to_capabilities(const std::string & base_url) const {
    json capabilities = json::array();
    std::string component_url = base_url + "/components/" + id;

    // Data capability (topics)
    if (!topics.publishes.empty() || !topics.subscribes.empty()) {
      capabilities.push_back({{"name", "data"}, {"href", component_url + "/data"}});
    }

    // Operations capability (services + actions)
    if (!services.empty() || !actions.empty()) {
      capabilities.push_back({{"name", "operations"}, {"href", component_url + "/operations"}});
    }

    // Configurations capability (parameters) - always present for ROS 2 nodes
    if (source == "node") {
      capabilities.push_back({{"name", "configurations"}, {"href", component_url + "/configurations"}});
    }

    // Depends-on capability
    if (!depends_on.empty()) {
      capabilities.push_back({{"name", "depends-on"}, {"href", component_url + "/depends-on"}});
    }

    return {{"id", id}, {"type", type}, {"capabilities", capabilities}};
  }
};

}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__COMPONENT_HPP_
