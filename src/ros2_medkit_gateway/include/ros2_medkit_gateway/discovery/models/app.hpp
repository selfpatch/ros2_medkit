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

#ifndef ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__APP_HPP_
#define ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__APP_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD App entity - represents a software application
 *
 * Apps are defined in manifests and represent software applications
 * that run on components. They can expose operations, data, and configurations.
 *
 * @note This is a stub for TASK_002. Full implementation pending.
 */
struct App {
  std::string id;                       ///< Unique identifier
  std::string name;                     ///< Human-readable name
  std::string translation_id;           ///< Internationalization key
  std::string description;              ///< Human-readable description
  std::string component_id;             ///< Host component (is-located-on relationship)
  std::vector<std::string> depends_on;  ///< Dependencies on other apps
  std::vector<std::string> tags;        ///< Tags for filtering
  bool external{false};                 ///< True if not a ROS 2 node

  /**
   * @brief Convert to JSON representation
   * @return JSON object with app data
   *
   * @note TODO: Implement in TASK_002
   */
  json to_json() const {
    // Stub implementation - will be expanded in TASK_002
    json j = {{"id", id}, {"type", "App"}};

    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }
    if (!description.empty()) {
      j["description"] = description;
    }
    if (!component_id.empty()) {
      j["componentId"] = component_id;
    }
    if (!depends_on.empty()) {
      j["dependsOn"] = depends_on;
    }
    if (!tags.empty()) {
      j["tags"] = tags;
    }
    if (external) {
      j["external"] = external;
    }

    return j;
  }

  /**
   * @brief Create SOVD EntityReference format
   * @param base_url Base URL for self links
   * @return JSON object in EntityReference format
   *
   * @note TODO: Implement in TASK_002
   */
  json to_entity_reference(const std::string & base_url) const {
    // Stub implementation
    json j = {{"id", id}, {"type", "App"}};
    if (!name.empty()) {
      j["name"] = name;
    }
    j["self"] = base_url + "/apps/" + id;
    return j;
  }
};

}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__APP_HPP_
