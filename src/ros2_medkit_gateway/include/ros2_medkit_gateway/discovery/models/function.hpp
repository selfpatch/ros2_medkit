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

#ifndef ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__FUNCTION_HPP_
#define ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__FUNCTION_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD Function entity - represents a functional grouping
 *
 * Functions are defined in manifests and represent logical groupings
 * of capabilities that can span multiple apps or components.
 *
 * @note This is a stub for TASK_002. Full implementation pending.
 */
struct Function {
  std::string id;                       ///< Unique identifier
  std::string name;                     ///< Human-readable name
  std::string translation_id;           ///< Internationalization key
  std::string description;              ///< Human-readable description
  std::vector<std::string> hosts;       ///< Host app or component IDs
  std::vector<std::string> depends_on;  ///< Dependencies on other functions
  std::vector<std::string> tags;        ///< Tags for filtering

  /**
   * @brief Convert to JSON representation
   * @return JSON object with function data
   *
   * @note TODO: Implement in TASK_002
   */
  json to_json() const {
    // Stub implementation - will be expanded in TASK_002
    json j = {{"id", id}, {"type", "Function"}};

    if (!name.empty()) {
      j["name"] = name;
    }
    if (!translation_id.empty()) {
      j["translationId"] = translation_id;
    }
    if (!description.empty()) {
      j["description"] = description;
    }
    if (!hosts.empty()) {
      j["hosts"] = hosts;
    }
    if (!depends_on.empty()) {
      j["dependsOn"] = depends_on;
    }
    if (!tags.empty()) {
      j["tags"] = tags;
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
    json j = {{"id", id}, {"type", "Function"}};
    if (!name.empty()) {
      j["name"] = name;
    }
    j["self"] = base_url + "/functions/" + id;
    return j;
  }
};

}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__FUNCTION_HPP_
