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

#ifndef ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__AREA_HPP_
#define ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__AREA_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD Area entity - represents a logical grouping (ROS 2 namespace)
 *
 * Areas are derived from ROS 2 namespaces or defined in manifests.
 * They provide a hierarchical organization of components.
 */
struct Area {
  std::string id;                 ///< Unique identifier (e.g., "powertrain")
  std::string name;               ///< Human-readable name (e.g., "Powertrain System")
  std::string namespace_path;     ///< ROS 2 namespace path (e.g., "/powertrain")
  std::string type = "Area";      ///< Entity type (always "Area")
  std::string translation_id;     ///< Internationalization key
  std::string description;        ///< Human-readable description
  std::vector<std::string> tags;  ///< Tags for filtering
  std::string parent_area_id;     ///< Parent area ID for sub-areas

  /**
   * @brief Convert to JSON representation
   * @return JSON object with area data
   */
  json to_json() const {
    json j = {{"id", id}, {"namespace", namespace_path}, {"type", type}};

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
    if (!tags.empty()) {
      j["tags"] = tags;
    }
    if (!parent_area_id.empty()) {
      j["parentAreaId"] = parent_area_id;
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
    j["self"] = base_url + "/areas/" + id;
    return j;
  }
};

}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__DISCOVERY__MODELS__AREA_HPP_
