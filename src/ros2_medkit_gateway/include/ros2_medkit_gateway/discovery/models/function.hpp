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

#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD Function entity - represents a functional group
 *
 * Functions are higher-level abstractions that group Apps and/or Components
 * representing a complete capability (e.g., "Autonomous Navigation").
 *
 * Functions are always manifest-defined and don't exist at runtime by themselves.
 * They aggregate data, operations, and faults from their hosted entities.
 */
struct Function {
  // === Required fields ===
  std::string id;    ///< Unique identifier
  std::string name;  ///< Human-readable name

  // === Optional SOVD fields ===
  std::string translation_id;     ///< For i18n support
  std::string description;        ///< Detailed description
  std::vector<std::string> tags;  ///< Tags for filtering

  // === Relationships ===
  std::vector<std::string> hosts;       ///< App or Component IDs this function hosts
  std::vector<std::string> depends_on;  ///< depends-on relationship (Function IDs)

  // === Discovery metadata ===
  std::string source = "manifest";  ///< Always "manifest" (functions don't exist at runtime)

  // === Serialization methods ===

  /**
   * @brief Serialize to full JSON representation
   */
  json to_json() const;

  /**
   * @brief Create SOVD EntityReference format
   * @param base_url Base URL for href (e.g., "/api/v1")
   */
  json to_entity_reference(const std::string & base_url) const;

  /**
   * @brief Create SOVD Entity Capabilities format
   * @param base_url Base URL for capability URIs
   */
  json to_capabilities(const std::string & base_url) const;
};

}  // namespace ros2_medkit_gateway
