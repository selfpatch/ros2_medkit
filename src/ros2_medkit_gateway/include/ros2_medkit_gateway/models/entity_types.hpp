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

#include <optional>
#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief SOVD Entity types
 *
 * Represents the hierarchy of diagnostic entities in the SOVD model.
 * Each entity type has different capabilities and supported resource collections.
 */
enum class SovdEntityType {
  SERVER,     ///< SOVDServer (root entity) - gateway itself
  AREA,       ///< Logical grouping (ROS 2 namespace or manifest area)
  COMPONENT,  ///< Hardware/virtual unit (ROS 2 node group or manifest component)
  APP,        ///< Software application - individual ROS 2 node
  FUNCTION,   ///< High-level capability (aggregates Apps)
  UNKNOWN     ///< Unknown/not found
};

/**
 * @brief SOVD Resource Collections (Table 7)
 *
 * Standardized collections of diagnostic resources that entities may expose.
 */
enum class ResourceCollection {
  CONFIGURATIONS,        ///< Configuration resources (ROS 2 parameters)
  DATA,                  ///< Static and dynamic data (topic subscriptions)
  FAULTS,                ///< Fault resources (DiagnosticStatus messages)
  OPERATIONS,            ///< Operation resources (services + actions)
  BULK_DATA,             ///< Bulk data resources (large topic payloads)
  DATA_LISTS,            ///< Combined data resources (multi-topic groups)
  LOCKS,                 ///< Lock resources (lifecycle states)
  MODES,                 ///< Mode resources (node modes)
  CYCLIC_SUBSCRIPTIONS,  ///< Cyclic subscriptions (topic polling)
  COMMUNICATION_LOGS,    ///< Communication logs (/rosout filtered)
  TRIGGERS,              ///< Trigger resources (event topics)
  SCRIPTS,               ///< Script resources (not mapped in ROS 2)
  UPDATES                ///< Update packages (not mapped in ROS 2)
};

/**
 * @brief Convert SovdEntityType to string
 * @param type Entity type
 * @return String representation ("Server", "Area", "Component", "App", "Function", "Unknown")
 */
std::string to_string(SovdEntityType type);

/**
 * @brief Convert ResourceCollection to string
 * @param col Resource collection
 * @return String representation (e.g., "configurations", "data")
 */
std::string to_string(ResourceCollection col);

/**
 * @brief Convert ResourceCollection to URL path segment
 * @param col Resource collection
 * @return Path segment (e.g., "data-lists" with hyphens)
 */
std::string to_path_segment(ResourceCollection col);

/**
 * @brief Parse ResourceCollection from path segment
 * @param segment Path segment (e.g., "data-lists")
 * @return Parsed collection, or std::nullopt if invalid
 */
std::optional<ResourceCollection> parse_resource_collection(const std::string & segment);

/**
 * @brief Parse SovdEntityType from string
 * @param str Type string (e.g., "Component", "App")
 * @return Parsed type, or UNKNOWN if invalid
 */
SovdEntityType parse_entity_type(const std::string & str);

}  // namespace ros2_medkit_gateway
