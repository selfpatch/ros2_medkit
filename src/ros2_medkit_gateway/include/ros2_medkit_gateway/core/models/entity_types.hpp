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

#include <cstddef>
#include <functional>
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
 * @brief An entity named by its type and its id - a member of a grouping
 * entity, the entity a lock is held on, an entity a plugin owns
 *
 * An App and a Component may share an id, so the id alone does not name an
 * entity: everything that resolves, dispatches to or asks the reachability of a
 * member, and every record kept per entity, takes the pair.
 */
struct MemberRef {
  SovdEntityType type{SovdEntityType::UNKNOWN};
  std::string id;

  bool operator==(const MemberRef & other) const {
    return type == other.type && id == other.id;
  }
  bool operator!=(const MemberRef & other) const {
    return !(*this == other);
  }
};

/// Hash of a MemberRef, for the maps that hold one record per entity - a lock,
/// a lock override, a plugin's ownership of an entity - and so key it by the
/// entity's type and id, never by the id alone.
struct MemberRefHash {
  size_t operator()(const MemberRef & ref) const noexcept {
    size_t seed = std::hash<std::string>{}(ref.id);
    seed ^= static_cast<size_t>(ref.type) + size_t{0x9e3779b9U} + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

/**
 * @brief SOVD Resource Collections (Table 7)
 *
 * Standardized collections of diagnostic resources that entities may expose.
 *
 * An enumerator here names a collection the SOVD model knows about. It says
 * nothing about whether the gateway serves it: a collection joins an entity's
 * capability list in `EntityCapabilities::for_type` only once an entity-scoped
 * route answers it, because every entry in that list becomes an `href` a client
 * follows and a path in the entity's `/docs` sub-document. No entity type lists
 * the four enumerators marked "no entity-scoped route" below for that reason -
 * `UPDATES` appears only in the SERVER list, whose collections are the ones
 * mounted at the API root. They are kept so a manifest or a plugin that later
 * serves one has a name for it, and so `parse_resource_collection` still
 * recognises the segment rather than reporting it as an unknown collection.
 */
enum class ResourceCollection {
  CONFIGURATIONS,        ///< Configuration resources (ROS 2 parameters)
  DATA,                  ///< Static and dynamic data (topic subscriptions)
  DATA_CATEGORIES,       ///< Data categories (route answers 501 - no ROS 2 mapping)
  DATA_GROUPS,           ///< Data groups (route answers 501 - no ROS 2 mapping)
  FAULTS,                ///< Fault resources (DiagnosticStatus messages)
  FAULT_TRIGGERS,        ///< Threshold rules that raise a fault (apps only, x-medkit)
  OPERATIONS,            ///< Operation resources (services + actions)
  BULK_DATA,             ///< Bulk data resources (large topic payloads)
  DATA_LISTS,            ///< Combined data resources - no entity-scoped route
  LOCKS,                 ///< Lock resources (exclusive entity access)
  MODES,                 ///< Mode resources - no entity-scoped route
  CYCLIC_SUBSCRIPTIONS,  ///< Cyclic subscriptions (topic polling)
  LOGS,                  ///< Application log entries (/rosout)
  COMMUNICATION_LOGS,    ///< Communication logs (CAN/UDS/DoIP) - no entity-scoped route
  TRIGGERS,              ///< Trigger resources (event topics)
  SCRIPTS,               ///< Diagnostic script resources
  UPDATES                ///< Update packages - server-scoped only, no entity-scoped route
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

/**
 * @brief The entity type an entity collection's path segment names
 * @param segment "areas", "components", "apps" or "functions"
 * @return The type, or UNKNOWN for any other segment
 */
SovdEntityType entity_type_from_collection(const std::string & segment);

}  // namespace ros2_medkit_gateway
