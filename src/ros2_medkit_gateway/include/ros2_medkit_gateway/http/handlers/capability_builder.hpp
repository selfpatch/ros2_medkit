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

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Utility class for building capability arrays.
 *
 * Generates capabilities JSON for entity responses, ensuring consistent
 * format across all entity types (areas, components, apps, functions).
 *
 * @example
 * using Cap = CapabilityBuilder::Capability;
 * std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS};
 * auto json = CapabilityBuilder::build_capabilities("components", "my-comp", caps);
 *
 * @verifies REQ_DISCOVERY_003 Entity capabilities
 */
class CapabilityBuilder {
 public:
  /**
   * @brief Available capability types for SOVD entities.
   */
  enum class Capability {
    DATA,                ///< Entity has data endpoints
    OPERATIONS,          ///< Entity has operations (services/actions)
    CONFIGURATIONS,      ///< Entity has configurations (parameters)
    FAULTS,              ///< Entity has fault management
    SUBAREAS,            ///< Entity has child areas (areas only)
    SUBCOMPONENTS,       ///< Entity has child components (components only)
    RELATED_COMPONENTS,  ///< Entity has related components (areas only)
    CONTAINS,            ///< Entity contains other entities (areas->components)
    RELATED_APPS,        ///< Entity has related apps (components only)
    HOSTS,               ///< Entity has host apps (functions/components)
    DEPENDS_ON           ///< Entity has dependencies (components only)
  };

  /**
   * @brief Build capabilities JSON array for an entity.
   *
   * @param entity_type The entity type (e.g., "areas", "components", "apps", "functions")
   * @param entity_id The entity identifier
   * @param capabilities Vector of capability types to include
   * @return JSON array of capability objects with name and href
   */
  static nlohmann::json build_capabilities(const std::string & entity_type, const std::string & entity_id,
                                           const std::vector<Capability> & capabilities);

  /**
   * @brief Convert capability enum to string name.
   *
   * @param cap The capability enum value
   * @return String name for the capability (e.g., "data", "operations")
   */
  static std::string capability_to_name(Capability cap);

  /**
   * @brief Convert capability enum to URL path segment.
   *
   * @param cap The capability enum value
   * @return URL path segment for the capability (e.g., "data", "operations")
   */
  static std::string capability_to_path(Capability cap);
};

/**
 * @brief Fluent builder for HATEOAS _links objects.
 *
 * Provides a fluent API for constructing _links JSON objects.
 *
 * @example
 * LinksBuilder links;
 * auto json = links.self("/components/my-comp")
 *                  .parent("/areas/powertrain")
 *                  .collection("/components")
 *                  .add("custom", "/custom/link")
 *                  .build();
 *
 * @verifies REQ_API_002 HATEOAS links
 */
class LinksBuilder {
 public:
  /**
   * @brief Construct a new LinksBuilder.
   */
  LinksBuilder() = default;

  /**
   * @brief Set the self link.
   *
   * @param href The href for the self link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & self(const std::string & href);

  /**
   * @brief Set the parent link.
   *
   * @param href The href for the parent link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & parent(const std::string & href);

  /**
   * @brief Set the collection link.
   *
   * @param href The href for the collection link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & collection(const std::string & href);

  /**
   * @brief Add a custom link.
   *
   * @param rel The relation name
   * @param href The href for the link
   * @return Reference to this builder for chaining
   */
  LinksBuilder & add(const std::string & rel, const std::string & href);

  /**
   * @brief Build the final _links JSON object.
   *
   * @return JSON object containing all configured links
   */
  nlohmann::json build() const;

 private:
  nlohmann::json links_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

