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

#include "ros2_medkit_gateway/models/entity_types.hpp"

#include <set>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief SOVD Entity Capabilities based on Table 8 and Table 10
 *
 * This class encapsulates which resource collections and resources
 * are supported by each entity type according to SOVD specification.
 *
 * Resource Collections:
 * - configurations: SERVER, COMPONENT, APP
 * - data: SERVER, COMPONENT, APP, FUNCTION*
 * - faults: SERVER, COMPONENT, APP
 * - operations: SERVER, COMPONENT, APP, FUNCTION*
 * - (others): SERVER, COMPONENT, APP
 *
 * Resources:
 * - docs: all
 * - version-info: SERVER only
 * - logs: SERVER, COMPONENT, APP
 * - hosts: COMPONENT, FUNCTION
 * - is-located-on: APP only
 * - contains: AREA only
 * - belongs-to: SERVER, COMPONENT, APP
 * - depends-on: SERVER, COMPONENT, APP, FUNCTION
 * - data-categories: SERVER, COMPONENT, APP
 * - data-groups: SERVER, COMPONENT, APP
 *
 * Note: FUNCTION data/operations are aggregated from hosted Apps (read-only).
 */
class EntityCapabilities {
 public:
  /**
   * @brief Get capabilities for an entity type
   * @param type SOVD entity type
   * @return EntityCapabilities instance with supported collections/resources
   */
  static EntityCapabilities for_type(SovdEntityType type);

  /**
   * @brief Check if entity supports a resource collection
   * @param col Resource collection to check
   * @return true if supported
   */
  bool supports_collection(ResourceCollection col) const;

  /**
   * @brief Check if entity supports a named resource
   * @param name Resource name (e.g., "docs", "hosts", "is-located-on")
   * @return true if supported
   */
  bool supports_resource(const std::string & name) const;

  /**
   * @brief Get all supported resource collections
   * @return Vector of supported collections
   */
  std::vector<ResourceCollection> collections() const;

  /**
   * @brief Get all supported resource names
   * @return Vector of supported resource names
   */
  std::vector<std::string> resources() const;

  /**
   * @brief Check if collection access is aggregated (from sub-entities)
   *
   * For FUNCTION type, data and operations are aggregated from hosted Apps.
   * For AREA type with x-medkit extension, operations can be aggregated.
   *
   * @param col Resource collection to check
   * @return true if access is aggregated
   */
  bool is_aggregated(ResourceCollection col) const;

 private:
  EntityCapabilities() = default;

  std::set<ResourceCollection> collections_;
  std::set<std::string> resources_;
  std::set<ResourceCollection> aggregated_collections_;
};

}  // namespace ros2_medkit_gateway
