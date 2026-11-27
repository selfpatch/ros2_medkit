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

#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

namespace ros2_medkit_gateway {

/// An entity a plugin introspected under an id an earlier entity of another
/// type in the same introspection already holds.
struct CrossTypeDuplicate {
  SovdEntityType kept_type{SovdEntityType::UNKNOWN};     ///< Type of the entity that keeps the id
  SovdEntityType dropped_type{SovdEntityType::UNKNOWN};  ///< Type of the entity left out
  std::string id;
};

/**
 * @brief Leave out each entity of a plugin's introspection whose id an earlier
 * entity of another type in it holds.
 *
 * A plugin's providers are called with an entity id alone, so the id has to
 * name one of the plugin's entities. The first entity in the order Areas,
 * Components, Apps, Functions keeps the id; a later one of another type is
 * removed from `entities`. A plugin entity sharing an id with an entity the
 * plugin did not introspect is not affected: their types keep them apart.
 *
 * @return The entities removed, in that order.
 */
std::vector<CrossTypeDuplicate> drop_cross_type_duplicates(NewEntities & entities);

/// The log line for one dropped duplicate of plugin `plugin_name`.
std::string describe_cross_type_duplicate(const std::string & plugin_name, const CrossTypeDuplicate & duplicate);

}  // namespace ros2_medkit_gateway
