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

#include "ros2_medkit_gateway/core/plugins/plugin_entities.hpp"

#include <algorithm>
#include <unordered_map>

namespace ros2_medkit_gateway {

namespace {

/// Remove from `list` every entity whose id `held` gives to another type, and
/// claim the ids of the rest for `type`.
template <class Entity>
void drop_held(std::vector<Entity> & list, SovdEntityType type, std::unordered_map<std::string, SovdEntityType> & held,
               std::vector<CrossTypeDuplicate> & dropped) {
  auto kept_end = std::remove_if(list.begin(), list.end(), [&](const Entity & entity) {
    auto [it, inserted] = held.emplace(entity.id, type);
    if (inserted || it->second == type) {
      return false;
    }
    dropped.push_back(CrossTypeDuplicate{it->second, type, entity.id});
    return true;
  });
  list.erase(kept_end, list.end());
}

}  // namespace

std::vector<CrossTypeDuplicate> drop_cross_type_duplicates(NewEntities & entities) {
  std::unordered_map<std::string, SovdEntityType> held;
  std::vector<CrossTypeDuplicate> dropped;
  drop_held(entities.areas, SovdEntityType::AREA, held, dropped);
  drop_held(entities.components, SovdEntityType::COMPONENT, held, dropped);
  drop_held(entities.apps, SovdEntityType::APP, held, dropped);
  drop_held(entities.functions, SovdEntityType::FUNCTION, held, dropped);
  return dropped;
}

std::string describe_cross_type_duplicate(const std::string & plugin_name, const CrossTypeDuplicate & duplicate) {
  const std::string kept = to_string(duplicate.kept_type);
  const std::string dropped = to_string(duplicate.dropped_type);
  return "Plugin '" + plugin_name + "' introspects both " + kept + " '" + duplicate.id + "' and " + dropped + " '" +
         duplicate.id + "'; the " + dropped + " is not served, since a plugin's providers are called with the id " +
         "alone and it has to name one entity";
}

}  // namespace ros2_medkit_gateway
