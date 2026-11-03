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

#include "ros2_medkit_gateway/core/aggregation/peer_ids.hpp"

#include <algorithm>
#include <array>
#include <string_view>
#include <utility>

#include "ros2_medkit_gateway/core/aggregation/entity_merger.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {

namespace {

/// True when `entity` is the one the merge renamed from `peer_id` for `peer`.
template <class Entity>
bool renamed_from(const Entity & entity, const std::string & peer, const std::string & peer_id) {
  return entity.original_id == peer_id && entity.source == "peer:" + peer;
}

SovdEntityType entity_type_named(const std::string & name) {
  if (name == "area") {
    return SovdEntityType::AREA;
  }
  if (name == "component") {
    return SovdEntityType::COMPONENT;
  }
  if (name == "app") {
    return SovdEntityType::APP;
  }
  if (name == "function") {
    return SovdEntityType::FUNCTION;
  }
  return SovdEntityType::UNKNOWN;
}

/// Rewrite the string `value` holds as the served ID of an entity of `type`.
bool rewrite_id(nlohmann::json & value, SovdEntityType type, const std::string & peer,
                const ThreadSafeEntityCache & cache) {
  if (!value.is_string()) {
    return false;
  }
  const auto peer_id = value.get<std::string>();
  auto served = served_entity_id(cache, type, peer, peer_id);
  if (served == peer_id) {
    return false;
  }
  value = std::move(served);
  return true;
}

/// rewrite_id over every string of the array `value` holds.
bool rewrite_ids(nlohmann::json & value, SovdEntityType type, const std::string & peer,
                 const ThreadSafeEntityCache & cache) {
  bool changed = false;
  if (value.is_array()) {
    for (auto & item : value) {
      changed = rewrite_id(item, type, peer, cache) || changed;
    }
  }
  return changed;
}

/// Rewrite an entity URI - `/api/v1/<collection>/<id>...`, or the same behind
/// `http(s)://host` - to name the entity by its served ID. Any other string is
/// left alone.
bool rewrite_uri(nlohmann::json & value, const std::string & peer, const ThreadSafeEntityCache & cache) {
  static constexpr std::string_view kApi = "/api/v1/";
  const auto & uri = value.get_ref<const std::string &>();
  const auto api = uri.find(kApi);
  if (api == std::string::npos) {
    return false;
  }
  if (api != 0 && uri.rfind("http://", 0) != 0 && uri.rfind("https://", 0) != 0) {
    return false;
  }
  // The collection has to follow the API prefix directly, as in every route.
  auto segment = locate_entity_segment(uri);
  if (!segment || segment->collection_pos != api + kApi.size() - 1) {
    return false;
  }
  const auto peer_id = uri.substr(segment->id_pos, segment->id_len);
  const auto served = served_entity_id(cache, segment->type, peer, peer_id);
  if (served == peer_id) {
    return false;
  }
  value = with_entity_id(uri, *segment, served);
  return true;
}

bool rewrite_node(nlohmann::json & node, const std::string & peer, const ThreadSafeEntityCache & cache);

/// The relationship IDs an `x-medkit` block carries, then anything else in it.
bool rewrite_x_medkit(nlohmann::json & xm, const std::string & peer, const ThreadSafeEntityCache & cache) {
  bool changed = false;
  for (auto it = xm.begin(); it != xm.end(); ++it) {
    const auto & key = it.key();
    if (key == "parent_area_id" || key == "area") {
      changed = rewrite_id(it.value(), SovdEntityType::AREA, peer, cache) || changed;
    } else if (key == "parentComponentId" || key == "component_id") {
      changed = rewrite_id(it.value(), SovdEntityType::COMPONENT, peer, cache) || changed;
    } else if (key == "dependsOn") {
      changed = rewrite_ids(it.value(), SovdEntityType::COMPONENT, peer, cache) || changed;
    } else if (key == "hosts") {
      if (it.value().is_array()) {
        for (auto & host : it.value()) {
          if (host.is_string()) {
            const auto peer_id = host.get<std::string>();
            auto served = served_member_id(cache, peer, peer_id);
            if (served != peer_id) {
              host = std::move(served);
              changed = true;
            }
          }
        }
      }
    } else {
      changed = rewrite_node(it.value(), peer, cache) || changed;
    }
  }
  return changed;
}

bool rewrite_node(nlohmann::json & node, const std::string & peer, const ThreadSafeEntityCache & cache) {
  if (node.is_string()) {
    return rewrite_uri(node, peer, cache);
  }
  bool changed = false;
  if (node.is_array()) {
    for (auto & item : node) {
      changed = rewrite_node(item, peer, cache) || changed;
    }
    return changed;
  }
  if (!node.is_object()) {
    return false;
  }
  // The entity an object describes names its type: a detail, or an item of a
  // relationship collection.
  const auto type = node.find("type");
  const auto id = node.find("id");
  if (type != node.end() && type->is_string() && id != node.end()) {
    const auto entity_type = entity_type_named(type->get<std::string>());
    if (entity_type != SovdEntityType::UNKNOWN) {
      changed = rewrite_id(*id, entity_type, peer, cache) || changed;
    }
  }
  for (auto it = node.begin(); it != node.end(); ++it) {
    if (it.key() == "id") {
      continue;
    }
    if (it.key() == "x-medkit" && it.value().is_object()) {
      changed = rewrite_x_medkit(it.value(), peer, cache) || changed;
    } else {
      changed = rewrite_node(it.value(), peer, cache) || changed;
    }
  }
  return changed;
}

}  // namespace

std::string served_entity_id(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & peer,
                             const std::string & peer_id) {
  if (peer.empty() || peer_id.empty()) {
    return peer_id;
  }
  // The merge generates exactly this ID for a renamed entity, or drops the
  // entity when the ID is taken, so there is one candidate to check.
  std::string candidate = peer + EntityMerger::SEPARATOR + peer_id;
  bool renamed = false;
  switch (type) {
    case SovdEntityType::AREA:
      if (auto area = cache.get_area(candidate)) {
        renamed = renamed_from(*area, peer, peer_id);
      }
      break;
    case SovdEntityType::COMPONENT:
      if (auto comp = cache.get_component(candidate)) {
        renamed = renamed_from(*comp, peer, peer_id);
      }
      break;
    case SovdEntityType::APP:
      if (auto app = cache.get_app(candidate)) {
        renamed = renamed_from(*app, peer, peer_id);
      }
      break;
    case SovdEntityType::FUNCTION:
      if (auto func = cache.get_function(candidate)) {
        renamed = renamed_from(*func, peer, peer_id);
      }
      break;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return renamed ? candidate : peer_id;
}

std::string served_member_id(const ThreadSafeEntityCache & cache, const std::string & peer,
                             const std::string & member_id) {
  auto as_app = served_entity_id(cache, SovdEntityType::APP, peer, member_id);
  if (as_app != member_id) {
    return as_app;
  }
  return served_entity_id(cache, SovdEntityType::COMPONENT, peer, member_id);
}

bool is_discovery_route(const std::string & path) {
  auto segment = locate_entity_segment(path);
  if (!segment) {
    return false;
  }
  if (!segment->has_subpath) {
    return true;
  }
  static const std::array<std::string_view, 8> kRelations = {"subareas", "components", "contains",      "subcomponents",
                                                             "hosts",    "depends-on", "is-located-on", "belongs-to"};
  const std::string_view relation = std::string_view(path).substr(segment->id_pos + segment->id_len + 1);
  return std::find(kRelations.begin(), kRelations.end(), relation) != kRelations.end();
}

bool rewrite_to_served_ids(nlohmann::json & body, const std::string & peer, const ThreadSafeEntityCache & cache) {
  if (peer.empty()) {
    return false;
  }
  return rewrite_node(body, peer, cache);
}

}  // namespace ros2_medkit_gateway
