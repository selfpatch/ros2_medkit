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

#include <array>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class ThreadSafeEntityCache;

/// Where the entity a path names sits in it: the first
/// `/components|apps|areas|functions/<id>` segment.
struct EntitySegment {
  SovdEntityType type{SovdEntityType::UNKNOWN};
  size_t collection_pos{0};  ///< Offset of the `/` that starts `/<collection>/`
  size_t id_pos{0};          ///< Offset of `<id>` in the path
  size_t id_len{0};          ///< Length of `<id>`, never 0
  bool has_subpath{false};   ///< `/<id>/...`: a resource of the entity, not its detail
};

/// The first entity segment of `path`, std::nullopt when there is none or its
/// id is empty (`/components//logs`).
inline std::optional<EntitySegment> locate_entity_segment(const std::string & path) {
  static constexpr std::array<std::pair<std::string_view, SovdEntityType>, 4> kEntityCollections = {{
      {"/components/", SovdEntityType::COMPONENT},
      {"/apps/", SovdEntityType::APP},
      {"/areas/", SovdEntityType::AREA},
      {"/functions/", SovdEntityType::FUNCTION},
  }};
  size_t best = std::string::npos;
  size_t best_prefix_len = 0;
  SovdEntityType best_type = SovdEntityType::UNKNOWN;
  for (const auto & [prefix, type] : kEntityCollections) {
    auto start = path.find(prefix);
    if (start < best) {
      best = start;
      best_prefix_len = prefix.size();
      best_type = type;
    }
  }
  if (best == std::string::npos) {
    return std::nullopt;
  }
  const size_t id_pos = best + best_prefix_len;
  const size_t end = path.find('/', id_pos);
  const size_t id_len = (end == std::string::npos ? path.size() : end) - id_pos;
  if (id_len == 0) {
    return std::nullopt;
  }
  return EntitySegment{best_type, best, id_pos, id_len, end != std::string::npos};
}

/// `path` with the id of the entity segment `segment` replaced by `id`.
inline std::string with_entity_id(const std::string & path, const EntitySegment & segment, const std::string & id) {
  std::string out = path.substr(0, segment.id_pos);
  out += id;
  out += path.substr(segment.id_pos + segment.id_len);
  return out;
}

/// A peer that contributes an entity, and the ID that peer knows it by.
struct PeerTarget {
  std::string peer;  ///< Peer name, as in the entity's "peer:<name>" contributor
  std::string id;    ///< The entity's ID on that peer
};

inline bool operator==(const PeerTarget & a, const PeerTarget & b) {
  return a.peer == b.peer && a.id == b.id;
}

/// The ID `peer` knows `entity` by.
///
/// An entity the merge renamed (`original_id` set) came from one peer, its
/// `source`, and that peer knows it by the ID it was renamed from. Every other
/// contributor knows it by the ID it is served under: entities merge by ID.
template <class Entity>
std::string peer_entity_id(const Entity & entity, const std::string & peer) {
  static constexpr std::string_view kPeerPrefix = "peer:";
  const bool renamed_for_peer = !entity.original_id.empty() &&
                                entity.source.size() == kPeerPrefix.size() + peer.size() &&
                                entity.source.compare(0, kPeerPrefix.size(), kPeerPrefix) == 0 &&
                                entity.source.compare(kPeerPrefix.size(), std::string::npos, peer) == 0;
  return renamed_for_peer ? entity.original_id : entity.id;
}

/// The peers among `entity`'s contributors, in contributor order, each with the
/// ID it knows the entity by.
///
/// Read off the entity itself, so a caller holding an entity it looked up gets
/// the peers of that entity, from the same cache generation - not those of
/// another entity sharing its ID, nor those of a newer merge than the entity.
template <class Entity>
std::vector<PeerTarget> peer_targets_of(const Entity & entity) {
  static constexpr std::string_view kPeerPrefix = "peer:";
  std::vector<PeerTarget> targets;
  for (const auto & contributor : entity.contributors) {
    if (contributor.size() <= kPeerPrefix.size() || contributor.compare(0, kPeerPrefix.size(), kPeerPrefix) != 0) {
      continue;
    }
    std::string peer = contributor.substr(kPeerPrefix.size());
    bool seen = false;
    for (const auto & target : targets) {
      seen = seen || target.peer == peer;
    }
    if (!seen) {
      std::string id = peer_entity_id(entity, peer);
      targets.push_back(PeerTarget{std::move(peer), std::move(id)});
    }
  }
  return targets;
}

/// The ID this gateway serves an entity of `type` under that `peer` calls `peer_id`.
///
/// The reverse of peer_entity_id, for an ID a peer reports: an entity the merge
/// renamed is served as `<peer>__<peer_id>`, and the unprefixed ID names a
/// local entity here. Only a recorded rename is reversed - the cached entity
/// under the prefixed ID must be that peer's, renamed from `peer_id` - so a
/// peer's own entity whose ID merely reads like a prefix is left alone, and so
/// is another peer's entity that happens to carry the prefixed ID. Returns
/// `peer_id` when nothing was renamed, which is the ordinary case.
std::string served_entity_id(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & peer,
                             const std::string & peer_id);

/// served_entity_id for a member a peer names without saying its type: its App
/// when one was renamed from that ID, else its Component.
std::string served_member_id(const ThreadSafeEntityCache & cache, const std::string & peer,
                             const std::string & member_id);

/// True for a discovery route: an entity's detail (`/<collection>/<id>`) or one
/// of the relationship collections discovery serves under it (`subareas`,
/// `components`, `contains`, `subcomponents`, `hosts`, `depends-on`,
/// `is-located-on`, `belongs-to`).
bool is_discovery_route(const std::string & path);

/// Rewrite a discovery response `peer` answered so that every entity it names
/// is named by the ID this gateway serves it under (served_entity_id).
///
/// A peer describes its own tree in its own IDs, and an entity this gateway
/// renamed on collision is something else here under the peer's ID: followed
/// through this gateway, the peer's ID addresses the local entity that took
/// its place. Rewritten are the `id` of every object that names its `type`
/// (the detail itself, the items of a relationship collection), every entity
/// URI - the self link, the resource collection links, the relationship
/// links, relative or absolute - and the relationship IDs `x-medkit` carries
/// (`parent_area_id`, `parentComponentId`, `dependsOn`, `area`,
/// `component_id`, `hosts`). IDs nothing was renamed from are left as they are.
///
/// @return true when anything was rewritten
bool rewrite_to_served_ids(nlohmann::json & body, const std::string & peer, const ThreadSafeEntityCache & cache);

}  // namespace ros2_medkit_gateway
