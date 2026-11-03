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
#include <string_view>
#include <vector>

namespace ros2_medkit_gateway {

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

}  // namespace ros2_medkit_gateway
