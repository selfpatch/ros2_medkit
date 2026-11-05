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
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

/// The IDs of one peer's entities the merge left out, one set per type, by the
/// peer's own IDs. Such an entity is not served here, and its ID is held here
/// by another entity.
struct PeerDrops {
  std::unordered_set<std::string> areas;
  std::unordered_set<std::string> components;
  std::unordered_set<std::string> apps;
  std::unordered_set<std::string> functions;
};

/// What the merge did to each peer's IDs: the entities it admitted, the ones
/// among them it renamed (served under `<peer>__<id>`), and the ones it
/// dropped.
///
/// Everything that maps an ID a peer reports back to this gateway reads it: a
/// renamed entity is named by its served ID, a dropped one by nothing, because
/// the peer's ID names the local entity that took its place. So is an ID the
/// peer never declared: a peer answering about its App `sensor` with links to
/// `/apps/admin` names, through this gateway, a local `admin` it has nothing to
/// do with. Only an admitted ID is the peer's to name. Built from the
/// served entities and the merge's drops, and published in the entity cache by
/// the same update as the entities (ThreadSafeEntityCache::update_all), so a
/// reader that takes one snapshot of it reads one generation.
struct PeerIdIndex {
  /// A renamed entity, as it is served here.
  struct Renamed {
    std::string id;
    std::string name;
  };

  /// One peer's IDs of one entity type.
  struct Ids {
    std::unordered_map<std::string, Renamed> renamed;  ///< peer ID -> how it is served
    std::unordered_set<std::string> dropped;           ///< peer IDs not served
    std::unordered_set<std::string> admitted;          ///< peer IDs served, renamed or not
  };

  /// One peer's IDs, per type.
  struct Peer {
    Ids areas;
    Ids components;
    Ids apps;
    Ids functions;

    /// The IDs of `type`; an empty set for SERVER and UNKNOWN.
    const Ids & of(SovdEntityType type) const;
  };

  std::unordered_map<std::string, Peer> peers;

  /// The IDs of `peer`; empty ones for a peer nothing was renamed or dropped for.
  const Peer & peer(const std::string & name) const;
};

/// The index of the served entities - every one a peer contributes, by the ID
/// that peer knows it by, and those whose `original_id` is set as renamed, by
/// the peer their `source` names - and of `drops`, peer name -> that peer's
/// drops.
PeerIdIndex make_peer_id_index(const std::vector<Area> & areas, const std::vector<Component> & components,
                               const std::vector<App> & apps, const std::vector<Function> & functions,
                               const std::unordered_map<std::string, PeerDrops> & drops);

}  // namespace ros2_medkit_gateway
