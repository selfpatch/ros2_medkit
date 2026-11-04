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

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

struct PeerIdIndex;

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
///
/// Only the path part is read: a `?` query or `#` fragment ends it, so in
/// `/api/v1/apps/x?view=full` the id is `x`, and with_entity_id keeps the query
/// and fragment as they are.
inline std::optional<EntitySegment> locate_entity_segment(const std::string & path) {
  static constexpr std::array<std::pair<std::string_view, SovdEntityType>, 4> kEntityCollections = {{
      {"/components/", SovdEntityType::COMPONENT},
      {"/apps/", SovdEntityType::APP},
      {"/areas/", SovdEntityType::AREA},
      {"/functions/", SovdEntityType::FUNCTION},
  }};
  const size_t path_end = std::min(path.find('?'), path.find('#'));
  size_t best = std::string::npos;
  size_t best_prefix_len = 0;
  SovdEntityType best_type = SovdEntityType::UNKNOWN;
  for (const auto & [prefix, type] : kEntityCollections) {
    auto start = path.find(prefix);
    if (start < best && (path_end == std::string::npos || start + prefix.size() <= path_end)) {
      best = start;
      best_prefix_len = prefix.size();
      best_type = type;
    }
  }
  if (best == std::string::npos) {
    return std::nullopt;
  }
  const size_t id_pos = best + best_prefix_len;
  const size_t slash = path.find('/', id_pos);
  const size_t end = std::min(slash, path_end);
  const size_t id_len = (end == std::string::npos ? path.size() : end) - id_pos;
  if (id_len == 0) {
    return std::nullopt;
  }
  const bool has_subpath = slash != std::string::npos && (path_end == std::string::npos || slash < path_end);
  return EntitySegment{best_type, best, id_pos, id_len, has_subpath};
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
/// The reverse of peer_entity_id, for an ID a peer reports, read from
/// `peer_ids`, the peer ID index the cache publishes with the entities
/// (PeerIdIndex), or nullptr for none. Pass the index of the generation the
/// request was validated against (EntityInfo::peer_ids): what the peer answers
/// describes the entity that request named, and a refresh that renames or
/// drops it while the answer is on its way makes the newest index describe
/// another entity. An entity the merge renamed is served as
/// `<peer>__<peer_id>`, and one it dropped is not served at all - in both
/// cases the unprefixed ID names a local entity here.
/// Only recorded renames and drops count, so a peer's own entity whose ID
/// merely reads like a prefix is left alone. Returns `peer_id` for an entity
/// the merge admitted under it, which is the ordinary case, and std::nullopt
/// for a dropped entity or an ID the peer never declared: nothing here may be
/// named for either, since the ID names whatever holds it here. Without an
/// index - none published yet - every ID is returned as it is.
std::optional<std::string> served_entity_id(const std::shared_ptr<const PeerIdIndex> & peer_ids, SovdEntityType type,
                                            const std::string & peer, const std::string & peer_id);

/// served_entity_id for a member a peer names without saying its type: its App
/// when the peer declared an App by that ID, else its Component.
std::optional<std::string> served_member_id(const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                            const std::string & peer, const std::string & member_id);

/// The id of an item `peer` listed in a fan-out collection - a configuration,
/// a data item, an operation - with the member it is qualified by named by the
/// ID this gateway serves that member under (served_member_id).
///
/// A member-qualified id is `<member>:<item>`, split at the first colon. This
/// gateway splits an id it is asked for the same way and reads the member half
/// against its own members, so the peer's `x:gain`, for a member `x` the merge
/// renamed, addresses the local `x` here and a write to it changes the local
/// parameter. The half is read as a member only when it can be an entity ID,
/// which a colon in an item's own name (`ns.a:b`) does not make it. Returns the
/// id unchanged when it carries no such half or the member keeps its ID,
/// `<served>:<item>` for a renamed member, and std::nullopt for a member the
/// merge dropped or the peer never declared: no id here addresses that item.
std::optional<std::string> served_member_qualified_id(const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                                      const std::string & peer, const std::string & item_id);

/// `uri`, an entity URI `peer` reported - `/api/v1/<collection>/<id>...`, or
/// the same behind `http(s)://host` - with the entity it names named by the
/// ID this gateway serves it under (served_entity_id), read from `peer_ids`,
/// the index of the generation the request was validated against, and always
/// as a path on this gateway: an absolute URI loses the peer's scheme and
/// authority, whether its ID is renamed or kept, so it resolves through this
/// gateway - with the peer's authority, a served ID reached a peer that does
/// not know it. Returns `uri` as it is when it names no entity, or is a path
/// naming one the merge left alone, and std::nullopt when it names one this
/// gateway does not serve: its ID there names another entity. For a header such
/// as `Location`, which a peer builds from the route it received - a route in
/// its own IDs.
std::optional<std::string> served_entity_uri(const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                             const std::string & peer, const std::string & uri);

/// True for a discovery route: an entity's detail (`/<collection>/<id>`) or one
/// of the relationship collections discovery serves under it (`subareas`,
/// `components`, `contains`, `subcomponents`, `hosts`, `depends-on`,
/// `is-located-on`, `belongs-to`).
bool is_discovery_route(const std::string & path);

/// What a relationship collection lists, by the entity its route names and the
/// relationship: an App's `depends-on` lists Apps, its `is-located-on`
/// Components and its `belongs-to` Areas; a Component's `subcomponents` and
/// `depends-on` list Components and its `hosts` Apps; an Area's `subareas`
/// lists Areas and its `components` and `contains` Components; a Function's
/// `hosts` lists Apps or Components.
struct RelationshipItems {
  /// The type of each item, UNKNOWN for a relationship this gateway knows no
  /// item type of.
  SovdEntityType type{SovdEntityType::UNKNOWN};
  /// An App or a Component, as a Function's hosts: the App when the peer
  /// declared one by the item's ID, else its Component (served_member_id).
  bool member{false};
};

/// The items of the relationship collection `path` names - a discovery route
/// with a relationship after the entity - or std::nullopt for any other path.
std::optional<RelationshipItems> relationship_items(const std::string & path);

/// Rewrite a discovery response `peer` answered so that every entity it names
/// is named by the ID this gateway serves it under (served_entity_id).
///
/// A peer describes its own tree in its own IDs, and an entity this gateway
/// renamed or dropped on collision is something else here under the peer's
/// ID: followed through this gateway, the peer's ID addresses the local entity
/// that took its place. Rewritten are the `id` of every object that names its
/// `type` (the detail itself, the items of a relationship collection), and of
/// the body itself when it names none and `root_type` is given - the type of
/// the entity a detail route asked for - together with its `name`, which
/// takes the served one as the listings show it; every
/// entity URI - the self link, the resource collection links, the relationship
/// links, relative or absolute, an absolute one written as a path on this
/// gateway (served_entity_uri); and the relationship IDs `x-medkit` carries
/// (`parent_area_id`, `parentComponentId`, `dependsOn`, `area`,
/// `component_id`, `hosts`). On a relationship collection (`items`, given),
/// an item that names no type the walk recognizes - none, one that is not a
/// string, one no entity has - is of the type the relationship lists, and its
/// `id` and `name` are rewritten as that type's; one whose type cannot be
/// established that way is removed whole. Left as it was, such an item kept
/// the peer's ID while its links were rewritten or removed, and named, through
/// this gateway, the local entity that holds that ID. A reference to a dropped
/// entity, or to one the peer never declared, is removed: the item of a collection that describes
/// it, the link or relationship field that names it, the entry of a list of
/// IDs. IDs the merge admitted unrenamed are left as they are. Every lookup reads `peer_ids`, the index of
/// the generation the request was validated against (see served_entity_id).
///
/// The walk descends at most kMaxPeerJsonDepth levels (see peer_json.hpp): the
/// body comes from a peer, and a walk that followed it down without a bound
/// would let a deeply nested answer overflow the stack. A deeper body returns
/// kTooDeep with the walk abandoned and `body` partly rewritten; the caller
/// must not serve it as a translated response.
///
/// What the walk writes is bounded too. Each replacement copies a served ID or
/// a served name into the body once per object that names the entity, so a
/// small body naming one renamed entity many times grows by that many copies:
/// a name of 8 MB repeated in 10,000 objects of a valid detail comes to about
/// 78 GB. Every replacement is charged, before its value is built, by what it
/// adds to the body's JSON text, against kMaxServedIdRewriteGrowth; one that
/// would take the total past it stops the walk and returns kTooLarge, with
/// `body` partly rewritten and not to be served.
enum class ServedIdRewrite {
  kUnchanged,  ///< Nothing in the body names a renamed or dropped entity.
  kRewritten,  ///< Renamed entities named by their served IDs, dropped ones removed.
  kTooDeep,    ///< The body nests past the bound; it is not a discovery response.
  kTooLarge,   ///< Rewritten, the body would grow past kMaxServedIdRewriteGrowth.
};

/// The most bytes rewrite_to_served_ids adds to a body's JSON text: as much
/// again as the largest body a peer is read for (10 MB). A rename adds the
/// peer's name and a separator to an ID and to a name, so a discovery response
/// grows by a fraction of its own size; only a body that names one renamed
/// entity far more often than its own size would explain comes near it.
inline constexpr size_t kMaxServedIdRewriteGrowth = 10 * 1024 * 1024;
ServedIdRewrite rewrite_to_served_ids(nlohmann::json & body, const std::string & peer,
                                      const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                      std::optional<SovdEntityType> root_type = std::nullopt,
                                      std::optional<RelationshipItems> items = std::nullopt);

}  // namespace ros2_medkit_gateway
