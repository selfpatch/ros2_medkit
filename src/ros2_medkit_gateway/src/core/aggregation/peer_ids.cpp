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
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/peer_id_index.hpp"
#include "ros2_medkit_gateway/core/aggregation/peer_json.hpp"

namespace ros2_medkit_gateway {

namespace {

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

/// What the merge did to one ID of one peer.
struct Fate {
  enum Kind {
    kKept,     ///< Admitted under the peer's own ID.
    kRenamed,  ///< Admitted under `renamed->id`.
    kDropped,  ///< Left out; the ID names another entity here.
    kUnknown,  ///< Never declared by the peer; here it names whatever holds it.
  };
  Kind kind{kKept};
  const PeerIdIndex::Renamed * renamed{nullptr};  ///< Set for kRenamed
};

/// The fate of `peer_id` among `ids`, one type of one peer's IDs; nullptr
/// when no index was published, in which case every ID is taken as it is.
Fate fate_of(const PeerIdIndex::Ids * ids, const std::string & peer_id) {
  if (ids == nullptr) {
    return Fate{};
  }
  if (auto it = ids->renamed.find(peer_id); it != ids->renamed.end()) {
    return Fate{Fate::kRenamed, &it->second};
  }
  if (ids->dropped.count(peer_id) > 0u) {
    return Fate{Fate::kDropped, nullptr};
  }
  if (ids->admitted.count(peer_id) > 0u) {
    return Fate{};
  }
  return Fate{Fate::kUnknown, nullptr};
}

/// A member names the peer's App when the peer declared an App by that ID,
/// and its Component otherwise.
Fate member_fate_of(const PeerIdIndex::Peer * ids, const std::string & member_id) {
  if (ids == nullptr) {
    return Fate{};
  }
  auto as_app = fate_of(&ids->apps, member_id);
  return as_app.kind != Fate::kUnknown ? as_app : fate_of(&ids->components, member_id);
}

/// The ID index of `peer` in the snapshot `index` (an empty one for a peer
/// that declared nothing), or nullptr when there is no snapshot.
const PeerIdIndex::Peer * peer_ids_of(const std::shared_ptr<const PeerIdIndex> & index, const std::string & peer) {
  return index ? &index->peer(peer) : nullptr;
}

/// The IDs of `type` in `ids`, or nullptr when there is no index.
const PeerIdIndex::Ids * ids_of(const PeerIdIndex::Peer * ids, SovdEntityType type) {
  return ids != nullptr ? &ids->of(type) : nullptr;
}

/// The entity segment of `text` when it is an entity URI - `/api/v1/<collection>/<id>...`,
/// or the same behind `http(s)://host` - and std::nullopt for any other string.
std::optional<EntitySegment> entity_uri_segment(const std::string & text) {
  static constexpr std::string_view kApi = "/api/v1/";
  const auto api = text.find(kApi);
  if (api == std::string::npos) {
    return std::nullopt;
  }
  if (api != 0 && text.rfind("http://", 0) != 0 && text.rfind("https://", 0) != 0) {
    return std::nullopt;
  }
  // The collection has to follow the API prefix directly, as in every route.
  auto segment = locate_entity_segment(text);
  if (!segment || segment->collection_pos != api + kApi.size() - 1) {
    return std::nullopt;
  }
  return segment;
}

std::optional<std::string> served(const Fate & fate, const std::string & peer_id) {
  switch (fate.kind) {
    case Fate::kRenamed:
      return fate.renamed->id;
    case Fate::kDropped:
    case Fate::kUnknown:
      return std::nullopt;
    case Fate::kKept:
      break;
  }
  return peer_id;
}

/// The walk rewrite_to_served_ids makes over a body, against one snapshot of
/// one peer's IDs. It descends at most kMaxPeerJsonDepth levels; a deeper body
/// stops the walk and is reported as too deep, never as rewritten in part.
class ServedIdWalk {
 public:
  /// `root_type`, when given, is the type of the entity the body describes
  /// when it names none itself: a detail route's.
  ServedIdWalk(const PeerIdIndex::Peer * ids, std::optional<SovdEntityType> root_type)
    : ids_(ids), root_type_(root_type) {
  }

  ServedIdRewrite run(nlohmann::json & body) {
    // The body itself cannot be removed; a body that is one dropped reference
    // is not a discovery response this gateway asks for.
    (void)node(body, 0);
    if (too_deep_) {
      return ServedIdRewrite::kTooDeep;
    }
    if (too_large_) {
      return ServedIdRewrite::kTooLarge;
    }
    return changed_ ? ServedIdRewrite::kRewritten : ServedIdRewrite::kUnchanged;
  }

 private:
  /// True once the walk is over: the body nests too deep, or its rewrite grew
  /// past the budget.
  bool halted() const {
    return too_deep_ || too_large_;
  }

  /// True, and the walk over, when `depth` is past the bound.
  bool past_bound(size_t depth) {
    if (depth >= kMaxPeerJsonDepth) {
      too_deep_ = true;
    }
    return halted();
  }

  /// The length of `text` as a JSON string in the served body: quoted, with
  /// the escapes nlohmann's dump writes (two bytes for a quote, a backslash
  /// and \b \f \n \r \t, six for any other control character).
  static size_t json_text_size(const std::string & text) {
    size_t size = 2;
    for (const char c : text) {
      const auto byte = static_cast<unsigned char>(c);
      if (c == '"' || c == '\\' || c == '\b' || c == '\f' || c == '\n' || c == '\r' || c == '\t') {
        size += 2;
      } else if (byte < 0x20) {
        size += 6;
      } else {
        size += 1;
      }
    }
    return size;
  }

  /// Charge replacing the string `current` by `replacement` against the growth
  /// budget, before the replacement is copied in. False, and the walk over,
  /// when it would take the body past kMaxServedIdRewriteGrowth.
  bool charge(const std::string & current, const std::string & replacement) {
    if (too_large_) {
      return false;
    }
    const size_t before = json_text_size(current);
    const size_t after = json_text_size(replacement);
    if (after <= before) {
      return true;
    }
    if (after - before > growth_left_) {
      too_large_ = true;
      return false;
    }
    growth_left_ -= after - before;
    return true;
  }

  /// Apply `fate` to the ID `value` holds. True when the value is to be removed.
  bool apply(nlohmann::json & value, const Fate & fate) {
    switch (fate.kind) {
      case Fate::kRenamed:
        if (!charge(value.get_ref<const std::string &>(), fate.renamed->id)) {
          return false;
        }
        value = fate.renamed->id;
        changed_ = true;
        return false;
      case Fate::kDropped:
      case Fate::kUnknown:
        changed_ = true;
        return true;
      case Fate::kKept:
        break;
    }
    return false;
  }

  bool id(nlohmann::json & value, SovdEntityType type) {
    return value.is_string() && apply(value, fate_of(ids_of(ids_, type), value.get<std::string>()));
  }

  bool member(nlohmann::json & value) {
    return value.is_string() && apply(value, member_fate_of(ids_, value.get<std::string>()));
  }

  /// An entity URI - `/api/v1/<collection>/<id>...`, or the same behind
  /// `http(s)://host` - named by the served ID; any other string is left
  /// alone. True when the URI names a dropped entity.
  bool uri(nlohmann::json & value) {
    const auto & text = value.get_ref<const std::string &>();
    const auto segment = entity_uri_segment(text);
    if (!segment) {
      return false;
    }
    const auto fate = fate_of(ids_of(ids_, segment->type), text.substr(segment->id_pos, segment->id_len));
    switch (fate.kind) {
      case Fate::kRenamed: {
        // The ID is the only part of the URI that changes, and it is an entity
        // ID on both sides, which JSON never escapes.
        const size_t grown = fate.renamed->id.size() > segment->id_len ? fate.renamed->id.size() - segment->id_len : 0;
        if (too_large_ || grown > growth_left_) {
          too_large_ = true;
          return false;
        }
        growth_left_ -= grown;
        value = with_entity_id(text, *segment, fate.renamed->id);
        changed_ = true;
        return false;
      }
      case Fate::kDropped:
      case Fate::kUnknown:
        changed_ = true;
        return true;
      case Fate::kKept:
        break;
    }
    return false;
  }

  /// Remove the entries of the array `value` that `drop` says to, in one pass:
  /// each kept entry moves down once. Erasing each dropped entry where it
  /// stands shifts everything after it, so an array the peer filled with
  /// dropped references cost the square of its length. `drop` may rewrite the
  /// entry it is given, and is called once per entry, in order.
  template <class Drop>
  void erase_from(nlohmann::json & value, Drop drop) {
    if (!value.is_array()) {
      return;
    }
    auto & entries = value.get_ref<nlohmann::json::array_t &>();
    size_t kept = 0;
    for (size_t i = 0; i < entries.size(); ++i) {
      if (drop(entries[i])) {
        continue;
      }
      if (kept != i) {
        entries[kept] = std::move(entries[i]);
      }
      ++kept;
    }
    entries.resize(kept);
  }

  /// The relationship IDs an `x-medkit` block carries, then anything else in it.
  void x_medkit(nlohmann::json & xm, size_t depth) {
    if (past_bound(depth)) {
      return;
    }
    std::vector<std::string> removed;
    for (auto it = xm.begin(); it != xm.end() && !halted(); ++it) {
      const auto & key = it.key();
      bool remove = false;
      if (key == "parent_area_id" || key == "area") {
        remove = id(it.value(), SovdEntityType::AREA);
      } else if (key == "parentComponentId" || key == "component_id") {
        remove = id(it.value(), SovdEntityType::COMPONENT);
      } else if (key == "dependsOn") {
        erase_from(it.value(), [this](nlohmann::json & entry) {
          return id(entry, SovdEntityType::COMPONENT);
        });
      } else if (key == "hosts") {
        erase_from(it.value(), [this](nlohmann::json & entry) {
          return member(entry);
        });
      } else {
        remove = node(it.value(), depth + 1);
      }
      if (remove) {
        removed.push_back(key);
      }
    }
    for (const auto & key : removed) {
      xm.erase(key);
    }
  }

  /// Rewrite `value`. True when it names a dropped entity and is to be removed
  /// from the array or object holding it.
  bool node(nlohmann::json & value, size_t depth) {
    if (value.is_string()) {
      return uri(value);
    }
    if (!value.is_array() && !value.is_object()) {
      return false;
    }
    if (past_bound(depth)) {
      return false;
    }
    if (value.is_array()) {
      erase_from(value, [this, depth](nlohmann::json & item) {
        return !halted() && node(item, depth + 1);
      });
      return false;
    }
    // The entity an object describes names its type: a detail, or an item of a
    // relationship collection. A renamed one takes its served ID and the name
    // the listings serve it under; a dropped one is removed whole. A detail
    // that names no type is of the type its route asked for.
    //
    // Read through contains() and operator[], not find(): with the type
    // branches below, GCC 13 inlines the find() iterator's dereference into
    // -Wnull-dereference false positives (see document_checks.cpp). value is an
    // object, so operator[] only reads keys contains() has found.
    auto entity_type = SovdEntityType::UNKNOWN;
    if (value.contains("type")) {
      if (const nlohmann::json & type = value["type"]; type.is_string()) {
        entity_type = entity_type_named(type.get<std::string>());
      }
    } else if (depth == 0 && root_type_) {
      entity_type = *root_type_;
    }
    if (entity_type != SovdEntityType::UNKNOWN && value.contains("id") && value["id"].is_string()) {
      nlohmann::json & entity_id = value["id"];
      const auto fate = fate_of(ids_of(ids_, entity_type), entity_id.get<std::string>());
      if (fate.kind == Fate::kDropped || fate.kind == Fate::kUnknown) {
        changed_ = true;
        return true;
      }
      if (fate.kind == Fate::kRenamed) {
        if (!charge(entity_id.get_ref<const std::string &>(), fate.renamed->id)) {
          return false;
        }
        entity_id = fate.renamed->id;
        if (value.contains("name") && value["name"].is_string()) {
          if (!charge(value["name"].get_ref<const std::string &>(), fate.renamed->name)) {
            return false;
          }
          value["name"] = fate.renamed->name;
        }
        changed_ = true;
      }
    }
    std::vector<std::string> removed;
    for (auto it = value.begin(); it != value.end() && !halted(); ++it) {
      if (it.key() == "id" || it.key() == "name") {
        continue;
      }
      bool remove = false;
      if (it.key() == "x-medkit" && it.value().is_object()) {
        x_medkit(it.value(), depth + 1);
      } else {
        remove = node(it.value(), depth + 1);
      }
      if (remove) {
        removed.push_back(it.key());
      }
    }
    for (const auto & key : removed) {
      value.erase(key);
    }
    return false;
  }

  const PeerIdIndex::Peer * ids_;
  std::optional<SovdEntityType> root_type_;
  bool changed_{false};
  bool too_deep_{false};
  bool too_large_{false};
  size_t growth_left_{kMaxServedIdRewriteGrowth};
};

}  // namespace

std::optional<std::string> served_entity_id(const std::shared_ptr<const PeerIdIndex> & peer_ids, SovdEntityType type,
                                            const std::string & peer, const std::string & peer_id) {
  if (peer.empty() || peer_id.empty()) {
    return peer_id;
  }
  return served(fate_of(ids_of(peer_ids_of(peer_ids, peer), type), peer_id), peer_id);
}

std::optional<std::string> served_member_id(const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                            const std::string & peer, const std::string & member_id) {
  if (peer.empty() || member_id.empty()) {
    return member_id;
  }
  return served(member_fate_of(peer_ids_of(peer_ids, peer), member_id), member_id);
}

std::optional<std::string> served_entity_uri(const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                             const std::string & peer, const std::string & uri) {
  const auto segment = entity_uri_segment(uri);
  if (peer.empty() || !segment) {
    return uri;
  }
  const auto fate =
      fate_of(ids_of(peer_ids_of(peer_ids, peer), segment->type), uri.substr(segment->id_pos, segment->id_len));
  switch (fate.kind) {
    case Fate::kRenamed:
      return with_entity_id(uri, *segment, fate.renamed->id);
    case Fate::kDropped:
    case Fate::kUnknown:
      return std::nullopt;
    case Fate::kKept:
      break;
  }
  return uri;
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
  std::string_view relation = std::string_view(path).substr(segment->id_pos + segment->id_len + 1);
  relation = relation.substr(0, relation.find_first_of("?#"));
  return std::find(kRelations.begin(), kRelations.end(), relation) != kRelations.end();
}

ServedIdRewrite rewrite_to_served_ids(nlohmann::json & body, const std::string & peer,
                                      const std::shared_ptr<const PeerIdIndex> & peer_ids,
                                      std::optional<SovdEntityType> root_type) {
  if (peer.empty()) {
    return ServedIdRewrite::kUnchanged;
  }
  // One snapshot for the whole body: every rename and drop it applies is of
  // the same generation.
  return ServedIdWalk(peer_ids_of(peer_ids, peer), root_type).run(body);
}

}  // namespace ros2_medkit_gateway
