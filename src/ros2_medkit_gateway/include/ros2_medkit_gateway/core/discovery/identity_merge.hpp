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

#include "ros2_medkit_gateway/core/discovery/models/asset_identity.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Configuration for identity merging.
 *
 * `source_precedence` ranks identity authority from highest to lowest. Entries are
 * matched against a source's canonical identifier - the contributing entity's
 * `Component.source` field ("manifest", "plugin", "runtime", "node", "heuristic",
 * "config", or a protocol-class tag a provider sets such as "opcua"/"s7"), NOT the
 * free-form discovery-layer / plugin name. A source not in the list ranks lowest: it
 * can still fill empty fields but never overrides a known source.
 *
 * Identity authority is deliberately decoupled from the structural merge policy: a
 * manifest may be the authoritative *structure* source while a live protocol read is
 * the authoritative *identity* source.
 *
 * Default precedence (highest first): a live protocol device-info read (a `plugin`
 * source, or a protocol-specific source tag) beats the hand-authored `manifest`,
 * which beats whatever runtime discovery guessed. The protocol-specific tags lead the
 * list so that a provider which sets a concrete `Component.source` (e.g. "opcua") is
 * honoured; the generic "plugin" tag covers the common case where the plugin layer
 * stamps every plugin entity with source="plugin".
 */
struct IdentityMergeConfig {
  std::vector<std::string> source_precedence{"opcua",    "s7",     "ethernet_ip", "modbus", "ads",
                                             "profinet", "plugin", "manifest",    "config", "runtime",
                                             "node",     "topic",  "heuristic"};
};

/**
 * @brief Rank of a source: lower number = higher authority. Unknown sources rank
 * just below every listed source (all unknowns share the same lowest rank).
 */
inline size_t source_rank(const std::string & source, const IdentityMergeConfig & config) {
  for (size_t i = 0; i < config.source_precedence.size(); ++i) {
    if (config.source_precedence[i] == source) {
      return i;
    }
  }
  return config.source_precedence.size();
}

namespace detail {

/// All typed identity fields as (provenance-key, member-pointer) pairs.
inline const std::array<std::pair<const char *, std::string AssetIdentity::*>, 8> & identity_fields() {
  static const std::array<std::pair<const char *, std::string AssetIdentity::*>, 8> fields{{
      {"manufacturer", &AssetIdentity::manufacturer},
      {"model", &AssetIdentity::model},
      {"serial_number", &AssetIdentity::serial_number},
      {"hardware_revision", &AssetIdentity::hardware_revision},
      {"firmware_version", &AssetIdentity::firmware_version},
      {"software_version", &AssetIdentity::software_version},
      {"network_endpoint", &AssetIdentity::network_endpoint},
      {"role", &AssetIdentity::role},
  }};
  return fields;
}

/// Decide whether an incoming value should overwrite the current field value.
/// @param field_set     Whether the target field already holds a value.
/// @param current_owner Source that owns the current value ("" if unstamped/unknown).
inline bool incoming_wins(bool field_set, const std::string & current_owner, const std::string & incoming_source,
                          const IdentityMergeConfig & config) {
  if (!field_set) {
    return true;  // field unset -> fill it
  }
  // Unstamped existing value -> treat its owner as the lowest authority.
  const size_t current_rank =
      current_owner.empty() ? config.source_precedence.size() : source_rank(current_owner, config);
  // Strictly higher authority (lower rank) wins; ties keep the existing value.
  return source_rank(incoming_source, config) < current_rank;
}

}  // namespace detail

/**
 * @brief Stamp provenance for every populated typed field / extra of `identity`
 *        to `source`, unless that field already has a provenance entry.
 *
 * Used to seed provenance for the first (base) source before merging others.
 * A no-op when `source` is empty: an unknown owner is the implicit default, so
 * stamping it would only add `_provenance` entries with empty values.
 */
inline void stamp_identity_provenance(AssetIdentity & identity, const std::string & source) {
  if (source.empty()) {
    return;
  }
  for (const auto & [prov_key, member] : detail::identity_fields()) {
    if (!(identity.*member).empty() && identity.provenance.find(prov_key) == identity.provenance.end()) {
      identity.provenance[prov_key] = source;
    }
  }
  for (const auto & [key, value] : identity.extra) {
    const std::string prov_key = "extra." + key;
    if (!value.empty() && identity.provenance.find(prov_key) == identity.provenance.end()) {
      identity.provenance[prov_key] = source;
    }
  }
}

/**
 * @brief Merge `source` identity (tagged `source_name`) into `target` in place.
 *
 * For each field: if the target field is unset, take the incoming value; otherwise
 * the incoming value wins only if `source_name` has strictly higher authority than
 * the source currently owning the field (per `config.source_precedence`). Empty
 * incoming values never overwrite.
 *
 * Authority for the override decision is always ranked on `source_name`, but the
 * provenance recorded for a written field is the incoming field's OWN provenance
 * when `source` carries one (`source.provenance`), falling back to `source_name`
 * otherwise. This keeps a field's recorded origin identical whether it arrives via
 * this in-place merge (collision gap-fill) or via a wholesale copy of the source
 * (no-collision add): e.g. a peer relaying a field it read over "opcua" keeps
 * provenance "opcua" instead of being re-stamped "peer:<name>", while the peer's
 * low authority for overriding local values is preserved.
 *
 * `target` should have had ::stamp_identity_provenance called on it (directly or via
 * a previous merge) so existing fields carry provenance; otherwise existing fields
 * are treated as owned by an unknown (lowest authority) source.
 */
inline void merge_identity(AssetIdentity & target, const AssetIdentity & source, const std::string & source_name,
                           const IdentityMergeConfig & config) {
  // Provenance to record for an incoming field: the source's own provenance for
  // that key when present, else the authority tag we merged it under.
  const auto recorded_provenance = [&](const std::string & prov_key) -> const std::string & {
    auto it = source.provenance.find(prov_key);
    if (it != source.provenance.end() && !it->second.empty()) {
      return it->second;
    }
    return source_name;
  };

  for (const auto & [prov_key, member] : detail::identity_fields()) {
    const std::string & incoming = source.*member;
    if (incoming.empty()) {
      continue;
    }
    auto prov_it = target.provenance.find(prov_key);
    const std::string current_owner = prov_it != target.provenance.end() ? prov_it->second : std::string{};
    const bool target_set = !(target.*member).empty();
    if (detail::incoming_wins(target_set, current_owner, source_name, config)) {
      target.*member = incoming;
      target.provenance[prov_key] = recorded_provenance(prov_key);
    }
  }

  for (const auto & [key, value] : source.extra) {
    if (value.empty()) {
      continue;
    }
    const std::string prov_key = "extra." + key;
    auto prov_it = target.provenance.find(prov_key);
    const std::string current_owner = prov_it != target.provenance.end() ? prov_it->second : std::string{};
    auto extra_it = target.extra.find(key);
    const bool target_set = extra_it != target.extra.end() && !extra_it->second.empty();
    if (detail::incoming_wins(target_set, current_owner, source_name, config)) {
      target.extra[key] = value;
      target.provenance[prov_key] = recorded_provenance(prov_key);
    }
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
