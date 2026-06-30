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

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief How the identity key for an asset is derived.
 *
 * The identity key is what decides whether two records describe the *same* asset
 * and should therefore have their identity merged.
 */
enum class IdentityKeyStrategy {
  AUTO,           ///< serial -> model+slot -> endpoint -> configured id (first that resolves)
  SERIAL,         ///< serial number only
  ORDER_CODE_SLOT,  ///< model (order code) + extra["slot"]
  ENDPOINT,       ///< network endpoint
  CONFIGURED_ID   ///< the configured Component id (always available, never cross-source)
};

inline std::optional<IdentityKeyStrategy> identity_key_strategy_from_string(const std::string & s) {
  if (s == "auto") {
    return IdentityKeyStrategy::AUTO;
  }
  if (s == "serial") {
    return IdentityKeyStrategy::SERIAL;
  }
  if (s == "order_code_slot") {
    return IdentityKeyStrategy::ORDER_CODE_SLOT;
  }
  if (s == "endpoint") {
    return IdentityKeyStrategy::ENDPOINT;
  }
  if (s == "configured_id") {
    return IdentityKeyStrategy::CONFIGURED_ID;
  }
  return std::nullopt;
}

/**
 * @brief Configuration for identity merging.
 *
 * `source_precedence` lists source names from highest to lowest authority. A source
 * not in the list is treated as the lowest authority (it can still fill empty fields
 * but never overrides a known source). Identity authority is deliberately decoupled
 * from the structural merge policy: a manifest may be the authoritative *structure*
 * source while a live protocol read is the authoritative *identity* source.
 *
 * Default precedence (highest first): live protocol device-info beats the hand
 * authored manifest, which beats whatever runtime discovery guessed.
 */
struct IdentityMergeConfig {
  std::vector<std::string> source_precedence{"opcua",      "s7",       "ethernet_ip", "modbus",
                                             "ads",        "profinet", "manifest",    "config",
                                             "runtime",    "node",     "heuristic"};
  IdentityKeyStrategy key_strategy{IdentityKeyStrategy::AUTO};
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

/**
 * @brief Compute the identity key for an asset.
 * @param identity     The (possibly partial) asset identity.
 * @param configured_id Fallback stable id (typically Component.id).
 * @param strategy     Key derivation strategy.
 * @return A non-empty identity key, or empty string if the chosen strategy cannot
 *         resolve one (e.g. SERIAL strategy on an asset with no serial).
 */
inline std::string compute_identity_key(const AssetIdentity & identity, const std::string & configured_id,
                                        IdentityKeyStrategy strategy) {
  auto slot = [&]() -> std::string {
    auto it = identity.extra.find("slot");
    return it != identity.extra.end() ? it->second : std::string{};
  };
  switch (strategy) {
    case IdentityKeyStrategy::SERIAL:
      return identity.serial_number;
    case IdentityKeyStrategy::ORDER_CODE_SLOT:
      if (identity.model.empty()) {
        return std::string{};
      }
      return identity.model + "/" + slot();
    case IdentityKeyStrategy::ENDPOINT:
      return identity.network_endpoint;
    case IdentityKeyStrategy::CONFIGURED_ID:
      return configured_id;
    case IdentityKeyStrategy::AUTO:
    default:
      if (!identity.serial_number.empty()) {
        return "serial:" + identity.serial_number;
      }
      if (!identity.model.empty() && !slot().empty()) {
        return "ordercode:" + identity.model + "/" + slot();
      }
      if (!identity.network_endpoint.empty()) {
        return "endpoint:" + identity.network_endpoint;
      }
      return "id:" + configured_id;
  }
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
 */
inline void stamp_identity_provenance(AssetIdentity & identity, const std::string & source) {
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
 * the source currently owning the field (per `config.source_precedence`). Provenance
 * is updated whenever a value is written. Empty incoming values never overwrite.
 *
 * `target` should have had ::stamp_identity_provenance called on it (directly or via
 * a previous merge) so existing fields carry provenance; otherwise existing fields
 * are treated as owned by an unknown (lowest authority) source.
 */
inline void merge_identity(AssetIdentity & target, const AssetIdentity & source, const std::string & source_name,
                           const IdentityMergeConfig & config) {
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
      target.provenance[prov_key] = source_name;
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
      target.provenance[prov_key] = source_name;
    }
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
