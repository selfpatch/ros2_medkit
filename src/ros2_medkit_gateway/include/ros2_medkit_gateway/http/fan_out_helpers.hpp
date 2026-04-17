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
#include <optional>
#include <string>
#include <string_view>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

namespace ros2_medkit_gateway {

inline std::string url_encode_param(const std::string & value) {
  std::string result;
  result.reserve(value.size());
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (const auto ch : value) {
    auto c = static_cast<unsigned char>(ch);
    // Locale-independent RFC 3986 unreserved character check (ASCII-only).
    // std::isalnum is locale-dependent and could pass bytes >= 0x80.
    bool unreserved = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
                      c == '_' || c == '.' || c == '~';
    if (unreserved) {
      result += static_cast<char>(c);
    } else {
      result += '%';
      result += kHex[static_cast<size_t>(c >> 4u)];
      result += kHex[static_cast<size_t>(c & 0x0Fu)];
    }
  }
  return result;
}

// Extract the entity id from a per-entity collection path like
// `/api/v1/components/<id>/logs` or `/api/v1/apps/<id>/data/<sub>`.
// Matching is intentionally permissive: any path containing
// `/components|apps|areas|functions/<id>/<anything>` is treated as
// per-entity, regardless of what the segment after `<id>` is. This keeps
// future per-entity sub-endpoints eligible for target-filtered fan-out
// without requiring a whitelist bump every time.
//
// Returns std::nullopt for:
//   - global endpoints (`/api/v1/faults`, `/api/v1/health`, ...),
//   - entity-detail endpoints (`/api/v1/components/<id>` with no trailing
//     `/...` - these go through the forwarding path, not fan-out),
//   - malformed paths with an empty entity segment (`/components//logs`).
inline std::optional<std::string> extract_entity_id_for_fan_out(const std::string & path) {
  static constexpr std::array<std::string_view, 4> kEntityCollections = {"/components/", "/apps/", "/areas/",
                                                                         "/functions/"};
  for (const auto & prefix : kEntityCollections) {
    auto start = path.find(prefix);
    if (start == std::string::npos) {
      continue;
    }
    start += prefix.size();
    auto end = path.find('/', start);
    if (end == std::string::npos) {
      // Entity detail endpoint like `/components/<id>` (no trailing collection).
      // These are routed to peers via a different mechanism; not a fan-out case.
      return std::nullopt;
    }
    std::string id = path.substr(start, end - start);
    if (id.empty()) {
      return std::nullopt;
    }
    return id;
  }
  return std::nullopt;
}

inline std::string build_fan_out_path(const httplib::Request & req) {
  if (req.params.empty()) {
    return req.path;
  }
  std::string path = req.path;
  char sep = '?';
  for (const auto & [key, value] : req.params) {
    path += sep;
    path += url_encode_param(key);
    path += '=';
    path += url_encode_param(value);
    sep = '&';
  }
  return path;
}

inline void merge_peer_items(AggregationManager * agg, const httplib::Request & req, nlohmann::json & result,
                             XMedkit & ext) {
  if (agg == nullptr) {
    return;
  }
  if (req.has_header("X-Medkit-No-Fan-Out")) {
    return;
  }
  // Skip fan-out when no healthy peers to avoid blocking the httplib handler
  // thread on network I/O (up to timeout_ms per request). With fan-out on all
  // per-entity collection endpoints, concurrent requests during a peer outage
  // could exhaust httplib's thread pool if we don't bail out early here.
  if (agg->healthy_peer_count() == 0) {
    return;
  }
  // For per-entity collection paths, target only the peers that host or
  // contribute to the entity (routed leaves and merged / hierarchical
  // entities). Local-only entities produce an empty target list and skip
  // fan-out entirely, avoiding spurious `partial: true` / `failed_peers`
  // from peers that do not own the entity. Global collection endpoints
  // (paths without an entity id) keep fan-out-to-all behavior.
  std::optional<std::vector<std::string>> contributors_buffer;
  const std::vector<std::string> * target_peers = nullptr;
  if (auto entity_id = extract_entity_id_for_fan_out(req.path); entity_id.has_value()) {
    contributors_buffer = agg->get_peer_contributors(*entity_id);
    if (contributors_buffer->empty()) {
      return;  // local-only: no peer hosts this entity
    }
    target_peers = &contributors_buffer.value();
  }
  auto fan_path = build_fan_out_path(req);
  auto fan_result = agg->fan_out_get(fan_path, req.get_header_value("Authorization"), target_peers);
  if (fan_result.merged_items.is_array() && !fan_result.merged_items.empty()) {
    if (!result.contains("items") || !result["items"].is_array()) {
      result["items"] = nlohmann::json::array();
    }
    for (const auto & item : fan_result.merged_items) {
      if (item.is_object()) {
        result["items"].push_back(item);
      }
    }
  }
  if (fan_result.is_partial) {
    ext.add("partial", true);
    ext.add("failed_peers", fan_result.failed_peers);
  }
}

}  // namespace ros2_medkit_gateway
