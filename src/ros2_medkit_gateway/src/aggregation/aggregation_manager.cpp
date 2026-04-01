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

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"

#include <algorithm>
#include <future>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>

#include "ros2_medkit_gateway/aggregation/entity_merger.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"

namespace ros2_medkit_gateway {

namespace {

/**
 * @brief Validate a peer URL discovered via mDNS
 *
 * Rejects URLs that don't use HTTP(S), point to cloud metadata endpoints,
 * or use loopback addresses (which would be self-referential for mDNS peers).
 */
bool is_valid_peer_url(const std::string & url) {
  // Must start with http:// or https://
  if (url.rfind("http://", 0) != 0 && url.rfind("https://", 0) != 0) {
    return false;
  }
  // Block loopback addresses (self-referential for mDNS peers)
  if (url.find("://127.") != std::string::npos || url.find("://localhost") != std::string::npos ||
      url.find("://[::1]") != std::string::npos) {
    return false;
  }
  // Block all link-local (169.254.x.x) including cloud metadata endpoints
  if (url.find("://169.254.") != std::string::npos) {
    return false;
  }
  if (url.find("metadata.google") != std::string::npos) {
    return false;
  }
  return true;
}

}  // namespace

AggregationManager::AggregationManager(const AggregationConfig & config, rclcpp::Logger * logger) : config_(config) {
  for (const auto & peer_cfg : config_.peers) {
    // Validate scheme for static peers (http/https only).
    // Unlike mDNS peers, loopback addresses are valid for static config
    // (e.g., testing or same-host deployments).
    if (peer_cfg.url.rfind("http://", 0) != 0 && peer_cfg.url.rfind("https://", 0) != 0) {
      continue;
    }

    // TLS enforcement: reject http:// peers when require_tls is true,
    // warn about cleartext when require_tls is false.
    if (peer_cfg.url.rfind("http://", 0) == 0) {
      if (config_.require_tls) {
        if (logger) {
          RCLCPP_ERROR(*logger, "Aggregation: skipping peer '%s' at %s - require_tls is enabled but URL uses http://",
                       peer_cfg.name.c_str(), peer_cfg.url.c_str());
        }
        continue;
      }
      if (logger) {
        RCLCPP_WARN(*logger, "Aggregation: peer '%s' at %s uses cleartext HTTP - consider using HTTPS",
                    peer_cfg.name.c_str(), peer_cfg.url.c_str());
      }
    }

    peers_.push_back(
        std::make_unique<PeerClient>(peer_cfg.url, peer_cfg.name, config_.timeout_ms, config_.forward_auth));
  }
}

size_t AggregationManager::peer_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return peers_.size();
}

void AggregationManager::add_discovered_peer(const std::string & url, const std::string & name) {
  // Validate mDNS-discovered peer URLs (static config peers bypass this check)
  if (!is_valid_peer_url(url)) {
    return;
  }

  // TLS enforcement for discovered peers
  if (url.rfind("http://", 0) == 0 && config_.require_tls) {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Do not add if a peer with this name already exists
  if (find_peer(name) != nullptr) {
    return;
  }

  peers_.push_back(std::make_unique<PeerClient>(url, name, config_.timeout_ms, config_.forward_auth));
}

void AggregationManager::remove_discovered_peer(const std::string & name) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  auto it = std::remove_if(peers_.begin(), peers_.end(), [&name](const std::unique_ptr<PeerClient> & peer) {
    return peer->name() == name;
  });
  peers_.erase(it, peers_.end());
}

void AggregationManager::check_all_health() {
  // Shared lock blocks add/remove (which need unique lock) during health checks.
  // Health checks run in parallel via std::async to reduce worst-case latency
  // from N * timeout_ms (sequential) to just timeout_ms (parallel).
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::future<void>> futures;
  futures.reserve(peers_.size());
  for (auto & peer : peers_) {
    futures.push_back(std::async(std::launch::async, [&peer]() {
      peer->check_health();
    }));
  }
  for (auto & f : futures) {
    f.get();
  }
}

size_t AggregationManager::healthy_peer_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  size_t count = 0;
  for (const auto & peer : peers_) {
    if (peer->is_healthy()) {
      ++count;
    }
  }
  return count;
}

PeerEntities AggregationManager::fetch_all_peer_entities() {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  PeerEntities merged;
  for (auto & peer : peers_) {
    if (!peer->is_healthy()) {
      continue;
    }
    auto result = peer->fetch_entities();
    if (!result.has_value()) {
      continue;
    }

    const auto & entities = result.value();
    merged.areas.insert(merged.areas.end(), entities.areas.begin(), entities.areas.end());
    merged.components.insert(merged.components.end(), entities.components.begin(), entities.components.end());
    merged.apps.insert(merged.apps.end(), entities.apps.begin(), entities.apps.end());
    merged.functions.insert(merged.functions.end(), entities.functions.begin(), entities.functions.end());
  }

  return merged;
}

AggregationManager::MergedPeerResult AggregationManager::fetch_and_merge_peer_entities(
    const std::vector<Area> & local_areas, const std::vector<Component> & local_components,
    const std::vector<App> & local_apps, const std::vector<Function> & local_functions, size_t max_entities_per_peer,
    rclcpp::Logger * logger) {
  MergedPeerResult merged;
  merged.areas = local_areas;
  merged.components = local_components;
  merged.apps = local_apps;
  merged.functions = local_functions;

  // Hold shared lock during the entire fetch-and-merge operation so that
  // remove_discovered_peer() (which needs a unique lock) cannot destroy
  // PeerClient objects while we are iterating.
  std::shared_lock<std::shared_mutex> lock(mutex_);
  for (auto & peer : peers_) {
    if (!peer->is_healthy()) {
      continue;
    }

    auto result = peer->fetch_entities();
    if (!result.has_value()) {
      if (logger) {
        RCLCPP_WARN(*logger, "Failed to fetch entities from peer '%s': %s", peer->name().c_str(),
                    result.error().c_str());
      }
      continue;
    }

    size_t total = result->areas.size() + result->components.size() + result->apps.size() + result->functions.size();
    if (total > max_entities_per_peer) {
      if (logger) {
        RCLCPP_WARN(*logger, "Peer '%s' returned %zu entities (max %zu), skipping", peer->name().c_str(), total,
                    max_entities_per_peer);
      }
      continue;
    }

    EntityMerger merger(peer->name());
    merged.areas = merger.merge_areas(merged.areas, result->areas);
    merged.functions = merger.merge_functions(merged.functions, result->functions);
    merged.components = merger.merge_components(merged.components, result->components);
    merged.apps = merger.merge_apps(merged.apps, result->apps);

    for (const auto & [id, name] : merger.get_routing_table()) {
      merged.routing_table[id] = name;
      // Log collision-prefixed entities to help operators diagnose naming conflicts
      if (id.find(EntityMerger::SEPARATOR) != std::string::npos) {
        if (logger) {
          RCLCPP_WARN(*logger, "Entity ID collision: '%s' prefixed for peer '%s'", id.c_str(), name.c_str());
        }
      }
    }
  }

  return merged;
}

void AggregationManager::update_routing_table(const std::unordered_map<std::string, std::string> & table) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  routing_table_ = table;
}

std::optional<std::string> AggregationManager::find_peer_for_entity(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  auto it = routing_table_.find(entity_id);
  if (it != routing_table_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::string AggregationManager::get_peer_url(const std::string & peer_name) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const auto * peer = find_peer(peer_name);
  if (peer != nullptr) {
    return peer->url();
  }
  return "";
}

void AggregationManager::forward_request(const std::string & peer_name, const httplib::Request & req,
                                         httplib::Response & res) {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  auto * peer = find_peer(peer_name);
  if (peer == nullptr) {
    res.status = 502;
    nlohmann::json error_body;
    error_body["error_code"] = ERR_VENDOR_ERROR;
    error_body["vendor_code"] = "x-medkit-peer-unavailable";
    error_body["message"] = "Peer '" + peer_name + "' is not known to this gateway";
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Strip peer prefix from entity ID in the path if present.
  // When entity ID collision causes renaming (e.g., camera_driver -> peer_b__camera_driver),
  // the peer only knows the entity by its original ID (camera_driver), so we must strip
  // the prefix before forwarding.
  // Anchor to path segment boundary: the prefix must appear right after '/' to avoid
  // false matches inside other path segments (e.g., "v1" matching "/api/v1/").
  std::string forwarded_path = req.path;
  std::string prefix = peer_name + EntityMerger::SEPARATOR;
  auto prefix_pos = forwarded_path.find(prefix);
  if (prefix_pos != std::string::npos && prefix_pos > 0 && forwarded_path[prefix_pos - 1] == '/') {
    forwarded_path.erase(prefix_pos, prefix.size());
  }

  httplib::Request modified_req = req;
  modified_req.path = forwarded_path;

  peer->forward_request(modified_req, res);
}

AggregationManager::FanOutResult AggregationManager::fan_out_get(const std::string & path,
                                                                 const std::string & auth_header) {
  FanOutResult fan_out_result;
  fan_out_result.merged_items = nlohmann::json::array();

  // Hold shared lock for entire iteration to prevent dangling pointers
  // if remove_discovered_peer() runs concurrently
  std::shared_lock<std::shared_mutex> lock(mutex_);
  for (auto & peer : peers_) {
    if (!peer->is_healthy()) {
      continue;
    }

    auto result = peer->forward_and_get_json("GET", path, auth_header);
    if (!result.has_value()) {
      fan_out_result.is_partial = true;
      fan_out_result.failed_peers.push_back(peer->name());
      continue;
    }

    const auto & response_json = result.value();
    if (response_json.contains("items") && response_json["items"].is_array()) {
      for (const auto & item : response_json["items"]) {
        fan_out_result.merged_items.push_back(item);
      }
    }
  }

  return fan_out_result;
}

nlohmann::json AggregationManager::get_peer_status() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  nlohmann::json status_array = nlohmann::json::array();
  for (const auto & peer : peers_) {
    nlohmann::json peer_obj;
    peer_obj["name"] = peer->name();
    peer_obj["url"] = peer->url();
    peer_obj["status"] = peer->is_healthy() ? "online" : "offline";
    status_array.push_back(peer_obj);
  }
  return status_array;
}

PeerClient * AggregationManager::find_peer(const std::string & name) const {
  for (const auto & peer : peers_) {
    if (peer->name() == name) {
      return peer.get();
    }
  }
  return nullptr;
}

}  // namespace ros2_medkit_gateway
