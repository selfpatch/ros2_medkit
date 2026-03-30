// Copyright 2026 Selfpatch GmbH
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
#include <string>

#include "ros2_medkit_gateway/http/error_codes.hpp"

namespace ros2_medkit_gateway {

AggregationManager::AggregationManager(const AggregationConfig & config) : config_(config) {
  for (const auto & peer_cfg : config_.peers) {
    peers_.push_back(std::make_unique<PeerClient>(peer_cfg.url, peer_cfg.name, config_.timeout_ms));
  }
}

size_t AggregationManager::peer_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return peers_.size();
}

void AggregationManager::add_discovered_peer(const std::string & url, const std::string & name) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Do not add if a peer with this name already exists
  if (find_peer(name) != nullptr) {
    return;
  }

  peers_.push_back(std::make_unique<PeerClient>(url, name, config_.timeout_ms));
}

void AggregationManager::remove_discovered_peer(const std::string & name) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  auto it = std::remove_if(peers_.begin(), peers_.end(), [&name](const std::unique_ptr<PeerClient> & peer) {
    return peer->name() == name;
  });
  peers_.erase(it, peers_.end());
}

void AggregationManager::check_all_health() {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  for (auto & peer : peers_) {
    peer->check_health();
  }
}

std::vector<PeerClient *> AggregationManager::healthy_peers() {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  std::vector<PeerClient *> result;
  for (auto & peer : peers_) {
    if (peer->is_healthy()) {
      result.push_back(peer.get());
    }
  }
  return result;
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

  peer->forward_request(req, res);
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
