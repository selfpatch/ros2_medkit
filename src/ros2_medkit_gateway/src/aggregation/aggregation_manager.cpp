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
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>

#include <rclcpp/logging.hpp>

#include "ros2_medkit_gateway/aggregation/entity_merger.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"

namespace ros2_medkit_gateway {

namespace {

/**
 * @brief Extract the host portion from a URL
 *
 * Handles both regular hosts and IPv6 bracket notation.
 * Returns empty string if the URL cannot be parsed.
 */
std::string extract_host(const std::string & url) {
  // Find the start of the host (after "://")
  auto scheme_end = url.find("://");
  if (scheme_end == std::string::npos) {
    return "";
  }
  size_t host_start = scheme_end + 3;
  if (host_start >= url.size()) {
    return "";
  }

  // IPv6 bracket notation: [::1] or [fe80::1%eth0]
  if (url[host_start] == '[') {
    auto bracket_end = url.find(']', host_start);
    if (bracket_end == std::string::npos) {
      return "";
    }
    return url.substr(host_start + 1, bracket_end - host_start - 1);
  }

  // Regular host: find end at ':', '/', or end of string
  size_t host_end = url.find_first_of(":/", host_start);
  if (host_end == std::string::npos) {
    host_end = url.size();
  }
  return url.substr(host_start, host_end - host_start);
}

/**
 * @brief Check if a resolved address is loopback, link-local, or unspecified
 *
 * Returns true if the address should be blocked for mDNS-discovered peers.
 */
bool is_blocked_address(const struct sockaddr * addr) {
  if (addr->sa_family == AF_INET) {
    const auto * sin = reinterpret_cast<const struct sockaddr_in *>(addr);
    uint32_t ip = ntohl(sin->sin_addr.s_addr);
    // 127.0.0.0/8 - loopback
    if ((ip >> 24) == 127) {
      return true;
    }
    // 0.0.0.0 - unspecified (binds all interfaces, self-referential)
    if (ip == 0) {
      return true;
    }
    // 169.254.0.0/16 - link-local (includes cloud metadata 169.254.169.254)
    if ((ip >> 16) == 0xA9FE) {
      return true;
    }
    return false;
  }

  if (addr->sa_family == AF_INET6) {
    const auto * sin6 = reinterpret_cast<const struct sockaddr_in6 *>(addr);
    // ::1 - IPv6 loopback
    if (IN6_IS_ADDR_LOOPBACK(&sin6->sin6_addr)) {
      return true;
    }
    // :: - IPv6 unspecified
    if (IN6_IS_ADDR_UNSPECIFIED(&sin6->sin6_addr)) {
      return true;
    }
    // fe80::/10 - IPv6 link-local
    if (IN6_IS_ADDR_LINKLOCAL(&sin6->sin6_addr)) {
      return true;
    }
    // ::ffff:127.x.x.x - IPv4-mapped loopback
    // ::ffff:0.0.0.0 - IPv4-mapped unspecified
    // ::ffff:169.254.x.x - IPv4-mapped link-local
    if (IN6_IS_ADDR_V4MAPPED(&sin6->sin6_addr)) {
      const auto * bytes = sin6->sin6_addr.s6_addr;
      // Last 4 bytes are the IPv4 address
      uint32_t ip = (static_cast<uint32_t>(bytes[12]) << 24) | (static_cast<uint32_t>(bytes[13]) << 16) |
                    (static_cast<uint32_t>(bytes[14]) << 8) | static_cast<uint32_t>(bytes[15]);
      if ((ip >> 24) == 127 || ip == 0 || (ip >> 16) == 0xA9FE) {
        return true;
      }
    }
    return false;
  }

  // Unknown address family - block by default
  return true;
}

/**
 * @brief Validate a peer URL discovered via mDNS
 *
 * Rejects URLs that don't use HTTP(S), point to cloud metadata endpoints,
 * or resolve to loopback/link-local/unspecified addresses. Uses getaddrinfo()
 * to resolve the host, which catches IPv4-mapped IPv6 loopback (::ffff:127.0.0.1),
 * expanded IPv6 loopback (0:0:0:0:0:0:0:1), 0.0.0.0, [::], and other bypass
 * variants that substring matching would miss.
 */
bool is_valid_peer_url(const std::string & url) {
  // Must start with http:// or https://
  if (url.rfind("http://", 0) != 0 && url.rfind("https://", 0) != 0) {
    return false;
  }

  // Block well-known cloud metadata hostnames regardless of resolution
  if (url.find("metadata.google") != std::string::npos) {
    return false;
  }

  std::string host = extract_host(url);
  if (host.empty()) {
    return false;
  }

  // Resolve the host to check the actual address
  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_NUMERICHOST;  // Try numeric first (fast, no DNS)

  struct addrinfo * result = nullptr;
  int ret = getaddrinfo(host.c_str(), nullptr, &hints, &result);

  if (ret != 0) {
    // Not a numeric address - try DNS resolution
    hints.ai_flags = 0;
    ret = getaddrinfo(host.c_str(), nullptr, &hints, &result);
    if (ret != 0) {
      // Cannot resolve - reject
      return false;
    }
  }

  // Check all resolved addresses - block if ANY resolves to a blocked range
  bool blocked = false;
  for (struct addrinfo * rp = result; rp != nullptr; rp = rp->ai_next) {
    if (is_blocked_address(rp->ai_addr)) {
      blocked = true;
      break;
    }
  }

  freeaddrinfo(result);
  return !blocked;
}

}  // namespace

AggregationManager::AggregationManager(const AggregationConfig & config, rclcpp::Logger * logger)
  : config_(config), logger_(logger ? *logger : rclcpp::get_logger("aggregation_manager")) {
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
        std::make_shared<PeerClient>(peer_cfg.url, peer_cfg.name, config_.timeout_ms, config_.forward_auth));
  }
  static_peer_count_ = peers_.size();
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

  // Enforce max_discovered_peers limit (static peers do not count)
  size_t discovered_count = peers_.size() - static_peer_count_;
  if (discovered_count >= config_.max_discovered_peers) {
    RCLCPP_WARN(logger_,
                "Aggregation: max_discovered_peers limit (%zu) reached, "
                "rejecting peer '%s' at %s",
                config_.max_discovered_peers, name.c_str(), url.c_str());
    return;
  }

  peers_.push_back(std::make_shared<PeerClient>(url, name, config_.timeout_ms, config_.forward_auth));
}

void AggregationManager::remove_discovered_peer(const std::string & name) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  auto it = std::remove_if(peers_.begin(), peers_.end(), [&name](const std::shared_ptr<PeerClient> & peer) {
    return peer->name() == name;
  });
  peers_.erase(it, peers_.end());
}

void AggregationManager::check_all_health() {
  // Snapshot shared_ptrs under lock, then release before I/O.
  // shared_ptr copies keep PeerClients alive even if remove_discovered_peer()
  // erases them from peers_ during health checks.
  std::vector<std::shared_ptr<PeerClient>> snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    snapshot.reserve(peers_.size());
    for (const auto & peer : peers_) {
      snapshot.push_back(peer);
    }
  }

  // Health checks run in parallel via std::async to reduce worst-case latency
  // from N * timeout_ms (sequential) to just timeout_ms (parallel).
  std::vector<std::future<void>> futures;
  futures.reserve(snapshot.size());
  for (auto & peer : snapshot) {
    futures.push_back(std::async(std::launch::async, [peer]() {
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
  // Snapshot healthy peers under lock, release before network I/O.
  std::vector<std::shared_ptr<PeerClient>> snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto & peer : peers_) {
      if (peer->is_healthy()) {
        snapshot.push_back(peer);
      }
    }
  }

  PeerEntities merged;
  for (auto & peer : snapshot) {
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

  // Snapshot healthy peers under lock, release before network I/O.
  // shared_ptr copies keep PeerClients alive even if remove_discovered_peer()
  // erases them from peers_ concurrently.
  std::vector<std::shared_ptr<PeerClient>> snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto & peer : peers_) {
      if (peer->is_healthy()) {
        snapshot.push_back(peer);
      }
    }
  }

  for (auto & peer : snapshot) {
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
  // Find peer under lock, take shared_ptr copy for lifetime safety, then release
  // before network I/O. The shared_ptr keeps the PeerClient alive even if
  // remove_discovered_peer() erases it from peers_ concurrently.
  std::shared_ptr<PeerClient> peer;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    peer = find_peer_shared(peer_name);
  }

  if (!peer) {
    res.status = 502;
    nlohmann::json error_body;
    error_body["error_code"] = ERR_VENDOR_ERROR;
    error_body["vendor_code"] = "x-medkit-peer-unavailable";
    error_body["message"] = "Peer '" + peer_name + "' is not known to this gateway";
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Validate forwarded path - only allow SOVD API paths to prevent SSRF
  // to internal peer endpoints (e.g., /metrics, /debug, /admin).
  if (req.path.rfind("/api/v1/", 0) != 0) {
    res.status = 400;
    nlohmann::json error_body;
    error_body["error_code"] = ERR_INVALID_REQUEST;
    error_body["message"] = "Forwarded request path must start with /api/v1/";
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
  // Per-peer result collected by each async task
  struct PeerResult {
    std::string peer_name;
    bool success{false};
    nlohmann::json items = nlohmann::json::array();
  };

  // Snapshot healthy peers under lock, release before network I/O.
  std::vector<std::shared_ptr<PeerClient>> snapshot;
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (const auto & peer : peers_) {
      if (peer->is_healthy()) {
        snapshot.push_back(peer);
      }
    }
  }

  // Fan out GET requests in parallel via std::async to reduce worst-case
  // latency from N * timeout_ms (sequential) to just timeout_ms (parallel).
  std::vector<std::future<PeerResult>> futures;
  futures.reserve(snapshot.size());
  for (auto & peer : snapshot) {
    futures.push_back(std::async(std::launch::async, [peer, &path, &auth_header]() {
      PeerResult pr;
      pr.peer_name = peer->name();

      auto result = peer->forward_and_get_json("GET", path, auth_header);
      if (!result.has_value()) {
        pr.success = false;
        return pr;
      }

      pr.success = true;
      const auto & response_json = result.value();
      if (response_json.contains("items") && response_json["items"].is_array()) {
        pr.items = response_json["items"];
      }
      return pr;
    }));
  }

  // Merge results from all peers
  FanOutResult fan_out_result;
  fan_out_result.merged_items = nlohmann::json::array();
  for (auto & f : futures) {
    auto pr = f.get();
    if (!pr.success) {
      fan_out_result.is_partial = true;
      fan_out_result.failed_peers.push_back(std::move(pr.peer_name));
      continue;
    }
    for (auto & item : pr.items) {
      fan_out_result.merged_items.push_back(std::move(item));
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

std::shared_ptr<PeerClient> AggregationManager::find_peer_shared(const std::string & name) const {
  for (const auto & peer : peers_) {
    if (peer->name() == name) {
      return peer;
    }
  }
  return nullptr;
}

}  // namespace ros2_medkit_gateway
