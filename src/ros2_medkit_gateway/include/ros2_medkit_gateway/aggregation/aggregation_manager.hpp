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

#pragma once

#include <httplib.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/aggregation/peer_client.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Configuration for peer aggregation
 */
struct AggregationConfig {
  bool enabled{false};
  int timeout_ms{2000};
  bool announce{false};
  bool discover{false};
  std::string mdns_service{"_medkit._tcp.local"};

  struct PeerConfig {
    std::string url;
    std::string name;
  };
  std::vector<PeerConfig> peers;
};

/**
 * @brief Coordinator for peer aggregation
 *
 * Manages PeerClients, health monitoring, routing table, entity merging,
 * and fan-out logic. Thread-safe: shared_mutex protects peers_ and
 * routing_table_. Readers use shared lock, writers use exclusive lock.
 */
class AggregationManager {
 public:
  /**
   * @brief Result of fetching and merging entities from all healthy peers
   *
   * Contains merged entity vectors and a routing table mapping remote entity
   * IDs to the peer name that owns them.
   */
  struct MergedPeerResult {
    std::vector<Area> areas;
    std::vector<Component> components;
    std::vector<App> apps;
    std::vector<Function> functions;
    std::unordered_map<std::string, std::string> routing_table;
  };

  /**
   * @brief Result of a fan-out GET across all healthy peers
   */
  struct FanOutResult {
    nlohmann::json merged_items;            ///< Merged "items" array from all peers
    bool is_partial{false};                 ///< True if some peers failed
    std::vector<std::string> failed_peers;  ///< Names of peers that failed
  };

  /**
   * @brief Construct an AggregationManager from config
   *
   * Creates a PeerClient for each statically configured peer.
   *
   * @param config Aggregation configuration
   */
  explicit AggregationManager(const AggregationConfig & config);

  /// Get the number of known peers (static + discovered)
  size_t peer_count() const;

  /**
   * @brief Add a dynamically discovered peer
   *
   * Thread-safe. If a peer with the given name already exists, this is a no-op.
   *
   * @param url Base URL of the peer
   * @param name Human-readable peer name
   */
  void add_discovered_peer(const std::string & url, const std::string & name);

  /**
   * @brief Remove a dynamically discovered peer by name
   *
   * Thread-safe. If the peer is not found, this is a no-op.
   *
   * @param name Peer name to remove
   */
  void remove_discovered_peer(const std::string & name);

  /**
   * @brief Check health of all peers
   *
   * Calls check_health() on each PeerClient.
   */
  void check_all_health();

  /**
   * @brief Get all currently healthy peers
   * @return Vector of raw pointers to healthy PeerClients (valid while lock held)
   */
  std::vector<PeerClient *> healthy_peers();

  /**
   * @brief Fetch entities from all healthy peers and merge them
   * @return Merged PeerEntities from all reachable peers
   */
  PeerEntities fetch_all_peer_entities();

  /**
   * @brief Fetch entities from all healthy peers, merge with local entities, and build routing table
   *
   * Holds the shared lock internally during iteration and entity fetching, avoiding
   * dangling pointer issues with healthy_peers(). Uses EntityMerger per-peer so that
   * collision-prefixed IDs are correctly tracked in the routing table.
   *
   * @param local_areas Local areas to merge with
   * @param local_components Local components to merge with
   * @param local_apps Local apps to merge with
   * @param local_functions Local functions to merge with
   * @param max_entities_per_peer Maximum total entities accepted from a single peer
   * @param logger Optional logger for warnings (pass nullptr to suppress)
   * @return MergedPeerResult with merged entity vectors and routing table
   */
  MergedPeerResult
  fetch_and_merge_peer_entities(const std::vector<Area> & local_areas, const std::vector<Component> & local_components,
                                const std::vector<App> & local_apps, const std::vector<Function> & local_functions,
                                size_t max_entities_per_peer = 10000, rclcpp::Logger * logger = nullptr);

  /**
   * @brief Update the routing table (entity_id -> peer_name)
   * @param table New routing table to replace the current one
   */
  void update_routing_table(const std::unordered_map<std::string, std::string> & table);

  /**
   * @brief Look up which peer owns a given entity
   * @param entity_id Entity ID to look up
   * @return Peer name if entity is remote, std::nullopt if local or unknown
   */
  std::optional<std::string> find_peer_for_entity(const std::string & entity_id) const;

  /**
   * @brief Get the URL for a known peer by name
   * @param peer_name Name of the peer
   * @return URL if found, empty string otherwise
   */
  std::string get_peer_url(const std::string & peer_name) const;

  /**
   * @brief Forward an HTTP request to a specific peer
   *
   * Finds the PeerClient by name and calls forward_request().
   * If the peer is not found, sends a 502 error response.
   *
   * @param peer_name Name of the target peer
   * @param req Incoming HTTP request to forward
   * @param res Outgoing HTTP response to populate
   */
  void forward_request(const std::string & peer_name, const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Fan-out a GET request to all healthy peers
   *
   * Sends GET requests to all healthy peers (sequentially), merges the
   * "items" arrays from their responses. Returns partial results if some
   * peers fail.
   *
   * @param path Request path (e.g., "/api/v1/components")
   * @param auth_header Authorization header value (empty to omit)
   * @return FanOutResult with merged items and failure info
   */
  FanOutResult fan_out_get(const std::string & path, const std::string & auth_header);

  /**
   * @brief Get peer status for /health endpoint
   * @return JSON array of peer objects with name, url, status
   */
  nlohmann::json get_peer_status() const;

 private:
  AggregationConfig config_;
  std::vector<std::unique_ptr<PeerClient>> peers_;
  std::unordered_map<std::string, std::string> routing_table_;
  mutable std::shared_mutex mutex_;

  /**
   * @brief Find a peer by name (caller must hold lock)
   * @param name Peer name to search for
   * @return Raw pointer to PeerClient, or nullptr if not found
   */
  PeerClient * find_peer(const std::string & name) const;
};

}  // namespace ros2_medkit_gateway
