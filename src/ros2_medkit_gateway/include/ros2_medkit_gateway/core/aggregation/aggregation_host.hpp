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

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/aggregation/mdns_discovery.hpp"
#include "ros2_medkit_gateway/core/logging.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Peer aggregation for a program that hosts the gateway core.
 *
 * Owns the AggregationManager built from an AggregationConfig. When `announce` or `discover` is set it also owns
 * the mDNS discovery that adds and removes peers. Discovered peers that resolve to this gateway's own name or to a
 * local address with its own port are ignored.
 *
 * The host passes the manager to RESTServer::set_aggregation_manager() and calls merge_peers() on every entity
 * refresh. The config is used as given: range checks of its fields belong to the code that reads it from a
 * configuration source.
 */
class AggregationHost {
 public:
  /// @param listen_port Port of this gateway's REST server, used for the mDNS announcement and the self-filter.
  /// @param mdns_name Instance name for the announcement. Empty means the hostname.
  AggregationHost(const AggregationConfig & config, int listen_port, const std::string & mdns_name, log::Logger logger);
  ~AggregationHost();

  AggregationHost(const AggregationHost &) = delete;
  AggregationHost & operator=(const AggregationHost &) = delete;
  AggregationHost(AggregationHost &&) = delete;
  AggregationHost & operator=(AggregationHost &&) = delete;

  AggregationManager & manager() {
    return *manager_;
  }

  /**
   * @brief Merges the entities of all peers into the local ones.
   *
   * Does nothing and returns nullopt while no peer is configured or discovered. Otherwise it checks the peers'
   * health, replaces the four vectors with the merged entities, publishes the routing table, the leaf warnings and
   * the per-entity peer contributors to the manager, and returns the routing table (entity id to peer name).
   */
  std::optional<std::unordered_map<std::string, std::string>> merge_peers(std::vector<Area> & areas,
                                                                          std::vector<Component> & components,
                                                                          std::vector<App> & apps,
                                                                          std::vector<Function> & functions);

  /// Stops mDNS discovery and cancels the calls in flight to peers. Safe to call more than once.
  /// Call it before the REST server stops: the server's workers may be waiting on a peer.
  void shutdown();

 private:
  log::Logger logger_;
  std::unique_ptr<AggregationManager> manager_;
  std::unique_ptr<MdnsDiscovery> mdns_;
};

}  // namespace ros2_medkit_gateway
