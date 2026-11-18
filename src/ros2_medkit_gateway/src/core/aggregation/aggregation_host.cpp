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

#include "ros2_medkit_gateway/core/aggregation/aggregation_host.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "ros2_medkit_gateway/core/aggregation/network_utils.hpp"
#include "ros2_medkit_gateway/core/providers/host_info_provider.hpp"

namespace ros2_medkit_gateway {

AggregationHost::AggregationHost(const AggregationConfig & config, int listen_port, const std::string & mdns_name,
                                 log::Logger logger)
  : logger_(std::move(logger)) {
  if (config.peers.empty() && !config.announce && !config.discover) {
    MEDKIT_LOG_WARN(logger_,
                    "Aggregation enabled but no static peers and mDNS disabled. "
                    "No peer communication will occur.");
  }
  // Security warning: forwarding auth tokens over cleartext is dangerous
  if (config.forward_auth && !config.require_tls) {
    MEDKIT_LOG_WARN(logger_,
                    "Aggregation: forward_auth is enabled but require_tls is false. "
                    "Authorization tokens may be sent to peers over cleartext HTTP. "
                    "Set aggregation.require_tls=true for production deployments.");
  }

  manager_ = std::make_unique<AggregationManager>(config, &logger_);

  if (config.announce || config.discover) {
    MdnsDiscovery::Config mdns_config;
    mdns_config.announce = config.announce;
    mdns_config.discover = config.discover;
    mdns_config.service = config.mdns_service;
    mdns_config.port = listen_port;
    mdns_config.peer_scheme = config.peer_scheme;
    // The instance name must be unique per gateway instance. MdnsDiscovery uses the hostname when it is empty.
    mdns_config.name = mdns_name;
    mdns_config.on_error = [this](const std::string & msg) {
      MEDKIT_LOG_ERROR(logger_, "mDNS: %s", msg.c_str());
    };
    mdns_config.on_log = [this](const std::string & msg) {
      MEDKIT_LOG_DEBUG(logger_, "mDNS: %s", msg.c_str());
    };
    mdns_ = std::make_unique<MdnsDiscovery>(mdns_config);
    // The resolved instance name, sanitized the way discovered peer names are, for the self-discovery filter.
    const std::string self_mdns_name = HostInfoProvider::sanitize_entity_id(mdns_->instance_name());

    // Name checks alone are not enough: a spoofed response can carry another name and our own address and port,
    // and cause a forwarding loop. The address set is refreshed every 60 s to follow network changes.
    auto local_addrs = std::make_shared<std::unordered_set<std::string>>(collect_local_addresses());
    auto last_addr_refresh = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());
    auto addr_mutex = std::make_shared<std::mutex>();

    mdns_->start(
        [this, self_mdns_name, local_addrs, last_addr_refresh, addr_mutex, listen_port](const std::string & url,
                                                                                        const std::string & name) {
          if (name == self_mdns_name) {
            return;
          }
          {
            std::lock_guard<std::mutex> lock(*addr_mutex);
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - *last_addr_refresh).count() >= 60) {
              *local_addrs = collect_local_addresses();
              *last_addr_refresh = now;
            }
          }
          auto [peer_host, peer_port] = parse_url_host_port(url);
          if (peer_port == listen_port && local_addrs->count(peer_host) > 0) {
            MEDKIT_LOG_WARN(logger_,
                            "mDNS: rejecting peer '%s' at %s - resolves to local address with our port "
                            "(possible spoofed mDNS response)",
                            name.c_str(), url.c_str());
            return;
          }
          manager_->add_discovered_peer(url, name);
        },
        [this](const std::string & name) {
          manager_->remove_discovered_peer(name);
        });
  }

  MEDKIT_LOG_INFO(logger_,
                  "Aggregation: enabled (timeout=%dms, announce=%s, discover=%s, "
                  "forward_auth=%s, require_tls=%s, peer_scheme=%s, max_discovered_peers=%zu)",
                  config.timeout_ms, config.announce ? "true" : "false", config.discover ? "true" : "false",
                  config.forward_auth ? "true" : "false", config.require_tls ? "true" : "false",
                  config.peer_scheme.c_str(), config.max_discovered_peers);
}

AggregationHost::~AggregationHost() {
  shutdown();
}

void AggregationHost::shutdown() {
  // mDNS first: it must not add peers while the calls are being cancelled.
  if (mdns_) {
    mdns_->stop();
  }
  manager_->shutdown();
}

std::optional<std::unordered_map<std::string, std::string>>
AggregationHost::merge_peers(std::vector<Area> & areas, std::vector<Component> & components, std::vector<App> & apps,
                             std::vector<Function> & functions) {
  if (manager_->peer_count() == 0) {
    return std::nullopt;
  }
  manager_->check_all_health();
  auto merged = manager_->fetch_and_merge_peer_entities(areas, components, apps, functions, 10000, &logger_);
  areas = std::move(merged.areas);
  components = std::move(merged.components);
  apps = std::move(merged.apps);
  functions = std::move(merged.functions);
  auto routing_table = std::move(merged.routing_table);
  manager_->update_routing_table(routing_table);
  manager_->set_leaf_warnings(std::move(merged.leaf_warnings));

  // Per-entity peer contributors: entity id -> names of the peers that host or contribute to it. This covers routed
  // leaves (also in the routing table) and merged or hierarchical entities (areas, functions, parent components).
  // Fan-out helpers use the map to ask only the peers that own the entity. EntityMerger writes "peer:<name>"
  // entries; anything else, including "local", is ignored.
  std::unordered_map<std::string, std::vector<std::string>> peer_contributors;
  static constexpr std::string_view kPeerPrefix = "peer:";
  auto collect_contributors = [&peer_contributors](const auto & entities) {
    for (const auto & e : entities) {
      for (const auto & c : e.contributors) {
        if (c.rfind(kPeerPrefix, 0) != 0) {
          continue;
        }
        std::string peer_name = c.substr(kPeerPrefix.size());
        if (peer_name.empty()) {
          continue;
        }
        auto & list = peer_contributors[e.id];
        if (std::find(list.begin(), list.end(), peer_name) == list.end()) {
          list.push_back(std::move(peer_name));
        }
      }
    }
  };
  collect_contributors(areas);
  collect_contributors(components);
  collect_contributors(apps);
  collect_contributors(functions);
  manager_->update_peer_contributors(std::move(peer_contributors));
  return routing_table;
}

}  // namespace ros2_medkit_gateway
