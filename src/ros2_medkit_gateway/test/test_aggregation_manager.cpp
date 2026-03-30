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

#include <gtest/gtest.h>
#include <httplib.h>

#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"

using namespace ros2_medkit_gateway;

// Use a port that nothing listens on for connection-refused tests
constexpr int DEAD_PORT = 59999;
static const std::string DEAD_URL = "http://127.0.0.1:" + std::to_string(DEAD_PORT);

// =============================================================================
// Helper: build AggregationConfig with unreachable static peers
// =============================================================================

static AggregationConfig make_config(size_t num_peers) {
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;  // Short timeout for fast test execution

  for (size_t i = 0; i < num_peers; ++i) {
    AggregationConfig::PeerConfig peer;
    peer.url = "http://127.0.0.1:" + std::to_string(DEAD_PORT + static_cast<int>(i));
    peer.name = "peer_" + std::to_string(i);
    config.peers.push_back(peer);
  }

  return config;
}

// =============================================================================
// Construction and peer count tests
// =============================================================================

TEST(AggregationManager, adds_static_peers_from_config) {
  auto config = make_config(3);
  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 3u);
}

TEST(AggregationManager, peer_count_reflects_config) {
  auto config_empty = make_config(0);
  AggregationManager manager_empty(config_empty);
  EXPECT_EQ(manager_empty.peer_count(), 0u);

  auto config_one = make_config(1);
  AggregationManager manager_one(config_one);
  EXPECT_EQ(manager_one.peer_count(), 1u);

  auto config_five = make_config(5);
  AggregationManager manager_five(config_five);
  EXPECT_EQ(manager_five.peer_count(), 5u);
}

// =============================================================================
// Dynamic peer management tests
// =============================================================================

TEST(AggregationManager, add_discovered_peer_increases_count) {
  auto config = make_config(1);
  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 1u);

  manager.add_discovered_peer("http://192.168.1.50:8081", "discovered_peer");
  EXPECT_EQ(manager.peer_count(), 2u);
}

TEST(AggregationManager, add_discovered_peer_is_idempotent) {
  auto config = make_config(0);
  AggregationManager manager(config);

  manager.add_discovered_peer("http://192.168.1.50:8081", "peer_alpha");
  EXPECT_EQ(manager.peer_count(), 1u);

  // Adding again with same name is a no-op
  manager.add_discovered_peer("http://192.168.1.50:8082", "peer_alpha");
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, remove_discovered_peer_decreases_count) {
  auto config = make_config(2);
  AggregationManager manager(config);

  EXPECT_EQ(manager.peer_count(), 2u);

  manager.remove_discovered_peer("peer_0");
  EXPECT_EQ(manager.peer_count(), 1u);
}

TEST(AggregationManager, remove_nonexistent_peer_is_noop) {
  auto config = make_config(2);
  AggregationManager manager(config);

  manager.remove_discovered_peer("no_such_peer");
  EXPECT_EQ(manager.peer_count(), 2u);
}

// =============================================================================
// Health monitoring tests
// =============================================================================

TEST(AggregationManager, healthy_peers_empty_when_all_unreachable) {
  auto config = make_config(3);
  AggregationManager manager(config);

  // Peers start unhealthy (never checked)
  auto healthy = manager.healthy_peers();
  EXPECT_TRUE(healthy.empty());

  // After health check, they should still be unhealthy (unreachable)
  manager.check_all_health();
  healthy = manager.healthy_peers();
  EXPECT_TRUE(healthy.empty());
}

// =============================================================================
// Fan-out tests
// =============================================================================

TEST(AggregationManager, fan_out_returns_partial_when_peers_unreachable) {
  auto config = make_config(2);
  AggregationManager manager(config);

  // No peers are healthy, so fan-out should return empty merged_items
  // with no failed_peers (they are simply not included as targets)
  auto result = manager.fan_out_get("/api/v1/components", "");

  EXPECT_TRUE(result.merged_items.is_array());
  EXPECT_TRUE(result.merged_items.empty());
  // No healthy peers means no targets, so no failures
  EXPECT_FALSE(result.is_partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

TEST(AggregationManager, fan_out_returns_empty_array_with_no_peers) {
  auto config = make_config(0);
  AggregationManager manager(config);

  auto result = manager.fan_out_get("/api/v1/apps", "Bearer token123");

  EXPECT_TRUE(result.merged_items.is_array());
  EXPECT_TRUE(result.merged_items.empty());
  EXPECT_FALSE(result.is_partial);
  EXPECT_TRUE(result.failed_peers.empty());
}

// =============================================================================
// Forward request tests
// =============================================================================

TEST(AggregationManager, forward_returns_502_for_unknown_peer) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components/abc/data";
  httplib::Response res;

  manager.forward_request("nonexistent_peer", req, res);

  EXPECT_EQ(res.status, 502);

  auto body = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-peer-unavailable");
  EXPECT_TRUE(body["message"].get<std::string>().find("nonexistent_peer") != std::string::npos);
}

TEST(AggregationManager, forward_returns_502_for_unreachable_peer) {
  auto config = make_config(1);
  AggregationManager manager(config);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components/abc/data";
  httplib::Response res;

  // peer_0 exists but is unreachable
  manager.forward_request("peer_0", req, res);

  EXPECT_EQ(res.status, 502);
}

// =============================================================================
// Peer status tests
// =============================================================================

TEST(AggregationManager, get_peer_status_includes_all_peers) {
  auto config = make_config(3);
  AggregationManager manager(config);

  auto status = manager.get_peer_status();

  ASSERT_TRUE(status.is_array());
  ASSERT_EQ(status.size(), 3u);

  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(status[i]["name"], "peer_" + std::to_string(i));
    EXPECT_TRUE(status[i].contains("url"));
    EXPECT_EQ(status[i]["status"], "offline");
  }
}

TEST(AggregationManager, get_peer_status_empty_when_no_peers) {
  auto config = make_config(0);
  AggregationManager manager(config);

  auto status = manager.get_peer_status();

  ASSERT_TRUE(status.is_array());
  EXPECT_TRUE(status.empty());
}

// =============================================================================
// Routing table tests
// =============================================================================

TEST(AggregationManager, routing_table_update_and_get) {
  auto config = make_config(0);
  AggregationManager manager(config);

  // Initially empty
  EXPECT_TRUE(manager.get_routing_table().empty());

  // Update with entries
  std::unordered_map<std::string, std::string> table;
  table["remote_component_a"] = "peer_x";
  table["remote_app_b"] = "peer_y";
  manager.update_routing_table(table);

  const auto & result = manager.get_routing_table();
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result.at("remote_component_a"), "peer_x");
  EXPECT_EQ(result.at("remote_app_b"), "peer_y");
}

TEST(AggregationManager, routing_table_replaces_on_update) {
  auto config = make_config(0);
  AggregationManager manager(config);

  std::unordered_map<std::string, std::string> table1;
  table1["entity_a"] = "peer_1";
  manager.update_routing_table(table1);
  EXPECT_EQ(manager.get_routing_table().size(), 1u);

  // Replace with different table
  std::unordered_map<std::string, std::string> table2;
  table2["entity_b"] = "peer_2";
  table2["entity_c"] = "peer_3";
  manager.update_routing_table(table2);

  const auto & result = manager.get_routing_table();
  EXPECT_EQ(result.size(), 2u);
  EXPECT_EQ(result.count("entity_a"), 0u);  // Old entry gone
  EXPECT_EQ(result.at("entity_b"), "peer_2");
  EXPECT_EQ(result.at("entity_c"), "peer_3");
}

// =============================================================================
// get_peer_url tests
// =============================================================================

TEST(AggregationManager, get_peer_url_returns_url_for_known_peer) {
  auto config = make_config(2);
  AggregationManager manager(config);

  std::string url = manager.get_peer_url("peer_0");
  EXPECT_EQ(url, "http://127.0.0.1:" + std::to_string(DEAD_PORT));

  std::string url1 = manager.get_peer_url("peer_1");
  EXPECT_EQ(url1, "http://127.0.0.1:" + std::to_string(DEAD_PORT + 1));
}

TEST(AggregationManager, get_peer_url_returns_empty_for_unknown_peer) {
  auto config = make_config(1);
  AggregationManager manager(config);

  std::string url = manager.get_peer_url("no_such_peer");
  EXPECT_TRUE(url.empty());
}

// =============================================================================
// fetch_all_peer_entities tests (with unreachable peers)
// =============================================================================

TEST(AggregationManager, fetch_all_peer_entities_returns_empty_when_none_healthy) {
  auto config = make_config(2);
  AggregationManager manager(config);

  // No peers healthy -> empty result
  auto entities = manager.fetch_all_peer_entities();

  EXPECT_TRUE(entities.areas.empty());
  EXPECT_TRUE(entities.components.empty());
  EXPECT_TRUE(entities.apps.empty());
  EXPECT_TRUE(entities.functions.empty());
}
