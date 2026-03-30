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

#include "ros2_medkit_gateway/aggregation/peer_client.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

using namespace ros2_medkit_gateway;

// Use a port that nothing listens on for connection-refused tests
constexpr int DEAD_PORT = 59999;
static const std::string DEAD_URL = "http://127.0.0.1:" + std::to_string(DEAD_PORT);

// =============================================================================
// Construction and basic accessor tests
// =============================================================================

TEST(PeerClient, constructs_with_url_and_name) {
  PeerClient client("http://localhost:8081", "subsystem_b", 500);

  EXPECT_EQ(client.url(), "http://localhost:8081");
  EXPECT_EQ(client.name(), "subsystem_b");
}

TEST(PeerClient, initially_not_healthy) {
  PeerClient client(DEAD_URL, "dead_peer", 500);

  EXPECT_FALSE(client.is_healthy());
}

// =============================================================================
// Health check tests (connection refused path)
// =============================================================================

TEST(PeerClient, health_check_marks_unhealthy_on_connection_refused) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  // Should not throw, just mark unhealthy
  client.check_health();
  EXPECT_FALSE(client.is_healthy());
}

// =============================================================================
// Forward request tests (connection failure path)
// =============================================================================

TEST(PeerClient, forward_sets_502_on_connection_error) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/components";
  httplib::Response res;

  client.forward_request(req, res);

  EXPECT_EQ(res.status, 502);

  auto body = nlohmann::json::parse(res.body, nullptr, false);
  ASSERT_FALSE(body.is_discarded());
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-peer-unavailable");
  EXPECT_TRUE(body["message"].get<std::string>().find("dead_peer") != std::string::npos);
}

// =============================================================================
// forward_and_get_json tests (connection failure path)
// =============================================================================

TEST(PeerClient, forward_and_get_json_returns_error_on_connection_refused) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  auto result = client.forward_and_get_json("GET", "/api/v1/health");

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("dead_peer") != std::string::npos);
  EXPECT_TRUE(result.error().find("Failed to connect") != std::string::npos);
}

// =============================================================================
// fetch_entities tests (connection failure path)
// =============================================================================

TEST(PeerClient, fetch_entities_returns_error_on_connection_refused) {
  PeerClient client(DEAD_URL, "dead_peer", 200);

  auto result = client.fetch_entities();

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("dead_peer") != std::string::npos);
}

// =============================================================================
// EntityInfo remote field tests
// =============================================================================

TEST(EntityInfoRemote, defaults_to_local) {
  EntityInfo info;
  EXPECT_FALSE(info.is_remote);
  EXPECT_TRUE(info.peer_url.empty());
  EXPECT_TRUE(info.peer_name.empty());
}

TEST(EntityInfoRemote, can_be_marked_remote) {
  EntityInfo info;
  info.is_remote = true;
  info.peer_url = "http://192.168.1.10:8081";
  info.peer_name = "subsystem_a";

  EXPECT_TRUE(info.is_remote);
  EXPECT_EQ(info.peer_url, "http://192.168.1.10:8081");
  EXPECT_EQ(info.peer_name, "subsystem_a");
}
