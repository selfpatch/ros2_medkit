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

// =============================================================================
// Mock server happy-path tests (local httplib::Server)
// =============================================================================

#include <thread>

TEST(PeerClientHappyPath, health_check_marks_healthy) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  EXPECT_FALSE(client.is_healthy());

  client.check_health();
  EXPECT_TRUE(client.is_healthy());

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, health_check_unhealthy_on_500) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.status = 500;
    res.set_content(R"({"status":"error"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  client.check_health();
  EXPECT_FALSE(client.is_healthy());

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, fetch_entities_parses_collections) {
  httplib::Server svr;
  svr.Get("/api/v1/areas", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"zone_a","name":"Zone A"}]})", "application/json");
  });
  svr.Get("/api/v1/components", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"ecu_1","name":"ECU 1"}]})", "application/json");
  });
  svr.Get("/api/v1/apps", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[{"id":"nav","name":"Navigation"}]})", "application/json");
  });
  svr.Get("/api/v1/functions", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"items":[]})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.fetch_entities();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->areas.size(), 1u);
  EXPECT_EQ(result->areas[0].id, "zone_a");
  EXPECT_EQ(result->areas[0].name, "Zone A");
  EXPECT_EQ(result->areas[0].source, "peer:test_peer");
  EXPECT_EQ(result->components.size(), 1u);
  EXPECT_EQ(result->components[0].id, "ecu_1");
  EXPECT_EQ(result->components[0].source, "peer:test_peer");
  EXPECT_EQ(result->apps.size(), 1u);
  EXPECT_EQ(result->apps[0].id, "nav");
  EXPECT_EQ(result->apps[0].source, "peer:test_peer");
  EXPECT_EQ(result->functions.size(), 0u);

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_request_proxies_response) {
  httplib::Server svr;
  svr.Get(R"(/api/v1/apps/nav/data)", [](const httplib::Request & req, httplib::Response & res) {
    nlohmann::json body;
    body["forwarded"] = true;
    if (req.has_header("Authorization")) {
      body["auth"] = req.get_header_value("Authorization");
    }
    res.set_content(body.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/apps/nav/data";
  req.set_header("Authorization", "Bearer test-token");

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body["forwarded"].get<bool>());
  EXPECT_EQ(body["auth"].get<std::string>(), "Bearer test-token");

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_filters_response_headers) {
  httplib::Server svr;
  svr.Get("/api/v1/test", [](const httplib::Request &, httplib::Response & res) {
    res.set_header("ETag", "\"abc123\"");
    res.set_header("Cache-Control", "no-cache");
    res.set_header("Set-Cookie", "evil=1");
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("X-Medkit-Custom", "allowed");
    res.set_content("{}", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);

  httplib::Request req;
  req.method = "GET";
  req.path = "/api/v1/test";

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  // Safe headers should be present
  EXPECT_TRUE(res.has_header("ETag"));
  EXPECT_TRUE(res.has_header("Cache-Control"));
  // x-medkit vendor headers should be forwarded
  EXPECT_TRUE(res.has_header("X-Medkit-Custom"));
  // Dangerous headers should be filtered
  EXPECT_FALSE(res.has_header("Set-Cookie"));
  EXPECT_FALSE(res.has_header("Access-Control-Allow-Origin"));

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_returns_parsed_json) {
  httplib::Server svr;
  svr.Get("/api/v1/components/ecu/data", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"temperature":42.5,"status":"ok"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.forward_and_get_json("GET", "/api/v1/components/ecu/data");

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->at("temperature").get<double>(), 42.5);
  EXPECT_EQ(result->at("status").get<std::string>(), "ok");

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_with_auth_header) {
  httplib::Server svr;
  svr.Get("/api/v1/health", [](const httplib::Request & req, httplib::Response & res) {
    nlohmann::json body;
    body["has_auth"] = req.has_header("Authorization");
    if (req.has_header("Authorization")) {
      body["auth_value"] = req.get_header_value("Authorization");
    }
    res.set_content(body.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.forward_and_get_json("GET", "/api/v1/health", "Bearer my-jwt");

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->at("has_auth").get<bool>());
  EXPECT_EQ(result->at("auth_value").get<std::string>(), "Bearer my-jwt");

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_and_get_json_error_on_non_2xx) {
  httplib::Server svr;
  svr.Get("/api/v1/missing", [](const httplib::Request &, httplib::Response & res) {
    res.status = 404;
    res.set_content(R"({"error":"not found"})", "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);
  auto result = client.forward_and_get_json("GET", "/api/v1/missing");

  ASSERT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("404") != std::string::npos);
  EXPECT_TRUE(result.error().find("test_peer") != std::string::npos);

  svr.stop();
  t.join();
}

TEST(PeerClientHappyPath, forward_post_request) {
  httplib::Server svr;
  svr.Post("/api/v1/apps/nav/operations/restart", [](const httplib::Request & req, httplib::Response & res) {
    auto body = nlohmann::json::parse(req.body, nullptr, false);
    nlohmann::json response;
    response["executed"] = true;
    response["received_body"] = body;
    res.set_content(response.dump(), "application/json");
  });

  int port = svr.bind_to_any_port("127.0.0.1");
  std::thread t([&]() {
    svr.listen_after_bind();
  });

  PeerClient client("http://127.0.0.1:" + std::to_string(port), "test_peer", 5000);

  httplib::Request req;
  req.method = "POST";
  req.path = "/api/v1/apps/nav/operations/restart";
  req.body = R"({"force":true})";
  req.set_header("Content-Type", "application/json");

  httplib::Response res;
  client.forward_request(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body["executed"].get<bool>());
  EXPECT_TRUE(body["received_body"]["force"].get<bool>());

  svr.stop();
  t.join();
}
