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

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/health_handlers.hpp"

using namespace std::chrono_literals;

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::HealthHandlers;
using ros2_medkit_gateway::openapi::RouteRegistry;

// HealthHandlers has no dependency on GatewayNode or AuthManager:
// - handle_health only calls HandlerContext::send_json() (static)
// - handle_version_info only calls HandlerContext::send_json() (static)
// - handle_root reads ctx_.auth_config() and ctx_.tls_config() (both disabled by default)
// All tests use a null GatewayNode and null AuthManager which is safe for these handlers.

namespace {

// No-op handler for route registration in tests
void noop_handler(const httplib::Request & /*req*/, httplib::Response & /*res*/) {
}

// Populate a test route registry with representative routes
void populate_test_routes(RouteRegistry & reg) {
  reg.get("/health", noop_handler).tag("Server").summary("Health check");
  reg.get("/", noop_handler).tag("Server").summary("API overview");
  reg.get("/version-info", noop_handler).tag("Server").summary("SOVD version information");
  reg.get("/areas", noop_handler).tag("Discovery").summary("List areas");
  reg.get("/apps", noop_handler).tag("Discovery").summary("List apps");
  reg.get("/components", noop_handler).tag("Discovery").summary("List components");
  reg.get("/functions", noop_handler).tag("Discovery").summary("List functions");
  reg.get("/faults", noop_handler).tag("Faults").summary("List all faults");
}

}  // namespace

class HealthHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_config_{};
  AuthConfig auth_config_{};  // enabled = false by default
  TlsConfig tls_config_{};    // enabled = false by default
  RouteRegistry route_registry_;
  HandlerContext ctx_{nullptr, cors_config_, auth_config_, tls_config_, nullptr};
  HealthHandlers handlers_{ctx_, &route_registry_};

  httplib::Request req_;
  httplib::Response res_;

  void SetUp() override {
    populate_test_routes(route_registry_);
  }

  HandlerContext make_context(const AuthConfig & auth, const TlsConfig & tls) {
    return HandlerContext(nullptr, cors_config_, auth, tls, nullptr);
  }
};

// --- handle_health ---

TEST_F(HealthHandlersTest, HandleHealthResponseContainsStatusHealthy) {
  handlers_.handle_health(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_EQ(body["status"], "healthy");
}

TEST_F(HealthHandlersTest, HandleHealthNullNodeOmitsDiscovery) {
  // ctx_ uses nullptr for GatewayNode, so discovery info should not be present
  handlers_.handle_health(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_EQ(body["status"], "healthy");
  EXPECT_FALSE(body.contains("discovery"));
}

TEST_F(HealthHandlersTest, HandleHealthResponseContainsTimestamp) {
  handlers_.handle_health(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_TRUE(body.contains("timestamp"));
  EXPECT_TRUE(body["timestamp"].is_number());
}

TEST_F(HealthHandlersTest, HandleHealthResponseIsValidJson) {
  handlers_.handle_health(req_, res_);
  EXPECT_NO_THROW(json::parse(res_.body));
}

// --- handle_version_info ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoContainsItemsArray) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  ASSERT_TRUE(body.contains("items"));
  ASSERT_TRUE(body["items"].is_array());
  EXPECT_FALSE(body["items"].empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasVersionField) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  auto & entry = body["items"][0];
  EXPECT_TRUE(entry.contains("version"));
  EXPECT_TRUE(entry["version"].is_string());
  EXPECT_FALSE(entry["version"].get<std::string>().empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasBaseUri) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  auto & entry = body["items"][0];
  EXPECT_TRUE(entry.contains("base_uri"));
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasVendorInfo) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  auto & entry = body["items"][0];
  EXPECT_TRUE(entry.contains("vendor_info"));
  EXPECT_TRUE(entry["vendor_info"].contains("name"));
  EXPECT_EQ(entry["vendor_info"]["name"], "ros2_medkit");
}

// --- handle_root ---

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootResponseContainsRequiredTopLevelFields) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_TRUE(body.contains("name"));
  EXPECT_FALSE(body["name"].get<std::string>().empty());
  EXPECT_TRUE(body.contains("version"));
  EXPECT_FALSE(body["version"].get<std::string>().empty());
  EXPECT_TRUE(body.contains("api_base"));
  EXPECT_FALSE(body["api_base"].get<std::string>().empty());
  EXPECT_TRUE(body.contains("endpoints"));
  EXPECT_TRUE(body.contains("capabilities"));
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootEndpointsIsNonEmptyArray) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  ASSERT_TRUE(body["endpoints"].is_array());
  EXPECT_FALSE(body["endpoints"].empty());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesContainsDiscovery) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  auto & caps = body["capabilities"];
  EXPECT_TRUE(caps.contains("discovery"));
  EXPECT_TRUE(caps["discovery"].get<bool>());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthDisabledNoAuthEndpoints) {
  // With auth disabled (default), auth endpoints must not appear in the list
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  for (const auto & ep : body["endpoints"]) {
    EXPECT_EQ(ep.get<std::string>().find("/auth/"), std::string::npos)
        << "Unexpected auth endpoint when auth is disabled: " << ep;
  }
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesAuthDisabled) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_FALSE(body["capabilities"]["authentication"].get<bool>());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesTlsDisabled) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_FALSE(body["capabilities"]["tls"].get<bool>());
  EXPECT_FALSE(body.contains("tls"));
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledAddsAuthEndpoints) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auto ctx_auth = make_context(auth_enabled, tls_config_);

  // Create registry with auth routes
  RouteRegistry auth_reg;
  populate_test_routes(auth_reg);
  auth_reg.post("/auth/authorize", noop_handler).tag("Authentication");
  auth_reg.post("/auth/token", noop_handler).tag("Authentication");
  auth_reg.post("/auth/revoke", noop_handler).tag("Authentication");

  HealthHandlers handlers_auth(ctx_auth, &auth_reg);

  handlers_auth.handle_root(req_, res_);
  auto body = json::parse(res_.body);

  bool has_auth_endpoint = false;
  for (const auto & ep : body["endpoints"]) {
    if (ep.get<std::string>().find("/auth/") != std::string::npos) {
      has_auth_endpoint = true;
      break;
    }
  }
  EXPECT_TRUE(has_auth_endpoint);
  EXPECT_TRUE(body["capabilities"]["authentication"].get<bool>());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledIncludesAuthMetadataBlock) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auth_enabled.require_auth_for = ros2_medkit_gateway::AuthRequirement::ALL;
  auth_enabled.jwt_algorithm = ros2_medkit_gateway::JwtAlgorithm::HS256;
  auto ctx_auth = make_context(auth_enabled, tls_config_);
  HealthHandlers handlers_auth(ctx_auth, &route_registry_);

  handlers_auth.handle_root(req_, res_);
  auto body = json::parse(res_.body);

  ASSERT_TRUE(body.contains("auth"));
  EXPECT_TRUE(body["auth"]["enabled"].get<bool>());
  EXPECT_EQ(body["auth"]["algorithm"], "HS256");
  EXPECT_EQ(body["auth"]["require_auth_for"], "all");
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootTlsEnabledIncludesTlsMetadataBlock) {
  TlsConfig tls_enabled{};
  tls_enabled.enabled = true;
  tls_enabled.min_version = "1.3";
  auto ctx_tls = make_context(auth_config_, tls_enabled);
  HealthHandlers handlers_tls(ctx_tls, &route_registry_);

  handlers_tls.handle_root(req_, res_);
  auto body = json::parse(res_.body);

  ASSERT_TRUE(body.contains("tls"));
  EXPECT_TRUE(body["tls"]["enabled"].get<bool>());
  EXPECT_EQ(body["tls"]["min_version"], "1.3");
  EXPECT_TRUE(body["capabilities"]["tls"].get<bool>());
}

// --- handle_health discovery block (requires live GatewayNode) ---

static constexpr const char * API_BASE_PATH = "/api/v1";

static int reserve_free_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(sock);
    return 0;
  }
  socklen_t len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

class HealthHandlersLiveTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    int free_port = reserve_free_port();
    ASSERT_NE(free_port, 0) << "Failed to reserve a free port for test";

    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("server.port", free_port)});
    node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(options);

    server_port_ = free_port;

    // Wait for the server to be ready
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    httplib::Client client("127.0.0.1", server_port_);
    const std::string health_ep = std::string(API_BASE_PATH) + "/health";
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (auto res = client.Get(health_ep)) {
        if (res->status == 200) {
          return;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "HTTP server failed to start within timeout";
  }

  void TearDown() override {
    node_.reset();
  }

  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  int server_port_{0};
};

TEST_F(HealthHandlersLiveTest, HealthDiscoveryBlockContainsExpectedFields) {
  httplib::Client client("127.0.0.1", server_port_);
  auto res = client.Get(std::string(API_BASE_PATH) + "/health");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  EXPECT_EQ(body["status"], "healthy");
  EXPECT_TRUE(body.contains("timestamp"));

  // With a live GatewayNode, the discovery block must be present
  ASSERT_TRUE(body.contains("discovery"));
  auto & disc = body["discovery"];

  // Must contain mode and strategy strings
  ASSERT_TRUE(disc.contains("mode"));
  EXPECT_TRUE(disc["mode"].is_string());
  EXPECT_FALSE(disc["mode"].get<std::string>().empty());
  // Default mode is runtime_only
  EXPECT_EQ(disc["mode"].get<std::string>(), "runtime_only");

  ASSERT_TRUE(disc.contains("strategy"));
  EXPECT_TRUE(disc["strategy"].is_string());
  EXPECT_FALSE(disc["strategy"].get<std::string>().empty());

  // In runtime_only mode, pipeline and linking are not present (only in hybrid mode)
  EXPECT_FALSE(disc.contains("pipeline"));
  EXPECT_FALSE(disc.contains("linking"));
}
