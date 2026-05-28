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
#include <functional>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>

#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/core/http/handlers/health_handlers.hpp"
#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using namespace std::chrono_literals;

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::dto::JsonWriter;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::HealthHandlers;
using ros2_medkit_gateway::http::TypedRequest;
using ros2_medkit_gateway::openapi::RouteRegistry;

namespace dto = ros2_medkit_gateway::dto;

// HealthHandlers has no dependency on GatewayNode or AuthManager:
// - get_health builds dto::Health (free-standing)
// - get_version_info builds dto::VersionInfo (free-standing)
// - get_root reads ctx_.auth_config() and ctx_.tls_config() (both disabled by default)
// All tests use a null GatewayNode and null AuthManager which is safe for these handlers.

namespace {

// Typed seed handlers used to populate a test route registry with the routes
// `handle_root` enumerates. The handler bodies are never invoked - the tests
// only inspect the registry's endpoint list.
ros2_medkit_gateway::http::Result<dto::Health> noop_get_health(ros2_medkit_gateway::http::TypedRequest /*req*/) {
  return dto::Health{};
}

ros2_medkit_gateway::http::Result<dto::Health> noop_post_health(ros2_medkit_gateway::http::TypedRequest /*req*/,
                                                                const dto::Health & /*body*/) {
  return dto::Health{};
}

void seed_get(RouteRegistry & reg, const std::string & path, const std::string & tag, const std::string & summary) {
  std::function<ros2_medkit_gateway::http::Result<dto::Health>(ros2_medkit_gateway::http::TypedRequest)> h =
      &noop_get_health;
  reg.get<dto::Health>(path, std::move(h)).tag(tag).summary(summary);
}

void seed_post(RouteRegistry & reg, const std::string & path, const std::string & tag) {
  std::function<ros2_medkit_gateway::http::Result<dto::Health>(ros2_medkit_gateway::http::TypedRequest, dto::Health)>
      h = &noop_post_health;
  reg.post<dto::Health, dto::Health>(path, std::move(h)).tag(tag);
}

// Populate a test route registry with representative routes
void populate_test_routes(RouteRegistry & reg) {
  seed_get(reg, "/health", "Server", "Health check");
  seed_get(reg, "/", "Server", "API overview");
  seed_get(reg, "/version-info", "Server", "SOVD version information");
  seed_get(reg, "/areas", "Discovery", "List areas");
  seed_get(reg, "/apps", "Discovery", "List apps");
  seed_get(reg, "/components", "Discovery", "List components");
  seed_get(reg, "/functions", "Discovery", "List functions");
  seed_get(reg, "/faults", "Faults", "List all faults");
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
  TypedRequest typed_req_{req_};

  void SetUp() override {
    populate_test_routes(route_registry_);
  }

  HandlerContext make_context(const AuthConfig & auth, const TlsConfig & tls) {
    return HandlerContext(nullptr, cors_config_, auth, tls, nullptr);
  }
};

// --- get_health ---

TEST_F(HealthHandlersTest, HandleHealthResponseContainsStatusHealthy) {
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "healthy");
}

TEST_F(HealthHandlersTest, HandleHealthNullNodeOmitsDiscovery) {
  // ctx_ uses nullptr for GatewayNode, so discovery info should not be present
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "healthy");
  EXPECT_FALSE(result->discovery.has_value());
}

TEST_F(HealthHandlersTest, HandleHealthResponseContainsTimestamp) {
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result->timestamp, 0);
}

TEST_F(HealthHandlersTest, HandleHealthResponseIsValidJson) {
  auto result = handlers_.get_health(typed_req_);
  ASSERT_TRUE(result.has_value());
  // The DTO writer must produce a valid JSON object for the success body.
  auto body = JsonWriter<dto::Health>::write(result.value());
  EXPECT_TRUE(body.is_object());
  EXPECT_EQ(body["status"], "healthy");
}

// --- get_version_info ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoContainsItemsArray) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->items.empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasVersionField) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->items.empty());
  EXPECT_FALSE(result->items[0].version.empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasBaseUri) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->items.empty());
  EXPECT_FALSE(result->items[0].base_uri.empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoItemsEntryHasVendorInfo) {
  auto result = handlers_.get_version_info(typed_req_);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->items.empty());
  ASSERT_TRUE(result->items[0].vendor_info.has_value());
  EXPECT_EQ(result->items[0].vendor_info->name, "ros2_medkit");
}

// --- get_root ---

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootResponseContainsRequiredTopLevelFields) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->name.empty());
  EXPECT_FALSE(result->version.empty());
  EXPECT_FALSE(result->api_base.empty());
  // endpoints + capabilities are required by the DTO schema; non-emptiness of
  // endpoints is checked by the dedicated test below.
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootEndpointsIsNonEmptyArray) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->endpoints.empty());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesContainsDiscovery) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->capabilities.discovery);
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthDisabledNoAuthEndpoints) {
  // With auth disabled (default), auth endpoints must not appear in the list
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  for (const auto & ep : result->endpoints) {
    EXPECT_EQ(ep.find("/auth/"), std::string::npos) << "Unexpected auth endpoint when auth is disabled: " << ep;
  }
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesAuthDisabled) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->capabilities.authentication);
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesTlsDisabled) {
  auto result = handlers_.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->capabilities.tls);
  EXPECT_FALSE(result->tls.has_value());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledAddsAuthEndpoints) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auto ctx_auth = make_context(auth_enabled, tls_config_);

  // Create registry with auth routes
  RouteRegistry auth_reg;
  populate_test_routes(auth_reg);
  seed_post(auth_reg, "/auth/authorize", "Authentication");
  seed_post(auth_reg, "/auth/token", "Authentication");
  seed_post(auth_reg, "/auth/revoke", "Authentication");

  HealthHandlers handlers_auth(ctx_auth, &auth_reg);

  auto result = handlers_auth.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());

  bool has_auth_endpoint = false;
  for (const auto & ep : result->endpoints) {
    if (ep.find("/auth/") != std::string::npos) {
      has_auth_endpoint = true;
      break;
    }
  }
  EXPECT_TRUE(has_auth_endpoint);
  EXPECT_TRUE(result->capabilities.authentication);
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledIncludesAuthMetadataBlock) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auth_enabled.require_auth_for = ros2_medkit_gateway::AuthRequirement::ALL;
  auth_enabled.jwt_algorithm = ros2_medkit_gateway::JwtAlgorithm::HS256;
  auto ctx_auth = make_context(auth_enabled, tls_config_);
  HealthHandlers handlers_auth(ctx_auth, &route_registry_);

  auto result = handlers_auth.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());

  ASSERT_TRUE(result->auth.has_value());
  EXPECT_TRUE(result->auth->enabled);
  EXPECT_EQ(result->auth->algorithm, "HS256");
  EXPECT_EQ(result->auth->require_auth_for, "all");
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootTlsEnabledIncludesTlsMetadataBlock) {
  TlsConfig tls_enabled{};
  tls_enabled.enabled = true;
  tls_enabled.min_version = "1.3";
  auto ctx_tls = make_context(auth_config_, tls_enabled);
  HealthHandlers handlers_tls(ctx_tls, &route_registry_);

  auto result = handlers_tls.get_root(typed_req_);
  ASSERT_TRUE(result.has_value());

  ASSERT_TRUE(result->tls.has_value());
  EXPECT_TRUE(result->tls->enabled);
  EXPECT_EQ(result->tls->min_version, "1.3");
  EXPECT_TRUE(result->capabilities.tls);
}

// --- live discovery block (requires live GatewayNode + real HTTP server) ---

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
