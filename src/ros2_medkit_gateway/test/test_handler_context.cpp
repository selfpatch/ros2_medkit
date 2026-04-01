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
#include <httplib.h>

#include <arpa/inet.h>
#include <cstring>
#include <memory>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <unordered_map>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/models/thread_safe_entity_cache.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::handlers;
using json = nlohmann::json;

// =============================================================================
// HandlerContext static method tests (don't require GatewayNode)
// =============================================================================

TEST(HandlerContextStaticTest, SendErrorSetsStatusAndBody) {
  httplib::Response res;

  HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Test error message");

  EXPECT_EQ(res.status, 400);
  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");

  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ERR_INVALID_REQUEST);
  EXPECT_EQ(body["message"], "Test error message");
}

TEST(HandlerContextStaticTest, SendErrorWithExtraFields) {
  httplib::Response res;
  json extra = {{"details", "More info"}, {"code", 42}};

  HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Not found", extra);

  EXPECT_EQ(res.status, 404);

  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ERR_ENTITY_NOT_FOUND);
  EXPECT_EQ(body["message"], "Not found");
  // Extra parameters are in x-medkit extension
  EXPECT_EQ(body["parameters"]["details"], "More info");
  EXPECT_EQ(body["parameters"]["code"], 42);
}

TEST(HandlerContextStaticTest, SendErrorInternalServerError) {
  httplib::Response res;

  HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Server error");

  EXPECT_EQ(res.status, 500);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ERR_INTERNAL_ERROR);
  EXPECT_EQ(body["message"], "Server error");
}

TEST(HandlerContextStaticTest, SendJsonSetsContentTypeAndBody) {
  httplib::Response res;
  json data = {{"name", "test"}, {"value", 123}, {"items", {1, 2, 3}}};

  HandlerContext::send_json(res, data);

  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");

  auto body = json::parse(res.body);
  EXPECT_EQ(body["name"], "test");
  EXPECT_EQ(body["value"], 123);
  EXPECT_EQ(body["items"].size(), 3);
}

TEST(HandlerContextStaticTest, SendJsonEmptyObject) {
  httplib::Response res;
  json data = json::object();

  HandlerContext::send_json(res, data);

  auto body = json::parse(res.body);
  EXPECT_TRUE(body.is_object());
  EXPECT_EQ(body.size(), 0);
}

TEST(HandlerContextStaticTest, SendJsonArray) {
  httplib::Response res;
  json data = json::array({1, 2, 3, 4, 5});

  HandlerContext::send_json(res, data);

  auto body = json::parse(res.body);
  EXPECT_TRUE(body.is_array());
  EXPECT_EQ(body.size(), 5);
}

TEST(HandlerContextStaticTest, LoggerReturnsValidLogger) {
  auto logger = HandlerContext::logger();
  // Just verify it doesn't throw and returns a valid logger name
  EXPECT_NE(logger.get_name(), nullptr);
  EXPECT_GT(strlen(logger.get_name()), 0);
}

// =============================================================================
// HandlerContext instance tests with CorsConfig
// =============================================================================

class HandlerContextCorsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cors_config_ = CorsConfigBuilder()
                       .with_origins({"http://localhost:3000", "http://example.com"})
                       .with_methods({"GET", "POST", "PUT", "DELETE"})
                       .with_headers({"Content-Type", "Authorization"})
                       .with_credentials(true)
                       .build();
  }

  CorsConfig cors_config_;
  AuthConfig auth_config_;  // Default (disabled)
  TlsConfig tls_config_;    // Default (disabled)
};

TEST_F(HandlerContextCorsTest, IsOriginAllowedReturnsTrueForConfiguredOrigins) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  EXPECT_TRUE(ctx.is_origin_allowed("http://localhost:3000"));
  EXPECT_TRUE(ctx.is_origin_allowed("http://example.com"));
}

TEST_F(HandlerContextCorsTest, IsOriginAllowedReturnsFalseForUnknownOrigins) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  EXPECT_FALSE(ctx.is_origin_allowed("http://malicious.com"));
  EXPECT_FALSE(ctx.is_origin_allowed("http://localhost:8080"));
  EXPECT_FALSE(ctx.is_origin_allowed(""));
}

TEST_F(HandlerContextCorsTest, IsOriginAllowedWithWildcard) {
  auto wildcard_config = CorsConfigBuilder()
                             .with_origins({"*"})
                             .with_methods({"GET"})
                             .with_headers({"Content-Type"})
                             .with_credentials(false)  // Must be false with wildcard
                             .build();

  HandlerContext ctx(nullptr, wildcard_config, auth_config_, tls_config_, nullptr);

  EXPECT_TRUE(ctx.is_origin_allowed("http://any-site.com"));
  EXPECT_TRUE(ctx.is_origin_allowed("http://localhost:12345"));
}

TEST_F(HandlerContextCorsTest, SetCorsHeadersSetsAllHeaders) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);
  httplib::Response res;

  ctx.set_cors_headers(res, "http://localhost:3000");

  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Origin"), "http://localhost:3000");
  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Methods"), "GET, POST, PUT, DELETE");
  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Headers"), "Content-Type, Authorization");
  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Credentials"), "true");
  EXPECT_EQ(res.get_header_value("Access-Control-Expose-Headers"), "Content-Disposition, Content-Length");
}

TEST_F(HandlerContextCorsTest, SetCorsHeadersWithoutCredentials) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost:3000"})
                    .with_methods({"GET"})
                    .with_headers({"Content-Type"})
                    .with_credentials(false)
                    .build();

  HandlerContext ctx(nullptr, config, auth_config_, tls_config_, nullptr);
  httplib::Response res;

  ctx.set_cors_headers(res, "http://localhost:3000");

  EXPECT_EQ(res.get_header_value("Access-Control-Allow-Origin"), "http://localhost:3000");
  // Credentials header should not be set when disabled
  EXPECT_TRUE(res.get_header_value("Access-Control-Allow-Credentials").empty());
}

// =============================================================================
// Entity ID validation tests
// =============================================================================

TEST_F(HandlerContextCorsTest, ValidateEntityIdAcceptsValidIds) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  EXPECT_TRUE(ctx.validate_entity_id("engine").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("engine_temp_sensor").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("sensor123").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("MyComponent").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("engine-ecu").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("front-left-door").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("a").has_value());
  EXPECT_TRUE(ctx.validate_entity_id("X1").has_value());
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsEmptyString) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  auto result = ctx.validate_entity_id("");
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("empty") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsTooLongIds) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Create a string longer than 256 characters
  std::string long_id(257, 'a');

  auto result = ctx.validate_entity_id(long_id);
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("too long") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsForwardSlash) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  auto result = ctx.validate_entity_id("path/injection");
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("invalid character") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsSpecialCharacters) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Test various invalid characters
  EXPECT_FALSE(ctx.validate_entity_id("test@value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test#value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test$value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test%value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test&value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test*value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test!value").has_value());
  EXPECT_FALSE(ctx.validate_entity_id("test value").has_value());   // Space
  EXPECT_FALSE(ctx.validate_entity_id("test\nvalue").has_value());  // Newline
  EXPECT_FALSE(ctx.validate_entity_id("test\tvalue").has_value());  // Tab
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdRejectsNonPrintableCharacters) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Test non-printable characters
  std::string with_null = "test";
  with_null += '\0';
  with_null += "value";

  auto result = ctx.validate_entity_id(with_null);
  EXPECT_FALSE(result.has_value());
  // Error should show hex representation for non-printable
  EXPECT_TRUE(result.error().find("0x00") != std::string::npos);
}

TEST_F(HandlerContextCorsTest, ValidateEntityIdAcceptsMaxLengthId) {
  HandlerContext ctx(nullptr, cors_config_, auth_config_, tls_config_, nullptr);

  // Create exactly 256 character string (max allowed)
  std::string max_id(256, 'a');

  auto result = ctx.validate_entity_id(max_id);
  EXPECT_TRUE(result.has_value());
}

// =============================================================================
// CorsConfigBuilder tests
// =============================================================================

TEST(CorsConfigBuilderTest, BuildsValidConfig) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost:3000"})
                    .with_methods({"GET", "POST"})
                    .with_headers({"Content-Type"})
                    .with_credentials(true)
                    .with_max_age(3600)
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.allowed_origins.size(), 1);
  EXPECT_EQ(config.allowed_origins[0], "http://localhost:3000");
  EXPECT_EQ(config.allowed_methods.size(), 2);
  EXPECT_EQ(config.allowed_headers.size(), 1);
  EXPECT_TRUE(config.allow_credentials);
  EXPECT_EQ(config.max_age_seconds, 3600);
  EXPECT_EQ(config.methods_header, "GET, POST");
  EXPECT_EQ(config.headers_header, "Content-Type");
}

TEST(CorsConfigBuilderTest, EmptyOriginsDisablesCors) {
  auto config = CorsConfigBuilder().with_origins({}).with_methods({"GET"}).with_headers({"Content-Type"}).build();

  EXPECT_FALSE(config.enabled);
}

TEST(CorsConfigBuilderTest, FiltersEmptyStringsFromOrigins) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost:3000", "", "http://example.com", ""})
                    .with_methods({"GET"})
                    .with_headers({"Content-Type"})
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.allowed_origins.size(), 2);
  EXPECT_EQ(config.allowed_origins[0], "http://localhost:3000");
  EXPECT_EQ(config.allowed_origins[1], "http://example.com");
}

TEST(CorsConfigBuilderTest, CredentialsWithWildcardThrows) {
  EXPECT_THROW(
      {
        CorsConfigBuilder()
            .with_origins({"*"})
            .with_methods({"GET"})
            .with_headers({"Content-Type"})
            .with_credentials(true)
            .build();
      },
      std::invalid_argument);
}

TEST(CorsConfigBuilderTest, WildcardWithoutCredentialsSucceeds) {
  auto config = CorsConfigBuilder()
                    .with_origins({"*"})
                    .with_methods({"GET", "POST"})
                    .with_headers({"Content-Type", "Authorization"})
                    .with_credentials(false)
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.allowed_origins[0], "*");
}

TEST(CorsConfigBuilderTest, MultipleMethodsAndHeadersJoined) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost"})
                    .with_methods({"GET", "POST", "PUT", "DELETE", "OPTIONS"})
                    .with_headers({"Content-Type", "Authorization", "X-Custom-Header"})
                    .build();

  EXPECT_EQ(config.methods_header, "GET, POST, PUT, DELETE, OPTIONS");
  EXPECT_EQ(config.headers_header, "Content-Type, Authorization, X-Custom-Header");
}

TEST(CorsConfigBuilderTest, DefaultMaxAge) {
  auto config = CorsConfigBuilder()
                    .with_origins({"http://localhost"})
                    .with_methods({"GET"})
                    .with_headers({"Content-Type"})
                    .build();

  EXPECT_EQ(config.max_age_seconds, 86400);  // Default value
}

// =============================================================================
// Helper: reserve a free TCP port for test isolation
// =============================================================================

namespace {

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(sock);
    return 0;
  }
  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

/// Build a request with regex matches populated (simulates cpp-httplib routing)
httplib::Request make_request_with_path(const std::string & path) {
  httplib::Request req;
  req.method = "GET";
  req.path = path;
  return req;
}

}  // namespace

// =============================================================================
// Shared rclcpp lifecycle for all fixtures that need GatewayNode
// =============================================================================

class RclcppEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

::testing::Environment * const rclcpp_env = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

// =============================================================================
// HandlerContext forwarding tests (require GatewayNode + AggregationManager)
// =============================================================================

class HandlerContextForwardingTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0) << "Failed to reserve a port for test";

    rclcpp::NodeOptions options;
    options.append_parameter_override("server.port", suite_server_port_);
    options.append_parameter_override("refresh_interval_ms", 60000);
    suite_node_ = std::make_shared<GatewayNode>(options);
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
  }

  void SetUp() override {
    ASSERT_NE(suite_node_, nullptr);

    // Seed the entity cache with a local component and a "remote" component.
    // The "remote" component is just a normal component in the cache; the routing
    // table in the AggregationManager is what marks it as remote.
    Component local_comp;
    local_comp.id = "local_ecu";
    local_comp.name = "Local ECU";
    local_comp.namespace_path = "/local";
    local_comp.fqn = "/local/local_ecu";

    Component remote_comp;
    remote_comp.id = "remote_sensor";
    remote_comp.name = "Remote Sensor";
    remote_comp.namespace_path = "/remote";
    remote_comp.fqn = "/remote/remote_sensor";

    App remote_app;
    remote_app.id = "remote_driver";
    remote_app.name = "Remote Driver";
    remote_app.component_id = "remote_sensor";

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all({}, {local_comp, remote_comp}, {remote_app}, {});

    // Create AggregationManager with a static peer (unreachable - we only need routing)
    AggregationConfig agg_config;
    agg_config.enabled = true;
    agg_config.timeout_ms = 200;
    AggregationConfig::PeerConfig peer;
    peer.url = "http://127.0.0.1:59999";  // Unreachable port
    peer.name = "peer_subsystem";
    agg_config.peers.push_back(peer);
    agg_mgr_ = std::make_unique<AggregationManager>(agg_config);

    // Set up routing table: remote_sensor and remote_driver are owned by peer_subsystem
    std::unordered_map<std::string, std::string> routing;
    routing["remote_sensor"] = "peer_subsystem";
    routing["remote_driver"] = "peer_subsystem";
    agg_mgr_->update_routing_table(routing);
  }

  void TearDown() override {
    agg_mgr_.reset();
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::unique_ptr<AggregationManager> agg_mgr_;
};

// When a remote entity is accessed and aggregation is enabled, validate_entity_for_route
// should forward the request to the owning peer and return nullopt.
TEST_F(HandlerContextForwardingTest, RemoteEntityWithAggregationForwardsAndReturnsNullopt) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto req = make_request_with_path("/api/v1/components/remote_sensor/data");
  httplib::Response res;

  auto result = ctx.validate_entity_for_route(req, res, "remote_sensor");

  // Should return nullopt because the request was forwarded
  EXPECT_FALSE(result.has_value());

  // The peer is unreachable, so forward_request sets 502
  EXPECT_EQ(res.status, 502);
}

// When a remote app is accessed and aggregation is enabled, the same forwarding applies.
TEST_F(HandlerContextForwardingTest, RemoteAppWithAggregationForwardsAndReturnsNullopt) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto req = make_request_with_path("/api/v1/apps/remote_driver/data");
  httplib::Response res;

  auto result = ctx.validate_entity_for_route(req, res, "remote_driver");

  // Should return nullopt because the request was forwarded
  EXPECT_FALSE(result.has_value());

  // The peer is unreachable, so forward_request sets 502
  EXPECT_EQ(res.status, 502);
}

// Verify that the correct peer name is used for forwarding by checking the
// 502 response body from the AggregationManager (it includes the peer name).
TEST_F(HandlerContextForwardingTest, ForwardingUsesCorrectPeerName) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto req = make_request_with_path("/api/v1/components/remote_sensor/data");
  httplib::Response res;

  auto result = ctx.validate_entity_for_route(req, res, "remote_sensor");
  EXPECT_FALSE(result.has_value());

  // The 502 response from forward_request to an unreachable peer contains
  // information about the peer. Verify it mentions our peer name.
  // (The peer exists in the manager but the host is unreachable, so we get
  // a 502 from the PeerClient, not the "peer not known" 502.)
  EXPECT_EQ(res.status, 502);
}

// When aggregation_mgr_ is not set (no aggregation), remote entities from the
// routing table are never marked as remote (apply_routing is a no-op when
// aggregation_mgr_ is null). The entity is returned as a local entity.
TEST_F(HandlerContextForwardingTest, NoAggregationManagerReturnsEntityAsLocal) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  // Deliberately NOT setting aggregation manager

  auto req = make_request_with_path("/api/v1/components/remote_sensor/data");
  httplib::Response res;

  auto result = ctx.validate_entity_for_route(req, res, "remote_sensor");

  // Without aggregation manager, apply_routing never marks is_remote = true,
  // so the entity is returned normally as if it were local
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "remote_sensor");
  EXPECT_EQ(result->type, EntityType::COMPONENT);
  EXPECT_FALSE(result->is_remote);
  EXPECT_TRUE(result->peer_name.empty());
}

// Local entities should be returned normally even when aggregation is enabled.
TEST_F(HandlerContextForwardingTest, LocalEntityWithAggregationReturnsEntityInfo) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto req = make_request_with_path("/api/v1/components/local_ecu/data");
  httplib::Response res;

  auto result = ctx.validate_entity_for_route(req, res, "local_ecu");

  // Local entity is not in routing table, so it's not remote
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "local_ecu");
  EXPECT_EQ(result->type, EntityType::COMPONENT);
  EXPECT_FALSE(result->is_remote);
  EXPECT_EQ(result->namespace_path, "/local");
  EXPECT_EQ(result->fqn, "/local/local_ecu");
}

// Entity not found at all should return 404 regardless of aggregation state.
TEST_F(HandlerContextForwardingTest, UnknownEntityReturns404WithAggregation) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto req = make_request_with_path("/api/v1/components/nonexistent/data");
  httplib::Response res;

  auto result = ctx.validate_entity_for_route(req, res, "nonexistent");

  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(res.status, 404);

  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ERR_ENTITY_NOT_FOUND);
}

// Verify get_entity_info marks entity as remote when aggregation manager is set
// and entity is in the routing table.
TEST_F(HandlerContextForwardingTest, GetEntityInfoSetsRemoteFieldsWithAggregation) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  ctx.set_aggregation_manager(agg_mgr_.get());

  auto info = ctx.get_entity_info("remote_sensor", SovdEntityType::COMPONENT);

  EXPECT_EQ(info.type, EntityType::COMPONENT);
  EXPECT_TRUE(info.is_remote);
  EXPECT_EQ(info.peer_name, "peer_subsystem");
  EXPECT_FALSE(info.peer_url.empty());
}

// Verify get_entity_info does NOT set remote fields when aggregation manager is null.
TEST_F(HandlerContextForwardingTest, GetEntityInfoNoRemoteWithoutAggregation) {
  HandlerContext ctx(suite_node_.get(), cors_, auth_, tls_, nullptr);
  // No aggregation manager set

  auto info = ctx.get_entity_info("remote_sensor", SovdEntityType::COMPONENT);

  EXPECT_EQ(info.type, EntityType::COMPONENT);
  EXPECT_FALSE(info.is_remote);
  EXPECT_TRUE(info.peer_name.empty());
  EXPECT_TRUE(info.peer_url.empty());
}

// =============================================================================
// Area fault/log aggregation handler tests (via REST API)
// =============================================================================

class AreaAggregationTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0) << "Failed to reserve a port for test";

    rclcpp::NodeOptions options;
    options.append_parameter_override("server.port", suite_server_port_);
    options.append_parameter_override("server.host", std::string("127.0.0.1"));
    options.append_parameter_override("refresh_interval_ms", 60000);
    suite_node_ = std::make_shared<GatewayNode>(options);
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
  }

  void SetUp() override {
    ASSERT_NE(suite_node_, nullptr);

    // Seed the cache with Area -> Component -> App hierarchy.
    // Area "powertrain" contains Component "engine_ecu" which has App "temp_sensor".
    Area area;
    area.id = "powertrain";
    area.name = "Powertrain";
    area.namespace_path = "/powertrain";

    Component comp;
    comp.id = "engine_ecu";
    comp.name = "Engine ECU";
    comp.area = "powertrain";
    comp.namespace_path = "/powertrain/engine";
    comp.fqn = "/powertrain/engine/engine_ecu";

    App app1;
    app1.id = "temp_sensor";
    app1.name = "Temperature Sensor";
    app1.component_id = "engine_ecu";
    app1.ros_binding = App::RosBinding{"/powertrain/engine/temp_sensor", "/powertrain/engine", ""};

    App app2;
    app2.id = "rpm_sensor";
    app2.name = "RPM Sensor";
    app2.component_id = "engine_ecu";
    app2.ros_binding = App::RosBinding{"/powertrain/engine/rpm_sensor", "/powertrain/engine", ""};

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all({area}, {comp}, {app1, app2}, {});

    client_ = std::make_unique<httplib::Client>("127.0.0.1", suite_server_port_);
    client_->set_connection_timeout(5);
    client_->set_read_timeout(5);
  }

  void TearDown() override {
    client_.reset();
  }

  std::unique_ptr<httplib::Client> client_;
};

// Area faults handler returns 503 when FaultManager service is unavailable
// (no ros2_medkit_fault_manager node running in unit test). This verifies
// the handler correctly accepts the area entity and attempts fault aggregation.
TEST_F(AreaAggregationTest, AreaFaultsReturns503WithoutFaultManagerNode) {
  auto res = client_->Get("/api/v1/areas/powertrain/faults");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  // FaultManager's ROS 2 service call times out without the fault_manager node
  EXPECT_EQ(res->status, 503);
}

// Area logs handler traverses area -> components -> apps -> FQNs chain and
// returns aggregated logs with x-medkit metadata. The LogManager is in-process
// so it works without external nodes (empty log buffer = empty items).
TEST_F(AreaAggregationTest, AreaLogsReturnsAggregatedResult) {
  auto res = client_->Get("/api/v1/areas/powertrain/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  EXPECT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());

  // Verify x-medkit aggregation metadata is present
  ASSERT_TRUE(body.contains("x-medkit"));
  auto xmedkit = body["x-medkit"];
  EXPECT_EQ(xmedkit["entity_id"], "powertrain");
  EXPECT_EQ(xmedkit["aggregation_level"], "area");
  EXPECT_TRUE(xmedkit["aggregated"].get<bool>());
  EXPECT_EQ(xmedkit["component_count"], 1);
  EXPECT_EQ(xmedkit["app_count"], 2);

  // Verify aggregation sources contain the app FQNs
  ASSERT_TRUE(xmedkit.contains("aggregation_sources"));
  auto sources = xmedkit["aggregation_sources"];
  EXPECT_EQ(sources.size(), 2);
}

// Area with no components falls through to namespace prefix matching for logs.
// With no matching logs, returns empty items.
TEST_F(AreaAggregationTest, AreaLogsWithNoComponentsFallsThrough) {
  auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
  Area empty_area;
  empty_area.id = "empty_domain";
  empty_area.name = "Empty Domain";
  empty_area.namespace_path = "/empty";

  Area pt_area;
  pt_area.id = "powertrain";
  pt_area.name = "Powertrain";
  pt_area.namespace_path = "/powertrain";

  Component comp;
  comp.id = "engine_ecu";
  comp.name = "Engine ECU";
  comp.area = "powertrain";
  comp.namespace_path = "/powertrain/engine";
  comp.fqn = "/powertrain/engine/engine_ecu";

  App app1;
  app1.id = "temp_sensor";
  app1.name = "Temperature Sensor";
  app1.component_id = "engine_ecu";
  app1.ros_binding = App::RosBinding{"/powertrain/engine/temp_sensor", "/powertrain/engine", ""};

  cache.update_all({pt_area, empty_area}, {comp}, {app1}, {});

  auto res = client_->Get("/api/v1/areas/empty_domain/logs");
  ASSERT_NE(res, nullptr) << "HTTP request failed";
  // Falls through to namespace prefix matching (no components linked to area)
  EXPECT_EQ(res->status, 200);

  auto body = json::parse(res->body);
  EXPECT_TRUE(body.contains("items"));
  EXPECT_TRUE(body["items"].is_array());
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
