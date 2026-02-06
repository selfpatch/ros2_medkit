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

#include <cstring>

#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::handlers;
using json = nlohmann::json;

// =============================================================================
// HandlerContext static method tests (don't require GatewayNode)
// =============================================================================

TEST(HandlerContextStaticTest, SendErrorSetsStatusAndBody) {
  httplib::Response res;

  HandlerContext::send_error(res, httplib::StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Test error message");

  EXPECT_EQ(res.status, 400);
  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");

  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ERR_INVALID_REQUEST);
  EXPECT_EQ(body["message"], "Test error message");
}

TEST(HandlerContextStaticTest, SendErrorWithExtraFields) {
  httplib::Response res;
  json extra = {{"details", "More info"}, {"code", 42}};

  HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Not found", extra);

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

  HandlerContext::send_error(res, httplib::StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Server error");

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

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
