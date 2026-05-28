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

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/auth/auth.hpp"
#include "ros2_medkit_gateway/core/http/handlers/auth_handlers.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::AuthConfigBuilder;
using ros2_medkit_gateway::AuthManager;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND;
using ros2_medkit_gateway::JwtAlgorithm;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::UserRole;
using ros2_medkit_gateway::handlers::AuthHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::http::TypedRequest;

namespace {

// Helper: build a JSON-bodied request and wrap it in a TypedRequest.
httplib::Request make_json_request(const std::string & body) {
  httplib::Request req;
  req.body = body;
  req.headers.emplace("Content-Type", "application/json");
  return req;
}

}  // namespace

// ============================================================================
// Auth Disabled tests
// All three endpoints surface an OAuth2-shaped 404 when authentication is not
// enabled. Per RFC 6749 §5.2, the auth endpoints render errors as
// `{error, error_description}` - the OAuth2 renderer wraps the SOVD error code
// (`resource-not-found`) under the `error` key.
// ============================================================================

class AuthHandlersDisabledTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};  // enabled = false by default
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  AuthHandlers handlers_{ctx_};
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersDisabledTest, AuthorizeReturns404WhenAuthDisabled) {
  httplib::Request req;
  TypedRequest typed_req(req);
  auto result = handlers_.post_authorize(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ERR_RESOURCE_NOT_FOUND);
}

// @verifies REQ_INTEROP_087
TEST_F(AuthHandlersDisabledTest, TokenReturns404WhenAuthDisabled) {
  httplib::Request req;
  TypedRequest typed_req(req);
  auto result = handlers_.post_token(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ERR_RESOURCE_NOT_FOUND);
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersDisabledTest, RevokeReturns404WhenAuthDisabled) {
  httplib::Request req;
  TypedRequest typed_req(req);
  auto result = handlers_.post_revoke(typed_req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ERR_RESOURCE_NOT_FOUND);
}

// ============================================================================
// post_authorize - input validation (auth enabled, null auth_manager)
// All assertions below exercise paths that return before auth_manager is used.
// The route registers `.error_renderer(kOAuth2Error)`, so failures surface as
// `ErrorInfo` whose `code` carries the OAuth2 error identifier (snake_case)
// verbatim - the framework wraps it into `{error, error_description}` at the
// wire boundary.
// ============================================================================

class AuthHandlersAuthorizeTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};

  void SetUp() override {
    auth_.enabled = true;
  }
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForWrongGrantType) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "password", "client_id": "c", "client_secret": "s"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "unsupported_grant_type");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForMissingClientId) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "client_credentials", "client_secret": "s"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForEmptyClientId) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "client_credentials", "client_id": "", "client_secret": "s"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForMissingClientSecret) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "client_credentials", "client_id": "c"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForEmptyClientSecret) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "client_credentials", "client_id": "c", "client_secret": ""})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ErrorCodeIsOAuth2Identifier) {
  // OAuth2 wire shape is enforced by the per-route renderer; the handler
  // carries the OAuth2 identifier in `ErrorInfo::code` so the renderer can
  // emit `{"error": "<identifier>"}` per RFC 6749 §5.2 without rewriting.
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "wrong"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  // OAuth2-shaped identifier (snake_case underscore), not SOVD's
  // `invalid-request` (hyphen).
  EXPECT_EQ(result.error().code, "unsupported_grant_type");
}

// ============================================================================
// post_token - input validation (auth enabled, null auth_manager)
// ============================================================================

class AuthHandlersTokenTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};

  void SetUp() override {
    auth_.enabled = true;
  }
};

// @verifies REQ_INTEROP_087
TEST_F(AuthHandlersTokenTest, ReturnsBadRequestForWrongGrantType) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "client_credentials"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_token(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "unsupported_grant_type");
}

// @verifies REQ_INTEROP_087
TEST_F(AuthHandlersTokenTest, ReturnsBadRequestForMissingRefreshToken) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "refresh_token"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_token(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_087
TEST_F(AuthHandlersTokenTest, ReturnsBadRequestForEmptyRefreshToken) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"grant_type": "refresh_token", "refresh_token": ""})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_token(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// ============================================================================
// post_revoke - input validation (auth enabled, null auth_manager)
// ============================================================================

class AuthHandlersRevokeTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};

  void SetUp() override {
    auth_.enabled = true;
  }
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersRevokeTest, ReturnsBadRequestForInvalidJson) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  httplib::Request raw;
  raw.body = "not valid json {";
  TypedRequest typed_req(raw);
  auto result = handlers.post_revoke(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersRevokeTest, ReturnsBadRequestForMissingTokenField) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"other_field": "value"})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_revoke(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersRevokeTest, ReturnsBadRequestForNonStringToken) {
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto raw = make_json_request(R"({"token": 12345})");
  TypedRequest typed_req(raw);
  auto result = handlers.post_revoke(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
  EXPECT_EQ(result.error().code, "invalid_request");
}

// ============================================================================
// AuthManager integration tests (auth enabled with live manager). Verifies the
// typed handlers return a DTO on success and an OAuth2 `ErrorInfo` on failure.
// ============================================================================

class AuthHandlersWithManagerTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_config_{};
  TlsConfig tls_{};
  std::unique_ptr<AuthManager> auth_manager_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<AuthHandlers> handlers_;

  void SetUp() override {
    auth_config_ = AuthConfigBuilder()
                       .with_enabled(true)
                       .with_jwt_secret("test_secret_key_for_jwt_signing_12345")
                       .with_algorithm(JwtAlgorithm::HS256)
                       .with_token_expiry(3600)
                       .with_refresh_token_expiry(86400)
                       .add_client("test_client", "test_secret", UserRole::ADMIN)
                       .build();

    auth_manager_ = std::make_unique<AuthManager>(auth_config_);
    ctx_ = std::make_unique<HandlerContext>(nullptr, cors_, auth_config_, tls_, auth_manager_.get());
    handlers_ = std::make_unique<AuthHandlers>(*ctx_);
  }

  ros2_medkit_gateway::http::Result<ros2_medkit_gateway::dto::AuthTokenResponse> authorize_admin() {
    auto raw = make_json_request(
        R"({"grant_type": "client_credentials", "client_id": "test_client", "client_secret": "test_secret"})");
    TypedRequest typed_req(raw);
    return handlers_->post_authorize(typed_req);
  }
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersWithManagerTest, AuthorizeReturnsTokensForValidCredentials) {
  auto result = authorize_admin();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->access_token.empty());
  ASSERT_TRUE(result->refresh_token.has_value());
  EXPECT_FALSE(result->refresh_token->empty());
  EXPECT_EQ(result->token_type, "Bearer");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersWithManagerTest, AuthorizeReturnsUnauthorizedForInvalidCredentials) {
  auto raw = make_json_request(
      R"({"grant_type": "client_credentials", "client_id": "test_client", "client_secret": "wrong_secret"})");
  TypedRequest typed_req(raw);
  auto result = handlers_->post_authorize(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 401);
  EXPECT_EQ(result.error().code, "invalid_client");
}

// @verifies REQ_INTEROP_087
TEST_F(AuthHandlersWithManagerTest, TokenReturnsNewAccessTokenForValidRefreshToken) {
  auto authorized = authorize_admin();
  ASSERT_TRUE(authorized.has_value());
  ASSERT_TRUE(authorized->refresh_token.has_value());
  const std::string refresh_token = *authorized->refresh_token;

  auto raw = make_json_request(json({{"grant_type", "refresh_token"}, {"refresh_token", refresh_token}}).dump());
  TypedRequest typed_req(raw);
  auto result = handlers_->post_token(typed_req);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->access_token.empty());
  EXPECT_EQ(result->token_type, "Bearer");
  ASSERT_TRUE(result->refresh_token.has_value());
  EXPECT_EQ(*result->refresh_token, refresh_token);
}

// @verifies REQ_INTEROP_087
TEST_F(AuthHandlersWithManagerTest, TokenReturnsUnauthorizedForInvalidRefreshToken) {
  auto raw = make_json_request(R"({"grant_type": "refresh_token", "refresh_token": "not.a.valid.refresh.token"})");
  TypedRequest typed_req(raw);
  auto result = handlers_->post_token(typed_req);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 401);
  EXPECT_EQ(result.error().code, "invalid_grant");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersWithManagerTest, RevokeRevokesRefreshTokenForSubsequentTokenRequest) {
  auto authorized = authorize_admin();
  ASSERT_TRUE(authorized.has_value());
  ASSERT_TRUE(authorized->refresh_token.has_value());
  const std::string refresh_token = *authorized->refresh_token;

  auto revoke_raw = make_json_request(json({{"token", refresh_token}}).dump());
  TypedRequest revoke_req(revoke_raw);
  auto revoke_result = handlers_->post_revoke(revoke_req);

  ASSERT_TRUE(revoke_result.has_value());
  EXPECT_EQ(revoke_result->status, "revoked");

  auto token_raw = make_json_request(json({{"grant_type", "refresh_token"}, {"refresh_token", refresh_token}}).dump());
  TypedRequest token_req(token_raw);
  auto token_result = handlers_->post_token(token_req);

  ASSERT_FALSE(token_result.has_value());
  EXPECT_EQ(token_result.error().http_status, 401);
  EXPECT_EQ(token_result.error().code, "invalid_grant");
}
