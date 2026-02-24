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

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/http/handlers/auth_handlers.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::AuthHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;

namespace {

// Helper: build a request with a JSON body and Content-Type header
httplib::Request make_json_request(const std::string & body)
{
  httplib::Request req;
  req.body = body;
  req.headers.emplace("Content-Type", "application/json");
  return req;
}

// Helper: build HandlerContext with auth disabled (default)
HandlerContext make_ctx_auth_disabled(CorsConfig & cors, AuthConfig & auth, TlsConfig & tls)
{
  auth.enabled = false;
  return HandlerContext(nullptr, cors, auth, tls, nullptr);
}

// Helper: build HandlerContext with auth enabled, no live auth_manager.
// Safe for tests that exercise input-validation paths which return before
// calling ctx_.auth_manager()->authenticate().
HandlerContext make_ctx_auth_enabled(CorsConfig & cors, AuthConfig & auth, TlsConfig & tls)
{
  auth.enabled = true;
  return HandlerContext(nullptr, cors, auth, tls, nullptr);
}

}  // namespace

// ============================================================================
// Auth Disabled tests
// All three endpoints return 404 when authentication is not enabled.
// ============================================================================

class AuthHandlersDisabledTest : public ::testing::Test
{
protected:
  CorsConfig cors_{};
  AuthConfig auth_{};   // enabled = false by default
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  AuthHandlers handlers_{ctx_};
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersDisabledTest, AuthorizeReturns404WhenAuthDisabled)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_auth_authorize(req, res);
  EXPECT_EQ(res.status, 404);
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersDisabledTest, AuthorizeErrorBodyContainsErrorCode)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_auth_authorize(req, res);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND);
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersDisabledTest, TokenReturns404WhenAuthDisabled)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_auth_token(req, res);
  EXPECT_EQ(res.status, 404);
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersDisabledTest, RevokeReturns404WhenAuthDisabled)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_auth_revoke(req, res);
  EXPECT_EQ(res.status, 404);
}

// ============================================================================
// handle_auth_authorize — input validation (auth enabled, null auth_manager)
// All assertions below exercise paths that return before auth_manager is used.
// ============================================================================

class AuthHandlersAuthorizeTest : public ::testing::Test
{
protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};

  void SetUp() override
  {
    auth_.enabled = true;
  }
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForWrongGrantType)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "password", "client_id": "c", "client_secret": "s"})");
  httplib::Response res;
  handlers.handle_auth_authorize(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "unsupported_grant_type");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForMissingClientId)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "client_credentials", "client_secret": "s"})");
  httplib::Response res;
  handlers.handle_auth_authorize(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForEmptyClientId)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "client_credentials", "client_id": "", "client_secret": "s"})");
  httplib::Response res;
  handlers.handle_auth_authorize(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, ReturnsBadRequestForMissingClientSecret)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "client_credentials", "client_id": "c"})");
  httplib::Response res;
  handlers.handle_auth_authorize(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersAuthorizeTest, AuthorizeErrorBodyFollowsOAuth2Format)
{
  // Verify that error responses follow RFC 6749 OAuth2 error format
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "wrong"})");
  httplib::Response res;
  handlers.handle_auth_authorize(req, res);

  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("error"));
  EXPECT_TRUE(body.contains("error_description"));
}

// ============================================================================
// handle_auth_token — input validation (auth enabled, null auth_manager)
// ============================================================================

class AuthHandlersTokenTest : public ::testing::Test
{
protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};

  void SetUp() override
  {
    auth_.enabled = true;
  }
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersTokenTest, ReturnsBadRequestForWrongGrantType)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "client_credentials"})");
  httplib::Response res;
  handlers.handle_auth_token(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "unsupported_grant_type");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersTokenTest, ReturnsBadRequestForMissingRefreshToken)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "refresh_token"})");
  httplib::Response res;
  handlers.handle_auth_token(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersTokenTest, ReturnsBadRequestForEmptyRefreshToken)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"grant_type": "refresh_token", "refresh_token": ""})");
  httplib::Response res;
  handlers.handle_auth_token(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// ============================================================================
// handle_auth_revoke — input validation (auth enabled, null auth_manager)
// ============================================================================

class AuthHandlersRevokeTest : public ::testing::Test
{
protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};

  void SetUp() override
  {
    auth_.enabled = true;
  }
};

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersRevokeTest, ReturnsBadRequestForInvalidJson)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  httplib::Request req;
  req.body = "not valid json {";
  httplib::Response res;
  handlers.handle_auth_revoke(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersRevokeTest, ReturnsBadRequestForMissingTokenField)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"other_field": "value"})");
  httplib::Response res;
  handlers.handle_auth_revoke(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthHandlersRevokeTest, ReturnsBadRequestForNonStringToken)
{
  HandlerContext ctx(nullptr, cors_, auth_, tls_, nullptr);
  AuthHandlers handlers(ctx);

  auto req = make_json_request(R"({"token": 12345})");
  httplib::Response res;
  handlers.handle_auth_revoke(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_request");
}
