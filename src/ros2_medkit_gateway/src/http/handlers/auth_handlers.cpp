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

#include "ros2_medkit_gateway/core/http/handlers/auth_handlers.hpp"

#include <string>
#include <utility>

#include "ros2_medkit_gateway/core/auth/auth_models.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Build an OAuth2-shaped ErrorInfo. The framework's `kOAuth2Error` renderer
/// emits `{"error": err.code, "error_description": err.message}`, so the
/// `code` field MUST carry the OAuth2 error identifier (e.g. `invalid_request`,
/// `invalid_grant`) verbatim.
ErrorInfo make_oauth2_error(int status, const std::string & error_code, std::string description) {
  ErrorInfo info;
  info.code = error_code;
  info.message = std::move(description);
  info.http_status = status;
  return info;
}

/// Translate the `AuthErrorResponse` returned by parse helpers / AuthManager
/// into an `ErrorInfo` carrying the OAuth2 error identifier verbatim.
ErrorInfo from_auth_error(int status, const AuthErrorResponse & err) {
  return make_oauth2_error(status, err.error, err.error_description);
}

/// Build an AuthTokenResponse DTO from a TokenResponse (auth_models.hpp).
dto::AuthTokenResponse to_auth_token_response(const TokenResponse & tr) {
  dto::AuthTokenResponse resp;
  resp.access_token = tr.access_token;
  resp.token_type = tr.token_type;
  resp.expires_in = tr.expires_in;
  resp.scope = tr.scope;
  resp.refresh_token = tr.refresh_token;
  return resp;
}

/// Build a 404 `resource-not-found`-equivalent OAuth2 error for the
/// auth-disabled path. The renderer emits `{error: "resource-not-found",
/// error_description: "..."}` so clients still see a structured OAuth2 body
/// rather than a SOVD GenericError.
ErrorInfo auth_disabled_error() {
  return make_oauth2_error(404, ERR_RESOURCE_NOT_FOUND, "Authentication is not enabled");
}

/// Parse the AuthorizeRequest body (JSON or form-urlencoded) and surface the
/// OAuth2 error verbatim on failure.
tl::expected<AuthorizeRequest, ErrorInfo> parse_authorize_body(const http::TypedRequest & req) {
  std::string content_type = req.header("Content-Type").value_or("");
  // Framework escape hatch: the auth endpoints accept non-JSON bodies, so the
  // typed-body parser cannot be used here. The deprecation warning is
  // intentional: this is the documented escape hatch for handlers that must
  // see the raw request body.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw = req.raw_for_framework();
#pragma GCC diagnostic pop
  auto parse_result = AuthorizeRequest::parse_request(content_type, raw.body);
  if (!parse_result) {
    return tl::make_unexpected(from_auth_error(400, parse_result.error()));
  }
  return parse_result.value();
}

}  // namespace

http::Result<dto::AuthTokenResponse> AuthHandlers::post_authorize(const http::TypedRequest & req) {
  try {
    const auto & auth_config = ctx_.auth_config();
    if (!auth_config.enabled) {
      return tl::make_unexpected(auth_disabled_error());
    }

    auto parsed = parse_authorize_body(req);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    const auto & auth_req = parsed.value();

    if (auth_req.grant_type != "client_credentials") {
      return tl::make_unexpected(from_auth_error(
          400, AuthErrorResponse::unsupported_grant_type("Only 'client_credentials' grant type is supported")));
    }

    if (!auth_req.client_id.has_value() || auth_req.client_id->empty()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("client_id is required")));
    }

    if (!auth_req.client_secret.has_value() || auth_req.client_secret->empty()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("client_secret is required")));
    }

    auto * auth_manager = ctx_.auth_manager();
    if (auth_manager == nullptr) {
      // Defensive: the route is registered while auth is enabled, but the
      // manager was never wired up. Surface as an OAuth2 server error rather
      // than crashing on a null deref.
      return tl::make_unexpected(make_oauth2_error(500, "server_error", "Authentication manager unavailable"));
    }

    auto result = auth_manager->authenticate(auth_req.client_id.value(), auth_req.client_secret.value());
    if (!result) {
      return tl::make_unexpected(from_auth_error(401, result.error()));
    }
    return to_auth_token_response(*result);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in post_authorize: %s", e.what());
    return tl::make_unexpected(
        make_oauth2_error(500, "server_error", std::string("Internal server error: ") + e.what()));
  }
}

http::Result<dto::AuthTokenResponse> AuthHandlers::post_token(const http::TypedRequest & req) {
  try {
    const auto & auth_config = ctx_.auth_config();
    if (!auth_config.enabled) {
      return tl::make_unexpected(auth_disabled_error());
    }

    auto parsed = parse_authorize_body(req);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    const auto & auth_req = parsed.value();

    if (auth_req.grant_type != "refresh_token") {
      return tl::make_unexpected(from_auth_error(
          400,
          AuthErrorResponse::unsupported_grant_type("Only 'refresh_token' grant type is supported on this endpoint")));
    }

    if (!auth_req.refresh_token.has_value() || auth_req.refresh_token->empty()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("refresh_token is required")));
    }

    auto * auth_manager = ctx_.auth_manager();
    if (auth_manager == nullptr) {
      return tl::make_unexpected(make_oauth2_error(500, "server_error", "Authentication manager unavailable"));
    }

    auto result = auth_manager->refresh_access_token(auth_req.refresh_token.value());
    if (!result) {
      return tl::make_unexpected(from_auth_error(401, result.error()));
    }
    return to_auth_token_response(*result);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in post_token: %s", e.what());
    return tl::make_unexpected(
        make_oauth2_error(500, "server_error", std::string("Internal server error: ") + e.what()));
  }
}

http::Result<dto::AuthRevokeResponse> AuthHandlers::post_revoke(const http::TypedRequest & req) {
  try {
    const auto & auth_config = ctx_.auth_config();
    if (!auth_config.enabled) {
      return tl::make_unexpected(auth_disabled_error());
    }

    // Framework escape hatch: /auth/revoke historically accepts a JSON body
    // only (not form-urlencoded). Parse manually to preserve the OAuth2
    // `invalid_request` error code on malformed input.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto & raw = req.raw_for_framework();
#pragma GCC diagnostic pop

    json body;
    try {
      body = json::parse(raw.body);
    } catch (const json::parse_error & e) {
      return tl::make_unexpected(
          from_auth_error(400, AuthErrorResponse::invalid_request(std::string("Invalid JSON: ") + e.what())));
    }

    if (!body.contains("token") || !body["token"].is_string()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("token is required")));
    }

    const std::string token = body["token"].get<std::string>();

    auto * auth_manager = ctx_.auth_manager();
    if (auth_manager == nullptr) {
      return tl::make_unexpected(make_oauth2_error(500, "server_error", "Authentication manager unavailable"));
    }

    // Per RFC 7009 §2.2, the revoke endpoint must not indicate whether the
    // submitted token was valid - always respond with the same 200 body.
    auth_manager->revoke_refresh_token(token);

    dto::AuthRevokeResponse resp;
    resp.status = "revoked";
    return resp;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(HandlerContext::logger(), "Error in post_revoke: %s", e.what());
    return tl::make_unexpected(
        make_oauth2_error(500, "server_error", std::string("Internal server error: ") + e.what()));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
