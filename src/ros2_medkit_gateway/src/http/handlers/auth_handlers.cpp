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
#include "ros2_medkit_gateway/core/logging.hpp"

#include <optional>
#include <string>
#include <utility>

#include "ros2_medkit_gateway/core/auth/auth_models.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// RFC 7235 §3.1 challenge on every 401 `invalid_client`, Basic being the scheme the client may use. The realm of
/// the bearer challenge.
constexpr const char * kBasicChallenge = "Basic realm=\"ros2_medkit_gateway\"";

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

/// RFC 6749 §5.1: no answer of the token endpoints is cached. `write_oauth2_error` sets the same on errors.
http::ResponseAttachments no_store() {
  return http::ResponseAttachments{}.with_header("Cache-Control", "no-store").with_header("Pragma", "no-cache");
}

/// Build an AuthTokenResponse DTO from a TokenResponse (auth_models.hpp).
std::pair<dto::AuthTokenResponse, http::ResponseAttachments> to_auth_token_response(const TokenResponse & tr) {
  dto::AuthTokenResponse resp;
  resp.access_token = tr.access_token;
  resp.token_type = tr.token_type;
  resp.expires_in = tr.expires_in;
  resp.scope = tr.scope;
  resp.refresh_token = tr.refresh_token;
  return {std::move(resp), no_store()};
}

/// Build a 404 `resource-not-found`-equivalent OAuth2 error for the
/// auth-disabled path. The renderer emits `{error: "resource-not-found",
/// error_description: "..."}` so clients still see a structured OAuth2 body
/// rather than a SOVD GenericError.
ErrorInfo auth_disabled_error() {
  return make_oauth2_error(404, ERR_RESOURCE_NOT_FOUND, "Authentication is not enabled");
}

/// The raw request body. The auth endpoints accept form-urlencoded bodies, which the typed-body parser does not.
const std::string & raw_body(const http::TypedRequest & req) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return req.raw_for_framework().body;
#pragma GCC diagnostic pop
}

/// How many `Authorization` headers the request carries. TypedRequest::header() reads the first only.
size_t authorization_header_count(const http::TypedRequest & req) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return req.raw_for_framework().get_header_value_count("Authorization");
#pragma GCC diagnostic pop
}

/// Parse the AuthorizeRequest body (JSON or form-urlencoded) and surface the
/// OAuth2 error verbatim on failure.
tl::expected<AuthorizeRequest, ErrorInfo> parse_authorize_body(const http::TypedRequest & req) {
  auto parse_result = AuthorizeRequest::parse_request(req.header("Content-Type").value_or(""), raw_body(req));
  if (!parse_result) {
    return tl::make_unexpected(from_auth_error(400, parse_result.error()));
  }
  return parse_result.value();
}

/// The client's credentials.
struct PresentedClient {
  std::string client_id;
  std::string client_secret;
};

/// A 401 `invalid_client` with the Basic challenge, also for credentials sent in the body.
ErrorInfo client_refused(const AuthErrorResponse & err) {
  auto refused = from_auth_error(401, err);
  refused.params["www_authenticate"] = kBasicChallenge;
  return refused;
}

/// Read the client's credentials (RFC 6749 §2.3.1): HTTP Basic, or `client_id` and `client_secret` in the body.
/// 400 `invalid_request`: more than one `Authorization` header, or both methods (§2.3). 401 `invalid_client`: Basic
/// that does not decode or has an empty id or secret, or no credentials.
/// An `Authorization` header of another scheme is not client authentication and is ignored.
tl::expected<PresentedClient, ErrorInfo> presented_client(const http::TypedRequest & req,
                                                          const std::optional<std::string> & body_id,
                                                          const std::optional<std::string> & body_secret) {
  if (authorization_header_count(req) > 1) {
    return tl::make_unexpected(
        from_auth_error(400, AuthErrorResponse::invalid_request("The request has more than one Authorization header")));
  }
  const bool body_has_secret = body_secret.has_value() && !body_secret->empty();
  const auto authorization = req.header("Authorization");
  if (authorization.has_value()) {
    auto basic = parse_basic_authorization(*authorization);
    const bool uses_basic = !basic || basic->has_value();
    if (uses_basic && body_has_secret) {
      return tl::make_unexpected(from_auth_error(
          400, AuthErrorResponse::invalid_request("The request uses more than one client authentication method")));
    }
    if (!basic) {
      return tl::make_unexpected(client_refused(AuthErrorResponse::client_authentication_failed()));
    }
    if (basic->has_value()) {
      if (body_id.has_value() && !body_id->empty() && *body_id != (*basic)->client_id) {
        return tl::make_unexpected(from_auth_error(
            400, AuthErrorResponse::invalid_request("client_id in the body differs from the Basic credentials")));
      }
      if ((*basic)->client_id.empty() || (*basic)->client_secret.empty()) {
        return tl::make_unexpected(client_refused(AuthErrorResponse::client_authentication_failed()));
      }
      return PresentedClient{std::move((*basic)->client_id), std::move((*basic)->client_secret)};
    }
  }
  if (!body_id.has_value() || body_id->empty() || !body_has_secret) {
    return tl::make_unexpected(client_refused(AuthErrorResponse::client_authentication_failed()));
  }
  return PresentedClient{*body_id, *body_secret};
}

}  // namespace

http::Result<std::pair<dto::AuthTokenResponse, http::ResponseAttachments>>
AuthHandlers::post_authorize(const http::TypedRequest & req) {
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

    // RFC 6749 §3.1: an empty parameter is an omitted one.
    if (auth_req.grant_type.empty()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("grant_type is required")));
    }

    if (auth_req.grant_type != "client_credentials") {
      return tl::make_unexpected(from_auth_error(
          400, AuthErrorResponse::unsupported_grant_type("Only 'client_credentials' grant type is supported")));
    }

    auto client = presented_client(req, auth_req.client_id, auth_req.client_secret);
    if (!client) {
      return tl::make_unexpected(client.error());
    }

    auto * auth_manager = ctx_.auth_manager();
    if (auth_manager == nullptr) {
      // Defensive: the route is registered while auth is enabled, but the
      // manager was never wired up. Surface as an OAuth2 server error rather
      // than crashing on a null deref.
      return tl::make_unexpected(make_oauth2_error(500, "server_error", "Authentication manager unavailable"));
    }

    auto result = auth_manager->authenticate(client->client_id, client->client_secret);
    if (!result) {
      const auto & error = result.error();
      if (error.retry_after_seconds.has_value()) {
        // The credentials were good; the client's refresh token slots are all held by revoked tokens.
        auto busy = from_auth_error(429, error);
        busy.params["retry_after_seconds"] = *error.retry_after_seconds;
        return tl::make_unexpected(std::move(busy));
      }
      return tl::make_unexpected(client_refused(error));
    }
    return to_auth_token_response(*result);
  } catch (const std::exception & e) {
    MEDKIT_LOG_ERROR(HandlerContext::logger(), "Error in post_authorize: %s", e.what());
    return tl::make_unexpected(
        make_oauth2_error(500, "server_error", std::string("Internal server error: ") + e.what()));
  }
}

http::Result<std::pair<dto::AuthTokenResponse, http::ResponseAttachments>>
AuthHandlers::post_token(const http::TypedRequest & req) {
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

    if (auth_req.grant_type.empty()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("grant_type is required")));
    }

    if (auth_req.grant_type != "refresh_token") {
      return tl::make_unexpected(from_auth_error(
          400,
          AuthErrorResponse::unsupported_grant_type("Only 'refresh_token' grant type is supported on this endpoint")));
    }

    if (!auth_req.refresh_token.has_value() || auth_req.refresh_token->empty()) {
      return tl::make_unexpected(from_auth_error(400, AuthErrorResponse::invalid_request("refresh_token is required")));
    }

    auto client = presented_client(req, auth_req.client_id, auth_req.client_secret);
    if (!client) {
      return tl::make_unexpected(client.error());
    }

    auto * auth_manager = ctx_.auth_manager();
    if (auth_manager == nullptr) {
      return tl::make_unexpected(make_oauth2_error(500, "server_error", "Authentication manager unavailable"));
    }

    // RFC 6749 §6: a confidential client authenticates before it refreshes.
    if (auto authenticated = auth_manager->authenticate_client(client->client_id, client->client_secret);
        !authenticated) {
      return tl::make_unexpected(client_refused(authenticated.error()));
    }

    auto result = auth_manager->refresh_access_token(auth_req.refresh_token.value(), client->client_id);
    if (!result) {
      // RFC 6749 §5.2: an invalid, expired, revoked or foreign refresh token is `invalid_grant` with 400.
      return tl::make_unexpected(from_auth_error(400, result.error()));
    }
    return to_auth_token_response(*result);
  } catch (const std::exception & e) {
    MEDKIT_LOG_ERROR(HandlerContext::logger(), "Error in post_token: %s", e.what());
    return tl::make_unexpected(
        make_oauth2_error(500, "server_error", std::string("Internal server error: ") + e.what()));
  }
}

http::Result<std::pair<dto::AuthRevokeResponse, http::ResponseAttachments>>
AuthHandlers::post_revoke(const http::TypedRequest & req) {
  try {
    const auto & auth_config = ctx_.auth_config();
    if (!auth_config.enabled) {
      return tl::make_unexpected(auth_disabled_error());
    }

    auto parsed = RevokeRequest::parse_request(req.header("Content-Type").value_or(""), raw_body(req));
    if (!parsed) {
      return tl::make_unexpected(from_auth_error(400, parsed.error()));
    }
    const auto & revoke_req = parsed.value();

    auto client = presented_client(req, revoke_req.client_id, revoke_req.client_secret);
    if (!client) {
      return tl::make_unexpected(client.error());
    }

    auto * auth_manager = ctx_.auth_manager();
    if (auth_manager == nullptr) {
      return tl::make_unexpected(make_oauth2_error(500, "server_error", "Authentication manager unavailable"));
    }

    if (auto authenticated = auth_manager->authenticate_client(client->client_id, client->client_secret);
        !authenticated) {
      return tl::make_unexpected(client_refused(authenticated.error()));
    }

    // RFC 7009 §2.2: a token that does not verify answers 200, so the answer does not tell whether it was valid.
    // The refusals: an access token (§2.2.1), another client's refresh token (§2.1), and a full pool (503, §2.2.1).
    const auto outcome = auth_manager->try_revoke_refresh_token(revoke_req.token, client->client_id);
    if (!outcome) {
      const auto & error = outcome.error();
      if (error.error != "temporarily_unavailable") {
        return tl::make_unexpected(from_auth_error(400, error));
      }
      auto busy = from_auth_error(503, error);
      busy.params["retry_after_seconds"] = error.retry_after_seconds.value_or(1);
      return tl::make_unexpected(std::move(busy));
    }

    dto::AuthRevokeResponse resp;
    resp.status = "revoked";
    return std::make_pair(std::move(resp), no_store());
  } catch (const std::exception & e) {
    MEDKIT_LOG_ERROR(HandlerContext::logger(), "Error in post_revoke: %s", e.what());
    return tl::make_unexpected(
        make_oauth2_error(500, "server_error", std::string("Internal server error: ") + e.what()));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
