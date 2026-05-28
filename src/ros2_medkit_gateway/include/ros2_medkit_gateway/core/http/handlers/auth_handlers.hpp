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

#pragma once

#include "ros2_medkit_gateway/dto/auth.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for authentication REST API endpoints.
 *
 * Implements OAuth2-like authentication flow:
 * - POST /auth/authorize - Authenticate with client credentials
 * - POST /auth/token     - Refresh access token (RFC 6749 token endpoint)
 * - POST /auth/revoke    - Revoke refresh token (RFC 7009)
 *
 * All three handlers follow the PR-403 typed RouteRegistry convention:
 *
 *   http::Result<dto::TResponse> X(const http::TypedRequest & req);
 *
 * The body is read directly from `req.body` (via the framework escape hatch)
 * because the auth endpoints accept BOTH `application/json` AND
 * `application/x-www-form-urlencoded` (RFC 6749 §4.1.3). The framework's
 * typed-body parser only speaks JSON and would also emit SOVD's
 * `invalid-request` (dash) code instead of OAuth2's `invalid_request`
 * (underscore). The routes set `.error_renderer(kOAuth2Error)` so any returned
 * `ErrorInfo` is rendered as `{error, error_description}` per RFC 6749 §5.2.
 *
 * @note These endpoints are only registered/exposed when authentication is
 * enabled. The handlers still defensively return 404 when `auth.enabled` is
 * false, so a misconfiguration that leaves the routes wired without a manager
 * does not crash.
 */
class AuthHandlers {
 public:
  /**
   * @brief Construct authentication handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit AuthHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /// POST /auth/authorize - authenticate with client_credentials grant.
  /// Returns an OAuth2 TokenResponse on success or an OAuth2 error on failure.
  http::Result<dto::AuthTokenResponse> post_authorize(const http::TypedRequest & req);

  /// POST /auth/token - refresh access token via the refresh_token grant.
  http::Result<dto::AuthTokenResponse> post_token(const http::TypedRequest & req);

  /// POST /auth/revoke - revoke a refresh token (RFC 7009).
  /// Always returns 200 + `{"status":"revoked"}` regardless of whether the
  /// token existed, to prevent token-enumeration side channels.
  http::Result<dto::AuthRevokeResponse> post_revoke(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
