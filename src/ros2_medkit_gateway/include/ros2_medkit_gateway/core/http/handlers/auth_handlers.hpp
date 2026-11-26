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

#include <utility>

#include "ros2_medkit_gateway/dto/auth.hpp"
#include "ros2_medkit_gateway/http/handler_result.hpp"
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
 * All three authenticate the client (RFC 6749 §2.3.1): HTTP Basic or
 * `client_id` and `client_secret` in the body. The body is read through the
 * framework escape hatch because it may be `application/json` or
 * `application/x-www-form-urlencoded`; the framework's typed-body parser only
 * speaks JSON and would also emit SOVD's `invalid-request` (dash) code instead
 * of OAuth2's `invalid_request` (underscore). The routes set
 * `.error_renderer(kOAuth2Error)` so any returned `ErrorInfo` is rendered as
 * `{error, error_description}` per RFC 6749 §5.2. A success carries
 * `Cache-Control: no-store` and `Pragma: no-cache` (RFC 6749 §5.1).
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
  http::Result<std::pair<dto::AuthTokenResponse, http::ResponseAttachments>>
  post_authorize(const http::TypedRequest & req);

  /// POST /auth/token - refresh access token via the refresh_token grant.
  /// The refresh token must have been issued to the authenticated client.
  http::Result<std::pair<dto::AuthTokenResponse, http::ResponseAttachments>> post_token(const http::TypedRequest & req);

  /// POST /auth/revoke - revoke a refresh token of the authenticated client (RFC 7009).
  /// A token that does not verify answers 200 `{"status":"revoked"}` and is not recorded.
  http::Result<std::pair<dto::AuthRevokeResponse, http::ResponseAttachments>>
  post_revoke(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
