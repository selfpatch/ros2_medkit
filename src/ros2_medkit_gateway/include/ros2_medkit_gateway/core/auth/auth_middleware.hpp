// Copyright 2025 bburda
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

#include <httplib.h>

#include <memory>
#include <optional>
#include <string>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/core/auth/auth_models.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief HTTP request abstraction for authentication
 *
 * This interface abstracts the HTTP request to decouple
 * auth logic from the HTTP library (cpp-httplib).
 */
struct AuthRequest {
  std::string method;
  std::string path;
  std::optional<std::string> authorization_header;
};

/**
 * @brief Result of authentication/authorization check
 */
struct AuthMiddlewareResult {
  bool allowed{false};
  int status_code{0};
  std::string error_body;
  std::string www_authenticate_header;
};

/**
 * @brief Middleware class for handling HTTP authentication/authorization
 *
 * Separates the authentication middleware logic from the REST server,
 * following the Single Responsibility Principle (SRP).
 *
 * This class:
 * - Extracts bearer tokens from Authorization headers
 * - Delegates token validation to AuthManager
 * - Produces appropriate HTTP responses for auth failures
 *
 * @verifies REQ_INTEROP_086
 */
class AuthMiddleware {
 public:
  /**
   * @brief Construct AuthMiddleware with configuration and auth manager
   * @param config Authentication configuration
   * @param auth_manager Pointer to the auth manager (not owned)
   */
  AuthMiddleware(const AuthConfig & config, AuthManager * auth_manager);

  /**
   * @brief Check if authentication is enabled
   * @return true if auth is enabled
   */
  bool is_enabled() const {
    return config_.enabled && auth_manager_ != nullptr;
  }

  /**
   * @brief Process an authentication request
   *
   * Checks if authentication is required, validates the token,
   * and checks authorization for the requested resource.
   *
   * @param request The HTTP request abstraction
   * @return AuthMiddlewareResult with success/failure and response details
   */
  AuthMiddlewareResult process(const AuthRequest & request) const;

  /**
   * @brief Whether this route needs a credential at all
   *
   * The same question `process` asks first, exposed so a caller that has to
   * decide something BEFORE running `process` - the rate limiter, choosing
   * whether it may answer - reads the one policy object, so there is a single
   * copy of the rule.
   *
   * @param request The HTTP request abstraction
   * @return true if the route requires authentication
   */
  bool requires_authentication(const AuthRequest & request) const;

  /**
   * @brief Whether an exhausted limiter answers before the token is verified
   *
   * Verifying a signature is the expensive half of `process`, and an
   * over-limit caller is one the gateway has already decided to refuse. So
   * when all three hold - the allowance is gone, the caller presented an
   * Authorization header, and the route needs one - the 429 is the answer and
   * the verifier never runs.
   *
   * The header is what separates the two refusals, and the test is its
   * PRESENCE: the value is never parsed here, so a `Basic` header or a bare
   * word takes this path exactly as a bearer does. With no header at all,
   * `process` short-circuits before it extracts or verifies anything, so the
   * anonymous 401 already costs nothing and stays the answer.
   *
   * The 429 this produces reports the refusal and nothing else - no
   * `Retry-After`, no `X-RateLimit-*` - because nothing about the caller has
   * been verified at this point. Limiter state reaches a caller the gateway
   * accepted, or one on a route needing no credential, and those are answered
   * further down.
   *
   * @param rate_limited                  The allowance for this caller is gone
   * @param has_authorization_header      The caller sent an Authorization header
   * @param route_requires_authentication The route needs a credential
   * @return true if the limiter answers and the verifier is skipped
   *
   * @note The caller evaluates the third argument only when the first two
   *       hold; the route lookup is work this predicate cannot avoid once it
   *       has been done.
   */
  static bool rate_limit_precedes_validation(bool rate_limited, bool has_authorization_header,
                                             bool route_requires_authentication) {
    return rate_limited && has_authorization_header && route_requires_authentication;
  }

  /**
   * @brief Extract bearer token from Authorization header
   * @param auth_header The Authorization header value
   * @return Token string if valid Bearer format, nullopt otherwise
   */
  static std::optional<std::string> extract_bearer_token(const std::string & auth_header);

  /**
   * @brief Build AuthRequest from httplib::Request
   * @param req The httplib request
   * @return AuthRequest abstraction
   */
  static AuthRequest from_httplib_request(const httplib::Request & req);

  /**
   * @brief Apply AuthMiddlewareResult to httplib::Response
   * @param result The auth result
   * @param res The httplib response to modify
   */
  static void apply_to_response(const AuthMiddlewareResult & result, httplib::Response & res);

 private:
  /**
   * @brief Build unauthorized response
   * @param error_message Error message
   * @param include_www_auth Whether to include WWW-Authenticate header
   * @return AuthMiddlewareResult with 401 status
   */
  static AuthMiddlewareResult make_unauthorized(const std::string & error_message, bool include_www_auth = true);

  /**
   * @brief Build forbidden response
   * @param error_message Error message
   * @return AuthMiddlewareResult with 403 status
   */
  static AuthMiddlewareResult make_forbidden(const std::string & error_message);

  /**
   * @brief Build success response
   * @return AuthMiddlewareResult with allowed=true
   */
  static AuthMiddlewareResult make_success();

  AuthConfig config_;
  AuthManager * auth_manager_;  ///< Non-owning pointer to auth manager
};

}  // namespace ros2_medkit_gateway
