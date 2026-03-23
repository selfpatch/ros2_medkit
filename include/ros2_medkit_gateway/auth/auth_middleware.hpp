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

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/auth/auth_models.hpp"

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
