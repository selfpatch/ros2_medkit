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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for authentication REST API endpoints.
 *
 * Implements OAuth2-like authentication flow:
 * - POST /auth/authorize - Authenticate with client credentials
 * - POST /auth/token - Refresh access token
 * - POST /auth/revoke - Revoke refresh token
 *
 * @note These endpoints are only available when authentication is enabled.
 */
class AuthHandlers {
 public:
  /**
   * @brief Construct authentication handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit AuthHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle POST /auth/authorize - authenticate with client credentials.
   */
  void handle_auth_authorize(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle POST /auth/token - refresh access token.
   */
  void handle_auth_token(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle POST /auth/revoke - revoke refresh token.
   */
  void handle_auth_revoke(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
