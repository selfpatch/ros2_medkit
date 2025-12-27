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

#include "ros2_medkit_gateway/auth/auth_middleware.hpp"

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

AuthMiddleware::AuthMiddleware(const AuthConfig & config, AuthManager * auth_manager)
  : config_(config), auth_manager_(auth_manager) {
}

AuthMiddlewareResult AuthMiddleware::process(const AuthRequest & request) const {
  // If auth is not enabled, allow all requests
  if (!config_.enabled || auth_manager_ == nullptr) {
    return make_success();
  }

  // Check if authentication is required for this request
  if (!auth_manager_->requires_authentication(request.method, request.path)) {
    return make_success();
  }

  // Extract token from Authorization header
  if (!request.authorization_header.has_value()) {
    return make_unauthorized("Missing Authorization header");
  }

  auto token = extract_bearer_token(request.authorization_header.value());
  if (!token.has_value()) {
    return make_unauthorized("Invalid Authorization header format. Expected: Bearer <token>");
  }

  // Validate token
  auto validation = auth_manager_->validate_token(token.value());
  if (!validation.valid) {
    return make_unauthorized(validation.error);
  }

  // Check authorization (RBAC)
  auto auth_result = auth_manager_->check_authorization(validation.claims->role, request.method, request.path);
  if (!auth_result.authorized) {
    return make_forbidden(auth_result.error);
  }

  return make_success();
}

std::optional<std::string> AuthMiddleware::extract_bearer_token(const std::string & auth_header) {
  if (auth_header.empty()) {
    return std::nullopt;
  }

  // Check for "Bearer " prefix (case-insensitive for "Bearer")
  const std::string bearer_prefix = "Bearer ";
  if (auth_header.length() < bearer_prefix.length()) {
    return std::nullopt;
  }

  std::string prefix = auth_header.substr(0, bearer_prefix.length());
  // Case-insensitive comparison for "Bearer "
  if (prefix != "Bearer " && prefix != "bearer ") {
    return std::nullopt;
  }

  std::string token = auth_header.substr(bearer_prefix.length());
  if (token.empty()) {
    return std::nullopt;
  }

  return token;
}

AuthRequest AuthMiddleware::from_httplib_request(const httplib::Request & req) {
  AuthRequest auth_req;
  auth_req.method = req.method;
  auth_req.path = req.path;

  std::string auth_header = req.get_header_value("Authorization");
  if (!auth_header.empty()) {
    auth_req.authorization_header = auth_header;
  }

  return auth_req;
}

void AuthMiddleware::apply_to_response(const AuthMiddlewareResult & result, httplib::Response & res) {
  if (result.allowed) {
    return;  // No modification needed for success
  }

  res.status = result.status_code;

  if (!result.www_authenticate_header.empty()) {
    res.set_header("WWW-Authenticate", result.www_authenticate_header);
  }

  if (!result.error_body.empty()) {
    res.set_content(result.error_body, "application/json");
  }
}

AuthMiddlewareResult AuthMiddleware::make_unauthorized(const std::string & error_message, bool include_www_auth) {
  AuthMiddlewareResult result;
  result.allowed = false;
  result.status_code = 401;
  result.error_body = AuthErrorResponse::invalid_token(error_message).to_json().dump(2);

  if (include_www_auth) {
    result.www_authenticate_header = "Bearer realm=\"ros2_medkit_gateway\", error=\"invalid_token\"";
  }

  return result;
}

AuthMiddlewareResult AuthMiddleware::make_forbidden(const std::string & error_message) {
  AuthMiddlewareResult result;
  result.allowed = false;
  result.status_code = 403;
  result.error_body = AuthErrorResponse::insufficient_scope(error_message).to_json().dump(2);
  return result;
}

AuthMiddlewareResult AuthMiddleware::make_success() {
  AuthMiddlewareResult result;
  result.allowed = true;
  return result;
}

}  // namespace ros2_medkit_gateway
