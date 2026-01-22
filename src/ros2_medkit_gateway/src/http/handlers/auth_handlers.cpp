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

#include "ros2_medkit_gateway/http/handlers/auth_handlers.hpp"

#include "ros2_medkit_gateway/auth/auth_models.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AuthHandlers::handle_auth_authorize(const httplib::Request & req, httplib::Response & res) {
  try {
    const auto & auth_config = ctx_.auth_config();

    if (!auth_config.enabled) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                 "Authentication is not enabled");
      return;
    }

    // Parse request using DRY helper
    auto parse_result = AuthorizeRequest::parse_request(req.get_header_value("Content-Type"), req.body);
    if (!parse_result) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(parse_result.error().to_json().dump(2), "application/json");
      return;
    }
    auto auth_req = parse_result.value();

    // Validate grant_type
    if (auth_req.grant_type != "client_credentials") {
      res.status = StatusCode::BadRequest_400;
      res.set_content(AuthErrorResponse::unsupported_grant_type("Only 'client_credentials' grant type is supported")
                          .to_json()
                          .dump(2),
                      "application/json");
      return;
    }

    // Validate required fields
    if (!auth_req.client_id.has_value() || auth_req.client_id->empty()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(AuthErrorResponse::invalid_request("client_id is required").to_json().dump(2),
                      "application/json");
      return;
    }

    if (!auth_req.client_secret.has_value() || auth_req.client_secret->empty()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(AuthErrorResponse::invalid_request("client_secret is required").to_json().dump(2),
                      "application/json");
      return;
    }

    // Authenticate
    auto auth_manager = ctx_.auth_manager();
    auto result = auth_manager->authenticate(auth_req.client_id.value(), auth_req.client_secret.value());

    if (result) {
      HandlerContext::send_json(res, result->to_json());
    } else {
      res.status = StatusCode::Unauthorized_401;
      res.set_content(result.error().to_json().dump(2), "application/json");
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_auth_authorize: %s", e.what());
  }
}

void AuthHandlers::handle_auth_token(const httplib::Request & req, httplib::Response & res) {
  try {
    const auto & auth_config = ctx_.auth_config();

    if (!auth_config.enabled) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                 "Authentication is not enabled");
      return;
    }

    // Parse request using DRY helper
    auto parse_result = AuthorizeRequest::parse_request(req.get_header_value("Content-Type"), req.body);
    if (!parse_result) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(parse_result.error().to_json().dump(2), "application/json");
      return;
    }
    auto auth_req = parse_result.value();

    // Validate grant_type
    if (auth_req.grant_type != "refresh_token") {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          AuthErrorResponse::unsupported_grant_type("Only 'refresh_token' grant type is supported on this endpoint")
              .to_json()
              .dump(2),
          "application/json");
      return;
    }

    // Validate required fields
    if (!auth_req.refresh_token.has_value() || auth_req.refresh_token->empty()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(AuthErrorResponse::invalid_request("refresh_token is required").to_json().dump(2),
                      "application/json");
      return;
    }

    // Refresh token
    auto auth_manager = ctx_.auth_manager();
    auto result = auth_manager->refresh_access_token(auth_req.refresh_token.value());

    if (result) {
      HandlerContext::send_json(res, result->to_json());
    } else {
      res.status = StatusCode::Unauthorized_401;
      res.set_content(result.error().to_json().dump(2), "application/json");
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_auth_token: %s", e.what());
  }
}

void AuthHandlers::handle_auth_revoke(const httplib::Request & req, httplib::Response & res) {
  try {
    const auto & auth_config = ctx_.auth_config();

    if (!auth_config.enabled) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                 "Authentication is not enabled");
      return;
    }

    // Parse request
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(AuthErrorResponse::invalid_request("Invalid JSON: " + std::string(e.what())).to_json().dump(2),
                      "application/json");
      return;
    }

    // Extract token to revoke
    if (!body.contains("token") || !body["token"].is_string()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(AuthErrorResponse::invalid_request("token is required").to_json().dump(2), "application/json");
      return;
    }

    std::string token = body["token"].get<std::string>();

    // Revoke token (do not reveal whether it existed to avoid token enumeration)
    auto auth_manager = ctx_.auth_manager();
    auth_manager->revoke_refresh_token(token);

    // Per OAuth2 RFC 7009, always return 200 and do not indicate token validity
    json response = {{"status", "revoked"}};
    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_auth_revoke: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
