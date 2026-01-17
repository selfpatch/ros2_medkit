// Copyright 2025 selfpatch
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

#include "ros2_medkit_gateway/http/handlers/discovery/app_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AppHandlers::handle_list_apps(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    // TODO: Implement in TASK_007
    // For now, return empty array since apps require manifest
    json apps_json = json::array();
    HandlerContext::send_json(res, apps_json);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_apps: %s", e.what());
  }
}

void AppHandlers::handle_get_app(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract app_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    // Validate app_id
    auto validation_result = ctx_.validate_entity_id(app_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid app ID",
                                 {{"details", validation_result.error()}, {"app_id", app_id}});
      return;
    }

    // TODO: Implement in TASK_007
    // For now, return 404 since apps require manifest
    HandlerContext::send_error(res, StatusCode::NotFound_404, "App not found", {{"app_id", app_id}});
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_app: %s", e.what());
  }
}

void AppHandlers::handle_related_apps(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract component_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    // Validate component_id
    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    // TODO: Implement in TASK_009
    // For now, return empty array since apps require manifest
    json apps_json = json::array();
    HandlerContext::send_json(res, apps_json);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_related_apps: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
