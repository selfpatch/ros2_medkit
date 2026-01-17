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

#include "ros2_medkit_gateway/http/handlers/discovery/function_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void FunctionHandlers::handle_list_functions(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    // TODO: Implement in TASK_008
    // For now, return empty array since functions require manifest
    json functions_json = json::array();
    HandlerContext::send_json(res, functions_json);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_functions: %s", e.what());
  }
}

void FunctionHandlers::handle_get_function(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract function_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    // Validate function_id
    auto validation_result = ctx_.validate_entity_id(function_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid function ID",
                                 {{"details", validation_result.error()}, {"function_id", function_id}});
      return;
    }

    // TODO: Implement in TASK_008
    // For now, return 404 since functions require manifest
    HandlerContext::send_error(res, StatusCode::NotFound_404, "Function not found", {{"function_id", function_id}});
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_function: %s", e.what());
  }
}

void FunctionHandlers::handle_function_hosts(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract function_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    // Validate function_id
    auto validation_result = ctx_.validate_entity_id(function_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid function ID",
                                 {{"details", validation_result.error()}, {"function_id", function_id}});
      return;
    }

    // TODO: Implement in TASK_008
    // For now, return 404 since functions require manifest
    HandlerContext::send_error(res, StatusCode::NotFound_404, "Function not found", {{"function_id", function_id}});
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_function_hosts: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
