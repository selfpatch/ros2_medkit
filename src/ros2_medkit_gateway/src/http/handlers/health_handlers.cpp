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

#include "ros2_medkit_gateway/http/handlers/health_handlers.hpp"

#include <chrono>

#include "ros2_medkit_gateway/auth/auth_models.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void HealthHandlers::handle_health(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    json response = {{"status", "healthy"}, {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}};

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_health: %s", e.what());
  }
}

void HealthHandlers::handle_root(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    json endpoints = json::array({
        // Health & Discovery
        "GET /api/v1/health",
        "GET /api/v1/version-info",
        // Areas
        "GET /api/v1/areas",
        "GET /api/v1/areas/{area_id}",
        "GET /api/v1/areas/{area_id}/subareas",
        "GET /api/v1/areas/{area_id}/components",
        "GET /api/v1/areas/{area_id}/contains",
        "GET /api/v1/areas/{area_id}/data",
        "GET /api/v1/areas/{area_id}/data/{data_id}",
        "PUT /api/v1/areas/{area_id}/data/{data_id}",
        "GET /api/v1/areas/{area_id}/operations",
        "GET /api/v1/areas/{area_id}/operations/{operation_id}",
        "POST /api/v1/areas/{area_id}/operations/{operation_id}/executions",
        "GET /api/v1/areas/{area_id}/operations/{operation_id}/executions",
        "GET /api/v1/areas/{area_id}/operations/{operation_id}/executions/{execution_id}",
        "PUT /api/v1/areas/{area_id}/operations/{operation_id}/executions/{execution_id}",
        "DELETE /api/v1/areas/{area_id}/operations/{operation_id}/executions/{execution_id}",
        "GET /api/v1/areas/{area_id}/configurations",
        "GET /api/v1/areas/{area_id}/configurations/{param_name}",
        "PUT /api/v1/areas/{area_id}/configurations/{param_name}",
        "DELETE /api/v1/areas/{area_id}/configurations/{param_name}",
        "DELETE /api/v1/areas/{area_id}/configurations",
        "GET /api/v1/areas/{area_id}/faults",
        "GET /api/v1/areas/{area_id}/faults/{fault_code}",
        "DELETE /api/v1/areas/{area_id}/faults/{fault_code}",
        "DELETE /api/v1/areas/{area_id}/faults",
        "GET /api/v1/areas/{area_id}/faults/{fault_code}/snapshots",
        // Components
        "GET /api/v1/components",
        "GET /api/v1/components/{component_id}",
        "GET /api/v1/components/{component_id}/subcomponents",
        "GET /api/v1/components/{component_id}/hosts",
        "GET /api/v1/components/{component_id}/depends-on",
        "GET /api/v1/components/{component_id}/data",
        "GET /api/v1/components/{component_id}/data/{data_id}",
        "PUT /api/v1/components/{component_id}/data/{data_id}",
        "GET /api/v1/components/{component_id}/operations",
        "GET /api/v1/components/{component_id}/operations/{operation_id}",
        "POST /api/v1/components/{component_id}/operations/{operation_id}/executions",
        "GET /api/v1/components/{component_id}/operations/{operation_id}/executions",
        "GET /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}",
        "PUT /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}",
        "DELETE /api/v1/components/{component_id}/operations/{operation_id}/executions/{execution_id}",
        "GET /api/v1/components/{component_id}/configurations",
        "GET /api/v1/components/{component_id}/configurations/{param_name}",
        "PUT /api/v1/components/{component_id}/configurations/{param_name}",
        "DELETE /api/v1/components/{component_id}/configurations/{param_name}",
        "DELETE /api/v1/components/{component_id}/configurations",
        "GET /api/v1/components/{component_id}/faults",
        "GET /api/v1/components/{component_id}/faults/{fault_code}",
        "DELETE /api/v1/components/{component_id}/faults/{fault_code}",
        "DELETE /api/v1/components/{component_id}/faults",
        "GET /api/v1/components/{component_id}/faults/{fault_code}/snapshots",
        // Apps
        "GET /api/v1/apps",
        "GET /api/v1/apps/{app_id}",
        "GET /api/v1/apps/{app_id}/depends-on",
        "GET /api/v1/apps/{app_id}/data",
        "GET /api/v1/apps/{app_id}/data/{data_id}",
        "PUT /api/v1/apps/{app_id}/data/{data_id}",
        "GET /api/v1/apps/{app_id}/data-categories",
        "GET /api/v1/apps/{app_id}/data-groups",
        "GET /api/v1/apps/{app_id}/operations",
        "GET /api/v1/apps/{app_id}/operations/{operation_id}",
        "POST /api/v1/apps/{app_id}/operations/{operation_id}/executions",
        "GET /api/v1/apps/{app_id}/operations/{operation_id}/executions",
        "GET /api/v1/apps/{app_id}/operations/{operation_id}/executions/{execution_id}",
        "PUT /api/v1/apps/{app_id}/operations/{operation_id}/executions/{execution_id}",
        "DELETE /api/v1/apps/{app_id}/operations/{operation_id}/executions/{execution_id}",
        "GET /api/v1/apps/{app_id}/configurations",
        "GET /api/v1/apps/{app_id}/configurations/{param_name}",
        "PUT /api/v1/apps/{app_id}/configurations/{param_name}",
        "DELETE /api/v1/apps/{app_id}/configurations/{param_name}",
        "DELETE /api/v1/apps/{app_id}/configurations",
        "GET /api/v1/apps/{app_id}/faults",
        "GET /api/v1/apps/{app_id}/faults/{fault_code}",
        "DELETE /api/v1/apps/{app_id}/faults/{fault_code}",
        "DELETE /api/v1/apps/{app_id}/faults",
        "GET /api/v1/apps/{app_id}/faults/{fault_code}/snapshots",
        // Functions
        "GET /api/v1/functions",
        "GET /api/v1/functions/{function_id}",
        "GET /api/v1/functions/{function_id}/hosts",
        "GET /api/v1/functions/{function_id}/data",
        "GET /api/v1/functions/{function_id}/data/{data_id}",
        "PUT /api/v1/functions/{function_id}/data/{data_id}",
        "GET /api/v1/functions/{function_id}/operations",
        "GET /api/v1/functions/{function_id}/operations/{operation_id}",
        "POST /api/v1/functions/{function_id}/operations/{operation_id}/executions",
        "GET /api/v1/functions/{function_id}/operations/{operation_id}/executions",
        "GET /api/v1/functions/{function_id}/operations/{operation_id}/executions/{execution_id}",
        "PUT /api/v1/functions/{function_id}/operations/{operation_id}/executions/{execution_id}",
        "DELETE /api/v1/functions/{function_id}/operations/{operation_id}/executions/{execution_id}",
        "GET /api/v1/functions/{function_id}/configurations",
        "GET /api/v1/functions/{function_id}/configurations/{param_name}",
        "PUT /api/v1/functions/{function_id}/configurations/{param_name}",
        "DELETE /api/v1/functions/{function_id}/configurations/{param_name}",
        "DELETE /api/v1/functions/{function_id}/configurations",
        "GET /api/v1/functions/{function_id}/faults",
        "GET /api/v1/functions/{function_id}/faults/{fault_code}",
        "DELETE /api/v1/functions/{function_id}/faults/{fault_code}",
        "DELETE /api/v1/functions/{function_id}/faults",
        "GET /api/v1/functions/{function_id}/faults/{fault_code}/snapshots",
        // Global Faults
        "GET /api/v1/faults",
        "GET /api/v1/faults/stream",
        "GET /api/v1/faults/{fault_code}/snapshots",
        "GET /api/v1/faults/{fault_code}/snapshots/bag",
    });

    const auto & auth_config = ctx_.auth_config();
    const auto & tls_config = ctx_.tls_config();

    // Add auth endpoints if auth is enabled
    if (auth_config.enabled) {
      endpoints.push_back("POST /api/v1/auth/authorize");
      endpoints.push_back("POST /api/v1/auth/token");
      endpoints.push_back("POST /api/v1/auth/revoke");
    }

    json capabilities = {
        {"discovery", true},
        {"data_access", true},
        {"operations", true},
        {"async_actions", true},
        {"configurations", true},
        {"faults", true},
        {"authentication", auth_config.enabled},
        {"tls", tls_config.enabled},
    };

    json response = {
        {"name", "ROS 2 Medkit Gateway"}, {"version", "0.1.0"},           {"api_base", API_BASE_PATH},
        {"endpoints", endpoints},         {"capabilities", capabilities},
    };

    // Add auth info if enabled
    if (auth_config.enabled) {
      response["auth"] = {
          {"enabled", true},
          {"algorithm", algorithm_to_string(auth_config.jwt_algorithm)},
          {"require_auth_for", auth_config.require_auth_for == AuthRequirement::NONE    ? "none"
                               : auth_config.require_auth_for == AuthRequirement::WRITE ? "write"
                                                                                        : "all"},
      };
    }

    // Add TLS info if enabled
    if (tls_config.enabled) {
      response["tls"] = {
          {"enabled", true}, {"min_version", tls_config.min_version},
          // TODO(future): Add mutual_tls when implemented
      };
    }

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_root: %s", e.what());
  }
}

void HealthHandlers::handle_version_info(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    // SOVD 7.4.1 compliant response format
    json sovd_info_entry = {
        {"version", "1.0.0"},                                             // SOVD standard version
        {"base_uri", API_BASE_PATH},                                      // Version-specific base URI
        {"vendor_info", {{"version", "0.1.0"}, {"name", "ros2_medkit"}}}  // Vendor-specific info
    };

    json response = {{"sovd_info", json::array({sovd_info_entry})}};

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_version_info: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
