// Copyright 2025-2026 bburda
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

#include <algorithm>
#include <chrono>

#include "ros2_medkit_gateway/auth/auth_models.hpp"
#include "ros2_medkit_gateway/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/version.hpp"

#include "../../openapi/route_registry.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

void HealthHandlers::handle_health(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    json response = {{"status", "healthy"}, {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}};

    // Add discovery info
    auto * dm = ctx_.node() ? ctx_.node()->get_discovery_manager() : nullptr;
    if (dm) {
      json discovery_info = {{"mode", discovery_mode_to_string(dm->get_mode())}, {"strategy", dm->get_strategy_name()}};

      auto report = dm->get_merge_report();
      if (report) {
        discovery_info["pipeline"] = report->to_json();
      }

      auto linking = dm->get_linking_result();
      if (linking) {
        json linking_info;
        linking_info["linked_count"] = linking->node_to_app.size();
        linking_info["orphan_count"] = linking->orphan_nodes.size();
        linking_info["binding_conflicts"] = linking->binding_conflicts;
        if (!linking->warnings.empty()) {
          linking_info["warnings"] = linking->warnings;
        }
        discovery_info["linking"] = linking_info;
      }

      response["discovery"] = std::move(discovery_info);
    }

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_health: %s", e.what());
  }
}

void HealthHandlers::handle_root(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    // Generate endpoint list from route registry (single source of truth)
    json endpoints = json::array();
    if (route_registry_) {
      auto ep_list = route_registry_->to_endpoint_list(API_BASE_PATH);
      for (auto & ep : ep_list) {
        endpoints.push_back(std::move(ep));
      }
    }

    // Read docs.enabled parameter (defaults to true)
    bool docs_enabled = true;
    if (ctx_.node()) {
      try {
        docs_enabled = ctx_.node()->get_parameter("docs.enabled").as_bool();
      } catch (...) {
        // Parameter may not be declared - default to true
      }
    }

    // Add docs endpoints (not in registry - registered directly with server)
    if (docs_enabled) {
      endpoints.push_back("GET " + std::string(API_BASE_PATH) + "/docs");
      endpoints.push_back("GET " + std::string(API_BASE_PATH) + "/{entity-path}/docs");
#ifdef ENABLE_SWAGGER_UI
      endpoints.push_back("GET " + std::string(API_BASE_PATH) + "/swagger-ui");
#endif
    }

    const auto & auth_config = ctx_.auth_config();
    const auto & tls_config = ctx_.tls_config();

    json capabilities = {
        {"discovery", true},
        {"data_access", true},
        {"operations", true},
        {"async_actions", true},
        {"configurations", true},
        {"faults", true},
        {"logs", true},
        {"bulk_data", true},
        {"cyclic_subscriptions", true},
        {"locking", ctx_.node() && ctx_.node()->get_lock_manager() != nullptr},
        {"updates", ctx_.node() && ctx_.node()->get_update_manager() != nullptr},
        {"authentication", auth_config.enabled},
        {"tls", tls_config.enabled},
        {"vendor_extensions",
         ctx_.node() && ctx_.node()->get_plugin_manager() && ctx_.node()->get_plugin_manager()->has_plugins()},
    };

    json response = {
        {"name", "ROS 2 Medkit Gateway"}, {"version", kGatewayVersion},   {"api_base", API_BASE_PATH},
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
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_root: %s", e.what());
  }
}

void HealthHandlers::handle_version_info(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    // SOVD 7.4.1 compliant response format
    json sovd_info_entry = {
        {"version", kSovdVersion},                                                // SOVD standard version
        {"base_uri", API_BASE_PATH},                                              // Version-specific base URI
        {"vendor_info", {{"version", kGatewayVersion}, {"name", "ros2_medkit"}}}  // Vendor-specific info
    };

    json response = {{"items", json::array({sovd_info_entry})}};

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_version_info: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
