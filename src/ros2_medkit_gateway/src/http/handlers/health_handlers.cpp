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

#include <chrono>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/auth/auth_models.hpp"
#include "ros2_medkit_gateway/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/warning_codes.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"
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

    // Surface subscription-executor and data-provider stats via x-medkit-*
    // vendor extensions. Atomic reads only; safe to serve from /health even
    // under load. External monitors (k8s liveness, systemd watchdog, Docker
    // HEALTHCHECK) can trigger on `x-medkit-subscription-executor.degraded`.
    if (ctx_.node()) {
      if (auto * tdp = ctx_.node()->get_topic_data_provider()) {
        auto x = tdp->x_medkit_stats();
        for (auto it = x.begin(); it != x.end(); ++it) {
          response[it.key()] = it.value();
        }
      }
    }

    // Add peer status when aggregation is active
    if (auto * agg = ctx_.aggregation_manager()) {
      response["peers"] = agg->get_peer_status();

      // Contract version for the warnings array. Increment whenever a code
      // is added or the shape of a warning object changes so typed clients
      // (MCP, Web UI, Foxglove) can feature-detect instead of relying on
      // capabilities.aggregation (a boolean, too coarse).
      // Keep in sync with docs/api/warning_codes.rst "Schema versioning".
      response["warning_schema_version"] = kWarningSchemaVersion;

      // Surface operator-actionable aggregation warnings (x-medkit extension).
      // Always an array when aggregation is active; empty means no active
      // warnings. Clients can feature-detect via /.capabilities.aggregation
      // in the root response.
      json warnings = json::array();
      for (const auto & w : agg->get_leaf_warnings()) {
        std::string peers_list;
        for (size_t i = 0; i < w.peer_names.size(); ++i) {
          if (i > 0u) {
            peers_list += ", ";
          }
          peers_list += w.peer_names[i];
        }
        std::string message = "Component '" + w.entity_id + "' is announced by multiple peers (" + peers_list +
                              "); routing falls back to last-writer-wins which is non-deterministic. Resolve by "
                              "renaming the Component on one side or by modelling it as a hierarchical parent "
                              "(declare a child Component with parentComponentId='" +
                              w.entity_id + "' on the owning peer).";
        warnings.push_back({
            {"code", WARN_LEAF_ID_COLLISION},
            {"message", std::move(message)},
            {"entity_ids", json::array({w.entity_id})},
            {"peer_names", w.peer_names},
        });
      }
      response["warnings"] = std::move(warnings);
    }

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_health: %s", e.what());
  }
}

void HealthHandlers::handle_subscription_pool(const httplib::Request & req, httplib::Response & res) {
  (void)req;
  try {
    json response;
    if (ctx_.node()) {
      if (auto * tdp = ctx_.node()->get_topic_data_provider()) {
        response["x-medkit-subscription-pool"] = tdp->x_medkit_pool_snapshot();
        response["x-medkit-data-provider"] = tdp->x_medkit_stats().value("x-medkit-data-provider", json::object());
        response["x-medkit-subscription-executor"] =
            tdp->x_medkit_stats().value("x-medkit-subscription-executor", json::object());
      } else {
        response["x-medkit-subscription-pool"] = json::array();
      }
    }
    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_subscription_pool: %s", e.what());
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
        {"triggers", ctx_.node() && ctx_.node()->get_trigger_manager() != nullptr},
        {"updates", ctx_.node() && ctx_.node()->get_update_manager() != nullptr},
        {"authentication", auth_config.enabled},
        {"tls", tls_config.enabled},
        {"scripts", ctx_.node() && ctx_.node()->get_script_manager() != nullptr &&
                        ctx_.node()->get_script_manager()->has_backend()},
        {"aggregation", ctx_.aggregation_manager() != nullptr},
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
  try {
    // SOVD 7.4.1 compliant response format
    json sovd_info_entry = {
        {"version", kSovdVersion},                                                // SOVD standard version
        {"base_uri", API_BASE_PATH},                                              // Version-specific base URI
        {"vendor_info", {{"version", kGatewayVersion}, {"name", "ros2_medkit"}}}  // Vendor-specific info
    };

    json response = {{"items", json::array({sovd_info_entry})}};

    XMedkit ext;
    merge_peer_items(ctx_.aggregation_manager(), req, response, ext);
    if (!ext.empty()) {
      response["x-medkit"] = ext.build();
    }
    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_version_info: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
