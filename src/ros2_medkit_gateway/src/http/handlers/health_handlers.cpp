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

#include "ros2_medkit_gateway/core/http/handlers/health_handlers.hpp"

#include <chrono>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/auth/auth_models.hpp"
#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/discovery/discovery_enums.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/http/warning_codes.hpp"
#include "ros2_medkit_gateway/core/version.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/dto/health.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

#include "../../openapi/route_registry.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

ErrorInfo make_internal_error(const char * where, const std::exception & e) {
  RCLCPP_ERROR(HandlerContext::logger(), "Error in %s: %s", where, e.what());
  ErrorInfo err;
  err.code = ERR_INTERNAL_ERROR;
  err.message = "Internal server error";
  err.http_status = 500;
  return err;
}

}  // namespace

http::Result<dto::Health> HealthHandlers::get_health(const http::TypedRequest & req) {
  (void)req;  // Unused parameter
  try {
    dto::Health response;
    response.status = "healthy";
    response.timestamp = std::chrono::system_clock::now().time_since_epoch().count();

    // Add discovery info
    auto * dm = ctx_.node() ? ctx_.node()->get_discovery_manager() : nullptr;
    if (dm) {
      dto::HealthDiscovery discovery;
      discovery.mode = discovery_mode_to_string(dm->get_mode());
      discovery.strategy = dm->get_strategy_name();

      auto report = dm->get_merge_report();
      if (report) {
        discovery.pipeline = report->to_json();
      }

      auto linking = dm->get_linking_result();
      if (linking) {
        dto::HealthDiscoveryLinking linking_dto;
        linking_dto.linked_count = static_cast<int64_t>(linking->node_to_app.size());
        linking_dto.orphan_count = static_cast<int64_t>(linking->orphan_nodes.size());
        linking_dto.binding_conflicts = static_cast<int64_t>(linking->binding_conflicts);
        if (!linking->warnings.empty()) {
          linking_dto.warnings = linking->warnings;
        }
        discovery.linking = linking_dto;
      }

      response.discovery = std::move(discovery);
    }

    // Surface subscription-executor and data-provider stats via x-medkit-*
    // vendor extensions. Atomic reads only; safe to serve from /health even
    // under load. External monitors (k8s liveness, systemd watchdog, Docker
    // HEALTHCHECK) can trigger on `x-medkit-subscription-executor.degraded`.
    if (ctx_.node()) {
      if (auto * tdp = ctx_.node()->get_topic_data_provider()) {
        auto x = tdp->x_medkit_stats();
        if (x.contains("x-medkit-data-provider")) {
          response.x_medkit_data_provider = x["x-medkit-data-provider"];
        }
        if (x.contains("x-medkit-subscription-executor")) {
          response.x_medkit_subscription_executor = x["x-medkit-subscription-executor"];
        }
      }
    }

    // Add peer status when aggregation is active
    if (auto * agg = ctx_.aggregation_manager()) {
      response.peers = agg->get_peer_status();

      // Contract version for the warnings array. Increment whenever a code
      // is added or the shape of a warning object changes so typed clients
      // (MCP, Web UI, Foxglove) can feature-detect instead of relying on
      // capabilities.aggregation (a boolean, too coarse).
      // Keep in sync with docs/api/warning_codes.rst "Schema versioning".
      response.warning_schema_version = static_cast<int64_t>(kWarningSchemaVersion);

      // Surface operator-actionable aggregation warnings (x-medkit extension).
      // Always an array when aggregation is active; empty means no active
      // warnings. Clients can feature-detect via /.capabilities.aggregation
      // in the root response.
      std::vector<dto::HealthAggregationWarning> warnings;
      for (const auto & w : agg->get_leaf_warnings()) {
        std::string peers_list;
        for (size_t i = 0; i < w.peer_names.size(); ++i) {
          if (i > 0u) {
            peers_list += ", ";
          }
          peers_list += w.peer_names[i];
        }
        dto::HealthAggregationWarning warning;
        warning.code = WARN_LEAF_ID_COLLISION;
        warning.message = "Component '" + w.entity_id + "' is announced by multiple peers (" + peers_list +
                          "); routing falls back to last-writer-wins which is non-deterministic. Resolve by "
                          "renaming the Component on one side or by modelling it as a hierarchical parent "
                          "(declare a child Component with parentComponentId='" +
                          w.entity_id + "' on the owning peer).";
        warning.entity_ids = {w.entity_id};
        warning.peer_names = w.peer_names;
        warnings.push_back(std::move(warning));
      }
      response.warnings = std::move(warnings);
    }

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_health", e));
  }
}

http::Result<dto::RootOverview> HealthHandlers::get_root(const http::TypedRequest & req) {
  (void)req;  // Unused parameter
  try {
    // Generate endpoint list from route registry (single source of truth)
    std::vector<std::string> endpoints;
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

    dto::RootCapabilities capabilities;
    capabilities.discovery = true;
    capabilities.data_access = true;
    capabilities.operations = true;
    capabilities.async_actions = true;
    capabilities.configurations = true;
    capabilities.faults = true;
    capabilities.logs = true;
    capabilities.bulk_data = true;
    capabilities.cyclic_subscriptions = true;
    capabilities.locking = ctx_.node() && ctx_.node()->get_lock_manager() != nullptr;
    capabilities.triggers = ctx_.node() && ctx_.node()->get_trigger_manager() != nullptr;
    capabilities.updates = ctx_.node() && ctx_.node()->get_update_manager() != nullptr;
    capabilities.authentication = auth_config.enabled;
    capabilities.tls = tls_config.enabled;
    capabilities.scripts =
        ctx_.node() && ctx_.node()->get_script_manager() != nullptr && ctx_.node()->get_script_manager()->has_backend();
    capabilities.aggregation = ctx_.aggregation_manager() != nullptr;
    capabilities.vendor_extensions =
        ctx_.node() && ctx_.node()->get_plugin_manager() && ctx_.node()->get_plugin_manager()->has_plugins();

    dto::RootOverview response;
    response.name = "ROS 2 Medkit Gateway";
    response.version = kGatewayVersion;
    response.api_base = API_BASE_PATH;
    response.endpoints = std::move(endpoints);
    response.capabilities = capabilities;

    // Add auth info if enabled
    if (auth_config.enabled) {
      dto::RootAuth auth;
      auth.enabled = true;
      auth.algorithm = algorithm_to_string(auth_config.jwt_algorithm);
      auth.require_auth_for = auth_config.require_auth_for == AuthRequirement::NONE    ? "none"
                              : auth_config.require_auth_for == AuthRequirement::WRITE ? "write"
                                                                                       : "all";
      response.auth = std::move(auth);
    }

    // Add TLS info if enabled
    if (tls_config.enabled) {
      dto::RootTls tls;
      tls.enabled = true;
      tls.min_version = tls_config.min_version;
      // TODO(future): Add mutual_tls when implemented
      response.tls = std::move(tls);
    }

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_root", e));
  }
}

http::Result<dto::VersionInfo> HealthHandlers::get_version_info(const http::TypedRequest & req) {
  try {
    // SOVD 7.4.1 compliant response format
    dto::VersionInfoVendor vendor;
    vendor.version = kGatewayVersion;
    vendor.name = "ros2_medkit";

    dto::VersionInfoEntry entry;
    entry.version = kSovdVersion;    // SOVD standard version
    entry.base_uri = API_BASE_PATH;  // Version-specific base URI
    entry.vendor_info = std::move(vendor);

    dto::VersionInfo response;
    response.items.push_back(std::move(entry));

    // Fan-out aggregation: merge items from peers and collect x-medkit metadata.
    // merge_peer_items operates on the raw cpp-httplib request; use the
    // framework-internal escape hatch deliberately (handlers do not own that
    // helper's signature yet - typed fan-out for non-collection endpoints lands
    // in a later commit of this PR).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
    json response_json = dto::JsonWriter<dto::VersionInfo>::write(response);
    dto::XMedkitVersionInfo ext_dto;
    json ext_json = json::object();
    merge_peer_items(ctx_.aggregation_manager(), raw_req, response_json, ext_json);
    if (!ext_json.empty()) {
      if (ext_json.contains("partial")) {
        ext_dto.partial = ext_json["partial"].get<bool>();
      }
      if (ext_json.contains("failed_peers")) {
        ext_dto.failed_peers = ext_json["failed_peers"].get<std::vector<std::string>>();
      }
    }

    // Re-parse merged items back into the DTO and attach x-medkit if present
    if (response_json.contains("items") && response_json["items"].is_array()) {
      response.items.clear();
      for (const auto & item : response_json["items"]) {
        auto parsed = dto::JsonReader<dto::VersionInfoEntry>::read(item);
        if (parsed.has_value()) {
          response.items.push_back(std::move(*parsed));
        }
      }
    }
    if (ext_dto.partial.has_value() || ext_dto.failed_peers.has_value()) {
      response.x_medkit = std::move(ext_dto);
    }

    return response;
  } catch (const std::exception & e) {
    return tl::unexpected(make_internal_error("get_version_info", e));
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
