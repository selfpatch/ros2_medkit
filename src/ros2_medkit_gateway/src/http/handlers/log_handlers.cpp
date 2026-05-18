// Copyright 2026 bburda
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

#include "ros2_medkit_gateway/core/http/handlers/log_handlers.hpp"

#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/logs.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {
using json = nlohmann::json;
}  // namespace

// ---------------------------------------------------------------------------
// handle_get_logs
// ---------------------------------------------------------------------------

void LogHandlers::handle_get_logs(const httplib::Request & req, httplib::Response & res) {
  if (req.matches.size() < 2) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
    return;
  }

  const auto entity_id = req.matches[1].str();
  auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity_opt) {
    return;
  }
  const auto & entity = *entity_opt;

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "LogManager not available");
    return;
  }

  // Validate optional query parameters (shared across all entity types)
  const std::string min_severity = req.get_param_value("severity");
  const std::string context_filter = req.get_param_value("context");

  if (!min_severity.empty() && !LogManager::is_valid_severity(min_severity)) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Invalid severity value: must be one of debug, info, warning, error, fatal");
    return;
  }

  static constexpr size_t kMaxContextFilterLen = 256;
  if (context_filter.size() > kMaxContextFilterLen) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "context filter exceeds maximum length of 256");
    return;
  }

  // -----------------------------------------------------------------------
  // FUNCTION - aggregate local hosted app logs + fan-out to peers
  // -----------------------------------------------------------------------
  if (entity.type == EntityType::FUNCTION) {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto func = cache.get_function(entity_id);

    json result;
    result["items"] = json::array();
    dto::LogListXMedkit xm;
    xm.entity_id = entity_id;
    xm.aggregation_level = "function";
    xm.aggregated = true;

    if (func && !func->hosts.empty()) {
      auto host_fqns = HandlerContext::resolve_app_host_fqns(cache, func->hosts);

      if (!host_fqns.empty()) {
        auto logs = log_mgr->get_logs(host_fqns, false, min_severity, context_filter, entity_id);
        if (!logs) {
          HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
          return;
        }
        result["items"] = std::move(*logs);
        xm.host_count = static_cast<int64_t>(host_fqns.size());
        std::vector<std::string> sources(host_fqns.begin(), host_fqns.end());
        xm.aggregation_sources = std::move(sources);
      }
    }

    auto xm_json = dto::JsonWriter<dto::LogListXMedkit>::write(xm);
    merge_peer_items(ctx_.aggregation_manager(), req, result, xm_json);
    result["x-medkit"] = std::move(xm_json);
    HandlerContext::send_json(res, result);
    return;
  }

  // -----------------------------------------------------------------------
  // AREA - aggregate local app logs from all components + fan-out to peers
  // -----------------------------------------------------------------------
  if (entity.type == EntityType::AREA) {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_ids = cache.get_components_for_area(entity_id);

    std::vector<std::string> host_fqns;
    for (const auto & comp_id : comp_ids) {
      auto comp_fqns = HandlerContext::resolve_app_host_fqns(cache, cache.get_apps_for_component(comp_id));
      host_fqns.insert(host_fqns.end(), std::make_move_iterator(comp_fqns.begin()),
                       std::make_move_iterator(comp_fqns.end()));
    }

    json result;
    dto::LogListXMedkit xm;
    xm.entity_id = entity_id;
    xm.aggregation_level = "area";
    xm.aggregated = true;

    if (!host_fqns.empty()) {
      auto logs = log_mgr->get_logs(host_fqns, false, min_severity, context_filter, entity_id);
      if (!logs) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
        return;
      }
      result["items"] = std::move(*logs);
      xm.component_count = static_cast<int64_t>(comp_ids.size());
      xm.app_count = static_cast<int64_t>(host_fqns.size());
      std::vector<std::string> sources(host_fqns.begin(), host_fqns.end());
      xm.aggregation_sources = std::move(sources);
    } else {
      auto logs = log_mgr->get_logs({entity.fqn}, true, min_severity, context_filter, entity_id);
      if (!logs) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
        return;
      }
      result["items"] = std::move(*logs);
    }

    auto xm_json = dto::JsonWriter<dto::LogListXMedkit>::write(xm);
    merge_peer_items(ctx_.aggregation_manager(), req, result, xm_json);
    result["x-medkit"] = std::move(xm_json);
    HandlerContext::send_json(res, result);
    return;
  }

  // -----------------------------------------------------------------------
  // COMPONENT - aggregate from hosted apps' fqns (mirrors AREA / FUNCTION
  // semantics) and fall through to the entity.fqn prefix path only when
  // the component has no hosted apps. Synthetic / runtime-discovered
  // components have an empty fqn, so the bare prefix-match query that
  // worked for manifest components silently returned zero items even
  // when the component grouped 20+ active nodes.
  // -----------------------------------------------------------------------
  if (entity.type == EntityType::COMPONENT) {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto host_fqns = HandlerContext::resolve_app_host_fqns(cache, cache.get_apps_for_component(entity_id));

    json result;
    result["items"] = json::array();
    dto::LogListXMedkit xm;
    xm.entity_id = entity_id;
    xm.aggregation_level = "component";
    xm.aggregated = true;

    if (!host_fqns.empty()) {
      auto logs = log_mgr->get_logs(host_fqns, false, min_severity, context_filter, entity_id);
      if (!logs) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
        return;
      }
      result["items"] = std::move(*logs);
      xm.app_count = static_cast<int64_t>(host_fqns.size());
      std::vector<std::string> sources(host_fqns.begin(), host_fqns.end());
      xm.aggregation_sources = std::move(sources);
    } else if (!entity.fqn.empty()) {
      // Manifest component without hosted apps - keep the original
      // namespace prefix path so manifest-only deployments still work.
      auto logs = log_mgr->get_logs({entity.fqn}, true, min_severity, context_filter, entity_id);
      if (!logs) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
        return;
      }
      result["items"] = std::move(*logs);
    }

    auto xm_json = dto::JsonWriter<dto::LogListXMedkit>::write(xm);
    merge_peer_items(ctx_.aggregation_manager(), req, result, xm_json);
    result["x-medkit"] = std::move(xm_json);
    HandlerContext::send_json(res, result);
    return;
  }

  // -----------------------------------------------------------------------
  // APP - local query + fan-out to peers
  // -----------------------------------------------------------------------
  auto logs = log_mgr->get_logs({entity.fqn}, false, min_severity, context_filter, entity_id);
  if (!logs) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
    return;
  }

  json result;
  result["items"] = std::move(*logs);

  dto::LogListXMedkit xm;
  auto xm_json = dto::JsonWriter<dto::LogListXMedkit>::write(xm);
  merge_peer_items(ctx_.aggregation_manager(), req, result, xm_json);
  if (!xm_json.empty()) {
    result["x-medkit"] = std::move(xm_json);
  }
  HandlerContext::send_json(res, result);
}

// ---------------------------------------------------------------------------
// handle_get_logs_configuration
// ---------------------------------------------------------------------------

void LogHandlers::handle_get_logs_configuration(const httplib::Request & req, httplib::Response & res) {
  if (req.matches.size() < 2) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
    return;
  }

  const auto entity_id = req.matches[1].str();
  auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity_opt) {
    return;
  }

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "LogManager not available");
    return;
  }

  auto cfg = log_mgr->get_config(entity_id);
  if (!cfg) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, cfg.error());
    return;
  }

  dto::LogConfiguration response;
  response.severity_filter = cfg->severity_filter;
  response.max_entries = static_cast<int64_t>(cfg->max_entries);
  HandlerContext::send_dto(res, response);
}

// ---------------------------------------------------------------------------
// handle_put_logs_configuration
// ---------------------------------------------------------------------------

void LogHandlers::handle_put_logs_configuration(const httplib::Request & req, httplib::Response & res) {
  if (req.matches.size() < 2) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
    return;
  }

  const auto entity_id = req.matches[1].str();
  auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity_opt) {
    return;
  }

  // Check lock access for logs
  if (ctx_.validate_lock_access(req, res, *entity_opt, "logs")) {
    return;
  }

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "LogManager not available");
    return;
  }

  auto body = ctx_.parse_body<dto::LogConfiguration>(req, res);
  if (!body) {
    return;
  }

  std::optional<std::string> severity_filter;
  std::optional<size_t> max_entries;

  if (body->severity_filter.has_value()) {
    severity_filter = body->severity_filter;
  }

  static constexpr int64_t kMaxEntriesCap = 10000;
  if (body->max_entries.has_value()) {
    const int64_t val = *body->max_entries;
    if (val <= 0) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "max_entries must be greater than 0");
      return;
    }
    if (val > kMaxEntriesCap) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "max_entries exceeds maximum allowed value of 10000");
      return;
    }
    max_entries = static_cast<size_t>(val);
  }

  const auto err = log_mgr->update_config(entity_id, severity_filter, max_entries);
  if (!err.empty()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, err);
    return;
  }

  res.status = 204;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
