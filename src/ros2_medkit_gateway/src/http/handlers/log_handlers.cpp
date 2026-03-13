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

#include "ros2_medkit_gateway/http/handlers/log_handlers.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"

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

  // Components and areas use prefix matching (all nodes under the namespace);
  // Apps use exact matching (single node FQN);
  // Functions aggregate logs from all hosted apps.
  if (entity.type == EntityType::FUNCTION) {
    // Aggregate logs from all hosted apps
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto func = cache.get_function(entity_id);
    if (!func || func->hosts.empty()) {
      json result;
      result["items"] = json::array();
      HandlerContext::send_json(res, result);
      return;
    }

    // Collect FQNs of hosted apps via effective_fqn() which falls back
    // to ros_binding in manifest_only mode where bound_fqn is not set.
    std::vector<std::string> host_fqns;
    for (const auto & app_id : func->hosts) {
      auto app = cache.get_app(app_id);
      if (!app) {
        continue;
      }
      auto fqn = app->effective_fqn();
      if (!fqn.empty()) {
        host_fqns.push_back(std::move(fqn));
      }
    }

    if (host_fqns.empty()) {
      json result;
      result["items"] = json::array();
      HandlerContext::send_json(res, result);
      return;
    }

    // get_logs accepts multiple FQNs with exact match - one call for all hosts
    auto logs = log_mgr->get_logs(host_fqns, false, min_severity, context_filter, entity_id);
    if (!logs) {
      HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
      return;
    }

    json result;
    result["items"] = std::move(*logs);
    HandlerContext::send_json(res, result);
    return;
  }

  const bool prefix_match = (entity.type == EntityType::COMPONENT || entity.type == EntityType::AREA);

  auto logs = log_mgr->get_logs({entity.fqn}, prefix_match, min_severity, context_filter, entity_id);
  if (!logs) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, logs.error());
    return;
  }

  json result;
  result["items"] = std::move(*logs);
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

  json result;
  result["severity_filter"] = cfg->severity_filter;
  result["max_entries"] = cfg->max_entries;
  HandlerContext::send_json(res, result);
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

  auto * log_mgr = ctx_.node()->get_log_manager();
  if (!log_mgr) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "LogManager not available");
    return;
  }

  json body;
  try {
    body = json::parse(req.body);
  } catch (const json::parse_error &) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON in request body");
    return;
  }

  std::optional<std::string> severity_filter;
  std::optional<size_t> max_entries;

  if (body.contains("severity_filter")) {
    if (!body["severity_filter"].is_string()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "severity_filter must be a string");
      return;
    }
    severity_filter = body["severity_filter"].get<std::string>();
  }

  static constexpr long long kMaxEntriesCap = 10000;
  if (body.contains("max_entries")) {
    const auto & me = body["max_entries"];
    if (!me.is_number_integer() && !me.is_number_unsigned()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "max_entries must be a positive integer");
      return;
    }
    long long val = 0;
    try {
      val = me.get<long long>();
    } catch (const nlohmann::json::exception &) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "max_entries value out of range");
      return;
    }
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

  // Warn about unrecognized fields (helps debug camelCase typos like "severityFilter")
  for (const auto & [key, _] : body.items()) {
    if (key != "severity_filter" && key != "max_entries") {
      RCLCPP_DEBUG(HandlerContext::logger(), "PUT /logs/configuration: ignoring unrecognized field '%s'", key.c_str());
    }
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
