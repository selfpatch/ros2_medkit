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
    HandlerContext::send_error(res, 503, ERR_INTERNAL_ERROR, "LogManager not available");
    return;
  }

  // Components use prefix matching (all nodes under the component namespace);
  // Apps use exact matching (single node FQN).
  const bool prefix_match = (entity.type == EntityType::COMPONENT);

  // Optional query parameters
  const std::string min_severity = req.get_param_value("severity");
  const std::string context_filter = req.get_param_value("context");

  auto logs = log_mgr->get_logs({entity.fqn}, prefix_match, min_severity, context_filter, entity_id);

  json result;
  result["items"] = std::move(logs);
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
    HandlerContext::send_error(res, 503, ERR_INTERNAL_ERROR, "LogManager not available");
    return;
  }

  const auto cfg = log_mgr->get_config(entity_id);

  json result;
  result["severity_filter"] = cfg.severity_filter;
  result["max_entries"] = cfg.max_entries;
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
    HandlerContext::send_error(res, 503, ERR_INTERNAL_ERROR, "LogManager not available");
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

  if (body.contains("severity_filter") && body["severity_filter"].is_string()) {
    severity_filter = body["severity_filter"].get<std::string>();
  }
  if (body.contains("max_entries") && body["max_entries"].is_number_unsigned()) {
    max_entries = body["max_entries"].get<size_t>();
  }

  const auto err = log_mgr->update_config(entity_id, severity_filter, max_entries);
  if (!err.empty()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, err);
    return;
  }

  const auto cfg = log_mgr->get_config(entity_id);
  json result;
  result["severity_filter"] = cfg.severity_filter;
  result["max_entries"] = cfg.max_entries;
  HandlerContext::send_json(res, result);
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
