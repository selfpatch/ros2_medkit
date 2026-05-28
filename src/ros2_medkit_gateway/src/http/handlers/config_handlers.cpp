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

#include "ros2_medkit_gateway/core/http/handlers/config_handlers.hpp"

#include <future>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

using json = nlohmann::json;

// =============================================================================
// Constants
// =============================================================================

/// Maximum length for aggregated parameter IDs.
///
/// Aggregated parameter IDs use the format `app_id:param_name`, so the max
/// length is:
///   - app_id    : up to 256 characters (entity ID limit)
///   - separator : 1 character (`:`)
///   - param_name: up to 256 characters (ROS 2 parameter name limit)
constexpr size_t kMaxAggregatedParamIdLength = 512;

// =============================================================================
// Helper structures and free functions
// =============================================================================

/// Build a SOVD-shaped ErrorInfo. Empty `params` are dropped so the wire body
/// matches the legacy `send_error` default and integration tests stay byte-
/// identical.
ErrorInfo make_error(int status, const std::string & code, std::string message, json params = {}) {
  ErrorInfo err;
  err.code = code;
  err.message = std::move(message);
  err.http_status = status;
  if (!params.is_null() && !params.empty()) {
    err.params = std::move(params);
  }
  return err;
}

/// Read the positional entity-id capture group from the typed request. cpp-
/// httplib only invokes the route when the regex matches, so the `nullopt`
/// branch is effectively unreachable; we surface it as 400 invalid-request to
/// match the legacy handlers' explicit `req.matches.size() < 2` guard.
tl::expected<std::string, ErrorInfo> read_entity_id(const http::TypedRequest & req) {
  auto raw = req.path_param("1");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Read the second positional capture group (the parameter id) with the same
/// "missing capture is treated as 400 invalid-request" semantics as
/// `read_entity_id`.
tl::expected<std::string, ErrorInfo> read_param_id(const http::TypedRequest & req) {
  auto raw = req.path_param("2");
  if (raw) {
    return *raw;
  }
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid request"));
}

/// Convert a ValidatorResult's error variant into a typed Result<T> error.
/// When the validator returned Forwarded, the proxy already wrote the wire
/// response, so the handler signals "do not render" via the framework-internal
/// sentinel (ERR_X_INTERNAL_FORWARDED) the typed wrapper detects.
ErrorInfo flatten_validator_error(const std::variant<ErrorInfo, http::Forwarded> & err) {
  return std::visit(
      [](auto && alt) -> ErrorInfo {
        using T = std::decay_t<decltype(alt)>;
        if constexpr (std::is_same_v<T, ErrorInfo>) {
          return alt;
        } else {
          return HandlerContext::forwarded_sentinel_error();
        }
      },
      err);
}

/// Parsed parameter ID with optional app prefix.
struct ParsedParamId {
  std::string app_id;      ///< Target app ID (empty if not prefixed)
  std::string param_name;  ///< Parameter name
  bool has_prefix{false};  ///< Whether the ID had an app prefix
};

/// Parse param_id which may carry an `app_id:param_name` prefix for
/// aggregated configurations. For aggregated entities the prefix
/// disambiguates which app's parameter is targeted; for non-aggregated
/// entities the colon (if any) is treated as part of the parameter name.
ParsedParamId parse_aggregated_param_id(const std::string & param_id, bool is_aggregated) {
  ParsedParamId result;
  result.param_name = param_id;

  auto colon_pos = param_id.find(':');
  if (colon_pos != std::string::npos && is_aggregated) {
    result.app_id = param_id.substr(0, colon_pos);
    result.param_name = param_id.substr(colon_pos + 1);
    result.has_prefix = true;
  }

  return result;
}

/// Find node info for a specific app ID in aggregated configurations.
const NodeConfigInfo * find_node_for_app(const std::vector<NodeConfigInfo> & nodes, const std::string & app_id) {
  for (const auto & node : nodes) {
    if (node.app_id == app_id) {
      return &node;
    }
  }
  return nullptr;
}

/// Error classification result for parameter operations.
struct ErrorClassification {
  int status_code;
  std::string error_code;
};

/// Map a structured `ParameterErrorCode` to an HTTP status + SOVD error code.
/// Identical mapping to the legacy `classify_error_code` helper.
ErrorClassification classify_error_code(ParameterErrorCode error_code) {
  ErrorClassification result;

  switch (error_code) {
    case ParameterErrorCode::NOT_FOUND:
    case ParameterErrorCode::NO_DEFAULTS_CACHED:
      result.status_code = 404;
      result.error_code = ERR_RESOURCE_NOT_FOUND;
      break;
    case ParameterErrorCode::READ_ONLY:
      result.status_code = 403;
      result.error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
      break;
    case ParameterErrorCode::SERVICE_UNAVAILABLE:
    case ParameterErrorCode::TIMEOUT:
      result.status_code = 503;
      result.error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
      break;
    case ParameterErrorCode::TYPE_MISMATCH:
    case ParameterErrorCode::INVALID_VALUE:
      result.status_code = 400;
      result.error_code = ERR_INVALID_PARAMETER;
      break;
    case ParameterErrorCode::NONE:
      // NONE means "no error" - caller (classify_parameter_error) pre-filters
      // this case, so reaching here indicates a programming error.
      result.status_code = 500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
    case ParameterErrorCode::SHUT_DOWN:
    case ParameterErrorCode::INTERNAL_ERROR:
    default:
      result.status_code = 500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
  }

  return result;
}

/// Classify a ParameterResult into HTTP status + error code. Prefers the
/// structured `error_code` field and falls back to string-matching the legacy
/// error messages for backward compatibility with older transports.
ErrorClassification classify_parameter_error(const ParameterResult & result) {
  // Prefer structured error code
  if (result.error_code != ParameterErrorCode::NONE) {
    return classify_error_code(result.error_code);
  }

  // Fallback to string parsing for backward compatibility
  ErrorClassification classification;
  const auto & error_message = result.error_message;

  if (error_message.find("not found") != std::string::npos ||
      error_message.find("Parameter not found") != std::string::npos) {
    classification.status_code = 404;
    classification.error_code = ERR_RESOURCE_NOT_FOUND;
  } else if (error_message.find("read-only") != std::string::npos ||
             error_message.find("read only") != std::string::npos ||
             error_message.find("is read_only") != std::string::npos) {
    classification.status_code = 403;
    classification.error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
  } else if (error_message.find("not available") != std::string::npos ||
             error_message.find("service not available") != std::string::npos ||
             error_message.find("timed out") != std::string::npos ||
             error_message.find("timeout") != std::string::npos) {
    classification.status_code = 503;
    classification.error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
  } else {
    classification.status_code = 400;
    classification.error_code = ERR_INVALID_REQUEST;
  }

  return classification;
}

/// Build a typed `ErrorInfo` for a failed parameter operation. Mirrors the
/// legacy `send_parameter_error` helper's wire shape (params: details +
/// entity_id + id, message: "Failed to <op> parameter").
ErrorInfo make_parameter_error(const ParameterResult & result, const std::string & operation_name,
                               const std::string & entity_id, const std::string & param_id) {
  auto err = classify_parameter_error(result);
  std::string message = "Failed to " + operation_name + " parameter";
  return make_error(err.status_code, err.error_code, std::move(message),
                    json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
}

/// Build a typed `ErrorInfo` for the GET path when a parameter is missing on
/// a specific node. Legacy uses message "Parameter not found" for 404 and
/// "Failed to get parameter" otherwise; we preserve that branching here so
/// the wire shape is byte-identical.
ErrorInfo make_get_parameter_error(const ParameterResult & result, const std::string & entity_id,
                                   const std::string & param_id) {
  auto err = classify_parameter_error(result);
  std::string message = err.status_code == 404 ? "Parameter not found" : "Failed to get parameter";
  return make_error(err.status_code, err.error_code, std::move(message),
                    json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
}

/// Build the `ConfigurationReadValue` DTO and accompanying `ConfigValueXMedkit`
/// payload for a successful get/set on a specific node. Shared between the
/// happy paths of `get_configuration` and `set_configuration`.
dto::ConfigurationReadValue make_read_value(const std::string & entity_id, const std::string & node_fqn,
                                            const std::string & param_id, const std::optional<std::string> & source_app,
                                            const json & param_data) {
  dto::ConfigValueXMedkit xm;
  xm.ros2 = dto::XMedkitRos2{};
  xm.ros2->node = node_fqn;
  xm.entity_id = entity_id;
  xm.source = "runtime";
  xm.parameter = param_data;
  if (source_app.has_value()) {
    xm.source_app = *source_app;
  }

  dto::ConfigurationReadValue resp;
  resp.id = param_id;
  resp.data = param_data.contains("value") ? param_data["value"] : param_data;
  resp.x_medkit = std::move(xm);
  return resp;
}

/// Build a single `ConfigurationMetaData` item for the list response.
dto::ConfigurationMetaData make_meta_item(const json & param, const NodeConfigInfo & node_info, bool is_aggregated) {
  dto::ConfigurationMetaData meta;
  std::string param_name = param.value("name", "");

  // Unique ID for aggregated configs is `app_id:param_name` so callers can
  // disambiguate same-named parameters across nodes hosted by the same SOVD
  // entity. Non-aggregated entities just use the bare parameter name.
  meta.id = is_aggregated ? (node_info.app_id + ":" + param_name) : param_name;
  meta.name = param_name;
  meta.type = "parameter";

  if (is_aggregated) {
    dto::ConfigXMedkitItem item_xm;
    item_xm.source = node_info.app_id;
    meta.x_medkit = std::move(item_xm);
  }
  return meta;
}

/// Single node-query result used by the parallel list-parameters fan-out.
struct NodeQueryResult {
  NodeConfigInfo node_info;
  ParameterResult result;
};

/// Run `ConfigurationManager::list_parameters` in parallel against every
/// node in `agg_configs.nodes` and return the per-node results. The order
/// matches `agg_configs.nodes` to keep deterministic per-test output.
std::vector<NodeQueryResult> query_nodes_in_parallel(ConfigurationManager * config_mgr,
                                                     const AggregatedConfigurations & agg_configs) {
  std::vector<std::future<NodeQueryResult>> futures;
  futures.reserve(agg_configs.nodes.size());

  for (const auto & node_info : agg_configs.nodes) {
    futures.push_back(std::async(std::launch::async, [config_mgr, node_info]() {
      NodeQueryResult query_result;
      query_result.node_info = node_info;
      query_result.result = config_mgr->list_parameters(node_info.node_fqn);
      return query_result;
    }));
  }

  std::vector<NodeQueryResult> results;
  results.reserve(futures.size());
  for (auto & future : futures) {
    results.push_back(future.get());
  }
  return results;
}

/// Fold the partial/failed_peers/peer_dropped_items observability fields
/// from a `FanOutResult<ConfigurationMetaData>` into the typed
/// `ConfigListXMedkit`. Mirrors the legacy behaviour where
/// `merge_peer_items` injected `partial` and `failed_peers` keys into the
/// x-medkit JSON; here the typed equivalent is a direct field assignment
/// plus dropped-item enrichment from the typed reader.
void apply_fan_out_observability(dto::ConfigListXMedkit & xm,
                                 const FanOutResult<dto::ConfigurationMetaData> & fan_out) {
  if (fan_out.partial) {
    xm.partial = true;
    xm.failed_peers = fan_out.failed_peers;
  }
  if (!fan_out.dropped_items.empty()) {
    xm.peer_dropped_items = fan_out.dropped_items;
  }
}

}  // namespace

// ===========================================================================
// GET /{entity-path}/configurations
// ===========================================================================

http::Result<dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit>>
ConfigHandlers::list_configurations(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit> response;
  dto::ConfigListXMedkit xm;
  xm.entity_id = entity_id;
  xm.source = "runtime";
  xm.aggregation_level = agg_configs.aggregation_level;
  xm.is_aggregated = agg_configs.is_aggregated;

  json all_parameters = json::array();
  std::vector<std::string> queried_nodes;

  if (!agg_configs.nodes.empty()) {
    auto * config_mgr = ctx_.node()->get_configuration_manager();
    const auto node_results = query_nodes_in_parallel(config_mgr, agg_configs);

    bool any_success = false;
    std::string first_error;
    std::string first_error_node;  // Track which node failed for better diagnostics

    for (const auto & qr : node_results) {
      const auto & node_info = qr.node_info;
      const auto & result = qr.result;

      if (result.success) {
        any_success = true;
        queried_nodes.push_back(node_info.node_fqn);

        if (result.data.is_array()) {
          for (const auto & param : result.data) {
            response.items.push_back(make_meta_item(param, node_info, agg_configs.is_aggregated));

            // Also track the full parameter details (with x-medkit source/node)
            // inside the x-medkit `parameters` array. Free-form JSON because the
            // ConfigurationManager produces vendor-specific raw output here.
            json param_with_source = param;
            dto::ConfigXMedkitItem param_xm;
            param_xm.source = node_info.app_id;
            param_xm.node = node_info.node_fqn;
            param_with_source["x-medkit"] = dto::JsonWriter<dto::ConfigXMedkitItem>::write(param_xm);
            all_parameters.push_back(std::move(param_with_source));
          }
        }
      } else if (first_error.empty()) {
        first_error = result.error_message;
        first_error_node = node_info.node_fqn;
      }
    }

    if (!any_success) {
      return tl::unexpected(
          make_error(503, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE, "Failed to list parameters from any node",
                     json{{"details", first_error}, {"entity_id", entity_id}, {"failed_node", first_error_node}}));
    }

    xm.parameters = all_parameters;
    if (!agg_configs.source_ids.empty()) {
      xm.source_ids = agg_configs.source_ids;
    }
    if (!queried_nodes.empty()) {
      xm.queried_nodes = queried_nodes;
    }
  }

  // Typed fan-out for collection endpoints. Replaces the legacy raw-JSON
  // `merge_peer_items` mutator: peer items come back as parsed
  // `dto::ConfigurationMetaData` values via `JsonReader<ConfigurationMetaData>`,
  // malformed peer items are surfaced as `dropped_items` (folded into
  // xm.peer_dropped_items below), and partial/failed_peers go into the typed
  // xm fields. `fan_out_collection` still operates on the raw cpp-httplib
  // request (path + headers) - the typed-request raw escape hatch is used
  // deliberately here; a later commit will accept the typed request directly.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto & raw_req = req.raw_for_framework();
#pragma GCC diagnostic pop
  auto fan_out = fan_out_collection<dto::ConfigurationMetaData>(ctx_.aggregation_manager(), raw_req);
  for (auto & item : fan_out.items) {
    response.items.push_back(std::move(item));
  }
  apply_fan_out_observability(xm, fan_out);

  response.x_medkit = std::move(xm);
  return response;
}

// ===========================================================================
// GET /{entity-path}/configurations/{param_name}
// ===========================================================================

http::Result<dto::ConfigurationReadValue> ConfigHandlers::get_configuration(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto param_id_result = read_param_id(req);
  if (!param_id_result) {
    return tl::unexpected(param_id_result.error());
  }
  const std::string param_id = *param_id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }

  // Parameter ID may be prefixed with app_id: for aggregated configs.
  if (param_id.empty() || param_id.length() > kMaxAggregatedParamIdLength) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                     json{{"details", "Parameter ID is empty or too long"}}));
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  if (agg_configs.nodes.empty()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();
  auto parsed = parse_aggregated_param_id(param_id, agg_configs.is_aggregated);

  // If targeting a specific app in an aggregated entity, dispatch to that
  // app's node directly.
  if (parsed.has_prefix) {
    const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
    if (node_info == nullptr) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Source app not found in entity",
                     json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}}));
    }

    auto result = config_mgr->get_parameter(node_info->node_fqn, parsed.param_name);
    if (!result.success) {
      return tl::unexpected(make_get_parameter_error(result, entity_id, param_id));
    }
    return make_read_value(entity_id, node_info->node_fqn, param_id, parsed.app_id, result.data);
  }

  // For non-aggregated entities (or aggregated entities without a prefix) we
  // probe every backing node for the parameter and return the first success.
  // Track errors so a non-404 from any node (e.g. unavailable) wins over a
  // pure "not found" verdict in the no-success branch.
  ParameterResult last_result;
  bool all_not_found = true;

  for (const auto & node_info : agg_configs.nodes) {
    auto result = config_mgr->get_parameter(node_info.node_fqn, parsed.param_name);
    if (result.success) {
      std::optional<std::string> source_app;
      if (agg_configs.is_aggregated) {
        source_app = node_info.app_id;
      }
      return make_read_value(entity_id, node_info.node_fqn, parsed.param_name, source_app, result.data);
    }

    last_result = result;
    auto err = classify_parameter_error(result);
    if (err.status_code != 404) {
      all_not_found = false;
    }
  }

  if (all_not_found) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Parameter not found",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  // Some node reported a non-404 (e.g. unavailable); surface that.
  auto err = classify_parameter_error(last_result);
  return tl::unexpected(
      make_error(err.status_code, err.error_code, "Failed to get parameter from any node",
                 json{{"details", last_result.error_message}, {"entity_id", entity_id}, {"id", param_id}}));
}

// ===========================================================================
// PUT /{entity-path}/configurations/{param_name}
// ===========================================================================

http::Result<dto::ConfigurationReadValue> ConfigHandlers::set_configuration(const http::TypedRequest & req,
                                                                            dto::ConfigurationWriteRequest body) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto param_id_result = read_param_id(req);
  if (!param_id_result) {
    return tl::unexpected(param_id_result.error());
  }
  const std::string param_id = *param_id_result;

  // Validate entity_id format first so that pathological IDs get a 400
  // before we touch the cache.
  if (auto vr = ctx_.validate_entity_id(entity_id); !vr) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                     json{{"details", vr.error()}, {"entity_id", entity_id}}));
  }

  if (param_id.empty() || param_id.length() > kMaxAggregatedParamIdLength) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                     json{{"details", "Parameter ID is empty or too long"}}));
  }

  // Both `data` and `value` are optional at the DTO level so legacy clients
  // sending `{"value": ...}` are not rejected by the typed body parser. We
  // enforce "at least one present" here and prefer `data` over `value` to
  // match the legacy handler's behaviour bit-for-bit.
  json config_value;
  if (body.data.has_value()) {
    config_value = *body.data;
  } else if (body.value.has_value()) {
    config_value = *body.value;
  } else {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Request body must contain a 'data' field"));
  }

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "configurations"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  if (agg_configs.nodes.empty()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();
  auto parsed = parse_aggregated_param_id(param_id, agg_configs.is_aggregated);

  // Helper: turn a successful set into the typed response. The wire shape
  // matches the legacy handler exactly - the response id is the original
  // request param_id (which includes the app_id: prefix if any), so callers
  // who issued `PUT .../foo:bar` see `"id": "foo:bar"` come back.
  auto on_success = [&](const std::string & node_fqn, const std::optional<std::string> & source_app,
                        const json & param_data) -> dto::ConfigurationReadValue {
    return make_read_value(entity_id, node_fqn, param_id, source_app, param_data);
  };

  if (parsed.has_prefix) {
    const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
    if (node_info == nullptr) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Source app not found in entity",
                     json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}}));
    }

    auto result = config_mgr->set_parameter(node_info->node_fqn, parsed.param_name, config_value);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "set", entity_id, param_id));
    }
    return on_success(node_info->node_fqn, parsed.app_id, result.data);
  }

  // Non-aggregated entity: target the single backing node.
  if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
    const auto & node_info = agg_configs.nodes[0];
    auto result = config_mgr->set_parameter(node_info.node_fqn, parsed.param_name, config_value);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "set", entity_id, param_id));
    }
    // Non-aggregated case never carries source_app; keep the wire shape
    // legacy-compatible by passing nullopt.
    return on_success(node_info.node_fqn, std::nullopt, result.data);
  }

  // Aggregated entity but no prefix: ambiguous, reject.
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Aggregated configuration requires app_id prefix",
                                   json{{"details", "Use format 'app_id:param_name' for aggregated configurations"},
                                        {"entity_id", entity_id},
                                        {"id", param_id}}));
}

// ===========================================================================
// DELETE /{entity-path}/configurations/{param_name}
// ===========================================================================

http::Result<http::NoContent> ConfigHandlers::delete_configuration(const http::TypedRequest & req) {
  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto param_id_result = read_param_id(req);
  if (!param_id_result) {
    return tl::unexpected(param_id_result.error());
  }
  const std::string param_id = *param_id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "configurations"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  if (agg_configs.nodes.empty()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                     json{{"entity_id", entity_id}, {"id", param_id}}));
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();
  auto parsed = parse_aggregated_param_id(param_id, agg_configs.is_aggregated);

  if (parsed.has_prefix) {
    const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
    if (node_info == nullptr) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Source app not found in entity",
                     json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}}));
    }

    auto result = config_mgr->reset_parameter(node_info->node_fqn, parsed.param_name);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "reset", entity_id, param_id));
    }
    return http::NoContent{};
  }

  if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
    const auto & node_info = agg_configs.nodes[0];
    auto result = config_mgr->reset_parameter(node_info.node_fqn, parsed.param_name);
    if (!result.success) {
      return tl::unexpected(make_parameter_error(result, "reset", entity_id, param_id));
    }
    return http::NoContent{};
  }

  // Aggregated configs without a prefix - we cannot resolve a single target node.
  return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Aggregated configuration requires app_id prefix",
                                   json{{"details", "Use format 'app_id:param_name' for aggregated configurations"},
                                        {"entity_id", entity_id},
                                        {"id", param_id}}));
}

// ===========================================================================
// DELETE /{entity-path}/configurations - reset all (204 success / 207 partial)
// ===========================================================================

http::Result<std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>>
ConfigHandlers::delete_all_configurations(const http::TypedRequest & req) {
  using ResultVariant = std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>;

  auto id_result = read_entity_id(req);
  if (!id_result) {
    return tl::unexpected(id_result.error());
  }
  const std::string entity_id = *id_result;

  auto entity_result = ctx_.validate_entity_for_route(req, entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "configurations"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  const auto & cache = ctx_.node()->get_thread_safe_cache();
  auto agg_configs = cache.get_entity_configurations(entity_id);

  if (agg_configs.nodes.empty()) {
    // No backing nodes means nothing to reset; SOVD treats this as a success
    // with no content (204) - matches legacy behaviour.
    return ResultVariant{http::NoContent{}};
  }

  auto * config_mgr = ctx_.node()->get_configuration_manager();
  bool all_success = true;
  dto::ConfigurationDeleteMultiStatus multi_status;
  multi_status.entity_id = entity_id;

  for (const auto & node_info : agg_configs.nodes) {
    auto result = config_mgr->reset_all_parameters(node_info.node_fqn);

    dto::ConfigurationDeleteResultItem entry;
    entry.node = node_info.node_fqn;
    entry.app_id = node_info.app_id;
    if (result.success) {
      entry.success = true;
      if (result.data.is_object() || result.data.is_array()) {
        entry.details = result.data;
      }
    } else {
      all_success = false;
      entry.success = false;
      entry.error = result.error_message;
    }
    multi_status.results.push_back(std::move(entry));
  }

  if (all_success) {
    return ResultVariant{http::NoContent{}};
  }
  return ResultVariant{std::move(multi_status)};
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
