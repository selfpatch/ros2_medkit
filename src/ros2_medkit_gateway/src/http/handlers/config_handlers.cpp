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

#include "ros2_medkit_gateway/http/handlers/config_handlers.hpp"

#include <future>
#include <vector>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief Maximum length for aggregated parameter IDs
 *
 * Aggregated parameter IDs use format "app_id:param_name", so max length is:
 * - app_id: up to 256 characters (entity ID limit)
 * - separator: 1 character (":")
 * - param_name: up to 256 characters (ROS 2 parameter name limit)
 */
constexpr size_t MAX_AGGREGATED_PARAM_ID_LENGTH = 512;

// ============================================================================
// Helper structures and functions for configuration handlers
// ============================================================================

/**
 * @brief Parsed parameter ID with optional app prefix
 */
struct ParsedParamId {
  std::string app_id;      ///< Target app ID (empty if not prefixed)
  std::string param_name;  ///< Parameter name
  bool has_prefix{false};  ///< Whether the ID had an app prefix
};

/**
 * @brief Parse param_id which may contain "app_id:param_name" format
 *
 * For aggregated configurations, parameter IDs are prefixed with the source
 * app ID to disambiguate parameters with the same name across nodes.
 *
 * @param param_id The parameter ID to parse
 * @param is_aggregated Whether the entity is aggregated
 * @return ParsedParamId with separated app_id and param_name
 */
static ParsedParamId parse_aggregated_param_id(const std::string & param_id, bool is_aggregated) {
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

/**
 * @brief Find node info for a specific app ID in aggregated configurations
 *
 * @param nodes Vector of NodeConfigInfo to search
 * @param app_id Target app ID
 * @return Pointer to NodeConfigInfo if found, nullptr otherwise
 */
static const NodeConfigInfo * find_node_for_app(const std::vector<NodeConfigInfo> & nodes, const std::string & app_id) {
  for (const auto & node : nodes) {
    if (node.app_id == app_id) {
      return &node;
    }
  }
  return nullptr;
}

/**
 * @brief Error classification result for parameter operations
 */
struct ErrorClassification {
  httplib::StatusCode status_code;
  std::string error_code;
};

/**
 * @brief Classify ParameterErrorCode to HTTP status and error code
 *
 * Maps structured error codes from ConfigurationManager to HTTP responses.
 *
 * @param error_code The structured error code from ParameterResult
 * @return ErrorClassification with HTTP status code and error code string
 */
static ErrorClassification classify_error_code(ParameterErrorCode error_code) {
  ErrorClassification result;

  switch (error_code) {
    case ParameterErrorCode::NOT_FOUND:
    case ParameterErrorCode::NO_DEFAULTS_CACHED:
      result.status_code = StatusCode::NotFound_404;
      result.error_code = ERR_RESOURCE_NOT_FOUND;
      break;
    case ParameterErrorCode::READ_ONLY:
      result.status_code = StatusCode::Forbidden_403;
      result.error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
      break;
    case ParameterErrorCode::SERVICE_UNAVAILABLE:
    case ParameterErrorCode::TIMEOUT:
      result.status_code = StatusCode::ServiceUnavailable_503;
      result.error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
      break;
    case ParameterErrorCode::TYPE_MISMATCH:
    case ParameterErrorCode::INVALID_VALUE:
      result.status_code = StatusCode::BadRequest_400;
      result.error_code = ERR_INVALID_PARAMETER;
      break;
    case ParameterErrorCode::INTERNAL_ERROR:
    default:
      result.status_code = StatusCode::InternalServerError_500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
  }

  return result;
}

/**
 * @brief Classify error from ParameterResult to HTTP status and error code
 *
 * Uses structured error_code if available, falls back to string parsing for
 * backward compatibility with older error messages.
 *
 * @param result The ParameterResult to classify
 * @return ErrorClassification with HTTP status code and error code string
 */
static ErrorClassification classify_parameter_error(const ParameterResult & result) {
  // Prefer structured error code
  if (result.error_code != ParameterErrorCode::NONE) {
    return classify_error_code(result.error_code);
  }

  // Fallback to string parsing for backward compatibility
  ErrorClassification classification;
  const auto & error_message = result.error_message;

  if (error_message.find("not found") != std::string::npos ||
      error_message.find("Parameter not found") != std::string::npos) {
    classification.status_code = StatusCode::NotFound_404;
    classification.error_code = ERR_RESOURCE_NOT_FOUND;
  } else if (error_message.find("read-only") != std::string::npos ||
             error_message.find("read only") != std::string::npos ||
             error_message.find("is read_only") != std::string::npos) {
    classification.status_code = StatusCode::Forbidden_403;
    classification.error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
  } else if (error_message.find("not available") != std::string::npos ||
             error_message.find("service not available") != std::string::npos ||
             error_message.find("timed out") != std::string::npos ||
             error_message.find("timeout") != std::string::npos) {
    classification.status_code = StatusCode::ServiceUnavailable_503;
    classification.error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
  } else {
    classification.status_code = StatusCode::BadRequest_400;
    classification.error_code = ERR_INVALID_REQUEST;
  }

  return classification;
}

/**
 * @brief Send error response for a failed parameter operation
 *
 * Common helper for set/get/reset parameter operations. Classifies the error
 * and sends appropriate HTTP response.
 *
 * @param res HTTP response to send
 * @param result ParameterResult containing error info
 * @param operation_name Name of the operation (e.g., "set", "reset", "get")
 * @param entity_id Entity ID for error context
 * @param param_id Parameter ID for error context
 */
static void send_parameter_error(httplib::Response & res, const ParameterResult & result,
                                 const std::string & operation_name, const std::string & entity_id,
                                 const std::string & param_id) {
  auto err = classify_parameter_error(result);
  std::string message = "Failed to " + operation_name + " parameter";
  HandlerContext::send_error(res, err.status_code, err.error_code, message,
                             json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
}

// ============================================================================
// Handler implementations
// ============================================================================

void ConfigHandlers::handle_list_configurations(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }

    // Get aggregated configurations info for this entity
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto agg_configs = cache.get_entity_configurations(entity_id);

    // If no nodes to query, return empty result
    if (agg_configs.nodes.empty()) {
      json response;
      response["items"] = json::array();

      XMedkit ext;
      ext.entity_id(entity_id).source("runtime");
      ext.add("aggregation_level", agg_configs.aggregation_level);
      ext.add("is_aggregated", agg_configs.is_aggregated);
      response["x-medkit"] = ext.build();

      HandlerContext::send_json(res, response);
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    json items = json::array();
    json all_parameters = json::array();
    std::vector<std::string> queried_nodes;
    bool any_success = false;
    std::string first_error;
    std::string first_error_node;  // Track which node failed for better diagnostics

    // Query nodes in parallel for better performance with large hierarchies
    // Each async task queries one node and returns its result
    struct NodeQueryResult {
      NodeConfigInfo node_info;
      ParameterResult result;
    };

    std::vector<std::future<NodeQueryResult>> futures;
    futures.reserve(agg_configs.nodes.size());

    // Launch parallel queries
    for (const auto & node_info : agg_configs.nodes) {
      futures.push_back(std::async(std::launch::async, [config_mgr, node_info]() {
        NodeQueryResult query_result;
        query_result.node_info = node_info;
        query_result.result = config_mgr->list_parameters(node_info.node_fqn);
        return query_result;
      }));
    }

    // Collect results
    for (auto & future : futures) {
      auto query_result = future.get();
      const auto & node_info = query_result.node_info;
      const auto & result = query_result.result;

      if (result.success) {
        any_success = true;
        queried_nodes.push_back(node_info.node_fqn);

        if (result.data.is_array()) {
          for (const auto & param : result.data) {
            json config_meta;
            std::string param_name = param.value("name", "");

            // Create unique ID for aggregated configs: app_id:param_name
            // This allows disambiguation when multiple apps have parameters with the same name
            std::string unique_id = param_name;
            if (agg_configs.is_aggregated) {
              unique_id = node_info.app_id + ":" + param_name;
            }

            config_meta["id"] = unique_id;
            config_meta["name"] = param_name;
            config_meta["type"] = "parameter";

            // Add source info for aggregated configurations
            if (agg_configs.is_aggregated) {
              config_meta["x-medkit-source"] = node_info.app_id;
            }

            items.push_back(config_meta);

            // Also track full parameter info
            json param_with_source = param;
            param_with_source["x-medkit-source"] = node_info.app_id;
            param_with_source["x-medkit-node"] = node_info.node_fqn;
            all_parameters.push_back(param_with_source);
          }
        }
      } else if (first_error.empty()) {
        first_error = result.error_message;
        first_error_node = node_info.node_fqn;
      }
    }

    // If no successful queries, return error
    if (!any_success) {
      HandlerContext::send_error(
          res, StatusCode::ServiceUnavailable_503, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE,
          "Failed to list parameters from any node",
          {{"details", first_error}, {"entity_id", entity_id}, {"failed_node", first_error_node}});
      return;
    }

    // Build x-medkit extension
    XMedkit ext;
    ext.entity_id(entity_id).source("runtime");
    ext.add("parameters", all_parameters);
    ext.add("aggregation_level", agg_configs.aggregation_level);
    ext.add("is_aggregated", agg_configs.is_aggregated);
    ext.add("source_ids", agg_configs.source_ids);
    ext.add("queried_nodes", queried_nodes);

    json response;
    response["items"] = items;
    response["x-medkit"] = ext.build();
    HandlerContext::send_json(res, response);

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to list configurations", {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_configurations for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void ConfigHandlers::handle_get_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string param_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    param_id = req.matches[2];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }

    // Parameter ID may be prefixed with app_id: for aggregated configs
    if (param_id.empty() || param_id.length() > MAX_AGGREGATED_PARAM_ID_LENGTH) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                 {{"details", "Parameter ID is empty or too long"}});
      return;
    }

    // Get aggregated configurations info
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto agg_configs = cache.get_entity_configurations(entity_id);

    if (agg_configs.nodes.empty()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                 json{{"entity_id", entity_id}, {"id", param_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();

    // Parse param_id for app_id prefix
    auto parsed = parse_aggregated_param_id(param_id, agg_configs.is_aggregated);

    // If targeting specific app in aggregated entity
    if (parsed.has_prefix) {
      const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
      if (!node_info) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                   "Source app not found in entity",
                                   json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}});
        return;
      }

      auto result = config_mgr->get_parameter(node_info->node_fqn, parsed.param_name);

      if (result.success) {
        json response;
        response["id"] = param_id;
        response["data"] = result.data.contains("value") ? result.data["value"] : result.data;

        XMedkit ext;
        ext.ros2_node(node_info->node_fqn).entity_id(entity_id).source("runtime");
        ext.add("parameter", result.data);
        ext.add("source_app", parsed.app_id);
        response["x-medkit"] = ext.build();

        HandlerContext::send_json(res, response);
      } else {
        auto err = classify_parameter_error(result);
        HandlerContext::send_error(res, err.status_code, err.error_code,
                                   err.status_code == StatusCode::NotFound_404 ? "Parameter not found"
                                                                               : "Failed to get parameter",
                                   json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
      }
      return;
    }

    // For non-aggregated or no prefix: search all nodes for the parameter
    // Track errors to provide meaningful response if parameter not found anywhere
    ParameterResult last_result;  // Track full result for error code info
    bool all_not_found = true;    // Track if all failures are "not found" vs other errors

    for (const auto & node_info : agg_configs.nodes) {
      auto result = config_mgr->get_parameter(node_info.node_fqn, parsed.param_name);

      if (result.success) {
        json response;
        response["id"] = parsed.param_name;
        response["data"] = result.data.contains("value") ? result.data["value"] : result.data;

        XMedkit ext;
        ext.ros2_node(node_info.node_fqn).entity_id(entity_id).source("runtime");
        ext.add("parameter", result.data);
        if (agg_configs.is_aggregated) {
          ext.add("source_app", node_info.app_id);
        }
        response["x-medkit"] = ext.build();

        HandlerContext::send_json(res, response);
        return;
      }

      // Track the error for later reporting
      last_result = result;
      auto err = classify_parameter_error(result);
      if (err.status_code != StatusCode::NotFound_404) {
        all_not_found = false;
      }
    }

    // Parameter not found in any node - report appropriate error
    if (all_not_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Parameter not found",
                                 json{{"entity_id", entity_id}, {"id", param_id}});
    } else {
      // Some nodes had non-"not found" errors (e.g., unavailable) - report 503
      auto err = classify_parameter_error(last_result);
      HandlerContext::send_error(
          res, err.status_code, err.error_code, "Failed to get parameter from any node",
          json{{"details", last_result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to get configuration",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"param_id", param_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_configuration for entity '%s', param '%s': %s",
                 entity_id.c_str(), param_id.c_str(), e.what());
  }
}

void ConfigHandlers::handle_set_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string param_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    param_id = req.matches[2];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }

    if (param_id.empty() || param_id.length() > MAX_AGGREGATED_PARAM_ID_LENGTH) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                 {{"details", "Parameter ID is empty or too long"}});
      return;
    }

    // Parse request body
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid JSON in request body",
                                 {{"details", e.what()}});
      return;
    }

    // SOVD uses "data" field, but also support legacy "value" field
    json value;
    if (body.contains("data")) {
      value = body["data"];
    } else if (body.contains("value")) {
      value = body["value"];
    } else {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Missing 'data' field",
                                 {{"details", "Request body must contain 'data' field"}});
      return;
    }

    // Get aggregated configurations info
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto agg_configs = cache.get_entity_configurations(entity_id);

    if (agg_configs.nodes.empty()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                 json{{"entity_id", entity_id}, {"id", param_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();

    // Parse param_id for app_id prefix
    auto parsed = parse_aggregated_param_id(param_id, agg_configs.is_aggregated);

    // Helper to handle set result and send response
    auto handle_set_result = [&](const auto & result, const std::string & node_fqn, const std::string & app_id) {
      if (result.success) {
        json response;
        response["id"] = param_id;
        response["data"] = result.data.contains("value") ? result.data["value"] : result.data;

        XMedkit ext;
        ext.ros2_node(node_fqn).entity_id(entity_id).source("runtime");
        ext.add("parameter", result.data);
        if (agg_configs.is_aggregated) {
          ext.add("source_app", app_id);
        }
        response["x-medkit"] = ext.build();

        HandlerContext::send_json(res, response);
        return true;
      }

      send_parameter_error(res, result, "set", entity_id, param_id);
      return false;
    };

    // If targeting specific app in aggregated entity
    if (parsed.has_prefix) {
      const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
      if (!node_info) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                   "Source app not found in entity",
                                   json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}});
        return;
      }

      auto result = config_mgr->set_parameter(node_info->node_fqn, parsed.param_name, value);
      handle_set_result(result, node_info->node_fqn, parsed.app_id);
      return;
    }

    // For non-aggregated: use the single node
    if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
      const auto & node_info = agg_configs.nodes[0];
      auto result = config_mgr->set_parameter(node_info.node_fqn, parsed.param_name, value);
      handle_set_result(result, node_info.node_fqn, node_info.app_id);
      return;
    }

    // For aggregated configs without prefix, we don't know which node to target
    HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST,
                               "Aggregated configuration requires app_id prefix",
                               {{"details", "Use format 'app_id:param_name' for aggregated configurations"},
                                {"entity_id", entity_id},
                                {"id", param_id}});

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to set configuration",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"param_id", param_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_set_configuration for entity '%s', param '%s': %s",
                 entity_id.c_str(), param_id.c_str(), e.what());
  }
}

void ConfigHandlers::handle_delete_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string param_id;

  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    param_id = req.matches[2];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }

    // Get aggregated configurations info
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto agg_configs = cache.get_entity_configurations(entity_id);

    if (agg_configs.nodes.empty()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "No nodes available",
                                 json{{"entity_id", entity_id}, {"id", param_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();

    // Parse param_id for app_id prefix
    auto parsed = parse_aggregated_param_id(param_id, agg_configs.is_aggregated);

    // Helper to handle reset result
    auto handle_reset_result = [&](const auto & result) {
      if (result.success) {
        res.status = StatusCode::NoContent_204;
        return true;
      }
      send_parameter_error(res, result, "reset", entity_id, param_id);
      return false;
    };

    // If targeting specific app in aggregated entity
    if (parsed.has_prefix) {
      const auto * node_info = find_node_for_app(agg_configs.nodes, parsed.app_id);
      if (!node_info) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                   "Source app not found in entity",
                                   json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", parsed.app_id}});
        return;
      }

      auto result = config_mgr->reset_parameter(node_info->node_fqn, parsed.param_name);
      handle_reset_result(result);
      return;
    }

    // For non-aggregated: use the single node
    if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
      const auto & node_info = agg_configs.nodes[0];
      auto result = config_mgr->reset_parameter(node_info.node_fqn, parsed.param_name);
      handle_reset_result(result);
      return;
    }

    // For aggregated configs without prefix, we don't know which node to target
    HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST,
                               "Aggregated configuration requires app_id prefix",
                               json{{"details", "Use format 'app_id:param_name' for aggregated configurations"},
                                    {"entity_id", entity_id},
                                    {"id", param_id}});

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to reset configuration",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"param_id", param_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_delete_configuration: %s", e.what());
  }
}

void ConfigHandlers::handle_delete_all_configurations(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;

  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }

    // Get aggregated configurations info
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto agg_configs = cache.get_entity_configurations(entity_id);

    if (agg_configs.nodes.empty()) {
      // No nodes means nothing to reset, success
      res.status = StatusCode::NoContent_204;
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    bool all_success = true;
    json multi_status = json::array();

    // Reset all parameters on all nodes
    for (const auto & node_info : agg_configs.nodes) {
      auto result = config_mgr->reset_all_parameters(node_info.node_fqn);
      if (!result.success) {
        all_success = false;
        json status_entry;
        status_entry["node"] = node_info.node_fqn;
        status_entry["app_id"] = node_info.app_id;
        status_entry["success"] = false;
        status_entry["error"] = result.error_message;
        multi_status.push_back(status_entry);
      } else {
        json status_entry;
        status_entry["node"] = node_info.node_fqn;
        status_entry["app_id"] = node_info.app_id;
        status_entry["success"] = true;
        if (result.data.is_object() || result.data.is_array()) {
          status_entry["details"] = result.data;
        }
        multi_status.push_back(status_entry);
      }
    }

    if (all_success) {
      // SOVD compliance: DELETE returns 204 No Content on complete success
      res.status = StatusCode::NoContent_204;
    } else {
      // Partial success - return 207 Multi-Status
      json response;
      response["entity_id"] = entity_id;
      response["results"] = multi_status;
      res.status = StatusCode::MultiStatus_207;
      res.set_content(response.dump(2), "application/json");
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to reset configurations", {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_delete_all_configurations: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
