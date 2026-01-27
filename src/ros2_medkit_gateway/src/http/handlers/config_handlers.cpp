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

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void ConfigHandlers::handle_list_configurations(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Use unified entity lookup
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
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

    // Query each node and aggregate results
    for (const auto & node_info : agg_configs.nodes) {
      auto result = config_mgr->list_parameters(node_info.node_fqn);

      if (result.success) {
        any_success = true;
        queried_nodes.push_back(node_info.node_fqn);

        if (result.data.is_array()) {
          for (const auto & param : result.data) {
            json config_meta;
            std::string param_name = param.value("name", "");

            // Create unique ID for aggregated configs: node_fqn:param_name
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
      }
    }

    // If no successful queries, return error
    if (!any_success) {
      HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE,
                                 "Failed to list parameters from any node",
                                 {{"details", first_error}, {"entity_id", entity_id}});
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

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Parameter ID may be prefixed with app_id: for aggregated configs
    if (param_id.empty() || param_id.length() > 512) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid parameter ID",
                                 {{"details", "Parameter ID is empty or too long"}});
      return;
    }

    // Use unified entity lookup
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
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

    // Check if param_id contains app_id prefix (for aggregated configs: "app_id:param_name")
    std::string target_app_id;
    std::string param_name = param_id;
    auto colon_pos = param_id.find(':');
    if (colon_pos != std::string::npos && agg_configs.is_aggregated) {
      target_app_id = param_id.substr(0, colon_pos);
      param_name = param_id.substr(colon_pos + 1);
    }

    // If targeting specific app in aggregated entity
    if (!target_app_id.empty()) {
      for (const auto & node_info : agg_configs.nodes) {
        if (node_info.app_id == target_app_id) {
          auto result = config_mgr->get_parameter(node_info.node_fqn, param_name);

          if (result.success) {
            json response;
            response["id"] = param_id;

            if (result.data.contains("value")) {
              response["data"] = result.data["value"];
            } else {
              response["data"] = result.data;
            }

            XMedkit ext;
            ext.ros2_node(node_info.node_fqn).entity_id(entity_id).source("runtime");
            ext.add("parameter", result.data);
            ext.add("source_app", target_app_id);
            response["x-medkit"] = ext.build();

            HandlerContext::send_json(res, response);
            return;
          } else {
            if (result.error_message.find("not found") != std::string::npos ||
                result.error_message.find("Parameter not found") != std::string::npos) {
              HandlerContext::send_error(
                  res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Parameter not found",
                  json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
            } else {
              HandlerContext::send_error(
                  res, StatusCode::ServiceUnavailable_503, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE,
                  "Failed to get parameter",
                  json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
            }
            return;
          }
        }
      }
      // App not found in this entity's children
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                 "Source app not found in entity",
                                 json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", target_app_id}});
      return;
    }

    // For non-aggregated or no prefix: search all nodes for the parameter
    for (const auto & node_info : agg_configs.nodes) {
      auto result = config_mgr->get_parameter(node_info.node_fqn, param_name);

      if (result.success) {
        json response;
        response["id"] = param_name;

        if (result.data.contains("value")) {
          response["data"] = result.data["value"];
        } else {
          response["data"] = result.data;
        }

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
    }

    // Parameter not found in any node
    HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Parameter not found",
                               {{"entity_id", entity_id}, {"id", param_id}});

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

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    if (param_id.empty() || param_id.length() > 512) {
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

    // Use unified entity lookup
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
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

    // Parse param_id for app_id prefix (for aggregated configs: "app_id:param_name")
    std::string target_app_id;
    std::string param_name = param_id;
    auto colon_pos = param_id.find(':');
    if (colon_pos != std::string::npos && agg_configs.is_aggregated) {
      target_app_id = param_id.substr(0, colon_pos);
      param_name = param_id.substr(colon_pos + 1);
    }

    // Helper to handle set result
    auto handle_set_result = [&](const auto & result, const std::string & node_fqn, const std::string & app_id) {
      if (result.success) {
        json response;
        response["id"] = param_id;

        if (result.data.contains("value")) {
          response["data"] = result.data["value"];
        } else {
          response["data"] = result.data;
        }

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

      // Determine error type
      std::string error_code;
      httplib::StatusCode status_code;
      if (result.error_message.find("read-only") != std::string::npos ||
          result.error_message.find("read only") != std::string::npos ||
          result.error_message.find("is read_only") != std::string::npos) {
        status_code = StatusCode::Forbidden_403;
        error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
      } else if (result.error_message.find("not found") != std::string::npos ||
                 result.error_message.find("Parameter not found") != std::string::npos) {
        status_code = StatusCode::NotFound_404;
        error_code = ERR_RESOURCE_NOT_FOUND;
      } else if (result.error_message.find("not available") != std::string::npos ||
                 result.error_message.find("service not available") != std::string::npos) {
        status_code = StatusCode::ServiceUnavailable_503;
        error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
      } else {
        status_code = StatusCode::BadRequest_400;
        error_code = ERR_INVALID_REQUEST;
      }
      HandlerContext::send_error(res, status_code, error_code, "Failed to set parameter",
                                 json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
      return false;
    };

    // If targeting specific app in aggregated entity
    if (!target_app_id.empty()) {
      for (const auto & node_info : agg_configs.nodes) {
        if (node_info.app_id == target_app_id) {
          auto result = config_mgr->set_parameter(node_info.node_fqn, param_name, value);
          handle_set_result(result, node_info.node_fqn, target_app_id);
          return;
        }
      }
      // App not found in this entity's children
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                 "Source app not found in entity",
                                 json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", target_app_id}});
      return;
    }

    // For non-aggregated or no prefix: use first node (or find matching param)
    if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
      const auto & node_info = agg_configs.nodes[0];
      auto result = config_mgr->set_parameter(node_info.node_fqn, param_name, value);
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

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Use unified entity lookup
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
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

    // Parse param_id for app_id prefix (for aggregated configs: "app_id:param_name")
    std::string target_app_id;
    std::string param_name = param_id;
    auto colon_pos = param_id.find(':');
    if (colon_pos != std::string::npos && agg_configs.is_aggregated) {
      target_app_id = param_id.substr(0, colon_pos);
      param_name = param_id.substr(colon_pos + 1);
    }

    // If targeting specific app in aggregated entity
    if (!target_app_id.empty()) {
      for (const auto & node_info : agg_configs.nodes) {
        if (node_info.app_id == target_app_id) {
          auto result = config_mgr->reset_parameter(node_info.node_fqn, param_name);
          if (result.success) {
            res.status = StatusCode::NoContent_204;
          } else {
            HandlerContext::send_error(
                res, StatusCode::ServiceUnavailable_503, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE,
                "Failed to reset parameter",
                json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
          }
          return;
        }
      }
      // App not found in this entity's children
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                 "Source app not found in entity",
                                 json{{"entity_id", entity_id}, {"id", param_id}, {"source_app", target_app_id}});
      return;
    }

    // For non-aggregated: use first node
    if (!agg_configs.is_aggregated && !agg_configs.nodes.empty()) {
      const auto & node_info = agg_configs.nodes[0];
      auto result = config_mgr->reset_parameter(node_info.node_fqn, param_name);
      if (result.success) {
        res.status = StatusCode::NoContent_204;
      } else {
        HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE,
                                   "Failed to reset parameter",
                                   json{{"details", result.error_message}, {"entity_id", entity_id}, {"id", param_id}});
      }
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

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Use unified entity lookup
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
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
