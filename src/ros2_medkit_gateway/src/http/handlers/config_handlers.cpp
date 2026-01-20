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

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void ConfigHandlers::handle_list_configurations(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Use unified entity lookup
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, "Entity not found", {{"entity_id", entity_id}});
      return;
    }

    // Get node name for parameter access
    std::string node_name = entity_info.fqn;
    if (node_name.empty()) {
      node_name = "/" + entity_id;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    auto result = config_mgr->list_parameters(node_name);

    if (result.success) {
      json response = {{entity_info.id_field, entity_id}, {"node_name", node_name}, {"parameters", result.data}};
      HandlerContext::send_json(res, response);
    } else {
      HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, "Failed to list parameters",
                                 {{"details", result.error_message}, {"node_name", node_name}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to list configurations",
                               {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_configurations for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void ConfigHandlers::handle_get_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string param_name;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    param_name = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Parameter names may contain dots, so we use a more permissive validation
    if (param_name.empty() || param_name.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid parameter name",
                                 {{"details", "Parameter name is empty or too long"}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();

    std::string node_name;
    bool entity_found = false;
    std::string id_field = "component_id";

    // Try components first
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        node_name = component.fqn;
        entity_found = true;
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          node_name = app.bound_fqn.value_or("/" + app.id);
          entity_found = true;
          id_field = "app_id";
          break;
        }
      }
    }

    if (!entity_found) {
      std::string error_msg = (id_field == "app_id") ? "App not found" : "Component not found";
      HandlerContext::send_error(res, StatusCode::NotFound_404, error_msg, {{id_field, entity_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    auto result = config_mgr->get_parameter(node_name, param_name);

    if (result.success) {
      json response = {{id_field, entity_id}, {"parameter", result.data}};
      HandlerContext::send_json(res, response);
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Parameter not found") != std::string::npos) {
        HandlerContext::send_error(
            res, StatusCode::NotFound_404, "Failed to get parameter",
            {{"details", result.error_message}, {id_field, entity_id}, {"param_name", param_name}});
      } else {
        HandlerContext::send_error(
            res, StatusCode::ServiceUnavailable_503, "Failed to get parameter",
            {{"details", result.error_message}, {id_field, entity_id}, {"param_name", param_name}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to get configuration",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"param_name", param_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_configuration for entity '%s', param '%s': %s",
                 entity_id.c_str(), param_name.c_str(), e.what());
  }
}

void ConfigHandlers::handle_set_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string param_name;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    param_name = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    if (param_name.empty() || param_name.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid parameter name",
                                 {{"details", "Parameter name is empty or too long"}});
      return;
    }

    // Parse request body
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid JSON in request body",
                                 {{"details", e.what()}});
      return;
    }

    // Extract value from request body
    if (!body.contains("value")) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Missing 'value' field",
                                 {{"details", "Request body must contain 'value' field"}});
      return;
    }

    json value = body["value"];

    const auto cache = ctx_.node()->get_entity_cache();

    std::string node_name;
    bool entity_found = false;
    std::string id_field = "component_id";

    // Try components first
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        node_name = component.fqn;
        entity_found = true;
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          node_name = app.bound_fqn.value_or("/" + app.id);
          entity_found = true;
          id_field = "app_id";
          break;
        }
      }
    }

    if (!entity_found) {
      std::string error_msg = (id_field == "app_id") ? "App not found" : "Component not found";
      HandlerContext::send_error(res, StatusCode::NotFound_404, error_msg, {{id_field, entity_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    auto result = config_mgr->set_parameter(node_name, param_name, value);

    if (result.success) {
      json response = {{"status", "success"}, {id_field, entity_id}, {"parameter", result.data}};
      HandlerContext::send_json(res, response);
    } else {
      // Check if it's a read-only, not found, or service unavailable error
      httplib::StatusCode status_code;
      if (result.error_message.find("read-only") != std::string::npos ||
          result.error_message.find("read only") != std::string::npos ||
          result.error_message.find("is read_only") != std::string::npos) {
        status_code = StatusCode::Forbidden_403;
      } else if (result.error_message.find("not found") != std::string::npos ||
                 result.error_message.find("Parameter not found") != std::string::npos) {
        status_code = StatusCode::NotFound_404;
      } else if (result.error_message.find("not available") != std::string::npos ||
                 result.error_message.find("service not available") != std::string::npos) {
        status_code = StatusCode::ServiceUnavailable_503;
      } else {
        status_code = StatusCode::BadRequest_400;
      }
      HandlerContext::send_error(
          res, status_code, "Failed to set parameter",
          {{"details", result.error_message}, {id_field, entity_id}, {"param_name", param_name}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to set configuration",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"param_name", param_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_set_configuration for entity '%s', param '%s': %s",
                 entity_id.c_str(), param_name.c_str(), e.what());
  }
}

void ConfigHandlers::handle_delete_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string param_name;

  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    param_name = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();

    // Find entity to get its namespace and node name
    std::string node_name;
    bool entity_found = false;
    std::string id_field = "component_id";

    // Try components first
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        node_name = component.fqn;
        entity_found = true;
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          node_name = app.bound_fqn.value_or("/" + app.id);
          entity_found = true;
          id_field = "app_id";
          break;
        }
      }
    }

    if (!entity_found) {
      std::string error_msg = (id_field == "app_id") ? "App not found" : "Component not found";
      HandlerContext::send_error(res, StatusCode::NotFound_404, error_msg, {{id_field, entity_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    auto result = config_mgr->reset_parameter(node_name, param_name);

    if (result.success) {
      HandlerContext::send_json(res, result.data);
    } else {
      HandlerContext::send_error(
          res, StatusCode::ServiceUnavailable_503, "Failed to reset parameter",
          {{"details", result.error_message}, {"node_name", node_name}, {"param_name", param_name}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to reset configuration",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"param_name", param_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_delete_configuration: %s", e.what());
  }
}

void ConfigHandlers::handle_delete_all_configurations(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;

  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();

    // Find entity to get its namespace and node name
    std::string node_name;
    bool entity_found = false;
    std::string id_field = "component_id";

    // Try components first
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        node_name = component.fqn;
        entity_found = true;
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          node_name = app.bound_fqn.value_or("/" + app.id);
          entity_found = true;
          id_field = "app_id";
          break;
        }
      }
    }

    if (!entity_found) {
      std::string error_msg = (id_field == "app_id") ? "App not found" : "Component not found";
      HandlerContext::send_error(res, StatusCode::NotFound_404, error_msg, {{id_field, entity_id}});
      return;
    }

    auto config_mgr = ctx_.node()->get_configuration_manager();
    auto result = config_mgr->reset_all_parameters(node_name);

    if (result.success) {
      HandlerContext::send_json(res, result.data);
    } else {
      // Partial success - some parameters were reset
      res.status = StatusCode::MultiStatus_207;
      res.set_content(result.data.dump(2), "application/json");
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to reset configurations",
                               {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_delete_all_configurations: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
