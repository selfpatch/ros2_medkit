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

#include "ros2_medkit_gateway/http/handlers/operation_handlers.hpp"

#include <unordered_set>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"
#include "ros2_medkit_gateway/operation_manager.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void OperationHandlers::handle_list_operations(const httplib::Request & req, httplib::Response & res) {
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

    const auto cache = ctx_.node()->get_entity_cache();

    // Find entity in cache - check components first, then apps
    bool entity_found = false;
    std::vector<ServiceInfo> services;
    std::vector<ActionInfo> actions;
    std::string entity_type = "component";

    // Try to find in components
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        services = component.services;
        actions = component.actions;

        // For synthetic components with no direct operations, aggregate from apps
        if (services.empty() && actions.empty()) {
          auto discovery = ctx_.node()->get_discovery_manager();
          auto apps = discovery->get_apps_for_component(entity_id);

          // Use sets to deduplicate operations by full_path
          std::unordered_set<std::string> seen_service_paths;
          std::unordered_set<std::string> seen_action_paths;

          for (const auto & app : apps) {
            for (const auto & svc : app.services) {
              if (seen_service_paths.insert(svc.full_path).second) {
                services.push_back(svc);
              }
            }
            for (const auto & act : app.actions) {
              if (seen_action_paths.insert(act.full_path).second) {
                actions.push_back(act);
              }
            }
          }
        }

        entity_found = true;
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          services = app.services;
          actions = app.actions;
          entity_found = true;
          entity_type = "app";
          break;
        }
      }
    }

    if (!entity_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    RCLCPP_DEBUG(HandlerContext::logger(), "Listing operations for %s '%s': %zu services, %zu actions",
                 entity_type.c_str(), entity_id.c_str(), services.size(), actions.size());

    // Build response with services and actions
    json operations = json::array();

    // Get type introspection for schema info
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto type_introspection = data_access_mgr->get_type_introspection();

    for (const auto & svc : services) {
      // SOVD-compliant response format
      json svc_json = {{"id", svc.name},
                       {"name", svc.name},
                       {"proximity_proof_required", false},
                       {"asynchronous_execution", false}};

      // Build x-medkit extension with ROS2-specific data
      auto x_medkit = XMedkit()
                          .ros2_service(svc.full_path)
                          .ros2_type(svc.type)
                          .ros2_kind("service")
                          .entity_id(entity_id)
                          .source("ros2_medkit_gateway");

      // Build type_info with request/response schemas for services
      try {
        json type_info_json;
        // Service types: pkg/srv/Type -> Request: pkg/srv/Type_Request, Response: pkg/srv/Type_Response
        auto request_info = type_introspection->get_type_info(svc.type + "_Request");
        auto response_info = type_introspection->get_type_info(svc.type + "_Response");
        type_info_json["request"] = request_info.schema;
        type_info_json["response"] = response_info.schema;
        x_medkit.add("type_info", type_info_json);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for service '%s': %s", svc.type.c_str(),
                     e.what());
      }

      svc_json["x-medkit"] = x_medkit.build();
      operations.push_back(svc_json);
    }

    for (const auto & act : actions) {
      // SOVD-compliant response format
      json act_json = {{"id", act.name},
                       {"name", act.name},
                       {"proximity_proof_required", false},
                       {"asynchronous_execution", true}};

      // Build x-medkit extension with ROS2-specific data
      auto x_medkit = XMedkit()
                          .ros2_action(act.full_path)
                          .ros2_type(act.type)
                          .ros2_kind("action")
                          .entity_id(entity_id)
                          .source("ros2_medkit_gateway");

      // Build type_info with goal/result/feedback schemas for actions
      try {
        json type_info_json;
        // Action types: pkg/action/Type -> Goal: pkg/action/Type_Goal, etc.
        auto goal_info = type_introspection->get_type_info(act.type + "_Goal");
        auto result_info = type_introspection->get_type_info(act.type + "_Result");
        auto feedback_info = type_introspection->get_type_info(act.type + "_Feedback");
        type_info_json["goal"] = goal_info.schema;
        type_info_json["result"] = result_info.schema;
        type_info_json["feedback"] = feedback_info.schema;
        x_medkit.add("type_info", type_info_json);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for action '%s': %s", act.type.c_str(),
                     e.what());
      }

      act_json["x-medkit"] = x_medkit.build();
      operations.push_back(act_json);
    }

    // Return SOVD-compliant response with items array
    json response;
    response["items"] = operations;
    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to list operations", {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_operations for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void OperationHandlers::handle_get_operation(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_id = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();
    auto discovery = ctx_.node()->get_discovery_manager();

    // Find entity and operation
    bool entity_found = false;
    std::optional<ServiceInfo> service_info;
    std::optional<ActionInfo> action_info;
    std::string entity_type = "component";

    // Try to find in components
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        entity_found = true;

        for (const auto & svc : component.services) {
          if (svc.name == operation_id) {
            service_info = svc;
            break;
          }
        }

        if (!service_info.has_value()) {
          for (const auto & act : component.actions) {
            if (act.name == operation_id) {
              action_info = act;
              break;
            }
          }
        }

        // For synthetic components, try to find operation in apps
        if (!service_info.has_value() && !action_info.has_value()) {
          auto apps = discovery->get_apps_for_component(entity_id);
          for (const auto & app : apps) {
            for (const auto & svc : app.services) {
              if (svc.name == operation_id) {
                service_info = svc;
                break;
              }
            }
            if (service_info.has_value()) {
              break;
            }

            for (const auto & act : app.actions) {
              if (act.name == operation_id) {
                action_info = act;
                break;
              }
            }
            if (action_info.has_value()) {
              break;
            }
          }
        }
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          entity_found = true;
          entity_type = "app";

          for (const auto & svc : app.services) {
            if (svc.name == operation_id) {
              service_info = svc;
              break;
            }
          }

          if (!service_info.has_value()) {
            for (const auto & act : app.actions) {
              if (act.name == operation_id) {
                action_info = act;
                break;
              }
            }
          }
          break;
        }
      }
    }

    if (!entity_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    if (!service_info.has_value() && !action_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                 {{"entity_id", entity_id}, {"operation_id", operation_id}});
      return;
    }

    // Get type introspection for schema info
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto type_introspection = data_access_mgr->get_type_introspection();

    // Build SOVD-compliant response
    json item;

    if (service_info.has_value()) {
      item["id"] = service_info->name;
      item["name"] = service_info->name;
      item["proximity_proof_required"] = false;
      item["asynchronous_execution"] = false;

      auto x_medkit = XMedkit()
                          .ros2_service(service_info->full_path)
                          .ros2_type(service_info->type)
                          .ros2_kind("service")
                          .entity_id(entity_id)
                          .source("ros2_medkit_gateway");

      try {
        json type_info_json;
        auto request_info = type_introspection->get_type_info(service_info->type + "_Request");
        auto response_info = type_introspection->get_type_info(service_info->type + "_Response");
        type_info_json["request"] = request_info.schema;
        type_info_json["response"] = response_info.schema;
        x_medkit.add("type_info", type_info_json);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for service '%s': %s",
                     service_info->type.c_str(), e.what());
      }

      item["x-medkit"] = x_medkit.build();
    } else {
      item["id"] = action_info->name;
      item["name"] = action_info->name;
      item["proximity_proof_required"] = false;
      item["asynchronous_execution"] = true;

      auto x_medkit = XMedkit()
                          .ros2_action(action_info->full_path)
                          .ros2_type(action_info->type)
                          .ros2_kind("action")
                          .entity_id(entity_id)
                          .source("ros2_medkit_gateway");

      try {
        json type_info_json;
        auto goal_info = type_introspection->get_type_info(action_info->type + "_Goal");
        auto result_info = type_introspection->get_type_info(action_info->type + "_Result");
        auto feedback_info = type_introspection->get_type_info(action_info->type + "_Feedback");
        type_info_json["goal"] = goal_info.schema;
        type_info_json["result"] = result_info.schema;
        type_info_json["feedback"] = feedback_info.schema;
        x_medkit.add("type_info", type_info_json);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for action '%s': %s", action_info->type.c_str(),
                     e.what());
      }

      item["x-medkit"] = x_medkit.build();
    }

    json response;
    response["item"] = item;
    HandlerContext::send_json(res, response);

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to get operation details",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"operation_id", operation_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_operation for entity '%s', operation '%s': %s",
                 entity_id.c_str(), operation_id.c_str(), e.what());
  }
}

void OperationHandlers::handle_component_operation(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_name;
  try {
    // Extract entity_id and operation_name from URL path
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate entity_id
    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Validate operation_name
    auto operation_validation = ctx_.validate_entity_id(operation_name);
    if (!operation_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid operation name",
                                 {{"details", operation_validation.error()}, {"operation_name", operation_name}});
      return;
    }

    // Parse request body (optional for services with no parameters)
    json body = json::object();
    if (!req.body.empty()) {
      try {
        body = json::parse(req.body);
      } catch (const json::parse_error & e) {
        HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid JSON in request body",
                                   {{"details", e.what()}});
        return;
      }
    }

    // Extract optional type override and request data
    std::optional<std::string> service_type;
    json request_data = json::object();

    if (body.contains("type") && body["type"].is_string()) {
      service_type = body["type"].get<std::string>();
    }

    if (body.contains("request")) {
      request_data = body["request"];
    }

    const auto cache = ctx_.node()->get_entity_cache();
    auto discovery = ctx_.node()->get_discovery_manager();

    // Find entity and operation - check components first, then apps
    bool entity_found = false;
    std::string entity_type = "component";
    std::optional<ServiceInfo> service_info;
    std::optional<ActionInfo> action_info;

    // Try to find in components
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        entity_found = true;

        // Search in component's services list (has full_path)
        for (const auto & svc : component.services) {
          if (svc.name == operation_name) {
            service_info = svc;
            break;
          }
        }

        // Search in component's actions list (has full_path)
        if (!service_info.has_value()) {
          for (const auto & act : component.actions) {
            if (act.name == operation_name) {
              action_info = act;
              break;
            }
          }
        }

        // For synthetic components, try to find operation in apps
        if (!service_info.has_value() && !action_info.has_value()) {
          auto apps = discovery->get_apps_for_component(entity_id);
          for (const auto & app : apps) {
            for (const auto & svc : app.services) {
              if (svc.name == operation_name) {
                service_info = svc;
                break;
              }
            }
            if (service_info.has_value()) {
              break;
            }

            for (const auto & act : app.actions) {
              if (act.name == operation_name) {
                action_info = act;
                break;
              }
            }
            if (action_info.has_value()) {
              break;
            }
          }
        }
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          entity_found = true;
          entity_type = "app";

          // Search in app's services list
          for (const auto & svc : app.services) {
            if (svc.name == operation_name) {
              service_info = svc;
              break;
            }
          }

          // Search in app's actions list
          if (!service_info.has_value()) {
            for (const auto & act : app.actions) {
              if (act.name == operation_name) {
                action_info = act;
                break;
              }
            }
          }
          break;
        }
      }
    }

    if (!entity_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Check if operation is a service or action
    auto operation_mgr = ctx_.node()->get_operation_manager();

    // Determine response field name based on entity type
    std::string id_field = (entity_type == "app") ? "app_id" : "component_id";

    // First, check if it's an action (use full_path from cache)
    if (action_info.has_value()) {
      // Extract goal data (from 'goal' field or root object)
      json goal_data = json::object();
      if (body.contains("goal")) {
        goal_data = body["goal"];
      } else if (!body.contains("type") && !body.contains("request")) {
        // If no 'type' or 'request', treat the whole body as goal (without system fields)
        goal_data = body;
      }

      // Use action type from cache or request body
      std::string action_type = action_info->type;
      if (body.contains("type") && body["type"].is_string()) {
        action_type = body["type"].get<std::string>();
      }

      // Use full_path from cache (e.g., "/waypoint_follower/navigate_to_pose")
      auto action_result = operation_mgr->send_action_goal(action_info->full_path, action_type, goal_data);

      if (action_result.success && action_result.goal_accepted) {
        auto tracked = operation_mgr->get_tracked_goal(action_result.goal_id);
        std::string status_str = tracked ? action_status_to_string(tracked->status) : "accepted";

        json response = {{"status", "success"},
                         {"kind", "action"},
                         {id_field, entity_id},
                         {"operation", operation_name},
                         {"goal_id", action_result.goal_id},
                         {"goal_status", status_str}};
        HandlerContext::send_json(res, response);
      } else if (action_result.success && !action_result.goal_accepted) {
        HandlerContext::send_error(
            res, StatusCode::BadRequest_400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, "Goal rejected",
            {{"kind", "action"},
             {id_field, entity_id},
             {"operation", operation_name},
             {"details", action_result.error_message.empty() ? "Goal rejected" : action_result.error_message}});
      } else {
        HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                                   "Action execution failed",
                                   {{"kind", "action"},
                                    {id_field, entity_id},
                                    {"operation", operation_name},
                                    {"details", action_result.error_message}});
      }
      return;
    }

    // Otherwise, check if it's a service call
    if (service_info.has_value()) {
      // Use service type from cache or request body
      std::string resolved_service_type = service_info->type;
      if (service_type.has_value() && !service_type->empty()) {
        resolved_service_type = *service_type;
      }

      // Use full_path from cache (e.g., "/waypoint_follower/get_available_states")
      auto result = operation_mgr->call_service(service_info->full_path, resolved_service_type, request_data);

      if (result.success) {
        json response = {{"status", "success"},
                         {"kind", "service"},
                         {id_field, entity_id},
                         {"operation", operation_name},
                         {"response", result.response}};
        HandlerContext::send_json(res, response);
      } else {
        HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_X_MEDKIT_ROS2_SERVICE_UNAVAILABLE,
                                   "Service call failed",
                                   {{"kind", "service"},
                                    {id_field, entity_id},
                                    {"operation", operation_name},
                                    {"details", result.error_message}});
      }
    } else {
      // Neither service nor action found
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                 {{id_field, entity_id}, {"operation_name", operation_name}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to execute operation",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"operation_name", operation_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_operation for entity '%s', operation '%s': %s",
                 entity_id.c_str(), operation_name.c_str(), e.what());
  }
}

void OperationHandlers::handle_action_status(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_name;
  try {
    // Extract entity_id and operation_name from URL path
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate IDs
    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}});
      return;
    }

    auto operation_validation = ctx_.validate_entity_id(operation_name);
    if (!operation_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid operation name",
                                 {{"details", operation_validation.error()}});
      return;
    }

    auto operation_mgr = ctx_.node()->get_operation_manager();

    // Check query parameters
    std::string goal_id;
    bool get_all = false;
    if (req.has_param("goal_id")) {
      goal_id = req.get_param_value("goal_id");
    }
    if (req.has_param("all") && req.get_param_value("all") == "true") {
      get_all = true;
    }

    // If specific goal_id provided, return that goal's status
    if (!goal_id.empty()) {
      auto goal_info = operation_mgr->get_tracked_goal(goal_id);
      if (!goal_info.has_value()) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Goal not found",
                                   {{"goal_id", goal_id}});
        return;
      }

      json response = {{"goal_id", goal_info->goal_id},
                       {"status", action_status_to_string(goal_info->status)},
                       {"action_path", goal_info->action_path},
                       {"action_type", goal_info->action_type}};
      if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
        response["last_feedback"] = goal_info->last_feedback;
      }
      HandlerContext::send_json(res, response);
      return;
    }

    // No goal_id provided - find goals by action path
    // Find entity (component or app) to get its namespace
    const auto cache = ctx_.node()->get_entity_cache();

    std::string namespace_path;
    bool entity_found = false;

    // Try components first
    for (const auto & c : cache.components) {
      if (c.id == entity_id) {
        namespace_path = c.namespace_path;
        entity_found = true;
        break;
      }
    }

    // Try apps if not found in components
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          // For apps, find the action in the app's actions list
          for (const auto & act : app.actions) {
            if (act.name == operation_name) {
              namespace_path = act.full_path.substr(0, act.full_path.rfind('/'));
              entity_found = true;
              break;
            }
          }
          if (entity_found) {
            break;
          }
        }
      }
    }

    if (!entity_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Build the action path: namespace + operation_name
    std::string action_path = namespace_path + "/" + operation_name;

    if (get_all) {
      // Return all goals for this action
      auto goals = operation_mgr->get_goals_for_action(action_path);
      json goals_array = json::array();
      for (const auto & goal : goals) {
        json goal_json = {{"goal_id", goal.goal_id},
                          {"status", action_status_to_string(goal.status)},
                          {"action_path", goal.action_path},
                          {"action_type", goal.action_type}};
        if (!goal.last_feedback.is_null() && !goal.last_feedback.empty()) {
          goal_json["last_feedback"] = goal.last_feedback;
        }
        goals_array.push_back(goal_json);
      }
      json response = {{"action_path", action_path}, {"goals", goals_array}, {"count", goals.size()}};
      HandlerContext::send_json(res, response);
    } else {
      // Return the most recent goal for this action
      auto goal_info = operation_mgr->get_latest_goal_for_action(action_path);
      if (!goal_info.has_value()) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                                   "No goals found for this action", {{"action_path", action_path}});
        return;
      }

      json response = {{"goal_id", goal_info->goal_id},
                       {"status", action_status_to_string(goal_info->status)},
                       {"action_path", goal_info->action_path},
                       {"action_type", goal_info->action_type}};
      if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
        response["last_feedback"] = goal_info->last_feedback;
      }
      HandlerContext::send_json(res, response);
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to get action status", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_action_status: %s", e.what());
  }
}

void OperationHandlers::handle_action_result(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string operation_name;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    component_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate IDs
    auto component_validation = ctx_.validate_entity_id(component_id);
    if (!component_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", component_validation.error()}});
      return;
    }

    // Get goal_id from query parameter
    std::string goal_id;
    if (req.has_param("goal_id")) {
      goal_id = req.get_param_value("goal_id");
    }

    if (goal_id.empty()) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Missing goal_id query parameter");
      return;
    }

    // Get tracked goal info to find action path and type
    auto operation_mgr = ctx_.node()->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(goal_id);

    if (!goal_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Goal not found",
                                 {{"goal_id", goal_id}});
      return;
    }

    // Get the result (this may block until the action completes)
    auto result = operation_mgr->get_action_result(goal_info->action_path, goal_info->action_type, goal_id);

    if (result.success) {
      json response = {
          {"goal_id", goal_id}, {"status", action_status_to_string(result.status)}, {"result", result.result}};
      HandlerContext::send_json(res, response);
    } else {
      HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                                 "Failed to get action result", {{"details", result.error_message}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to get action result", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_action_result: %s", e.what());
  }
}

void OperationHandlers::handle_action_cancel(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string operation_name;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    component_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate IDs
    auto component_validation = ctx_.validate_entity_id(component_id);
    if (!component_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", component_validation.error()}});
      return;
    }

    // Get goal_id from query parameter
    std::string goal_id;
    if (req.has_param("goal_id")) {
      goal_id = req.get_param_value("goal_id");
    }

    if (goal_id.empty()) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Missing goal_id query parameter");
      return;
    }

    // Get tracked goal info to find action path
    auto operation_mgr = ctx_.node()->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(goal_id);

    if (!goal_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Goal not found",
                                 {{"goal_id", goal_id}});
      return;
    }

    // Cancel the action
    auto result = operation_mgr->cancel_action_goal(goal_info->action_path, goal_id);

    if (result.success && result.return_code == 0) {
      json response = {{"status", "canceling"}, {"goal_id", goal_id}, {"message", "Cancel request accepted"}};
      HandlerContext::send_json(res, response);
    } else {
      std::string error_msg;
      switch (result.return_code) {
        case 1:
          error_msg = "Cancel request rejected";
          break;
        case 2:
          error_msg = "Unknown goal ID";
          break;
        case 3:
          error_msg = "Goal already terminated";
          break;
        default:
          error_msg = result.error_message.empty() ? "Cancel failed" : result.error_message;
      }
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, error_msg,
                                 {{"goal_id", goal_id}, {"return_code", result.return_code}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to cancel action",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_action_cancel: %s", e.what());
  }
}

// Helper function to convert ROS2 action status to SOVD ExecutionStatus
static std::string sovd_status_from_ros2(ActionGoalStatus status) {
  switch (status) {
    case ActionGoalStatus::ACCEPTED:
    case ActionGoalStatus::EXECUTING:
    case ActionGoalStatus::CANCELING:
      return "running";
    case ActionGoalStatus::SUCCEEDED:
      return "completed";
    case ActionGoalStatus::CANCELED:
    case ActionGoalStatus::ABORTED:
      return "failed";
    default:
      return "running";
  }
}

void OperationHandlers::handle_create_execution(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_id = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Parse request body
    json body = json::object();
    if (!req.body.empty()) {
      try {
        body = json::parse(req.body);
      } catch (const json::parse_error & e) {
        HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid JSON in request body",
                                   {{"details", e.what()}});
        return;
      }
    }

    const auto cache = ctx_.node()->get_entity_cache();
    auto discovery = ctx_.node()->get_discovery_manager();

    // Find entity and operation
    bool entity_found = false;
    std::optional<ServiceInfo> service_info;
    std::optional<ActionInfo> action_info;
    std::string entity_type = "component";

    // Try to find in components
    for (const auto & component : cache.components) {
      if (component.id == entity_id) {
        entity_found = true;

        for (const auto & svc : component.services) {
          if (svc.name == operation_id) {
            service_info = svc;
            break;
          }
        }

        if (!service_info.has_value()) {
          for (const auto & act : component.actions) {
            if (act.name == operation_id) {
              action_info = act;
              break;
            }
          }
        }

        // For synthetic components, try to find operation in apps
        if (!service_info.has_value() && !action_info.has_value()) {
          auto apps = discovery->get_apps_for_component(entity_id);
          for (const auto & app : apps) {
            for (const auto & svc : app.services) {
              if (svc.name == operation_id) {
                service_info = svc;
                break;
              }
            }
            if (service_info.has_value()) {
              break;
            }

            for (const auto & act : app.actions) {
              if (act.name == operation_id) {
                action_info = act;
                break;
              }
            }
            if (action_info.has_value()) {
              break;
            }
          }
        }
        break;
      }
    }

    // If not found in components, try apps
    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          entity_found = true;
          entity_type = "app";

          for (const auto & svc : app.services) {
            if (svc.name == operation_id) {
              service_info = svc;
              break;
            }
          }

          if (!service_info.has_value()) {
            for (const auto & act : app.actions) {
              if (act.name == operation_id) {
                action_info = act;
                break;
              }
            }
          }
          break;
        }
      }
    }

    if (!entity_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    if (!service_info.has_value() && !action_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                 {{"entity_id", entity_id}, {"operation_id", operation_id}});
      return;
    }

    auto operation_mgr = ctx_.node()->get_operation_manager();
    std::string id_field = (entity_type == "app") ? "app_id" : "component_id";

    // Handle actions (asynchronous execution)
    if (action_info.has_value()) {
      // Extract goal data from 'parameters' field (SOVD standard) or 'goal' field (legacy)
      json goal_data = json::object();
      if (body.contains("parameters")) {
        goal_data = body["parameters"];
      } else if (body.contains("goal")) {
        goal_data = body["goal"];
      }

      std::string action_type = action_info->type;
      if (body.contains("type") && body["type"].is_string()) {
        action_type = body["type"].get<std::string>();
      }

      auto action_result = operation_mgr->send_action_goal(action_info->full_path, action_type, goal_data);

      if (action_result.success && action_result.goal_accepted) {
        // SOVD-compliant: Return 202 Accepted with Location header for async operations
        json response = {{"id", action_result.goal_id}, {"status", "running"}};

        // Add Location header pointing to execution status endpoint
        std::string base_path = (entity_type == "app") ? "/api/v1/apps/" : "/api/v1/components/";
        std::string location = base_path + entity_id + "/operations/" + operation_id + "/executions/" +
                               action_result.goal_id;
        res.set_header("Location", location);

        res.status = StatusCode::Accepted_202;
        res.set_content(response.dump(), "application/json");
      } else if (action_result.success && !action_result.goal_accepted) {
        HandlerContext::send_error(
            res, StatusCode::BadRequest_400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, "Goal rejected",
            {{id_field, entity_id},
             {"operation_id", operation_id},
             {"details", action_result.error_message.empty() ? "Goal rejected" : action_result.error_message}});
      } else {
        HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE,
                                   "Action execution failed",
                                   {{id_field, entity_id},
                                    {"operation_id", operation_id},
                                    {"details", action_result.error_message}});
      }
      return;
    }

    // Handle services (synchronous execution)
    if (service_info.has_value()) {
      json request_data = json::object();
      if (body.contains("parameters")) {
        request_data = body["parameters"];
      } else if (body.contains("request")) {
        request_data = body["request"];
      }

      std::string service_type = service_info->type;
      if (body.contains("type") && body["type"].is_string()) {
        service_type = body["type"].get<std::string>();
      }

      auto result = operation_mgr->call_service(service_info->full_path, service_type, request_data);

      if (result.success) {
        // SOVD-compliant synchronous response
        json response = {{"parameters", result.response}};
        HandlerContext::send_json(res, response);
      } else {
        json error_response = {{"error",
                                {{"code", ERR_X_MEDKIT_ROS2_SERVICE_UNAVAILABLE},
                                 {"message", "Service call failed"},
                                 {"details", result.error_message}}}};
        res.status = StatusCode::InternalServerError_500;
        res.set_content(error_response.dump(), "application/json");
      }
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to execute operation",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"operation_id", operation_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_create_execution for entity '%s', operation '%s': %s",
                 entity_id.c_str(), operation_id.c_str(), e.what());
  }
}

void OperationHandlers::handle_list_executions(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_id = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Find entity to get namespace
    const auto cache = ctx_.node()->get_entity_cache();
    std::string namespace_path;
    bool entity_found = false;

    for (const auto & c : cache.components) {
      if (c.id == entity_id) {
        namespace_path = c.namespace_path;
        entity_found = true;
        break;
      }
    }

    if (!entity_found) {
      for (const auto & app : cache.apps) {
        if (app.id == entity_id) {
          for (const auto & act : app.actions) {
            if (act.name == operation_id) {
              namespace_path = act.full_path.substr(0, act.full_path.rfind('/'));
              entity_found = true;
              break;
            }
          }
          if (entity_found) {
            break;
          }
        }
      }
    }

    if (!entity_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Build action path and get all goals for this action
    std::string action_path = namespace_path + "/" + operation_id;
    auto operation_mgr = ctx_.node()->get_operation_manager();
    auto goals = operation_mgr->get_goals_for_action(action_path);

    // SOVD-compliant: return list of execution IDs
    json items = json::array();
    for (const auto & goal : goals) {
      items.push_back(goal.goal_id);
    }

    json response = {{"items", items}};
    HandlerContext::send_json(res, response);

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to list executions",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"operation_id", operation_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_executions: %s", e.what());
  }
}

void OperationHandlers::handle_get_execution(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_id;
  std::string execution_id;
  try {
    if (req.matches.size() < 4) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_id = req.matches[2];
    execution_id = req.matches[3];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    auto operation_mgr = ctx_.node()->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(execution_id);

    if (!goal_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
                                 {{"entity_id", entity_id},
                                  {"operation_id", operation_id},
                                  {"execution_id", execution_id}});
      return;
    }

    // SOVD-compliant response
    json response = {{"status", sovd_status_from_ros2(goal_info->status)}, {"capability", "execute"}};

    // Add feedback as parameters if available
    if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
      response["parameters"] = goal_info->last_feedback;
    }

    // Add x-medkit extension for ROS2-specific details
    auto x_medkit =
        XMedkit()
            .add("goal_id", execution_id)
            .add("ros2_status", action_status_to_string(goal_info->status))
            .ros2_action(goal_info->action_path)
            .ros2_type(goal_info->action_type);
    response["x-medkit"] = x_medkit.build();

    HandlerContext::send_json(res, response);

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to get execution status",
                               {{"details", e.what()},
                                {"entity_id", entity_id},
                                {"operation_id", operation_id},
                                {"execution_id", execution_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_execution: %s", e.what());
  }
}

void OperationHandlers::handle_cancel_execution(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string operation_id;
  std::string execution_id;
  try {
    if (req.matches.size() < 4) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    operation_id = req.matches[2];
    execution_id = req.matches[3];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    auto operation_mgr = ctx_.node()->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(execution_id);

    if (!goal_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
                                 {{"entity_id", entity_id},
                                  {"operation_id", operation_id},
                                  {"execution_id", execution_id}});
      return;
    }

    // Cancel the action
    auto result = operation_mgr->cancel_action_goal(goal_info->action_path, execution_id);

    if (result.success && result.return_code == 0) {
      // SOVD-compliant: return 204 No Content on successful cancellation request
      res.status = StatusCode::NoContent_204;
    } else {
      std::string error_msg;
      switch (result.return_code) {
        case 1:
          error_msg = "Cancel request rejected";
          break;
        case 2:
          error_msg = "Unknown execution ID";
          break;
        case 3:
          error_msg = "Execution already terminated";
          break;
        default:
          error_msg = result.error_message.empty() ? "Cancel failed" : result.error_message;
      }
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, error_msg,
                                 {{"entity_id", entity_id},
                                  {"operation_id", operation_id},
                                  {"execution_id", execution_id},
                                  {"return_code", result.return_code}});
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to cancel execution",
                               {{"details", e.what()},
                                {"entity_id", entity_id},
                                {"operation_id", operation_id},
                                {"execution_id", execution_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_cancel_execution: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
