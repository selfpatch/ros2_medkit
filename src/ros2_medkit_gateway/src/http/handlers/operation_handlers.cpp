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

    // Use ThreadSafeEntityCache for O(1) entity lookup and aggregated operations
    const auto & cache = ctx_.node()->get_thread_safe_cache();

    // Determine entity type and get aggregated operations
    AggregatedOperations ops;
    std::string entity_type;

    auto entity_ref = cache.find_entity(entity_id);
    if (!entity_ref) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    switch (entity_ref->type) {
      case SovdEntityType::COMPONENT:
        ops = cache.get_component_operations(entity_id);
        entity_type = "component";
        break;
      case SovdEntityType::APP:
        ops = cache.get_app_operations(entity_id);
        entity_type = "app";
        break;
      case SovdEntityType::AREA:
        ops = cache.get_area_operations(entity_id);
        entity_type = "area";
        break;
      case SovdEntityType::FUNCTION:
        ops = cache.get_function_operations(entity_id);
        entity_type = "function";
        break;
      default:
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                   {{"entity_id", entity_id}});
        return;
    }

    RCLCPP_DEBUG(HandlerContext::logger(), "Listing operations for %s '%s': %zu services, %zu actions",
                 entity_type.c_str(), entity_id.c_str(), ops.services.size(), ops.actions.size());

    // Build response with services and actions
    json operations = json::array();

    // Get type introspection for schema info
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto type_introspection = data_access_mgr->get_type_introspection();

    for (const auto & svc : ops.services) {
      // Response format
      json svc_json = {
          {"id", svc.name}, {"name", svc.name}, {"proximity_proof_required", false}, {"asynchronous_execution", false}};

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

    for (const auto & act : ops.actions) {
      // Response format
      json act_json = {
          {"id", act.name}, {"name", act.name}, {"proximity_proof_required", false}, {"asynchronous_execution", true}};

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

    // Return response with items array
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

    // Use ThreadSafeEntityCache for O(1) entity lookup
    const auto & cache = ctx_.node()->get_thread_safe_cache();

    // Find entity
    auto entity_ref = cache.find_entity(entity_id);
    if (!entity_ref) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Get aggregated operations based on entity type
    AggregatedOperations ops;
    std::string entity_type;
    switch (entity_ref->type) {
      case SovdEntityType::COMPONENT:
        ops = cache.get_component_operations(entity_id);
        entity_type = "component";
        break;
      case SovdEntityType::APP:
        ops = cache.get_app_operations(entity_id);
        entity_type = "app";
        break;
      case SovdEntityType::AREA:
        ops = cache.get_area_operations(entity_id);
        entity_type = "area";
        break;
      case SovdEntityType::FUNCTION:
        ops = cache.get_function_operations(entity_id);
        entity_type = "function";
        break;
      default:
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND,
                                   "Entity type does not support operations", {{"entity_id", entity_id}});
        return;
    }

    // Find operation by name - O(m) where m = operations in entity
    std::optional<ServiceInfo> service_info;
    std::optional<ActionInfo> action_info;

    for (const auto & svc : ops.services) {
      if (svc.name == operation_id) {
        service_info = svc;
        break;
      }
    }

    if (!service_info.has_value()) {
      for (const auto & act : ops.actions) {
        if (act.name == operation_id) {
          action_info = act;
          break;
        }
      }
    }

    if (!service_info.has_value() && !action_info.has_value()) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_OPERATION_NOT_FOUND, "Operation not found",
                                 {{"entity_id", entity_id}, {"operation_id", operation_id}});
      return;
    }

    // Get type introspection for schema info
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto type_introspection = data_access_mgr->get_type_introspection();

    // Build response
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

    // Use ThreadSafeEntityCache for O(1) entity lookup
    const auto & cache = ctx_.node()->get_thread_safe_cache();

    // Find entity and get its operations
    auto entity_ref = cache.find_entity(entity_id);
    if (!entity_ref) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Get aggregated operations based on entity type
    AggregatedOperations ops;
    std::string entity_type;
    switch (entity_ref->type) {
      case SovdEntityType::COMPONENT:
        ops = cache.get_component_operations(entity_id);
        entity_type = "component";
        break;
      case SovdEntityType::APP:
        ops = cache.get_app_operations(entity_id);
        entity_type = "app";
        break;
      case SovdEntityType::AREA:
        ops = cache.get_area_operations(entity_id);
        entity_type = "area";
        break;
      case SovdEntityType::FUNCTION:
        ops = cache.get_function_operations(entity_id);
        entity_type = "function";
        break;
      default:
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND,
                                   "Entity type does not support operations", {{"entity_id", entity_id}});
        return;
    }

    // Find operation by name - O(m) where m = operations in entity
    std::optional<ServiceInfo> service_info;
    std::optional<ActionInfo> action_info;

    for (const auto & svc : ops.services) {
      if (svc.name == operation_id) {
        service_info = svc;
        break;
      }
    }

    if (!service_info.has_value()) {
      for (const auto & act : ops.actions) {
        if (act.name == operation_id) {
          action_info = act;
          break;
        }
      }
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
        // Return 202 Accepted with Location header for async operations
        json response = {{"id", action_result.goal_id}, {"status", "running"}};

        // Add Location header pointing to execution status endpoint
        std::string base_path = (entity_type == "app") ? "/api/v1/apps/" : "/api/v1/components/";
        std::string location =
            base_path + entity_id + "/operations/" + operation_id + "/executions/" + action_result.goal_id;
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
        HandlerContext::send_error(
            res, StatusCode::InternalServerError_500, ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE, "Action execution failed",
            {{id_field, entity_id}, {"operation_id", operation_id}, {"details", action_result.error_message}});
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
        // Synchronous response
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

    // Find entity to get namespace (O(1) lookups)
    const auto& cache = ctx_.node()->get_thread_safe_cache();
    std::string namespace_path;
    bool entity_found = false;

    if (auto component = cache.get_component(entity_id)) {
      namespace_path = component->namespace_path;
      entity_found = true;
    }

    if (!entity_found) {
      if (auto app = cache.get_app(entity_id)) {
        for (const auto & act : app->actions) {
          if (act.name == operation_id) {
            namespace_path = act.full_path.substr(0, act.full_path.rfind('/'));
            entity_found = true;
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

    // Return list of execution objects with id field (Table 172)
    json items = json::array();
    for (const auto & goal : goals) {
      items.push_back({{"id", goal.goal_id}});
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
      HandlerContext::send_error(
          res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
          {{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}});
      return;
    }

    // Response
    json response = {{"status", sovd_status_from_ros2(goal_info->status)}, {"capability", "execute"}};

    // Add feedback as parameters if available
    if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
      response["parameters"] = goal_info->last_feedback;
    }

    // Add x-medkit extension for ROS2-specific details
    auto x_medkit = XMedkit()
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
      HandlerContext::send_error(
          res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
          {{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}});
      return;
    }

    // Cancel the action
    auto result = operation_mgr->cancel_action_goal(goal_info->action_path, execution_id);

    if (result.success && result.return_code == 0) {
      // Return 204 No Content on successful cancellation request
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

void OperationHandlers::handle_update_execution(const httplib::Request & req, httplib::Response & res) {
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

    // Validate required 'capability' field
    if (!body.contains("capability") || !body["capability"].is_string()) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Missing required 'capability' field");
      return;
    }

    std::string capability = body["capability"].get<std::string>();

    auto operation_mgr = ctx_.node()->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(execution_id);

    if (!goal_info.has_value()) {
      HandlerContext::send_error(
          res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Execution not found",
          {{"entity_id", entity_id}, {"operation_id", operation_id}, {"execution_id", execution_id}});
      return;
    }

    // Handle supported capabilities
    // SOVD capabilities: execute, freeze, reset, stop (and custom x-<ext>-* capabilities)
    // ROS 2 actions support: execute (re-send goal), stop (cancel)

    if (capability == "stop") {
      // Stop capability maps to ROS 2 action cancel
      auto result = operation_mgr->cancel_action_goal(goal_info->action_path, execution_id);

      if (result.success && result.return_code == 0) {
        // Return 202 Accepted with execution status
        std::string base_path = req.path.find("/apps/") != std::string::npos ? "/api/v1/apps/" : "/api/v1/components/";
        std::string location = base_path + entity_id + "/operations/" + operation_id + "/executions/" + execution_id;
        res.set_header("Location", location);

        json response = {{"id", execution_id}, {"status", "running"}};  // Canceling is still "running" in SOVD terms
        res.status = StatusCode::Accepted_202;
        res.set_content(response.dump(), "application/json");
      } else {
        std::string error_msg;
        switch (result.return_code) {
          case 1:
            error_msg = "Stop request rejected";
            break;
          case 2:
            error_msg = "Unknown execution ID";
            break;
          case 3:
            error_msg = "Execution already terminated";
            break;
          default:
            error_msg = result.error_message.empty() ? "Stop failed" : result.error_message;
        }
        HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_X_MEDKIT_ROS2_ACTION_REJECTED, error_msg,
                                   {{"entity_id", entity_id},
                                    {"operation_id", operation_id},
                                    {"execution_id", execution_id},
                                    {"capability", capability}});
      }
    } else if (capability == "execute") {
      // Re-execute with updated parameters is not directly supported by ROS 2 actions
      // The goal would need to be cancelled and a new one sent
      // For now, return 409 Conflict indicating the operation is still running
      HandlerContext::send_error(
          res, StatusCode::Conflict_409, ERR_INVALID_REQUEST,
          "Cannot re-execute while operation is running. Cancel first, then start new execution.",
          {{"entity_id", entity_id},
           {"operation_id", operation_id},
           {"execution_id", execution_id},
           {"capability", capability}});
    } else if (capability == "freeze" || capability == "reset") {
      // These I/O control capabilities are not applicable to ROS 2 actions
      // They are ECU-specific concepts for UDS-style I/O controls
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Capability not supported for ROS 2 actions",
                                 {{"entity_id", entity_id},
                                  {"operation_id", operation_id},
                                  {"execution_id", execution_id},
                                  {"capability", capability},
                                  {"supported_capabilities", json::array({"stop"})}});
    } else {
      // Unknown or custom capability
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Unknown capability",
                                 {{"entity_id", entity_id},
                                  {"operation_id", operation_id},
                                  {"execution_id", execution_id},
                                  {"capability", capability},
                                  {"supported_capabilities", json::array({"stop"})}});
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to update execution",
                               {{"details", e.what()},
                                {"entity_id", entity_id},
                                {"operation_id", operation_id},
                                {"execution_id", execution_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_update_execution: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
