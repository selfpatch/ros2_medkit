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

#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void FaultHandlers::handle_list_all_faults(const httplib::Request & req, httplib::Response & res) {
  try {
    auto filter = parse_fault_status_param(req);
    if (!filter.is_valid) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Invalid status parameter value",
                                 {{"allowed_values", "pending, confirmed, cleared, all"},
                                  {"parameter", "status"},
                                  {"value", req.get_param_value("status")}});
      return;
    }

    // Parse correlation query parameters
    bool include_muted = req.get_param_value("include_muted") == "true";
    bool include_clusters = req.get_param_value("include_clusters") == "true";

    auto fault_mgr = ctx_.node()->get_fault_manager();
    // Empty source_id = no filtering, return all faults
    auto result = fault_mgr->get_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                        include_muted, include_clusters);

    if (result.success) {
      json response = {{"faults", result.data["faults"]},
                       {"count", result.data["count"]},
                       {"muted_count", result.data["muted_count"]},
                       {"cluster_count", result.data["cluster_count"]}};

      // Include detailed correlation data if requested and present
      if (result.data.contains("muted_faults")) {
        response["muted_faults"] = result.data["muted_faults"];
      }
      if (result.data.contains("clusters")) {
        response["clusters"] = result.data["clusters"];
      }

      res.status = StatusCode::OK_200;
      HandlerContext::send_json(res, response);
    } else {
      HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                 "Failed to get faults", {{"details", result.error_message}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to list faults",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_all_faults: %s", e.what());
  }
}

void FaultHandlers::handle_list_faults(const httplib::Request & req, httplib::Response & res) {
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

    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }
    std::string namespace_path = entity_info.namespace_path;

    auto filter = parse_fault_status_param(req);
    if (!filter.is_valid) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Invalid status parameter value",
                                 {{"allowed_values", "pending, confirmed, cleared, all"},
                                  {"parameter", "status"},
                                  {"value", req.get_param_value("status")},
                                  {entity_info.id_field, entity_id}});
      return;
    }

    // Parse correlation query parameters
    bool include_muted = req.get_param_value("include_muted") == "true";
    bool include_clusters = req.get_param_value("include_clusters") == "true";

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->get_faults(namespace_path, filter.include_pending, filter.include_confirmed,
                                        filter.include_cleared, include_muted, include_clusters);

    if (result.success) {
      json response = {{entity_info.id_field, entity_id},           {"source_id", namespace_path},
                       {"faults", result.data["faults"]},           {"count", result.data["count"]},
                       {"muted_count", result.data["muted_count"]}, {"cluster_count", result.data["cluster_count"]}};

      // Include detailed correlation data if requested and present
      if (result.data.contains("muted_faults")) {
        response["muted_faults"] = result.data["muted_faults"];
      }
      if (result.data.contains("clusters")) {
        response["clusters"] = result.data["clusters"];
      }

      HandlerContext::send_json(res, response);
    } else {
      HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                 "Failed to get faults",
                                 {{"details", result.error_message}, {entity_info.id_field, entity_id}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to list faults",
                               {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_faults for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void FaultHandlers::handle_get_fault(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string fault_code;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    fault_code = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Fault codes may contain dots and underscores, validate basic constraints
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
      return;
    }

    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }
    std::string namespace_path = entity_info.namespace_path;

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->get_fault(fault_code, namespace_path);

    if (result.success) {
      json response = {{entity_info.id_field, entity_id}, {"fault", result.data}};
      HandlerContext::send_json(res, response);
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Fault not found") != std::string::npos) {
        HandlerContext::send_error(
            res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(
            res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE, "Failed to get fault",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to get fault",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"fault_code", fault_code}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_fault for entity '%s', fault '%s': %s",
                 entity_id.c_str(), fault_code.c_str(), e.what());
  }
}

void FaultHandlers::handle_clear_fault(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string fault_code;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    fault_code = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Validate fault code
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
      return;
    }

    // Verify entity exists
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->clear_fault(fault_code);

    if (result.success) {
      json response = {{"status", "success"},
                       {entity_info.id_field, entity_id},
                       {"fault_code", fault_code},
                       {"message", result.data.value("message", "Fault cleared")}};

      // Include auto-cleared symptom codes if present (correlation feature)
      if (result.data.contains("auto_cleared_codes")) {
        response["auto_cleared_codes"] = result.data["auto_cleared_codes"];
      }

      HandlerContext::send_json(res, response);
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Fault not found") != std::string::npos) {
        HandlerContext::send_error(
            res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(
            res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE, "Failed to clear fault",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to clear fault",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"fault_code", fault_code}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_clear_fault for entity '%s', fault '%s': %s",
                 entity_id.c_str(), fault_code.c_str(), e.what());
  }
}

void FaultHandlers::handle_get_snapshots(const httplib::Request & req, httplib::Response & res) {
  std::string fault_code;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    fault_code = req.matches[1];

    // Validate fault code
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
      return;
    }

    // Optional topic filter from query parameter
    std::string topic_filter = req.get_param_value("topic");

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->get_snapshots(fault_code, topic_filter);

    if (result.success) {
      HandlerContext::send_json(res, result.data);
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Fault not found") != std::string::npos) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
                                   {{"details", result.error_message}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                   "Failed to get snapshots",
                                   {{"details", result.error_message}, {"fault_code", fault_code}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to get snapshots",
                               {{"details", e.what()}, {"fault_code", fault_code}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_snapshots for fault '%s': %s", fault_code.c_str(),
                 e.what());
  }
}

void FaultHandlers::handle_get_component_snapshots(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string fault_code;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    fault_code = req.matches[2];

    auto entity_validation = ctx_.validate_entity_id(entity_id);
    if (!entity_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", entity_validation.error()}, {"entity_id", entity_id}});
      return;
    }

    // Validate fault code
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
      return;
    }

    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Optional topic filter from query parameter
    std::string topic_filter = req.get_param_value("topic");

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->get_snapshots(fault_code, topic_filter);

    if (result.success) {
      // Add entity context to response
      json response = result.data;
      response[entity_info.id_field] = entity_id;
      HandlerContext::send_json(res, response);
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Fault not found") != std::string::npos) {
        HandlerContext::send_error(
            res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(
            res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE, "Failed to get snapshots",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to get snapshots",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"fault_code", fault_code}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_component_snapshots for entity '%s', fault '%s': %s",
                 entity_id.c_str(), fault_code.c_str(), e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
