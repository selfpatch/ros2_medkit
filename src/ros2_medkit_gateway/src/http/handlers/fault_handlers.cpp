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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <vector>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Sanitize a string for use in HTTP Content-Disposition filename
/// Removes/replaces characters that could cause header injection or filesystem issues
std::string sanitize_filename(const std::string & input) {
  std::string result;
  result.reserve(input.size());
  for (char c : input) {
    // Allow only alphanumeric, underscore, hyphen, dot
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.') {
      result += c;
    } else {
      result += '_';  // Replace unsafe characters
    }
  }
  return result;
}

/// Generate a timestamp string in YYYYMMDD_HHMMSS format for filenames
std::string generate_timestamp() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm;
  gmtime_r(&now_time, &now_tm);  // Use UTC for consistency
  std::ostringstream oss;
  oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

/// Maximum allowed length for fault_code
constexpr size_t kMaxFaultCodeLength = 128;

/// Validate fault_code format (same rules as FaultManagerNode)
/// @param fault_code The fault code to validate
/// @return Empty string if valid, error message if invalid
std::string validate_fault_code(const std::string & fault_code) {
  if (fault_code.empty()) {
    return "fault_code cannot be empty";
  }
  if (fault_code.length() > kMaxFaultCodeLength) {
    return "fault_code exceeds maximum length of " + std::to_string(kMaxFaultCodeLength) + " characters";
  }
  for (char c : fault_code) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-' && c != '.') {
      return "fault_code contains invalid character '" + std::string(1, c) +
             "'. Only alphanumeric, underscore, hyphen, and dot are allowed";
    }
  }
  if (fault_code.find("..") != std::string::npos) {
    return "fault_code cannot contain '..' (path traversal not allowed)";
  }
  return "";
}

/// Helper to filter faults JSON array by a set of namespace prefixes
/// Keeps faults where any reporting_source starts with any of the given prefixes
json filter_faults_by_sources(const json & faults_array, const std::set<std::string> & source_prefixes) {
  json filtered = json::array();
  for (const auto & fault : faults_array) {
    if (!fault.contains("reporting_sources")) {
      continue;
    }
    const auto & sources = fault["reporting_sources"];
    bool matches = false;
    for (const auto & src : sources) {
      const std::string src_str = src.get<std::string>();
      for (const auto & prefix : source_prefixes) {
        if (src_str.rfind(prefix, 0) == 0) {
          matches = true;
          break;
        }
      }
      if (matches) {
        break;
      }
    }
    if (matches) {
      filtered.push_back(fault);
    }
  }
  return filtered;
}

}  // namespace

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
      // Format: items array at top level
      json response = {{"items", result.data["faults"]}};

      // x-medkit extension for ros2_medkit-specific fields
      XMedkit ext;
      ext.add("count", result.data["count"]);
      ext.add("muted_count", result.data["muted_count"]);
      ext.add("cluster_count", result.data["cluster_count"]);

      // Include detailed correlation data if requested and present
      if (result.data.contains("muted_faults")) {
        ext.add("muted_faults", result.data["muted_faults"]);
      }
      if (result.data.contains("clusters")) {
        ext.add("clusters", result.data["clusters"]);
      }

      if (!ext.empty()) {
        response["x-medkit"] = ext.build();
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

    // For Functions, aggregate faults from all host apps
    // Functions don't have a single namespace_path - they host apps from potentially different namespaces
    if (entity_info.type == EntityType::FUNCTION) {
      // Get all faults (no namespace filter)
      auto result = fault_mgr->get_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                          include_muted, include_clusters);

      if (!result.success) {
        HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                   "Failed to get faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }

      // Collect host app FQNs for filtering
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto agg_configs = cache.get_entity_configurations(entity_id);
      std::set<std::string> host_fqns;
      for (const auto & node : agg_configs.nodes) {
        if (!node.node_fqn.empty()) {
          host_fqns.insert(node.node_fqn);
        }
      }

      // Filter faults to only those from function's host apps
      json filtered_faults = filter_faults_by_sources(result.data["faults"], host_fqns);

      // Build response
      json response = {{"items", filtered_faults}};

      XMedkit ext;
      ext.entity_id(entity_id);
      ext.add("aggregation_level", "function");
      ext.add("count", filtered_faults.size());
      ext.add("host_count", host_fqns.size());

      response["x-medkit"] = ext.build();
      HandlerContext::send_json(res, response);
      return;
    }

    // For Components, aggregate faults from all hosted apps
    // Components group Apps, so we filter by the apps' FQNs rather than namespace (which is too broad)
    if (entity_info.type == EntityType::COMPONENT) {
      // Get all faults (no namespace filter)
      auto result = fault_mgr->get_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                          include_muted, include_clusters);

      if (!result.success) {
        HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                   "Failed to get faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }

      // Collect hosted app FQNs for filtering
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto app_ids = cache.get_apps_for_component(entity_id);
      std::set<std::string> app_fqns;
      for (const auto & app_id : app_ids) {
        auto app = cache.get_app(app_id);
        if (app && app->bound_fqn.has_value() && !app->bound_fqn->empty()) {
          app_fqns.insert(app->bound_fqn.value());
        }
      }

      // Filter faults to only those from component's hosted apps
      json filtered_faults = filter_faults_by_sources(result.data["faults"], app_fqns);

      // Build response
      json response = {{"items", filtered_faults}};

      XMedkit ext;
      ext.entity_id(entity_id);
      ext.add("aggregation_level", "component");
      ext.add("count", filtered_faults.size());
      ext.add("app_count", app_fqns.size());

      response["x-medkit"] = ext.build();
      HandlerContext::send_json(res, response);
      return;
    }

    // For other entity types (App, Area), use namespace_path filtering
    std::string namespace_path = entity_info.namespace_path;
    auto result = fault_mgr->get_faults(namespace_path, filter.include_pending, filter.include_confirmed,
                                        filter.include_cleared, include_muted, include_clusters);

    if (result.success) {
      // Format: items array at top level
      json response = {{"items", result.data["faults"]}};

      // x-medkit extension for ros2_medkit-specific fields
      XMedkit ext;
      ext.entity_id(entity_id);
      ext.add("source_id", namespace_path);
      ext.add("count", result.data["count"]);
      ext.add("muted_count", result.data["muted_count"]);
      ext.add("cluster_count", result.data["cluster_count"]);

      // Include detailed correlation data if requested and present
      if (result.data.contains("muted_faults")) {
        ext.add("muted_faults", result.data["muted_faults"]);
      }
      if (result.data.contains("clusters")) {
        ext.add("clusters", result.data["clusters"]);
      }

      response["x-medkit"] = ext.build();
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
      // Format: single item wrapped
      json response = {{"item", result.data}};

      // x-medkit extension for entity context
      XMedkit ext;
      ext.entity_id(entity_id);
      response["x-medkit"] = ext.build();

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
      // Format: return 204 No Content on successful delete
      res.status = StatusCode::NoContent_204;
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

void FaultHandlers::handle_clear_all_faults(const httplib::Request & req, httplib::Response & res) {
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

    // Verify entity exists
    auto entity_info = ctx_.get_entity_info(entity_id);
    if (entity_info.type == EntityType::UNKNOWN) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Get all faults for this entity
    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto faults_result = fault_mgr->get_faults(entity_info.namespace_path, "", "");

    if (!faults_result.success) {
      HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                 "Failed to retrieve faults",
                                 {{"details", faults_result.error_message}, {entity_info.id_field, entity_id}});
      return;
    }

    // Clear each fault
    if (faults_result.data.contains("faults") && faults_result.data["faults"].is_array()) {
      for (const auto & fault : faults_result.data["faults"]) {
        if (fault.contains("faultCode")) {
          std::string fault_code = fault["faultCode"].get<std::string>();
          auto clear_result = fault_mgr->clear_fault(fault_code);
          if (!clear_result.success) {
            RCLCPP_WARN(HandlerContext::logger(), "Failed to clear fault '%s' for entity '%s': %s", fault_code.c_str(),
                        entity_id.c_str(), clear_result.error_message.c_str());
          }
        }
      }
    }

    // Format: return 204 No Content on successful delete
    res.status = StatusCode::NoContent_204;

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Failed to clear faults",
                               {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_clear_all_faults for entity '%s': %s", entity_id.c_str(),
                 e.what());
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

void FaultHandlers::handle_get_rosbag(const httplib::Request & req, httplib::Response & res) {
  std::string fault_code;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request", {});
      return;
    }

    fault_code = req.matches[1];

    // Validate fault code format (prevents path traversal and injection attacks)
    std::string validation_error = validate_fault_code(fault_code);
    if (!validation_error.empty()) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", validation_error}, {"fault_code", fault_code}});
      return;
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->get_rosbag(fault_code);

    if (!result.success) {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("No rosbag") != std::string::npos) {
        HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Rosbag not found",
                                   {{"details", result.error_message}, {"fault_code", fault_code}});
      } else if (result.error_message.find("invalid") != std::string::npos) {
        // Validation error from service
        HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                   {{"details", result.error_message}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(res, StatusCode::ServiceUnavailable_503, ERR_SERVICE_UNAVAILABLE,
                                   "Failed to get rosbag",
                                   {{"details", result.error_message}, {"fault_code", fault_code}});
      }
      return;
    }

    // Get file path from result
    std::string file_path = result.data["file_path"].get<std::string>();
    std::string format = result.data["format"].get<std::string>();

    // Check if path is a directory (rosbag2 creates directories)
    std::filesystem::path bag_path(file_path);
    bool is_directory = std::filesystem::is_directory(bag_path);
    std::string archive_path;  // Will be set if we create a temp archive

    // Determine content type and filename based on what we're sending
    std::string content_type;
    std::string timestamp = generate_timestamp();
    std::string filename;

    if (is_directory) {
      // Create tar.gz archive of the entire bag directory (includes all segments + metadata)
      archive_path = std::filesystem::temp_directory_path().string() + "/rosbag_download_" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".tar.gz";

      std::string tar_cmd = "tar -czf " + archive_path + " -C " + bag_path.parent_path().string() + " " +
                            bag_path.filename().string() + " 2>/dev/null";

      int tar_result = std::system(tar_cmd.c_str());
      if (tar_result != 0) {
        HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                                   "Failed to create rosbag archive",
                                   {{"fault_code", fault_code}, {"path", file_path}});
        return;
      }

      file_path = archive_path;
      content_type = "application/gzip";
      filename = "fault_" + sanitize_filename(fault_code) + "_" + timestamp + ".tar.gz";
    } else {
      // Single file - determine type from format
      if (format == "mcap") {
        content_type = "application/x-mcap";
        filename = "fault_" + sanitize_filename(fault_code) + "_" + timestamp + ".mcap";
      } else {
        content_type = "application/octet-stream";
        filename = "fault_" + sanitize_filename(fault_code) + "_" + timestamp + ".db3";
      }
    }

    // Check file exists and get size
    std::error_code ec;
    auto file_size = std::filesystem::file_size(file_path, ec);
    if (ec) {
      if (!archive_path.empty()) {
        std::filesystem::remove(archive_path, ec);
      }
      HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                                 "Failed to read rosbag file", {{"fault_code", fault_code}, {"path", file_path}});
      return;
    }

    res.set_header("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    res.set_header("Content-Type", content_type);
    res.status = StatusCode::OK_200;

    // Use streaming response for large files to avoid loading entire bag into memory
    std::string path_copy = file_path;        // Capture for content provider lambda
    std::string archive_copy = archive_path;  // Capture for cleanup lambda
    res.set_content_provider(
        file_size, content_type,
        [path_copy](size_t offset, size_t length, httplib::DataSink & sink) {
          std::ifstream file(path_copy, std::ios::binary);
          if (!file) {
            return false;
          }
          file.seekg(static_cast<std::streamoff>(offset));
          constexpr size_t kChunkSize = 65536;  // 64KB chunks
          std::vector<char> buffer(std::min(length, kChunkSize));
          size_t remaining = length;
          while (remaining > 0 && file) {
            size_t to_read = std::min(remaining, kChunkSize);
            file.read(buffer.data(), static_cast<std::streamsize>(to_read));
            auto bytes_read = static_cast<size_t>(file.gcount());
            if (bytes_read == 0) {
              break;
            }
            sink.write(buffer.data(), bytes_read);
            remaining -= bytes_read;
          }
          return true;
        },
        [archive_copy](bool /*success*/) {
          // Resource releaser callback - clean up temp archive if we created one
          if (!archive_copy.empty()) {
            std::error_code cleanup_ec;
            std::filesystem::remove(archive_copy, cleanup_ec);
            // Ignore errors - temp file cleanup is best-effort
          }
        });

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to download rosbag", {{"details", e.what()}, {"fault_code", fault_code}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_rosbag for fault '%s': %s", fault_code.c_str(),
                 e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
