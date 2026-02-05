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
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

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

// ===== SOVD-compliant response helpers =====

/// Build SOVD status object from fault status string
/// Maps ROS 2 medkit status (PREFAILED, PREPASSED, CONFIRMED, HEALED, CLEARED)
/// to SOVD aggregated status (active, passive, cleared)
json build_status_object(const std::string & status) {
  json status_obj;

  // SOVD aggregatedStatus mapping:
  // - "active": fault is confirmed and currently active
  // - "passive": fault detected but not yet confirmed (pending)
  // - "cleared": fault resolved or manually cleared
  std::string aggregated = "cleared";
  bool test_failed = false;
  bool confirmed_dtc = false;
  bool pending_dtc = false;

  if (status == "CONFIRMED") {
    aggregated = "active";
    test_failed = true;
    confirmed_dtc = true;
  } else if (status == "PREFAILED") {
    aggregated = "passive";
    test_failed = true;
    pending_dtc = true;
  } else if (status == "PREPASSED") {
    aggregated = "passive";
    pending_dtc = true;
  } else if (status == "HEALED" || status == "CLEARED") {
    aggregated = "cleared";
  }

  status_obj["aggregatedStatus"] = aggregated;
  status_obj["testFailed"] = test_failed ? "1" : "0";
  status_obj["confirmedDTC"] = confirmed_dtc ? "1" : "0";
  status_obj["pendingDTC"] = pending_dtc ? "1" : "0";

  return status_obj;
}

/// Convert nanoseconds since epoch to ISO 8601 string with milliseconds
/// Delegates to shared utility in http_utils.hpp
std::string to_iso8601_ns(int64_t ns) {
  return ros2_medkit_gateway::format_timestamp_ns(ns);
}

/// Map fault severity level to human-readable label
std::string severity_to_label(uint8_t severity) {
  switch (severity) {
    case 0:
      return "DEBUG";
    case 1:
      return "INFO";
    case 2:
      return "WARN";
    case 3:
      return "ERROR";
    case 4:
      return "FATAL";
    default:
      return "UNKNOWN";
  }
}

/// Extract primary value from JSON data based on message type
/// For common ROS message types, extracts the main value field
json extract_primary_value(const std::string & message_type, const json & full_data) {
  // Map of message type to primary field
  static const std::unordered_map<std::string, std::string> primary_fields = {
      {"std_msgs/msg/Float64", "data"},
      {"std_msgs/msg/Float32", "data"},
      {"std_msgs/msg/Int32", "data"},
      {"std_msgs/msg/Int64", "data"},
      {"std_msgs/msg/Bool", "data"},
      {"std_msgs/msg/String", "data"},
      {"sensor_msgs/msg/Temperature", "temperature"},
      {"sensor_msgs/msg/BatteryState", "percentage"},
      {"sensor_msgs/msg/FluidPressure", "fluid_pressure"},
      {"sensor_msgs/msg/Range", "range"},
      {"geometry_msgs/msg/Twist", "linear"},  // Returns nested object
      {"nav_msgs/msg/Odometry", "pose"},      // Returns nested object
  };

  auto it = primary_fields.find(message_type);
  if (it != primary_fields.end() && full_data.contains(it->second)) {
    return full_data[it->second];
  }

  // Fallback: return full data
  return full_data;
}

}  // namespace

// Static method: Build SOVD-compliant fault response with environment data
json FaultHandlers::build_sovd_fault_response(const ros2_medkit_msgs::msg::Fault & fault,
                                              const ros2_medkit_msgs::msg::EnvironmentData & env_data,
                                              const std::string & entity_path) {
  json response;

  // === SOVD "item" structure ===
  response["item"] = {{"code", fault.fault_code},
                      {"fault_name", fault.description},
                      {"severity", fault.severity},
                      {"status", build_status_object(fault.status)}};

  // === SOVD "environment_data" ===
  json snapshots = json::array();

  for (const auto & s : env_data.snapshots) {
    json snap;
    snap["type"] = s.type;
    snap["name"] = s.name;

    if (s.type == "freeze_frame") {
      // Parse JSON data and extract primary value
      try {
        json full_data = json::parse(s.data);
        snap["data"] = extract_primary_value(s.message_type, full_data);
        snap["x-medkit"] = {{"topic", s.topic},
                            {"message_type", s.message_type},
                            {"full_data", full_data},
                            {"captured_at", to_iso8601_ns(s.captured_at_ns)}};
      } catch (const json::exception & e) {
        // Invalid JSON - include raw data
        snap["data"] = s.data;
        snap["x-medkit"] = {{"topic", s.topic}, {"message_type", s.message_type}, {"parse_error", e.what()}};
      }
    } else if (s.type == "rosbag") {
      // Build absolute URI using entity path + UUID
      snap["bulk_data_uri"] = entity_path + "/bulk-data/rosbags/" + s.bulk_data_id;
      snap["size_bytes"] = s.size_bytes;
      snap["duration_sec"] = s.duration_sec;
      snap["format"] = s.format;
    }

    snapshots.push_back(snap);
  }

  response["environment_data"] = {
      {"extended_data_records",
       {{"first_occurrence", to_iso8601_ns(env_data.extended_data_records.first_occurrence_ns)},
        {"last_occurrence", to_iso8601_ns(env_data.extended_data_records.last_occurrence_ns)}}},
      {"snapshots", snapshots}};

  // === x-medkit extensions ===
  json reporting_sources = json::array();
  for (const auto & src : fault.reporting_sources) {
    reporting_sources.push_back(src);
  }

  response["x-medkit"] = {{"occurrence_count", fault.occurrence_count},
                          {"reporting_sources", reporting_sources},
                          {"severity_label", severity_to_label(fault.severity)},
                          {"status_raw", fault.status}};

  return response;
}

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

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }
    auto entity_info = *entity_opt;

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

    // Parse entity path from URL to get entity_path for bulk_data_uri
    auto entity_path_info = parse_entity_path(req.path);
    if (!entity_path_info) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid entity path");
      return;
    }

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }
    auto entity_info = *entity_opt;

    // Fault codes may contain dots and underscores, validate basic constraints
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
      return;
    }

    std::string namespace_path = entity_info.namespace_path;

    auto fault_mgr = ctx_.node()->get_fault_manager();

    // Use get_fault_with_env to get fault with environment data
    auto result = fault_mgr->get_fault_with_env(fault_code, namespace_path);

    if (result.success) {
      // Build SOVD-compliant response with environment data
      auto response = build_sovd_fault_response(result.fault, result.environment_data, entity_path_info->entity_path);

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

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }
    auto entity_info = *entity_opt;

    // Validate fault code
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
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

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Error response already sent
    }
    auto entity_info = *entity_opt;

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

}  // namespace handlers
}  // namespace ros2_medkit_gateway
