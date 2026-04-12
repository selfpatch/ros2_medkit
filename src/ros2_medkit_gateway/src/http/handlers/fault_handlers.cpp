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

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"
#include "ros2_medkit_gateway/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/providers/fault_provider.hpp"

using json = nlohmann::json;

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
      // Build absolute URI using entity path + fault_code as the bulk-data ID.
      // This must match the download handler which looks up rosbags by fault_code,
      // and handle_list_descriptors which also uses fault_code as the descriptor ID.
      snap["bulk_data_uri"] = entity_path + "/bulk-data/rosbags/" + fault.fault_code;
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
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid status parameter value",
                                 {{"allowed_values", "pending, confirmed, cleared, healed, all"},
                                  {"parameter", "status"},
                                  {"value", req.get_param_value("status")}});
      return;
    }

    // Parse correlation query parameters
    bool include_muted = req.get_param_value("include_muted") == "true";
    bool include_clusters = req.get_param_value("include_clusters") == "true";

    auto fault_mgr = ctx_.node()->get_fault_manager();
    // Empty source_id = no filtering, return all faults
    auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                         filter.include_healed, include_muted, include_clusters);

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

      // Fan-out to peers: faults are managed by FaultManager, not cached,
      // so we need to query each peer's /faults endpoint and merge results.
      if (auto * agg = ctx_.aggregation_manager()) {
        auto fan_result = agg->fan_out_get(req.path, req.get_header_value("Authorization"));
        if (fan_result.merged_items.is_array()) {
          for (const auto & item : fan_result.merged_items) {
            response["items"].push_back(item);
          }
        }
        if (fan_result.is_partial) {
          ext.add("partial", true);
          ext.add("failed_peers", fan_result.failed_peers);
        }
      }

      if (!ext.empty()) {
        response["x-medkit"] = ext.build();
      }

      res.status = 200;
      HandlerContext::send_json(res, response);
    } else {
      HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                                 {{"details", result.error_message}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to list faults", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_all_faults: %s", e.what());
  }
}

void FaultHandlers::handle_list_faults(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }
    auto entity_info = *entity_opt;

    // Delegate to plugin FaultProvider if entity is plugin-owned
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      if (fault_prov) {
        try {
          auto result = fault_prov->list_faults(entity_id);
          if (result) {
            HandlerContext::send_json(res, *result);
          } else {
            HandlerContext::send_plugin_error(res, result.error().http_status, result.error().message,
                                              {{"entity_id", entity_id}});
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
        }
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No fault provider for plugin entity '" + entity_id + "'");
      return;
    }

    // Validate entity type supports faults collection (SOVD Table 8)
    if (auto err = HandlerContext::validate_collection_access(entity_info, ResourceCollection::FAULTS)) {
      HandlerContext::send_error(res, 400, ERR_COLLECTION_NOT_SUPPORTED, *err);
      return;
    }

    auto filter = parse_fault_status_param(req);
    if (!filter.is_valid) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid status parameter value",
                                 {{"allowed_values", "pending, confirmed, cleared, healed, all"},
                                  {"parameter", "status"},
                                  {"value", req.get_param_value("status")},
                                  {entity_info.id_field, entity_id}});
      return;
    }

    // Parse correlation query parameters
    bool include_muted = req.get_param_value("include_muted") == "true";
    bool include_clusters = req.get_param_value("include_clusters") == "true";

    auto fault_mgr = ctx_.node()->get_fault_manager();

    // For Functions, aggregate faults from all host apps.
    // Functions don't have a single namespace_path (it is always empty in EntityInfo)
    // because they host apps from potentially different namespaces.
    // Instead, we collect the FQNs of all host apps and filter by reporting_source.
    if (entity_info.type == EntityType::FUNCTION) {
      // Get all faults (no namespace filter)
      auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                           filter.include_healed, include_muted, include_clusters);

      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
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
      ext.add("aggregated", true);
      ext.add("host_count", host_fqns.size());
      // Include source app IDs for cross-referencing aggregated results
      json source_ids = json::array();
      for (const auto & fqn : host_fqns) {
        source_ids.push_back(fqn);
      }
      ext.add("aggregation_sources", source_ids);

      merge_peer_items(ctx_.aggregation_manager(), req, response, ext);
      ext.add("count", response["items"].size());
      response["x-medkit"] = ext.build();
      HandlerContext::send_json(res, response);
      return;
    }

    // For Components, aggregate faults from all hosted apps
    // Components group Apps, so we filter by the apps' FQNs rather than namespace (which is too broad)
    if (entity_info.type == EntityType::COMPONENT) {
      // Get all faults (no namespace filter)
      auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                           filter.include_healed, include_muted, include_clusters);

      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }

      // Collect hosted app FQNs for filtering
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto app_ids = cache.get_apps_for_component(entity_id);
      std::set<std::string> app_fqns;
      for (const auto & app_id : app_ids) {
        auto app = cache.get_app(app_id);
        if (app) {
          auto fqn = app->effective_fqn();
          if (!fqn.empty()) {
            app_fqns.insert(std::move(fqn));
          }
        }
      }

      // Filter faults to only those from component's hosted apps
      json filtered_faults = filter_faults_by_sources(result.data["faults"], app_fqns);

      // Build response
      json response = {{"items", filtered_faults}};

      XMedkit ext;
      ext.entity_id(entity_id);
      ext.add("aggregation_level", "component");
      ext.add("aggregated", true);
      ext.add("app_count", app_fqns.size());
      // Include source app FQNs for cross-referencing aggregated results
      json source_fqns = json::array();
      for (const auto & fqn : app_fqns) {
        source_fqns.push_back(fqn);
      }
      ext.add("aggregation_sources", source_fqns);

      merge_peer_items(ctx_.aggregation_manager(), req, response, ext);
      ext.add("count", response["items"].size());
      response["x-medkit"] = ext.build();
      HandlerContext::send_json(res, response);
      return;
    }

    // For Areas, aggregate faults from all apps in all components within the area
    // This is an x-medkit extension - SOVD spec does not define fault collections for Areas
    if (entity_info.type == EntityType::AREA) {
      // Get all faults (no namespace filter)
      auto result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed, filter.include_cleared,
                                           filter.include_healed, include_muted, include_clusters);

      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }

      // Collect FQNs from all apps in all components belonging to this area
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto comp_ids = cache.get_components_for_area(entity_id);
      std::set<std::string> app_fqns;
      for (const auto & comp_id : comp_ids) {
        auto app_ids = cache.get_apps_for_component(comp_id);
        for (const auto & app_id : app_ids) {
          auto app = cache.get_app(app_id);
          if (app) {
            auto fqn = app->effective_fqn();
            if (!fqn.empty()) {
              app_fqns.insert(std::move(fqn));
            }
          }
        }
      }

      // Filter faults to only those from the area's apps
      json filtered_faults = filter_faults_by_sources(result.data["faults"], app_fqns);

      // Build response
      json response = {{"items", filtered_faults}};

      XMedkit ext;
      ext.entity_id(entity_id);
      ext.add("aggregation_level", "area");
      ext.add("aggregated", true);
      ext.add("component_count", comp_ids.size());
      ext.add("app_count", app_fqns.size());
      // Include source app FQNs for cross-referencing aggregated results
      json area_source_fqns = json::array();
      for (const auto & fqn : app_fqns) {
        area_source_fqns.push_back(fqn);
      }
      ext.add("aggregation_sources", area_source_fqns);

      merge_peer_items(ctx_.aggregation_manager(), req, response, ext);
      ext.add("count", response["items"].size());
      response["x-medkit"] = ext.build();
      HandlerContext::send_json(res, response);
      return;
    }

    // For Apps, use namespace_path filtering
    std::string namespace_path = entity_info.namespace_path;
    auto result =
        fault_mgr->list_faults(namespace_path, filter.include_pending, filter.include_confirmed, filter.include_cleared,
                               filter.include_healed, include_muted, include_clusters);

    if (result.success) {
      // Format: items array at top level
      json response = {{"items", result.data["faults"]}};

      // x-medkit extension for ros2_medkit-specific fields
      XMedkit ext;
      ext.entity_id(entity_id);
      ext.add("source_id", namespace_path);
      ext.add("muted_count", result.data["muted_count"]);
      ext.add("cluster_count", result.data["cluster_count"]);

      // Include detailed correlation data if requested and present
      if (result.data.contains("muted_faults")) {
        ext.add("muted_faults", result.data["muted_faults"]);
      }
      if (result.data.contains("clusters")) {
        ext.add("clusters", result.data["clusters"]);
      }

      merge_peer_items(ctx_.aggregation_manager(), req, response, ext);
      ext.add("count", response["items"].size());
      response["x-medkit"] = ext.build();
      HandlerContext::send_json(res, response);
    } else {
      HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                                 {{"details", result.error_message}, {entity_info.id_field, entity_id}});
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to list faults",
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
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    fault_code = req.matches[2];

    // Parse entity path from URL to get entity_path for bulk_data_uri
    auto entity_path_info = parse_entity_path(req.path);
    if (!entity_path_info) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid entity path");
      return;
    }

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }
    auto entity_info = *entity_opt;

    // Delegate to plugin FaultProvider if entity is plugin-owned
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      if (fault_prov) {
        try {
          auto result = fault_prov->get_fault(entity_id, fault_code);
          if (result) {
            HandlerContext::send_json(res, *result);
          } else {
            HandlerContext::send_plugin_error(res, result.error().http_status, result.error().message,
                                              {{"entity_id", entity_id}});
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
        }
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No fault provider for plugin entity '" + entity_id + "'");
      return;
    }

    // Fault codes may contain dots and underscores, validate basic constraints
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid fault code",
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
            res, 404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(
            res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to get fault",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to get fault",
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
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    fault_code = req.matches[2];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }
    auto entity_info = *entity_opt;

    // Check lock access for faults (before plugin delegation - locks apply to all entities)
    if (ctx_.validate_lock_access(req, res, entity_info, "faults")) {
      return;
    }

    // Delegate to plugin FaultProvider if entity is plugin-owned
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      if (fault_prov) {
        try {
          auto result = fault_prov->clear_fault(entity_id, fault_code);
          if (result) {
            HandlerContext::send_json(res, *result);
          } else {
            HandlerContext::send_plugin_error(res, result.error().http_status, result.error().message,
                                              {{"entity_id", entity_id}});
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
        }
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No fault provider for plugin entity '" + entity_id + "'");
      return;
    }

    // Validate fault code
    if (fault_code.empty() || fault_code.length() > 256) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid fault code",
                                 {{"details", "Fault code must be between 1 and 256 characters"}});
      return;
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    auto result = fault_mgr->clear_fault(fault_code);

    if (result.success) {
      // Format: return 204 No Content on successful delete
      res.status = 204;
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Fault not found") != std::string::npos) {
        HandlerContext::send_error(
            res, 404, ERR_RESOURCE_NOT_FOUND, "Fault not found",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      } else {
        HandlerContext::send_error(
            res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to clear fault",
            {{"details", result.error_message}, {entity_info.id_field, entity_id}, {"fault_code", fault_code}});
      }
    }
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to clear fault",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"fault_code", fault_code}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_clear_fault for entity '%s', fault '%s': %s",
                 entity_id.c_str(), fault_code.c_str(), e.what());
  }
}

void FaultHandlers::handle_clear_all_faults(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }
    auto entity_info = *entity_opt;

    // Check lock access for faults (before plugin delegation - locks apply to all entities)
    if (ctx_.validate_lock_access(req, res, entity_info, "faults")) {
      return;
    }

    // Delegate to plugin FaultProvider if entity is plugin-owned
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * fault_prov = pmgr ? pmgr->get_fault_provider_for_entity(entity_id) : nullptr;
      if (fault_prov) {
        try {
          auto list_result = fault_prov->list_faults(entity_id);
          if (list_result && list_result->contains("items") && (*list_result)["items"].is_array()) {
            std::vector<std::string> failed_codes;
            for (const auto & fault : (*list_result)["items"]) {
              auto code = fault.value("code", "");
              if (!code.empty()) {
                auto clear_result = fault_prov->clear_fault(entity_id, code);
                if (!clear_result) {
                  failed_codes.push_back(code);
                }
              }
            }
            if (!failed_codes.empty()) {
              HandlerContext::send_plugin_error(res, 500,
                                                "Failed to clear " + std::to_string(failed_codes.size()) + " fault(s)",
                                                {{"entity_id", entity_id}, {"failed_codes", failed_codes}});
              return;
            }
          } else if (!list_result) {
            HandlerContext::send_plugin_error(res, list_result.error().http_status, list_result.error().message,
                                              {{"entity_id", entity_id}});
            return;
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
          return;
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin FaultProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
          return;
        }
        res.status = 204;
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No fault provider for plugin entity '" + entity_id + "'");
      return;
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();

    // For non-App entities (Functions, Components, Areas), namespace_path is
    // either empty or too broad for accurate filtering. Use the same FQN-based
    // approach as handle_list_faults: collect host app FQNs and filter by
    // reporting_source match.
    json faults_to_clear;

    if (entity_info.type == EntityType::FUNCTION) {
      auto result = fault_mgr->list_faults("");
      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto agg_configs = cache.get_entity_configurations(entity_id);
      std::set<std::string> host_fqns;
      for (const auto & node : agg_configs.nodes) {
        if (!node.node_fqn.empty()) {
          host_fqns.insert(node.node_fqn);
        }
      }
      faults_to_clear = filter_faults_by_sources(result.data["faults"], host_fqns);

    } else if (entity_info.type == EntityType::COMPONENT) {
      auto result = fault_mgr->list_faults("");
      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto app_ids = cache.get_apps_for_component(entity_id);
      std::set<std::string> app_fqns;
      for (const auto & app_id : app_ids) {
        auto app = cache.get_app(app_id);
        if (app) {
          auto fqn = app->effective_fqn();
          if (!fqn.empty()) {
            app_fqns.insert(std::move(fqn));
          }
        }
      }
      faults_to_clear = filter_faults_by_sources(result.data["faults"], app_fqns);

    } else if (entity_info.type == EntityType::AREA) {
      auto result = fault_mgr->list_faults("");
      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      auto comp_ids = cache.get_components_for_area(entity_id);
      std::set<std::string> app_fqns;
      for (const auto & comp_id : comp_ids) {
        auto app_ids_inner = cache.get_apps_for_component(comp_id);
        for (const auto & app_id : app_ids_inner) {
          auto app = cache.get_app(app_id);
          if (app) {
            auto fqn = app->effective_fqn();
            if (!fqn.empty()) {
              app_fqns.insert(std::move(fqn));
            }
          }
        }
      }
      faults_to_clear = filter_faults_by_sources(result.data["faults"], app_fqns);

    } else {
      // Apps: use namespace_path filtering directly
      auto result = fault_mgr->list_faults(entity_info.namespace_path);
      if (!result.success) {
        HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                                   {{"details", result.error_message}, {entity_info.id_field, entity_id}});
        return;
      }
      faults_to_clear = result.data["faults"];
    }

    // Clear each matching fault
    if (faults_to_clear.is_array()) {
      for (const auto & fault : faults_to_clear) {
        if (fault.contains("fault_code")) {
          std::string fault_code = fault["fault_code"].get<std::string>();
          auto clear_result = fault_mgr->clear_fault(fault_code);
          if (!clear_result.success) {
            RCLCPP_WARN(HandlerContext::logger(), "Failed to clear fault '%s' for entity '%s': %s", fault_code.c_str(),
                        entity_id.c_str(), clear_result.error_message.c_str());
          }
        }
      }
    }

    // Format: return 204 No Content on successful delete
    res.status = 204;

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to clear faults",
                               {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_clear_all_faults for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void FaultHandlers::handle_clear_all_faults_global(const httplib::Request & req, httplib::Response & res) {
  try {
    auto filter = parse_fault_status_param(req);
    if (!filter.is_valid) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid status parameter value",
                                 {{"allowed_values", "pending, confirmed, cleared, healed, all"},
                                  {"parameter", "status"},
                                  {"value", req.get_param_value("status")}});
      return;
    }

    auto fault_mgr = ctx_.node()->get_fault_manager();
    // Global clear is the "nuclear option" — always include muted (correlated) faults,
    // unlike per-entity clear which respects the default include_muted=false.
    auto faults_result = fault_mgr->list_faults("", filter.include_pending, filter.include_confirmed,
                                                filter.include_cleared, filter.include_healed, true);

    if (!faults_result.success) {
      HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Failed to retrieve faults",
                                 {{"details", faults_result.error_message}});
      return;
    }

    // Build FQN-to-entity-ID map for lock checking
    auto * lock_mgr = ctx_.node() ? ctx_.node()->get_lock_manager() : nullptr;
    std::unordered_map<std::string, std::string> fqn_to_entity;
    if (lock_mgr) {
      const auto & cache = ctx_.node()->get_thread_safe_cache();
      for (const auto & app : cache.get_apps()) {
        auto fqn = app.effective_fqn();
        if (!fqn.empty()) {
          fqn_to_entity[fqn] = app.id;
        }
      }
    }

    auto client_id = req.get_header_value("X-Client-Id");

    // Clear each fault, skipping those on locked entities
    if (faults_result.data.contains("faults") && faults_result.data["faults"].is_array()) {
      for (const auto & fault : faults_result.data["faults"]) {
        if (!fault.contains("fault_code")) {
          continue;
        }

        // Check if any reporting source is on a locked entity
        bool blocked = false;
        if (lock_mgr && fault.contains("reporting_sources")) {
          for (const auto & src : fault["reporting_sources"]) {
            auto src_str = src.get<std::string>();
            auto it = fqn_to_entity.find(src_str);
            if (it != fqn_to_entity.end()) {
              auto access = lock_mgr->check_access(it->second, client_id, "faults");
              if (!access.allowed) {
                blocked = true;
                break;
              }
            }
          }
        }

        if (blocked) {
          continue;  // Skip faults on locked entities
        }

        std::string fault_code = fault["fault_code"].get<std::string>();
        auto clear_result = fault_mgr->clear_fault(fault_code);
        if (!clear_result.success) {
          RCLCPP_WARN(HandlerContext::logger(), "Failed to clear fault '%s': %s", fault_code.c_str(),
                      clear_result.error_message.c_str());
        }
      }
    }

    // Design limitation: this only clears faults on the local FaultManager.
    // Peer faults visible via fan_out_get in handle_list_all_faults are NOT
    // cleared because that would require fan-out DELETE requests to each peer,
    // which introduces distributed transaction semantics (partial failures,
    // rollback) that are out of scope. Clients should clear peer faults by
    // calling each peer's clear endpoint directly.
    //
    // The X-Medkit-Local-Only header signals this to clients. A 204 response
    // cannot carry a JSON body per HTTP spec, so we use a header instead.
    res.set_header("X-Medkit-Local-Only", "true");
    res.status = 204;

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to clear faults", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_clear_all_faults_global: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
