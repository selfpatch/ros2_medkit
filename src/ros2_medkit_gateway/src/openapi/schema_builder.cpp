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

#include "schema_builder.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

SchemaBuilder::SchemaBuilder() : serializer_() {
}

nlohmann::json SchemaBuilder::from_ros_msg(const std::string & type_string) const {
  try {
    return serializer_.get_schema(type_string);
  } catch (const std::exception &) {
    // Expected for unknown ROS 2 message types - graceful degradation
    return {{"type", "object"}, {"x-medkit-schema-unavailable", true}};
  } catch (...) {
    return {{"type", "object"}, {"x-medkit-schema-unavailable", true}};
  }
}

nlohmann::json SchemaBuilder::from_ros_srv_request(const std::string & srv_type) const {
  // Service request type: "pkg/srv/Name" -> "pkg/srv/Name_Request"
  return from_ros_msg(srv_type + "_Request");
}

nlohmann::json SchemaBuilder::from_ros_srv_response(const std::string & srv_type) const {
  // Service response type: "pkg/srv/Name" -> "pkg/srv/Name_Response"
  return from_ros_msg(srv_type + "_Response");
}

nlohmann::json SchemaBuilder::generic_error() {
  return {
      {"type", "object"},
      {"properties",
       {{"error_code", {{"type", "string"}}}, {"message", {{"type", "string"}}}, {"parameters", {{"type", "object"}}}}},
      {"required", {"error_code", "message"}}};
}

nlohmann::json SchemaBuilder::fault_list_item_schema() {
  return {
      {"type", "object"},
      {"properties",
       {{"fault_code", {{"type", "string"}}},
        {"severity", {{"type", "integer"}, {"description", "Numeric severity level"}}},
        {"severity_label", {{"type", "string"}, {"enum", {"INFO", "WARN", "ERROR", "CRITICAL"}}}},
        {"description", {{"type", "string"}}},
        {"first_occurred", {{"type", "number"}, {"description", "Unix timestamp (seconds with nanosecond fraction)"}}},
        {"last_occurred", {{"type", "number"}, {"description", "Unix timestamp (seconds with nanosecond fraction)"}}},
        {"occurrence_count", {{"type", "integer"}}},
        {"status", {{"type", "string"}}},
        {"reporting_sources", {{"type", "array"}, {"items", {{"type", "string"}}}}}}},
      {"required", {"fault_code", "severity", "status"}}};
}

nlohmann::json SchemaBuilder::fault_detail_schema() {
  // SOVD nested structure from FaultHandlers::build_sovd_fault_response
  nlohmann::json status_schema = {
      {"type", "object"},
      {"properties",
       {{"aggregatedStatus", {{"type", "string"}, {"enum", {"active", "passive", "cleared"}}}},
        {"testFailed", {{"type", "string"}}},
        {"confirmedDTC", {{"type", "string"}}},
        {"pendingDTC", {{"type", "string"}}}}},
      {"required", {"aggregatedStatus"}}};

  nlohmann::json item_schema = {{"type", "object"},
                                {"properties",
                                 {{"code", {{"type", "string"}}},
                                  {"fault_name", {{"type", "string"}}},
                                  {"severity", {{"type", "integer"}}},
                                  {"status", status_schema}}},
                                {"required", {"code", "severity", "status"}}};

  nlohmann::json snapshot_schema = {{"type", "object"},
                                    {"properties",
                                     {{"type", {{"type", "string"}}},
                                      {"name", {{"type", "string"}}},
                                      {"data", {}},
                                      {"bulk_data_uri", {{"type", "string"}}},
                                      {"size_bytes", {{"type", "integer"}}},
                                      {"duration_sec", {{"type", "number"}}},
                                      {"format", {{"type", "string"}}},
                                      {"x-medkit", {{"type", "object"}}}}}};

  nlohmann::json env_data_schema = {{"type", "object"},
                                    {"properties",
                                     {{"extended_data_records",
                                       {{"type", "object"},
                                        {"properties",
                                         {{"first_occurrence", {{"type", "string"}, {"format", "date-time"}}},
                                          {"last_occurrence", {{"type", "string"}, {"format", "date-time"}}}}}}},
                                      {"snapshots", {{"type", "array"}, {"items", snapshot_schema}}}}}};

  nlohmann::json x_medkit_schema = {{"type", "object"},
                                    {"properties",
                                     {{"occurrence_count", {{"type", "integer"}}},
                                      {"reporting_sources", {{"type", "array"}, {"items", {{"type", "string"}}}}},
                                      {"severity_label", {{"type", "string"}}},
                                      {"status_raw", {{"type", "string"}}}}}};

  return {{"type", "object"},
          {"properties", {{"item", item_schema}, {"environment_data", env_data_schema}, {"x-medkit", x_medkit_schema}}},
          {"required", {"item", "environment_data"}}};
}

nlohmann::json SchemaBuilder::fault_list_schema() {
  return items_wrapper(fault_list_item_schema());
}

nlohmann::json SchemaBuilder::entity_detail_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"type", {{"type", "string"}}},
            {"uri", {{"type", "string"}}}}},
          {"required", {"id", "name"}}};
}

nlohmann::json SchemaBuilder::entity_list_schema() {
  return items_wrapper(entity_detail_schema());
}

nlohmann::json SchemaBuilder::items_wrapper(const nlohmann::json & item_schema) {
  return {{"type", "object"},
          {"properties", {{"items", {{"type", "array"}, {"items", item_schema}}}}},
          {"required", {"items"}}};
}

nlohmann::json SchemaBuilder::configuration_param_schema() {
  return {{"type", "object"},
          {"properties", {{"name", {{"type", "string"}}}, {"value", {}}, {"type", {{"type", "string"}}}}},
          {"required", {"name", "value"}}};
}

nlohmann::json SchemaBuilder::log_entry_schema() {
  nlohmann::json context_schema = {{"type", "object"},
                                   {"properties",
                                    {{"node", {{"type", "string"}}},
                                     {"function", {{"type", "string"}}},
                                     {"file", {{"type", "string"}}},
                                     {"line", {{"type", "integer"}}}}},
                                   {"required", {"node"}}};

  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}, {"description", "Log entry ID (e.g. log_123)"}}},
            {"timestamp", {{"type", "string"}, {"format", "date-time"}}},
            {"severity", {{"type", "string"}}},
            {"message", {{"type", "string"}}},
            {"context", context_schema}}},
          {"required", {"id", "timestamp", "severity", "message"}}};
}

nlohmann::json SchemaBuilder::health_schema() {
  nlohmann::json discovery_schema = {
      {"type", "object"},
      {"properties", {{"mode", {{"type", "string"}}}, {"strategy", {{"type", "string"}}}}},
      {"description", "Discovery subsystem status"}};

  return {{"type", "object"},
          {"properties",
           {{"status", {{"type", "string"}}}, {"timestamp", {{"type", "integer"}}}, {"discovery", discovery_schema}}},
          {"required", {"status"}}};
}

nlohmann::json SchemaBuilder::version_info_schema() {
  nlohmann::json vendor_info_schema = {
      {"type", "object"},
      {"properties", {{"version", {{"type", "string"}}}, {"name", {{"type", "string"}}}}},
      {"required", {"version", "name"}}};

  nlohmann::json info_entry_schema = {
      {"type", "object"},
      {"properties",
       {{"version", {{"type", "string"}}}, {"base_uri", {{"type", "string"}}}, {"vendor_info", vendor_info_schema}}},
      {"required", {"version", "base_uri"}}};

  return {{"type", "object"},
          {"properties", {{"items", {{"type", "array"}, {"items", info_entry_schema}}}}},
          {"required", {"items"}}};
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
