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

nlohmann::json SchemaBuilder::fault_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"fault_code", {{"type", "string"}}},
            {"entity_id", {{"type", "string"}}},
            {"severity", {{"type", "string"}}},
            {"status", {{"type", "object"}}},
            {"timestamp", {{"type", "string"}}}}},
          {"required", {"fault_code", "severity", "status"}}};
}

nlohmann::json SchemaBuilder::fault_list_schema() {
  return items_wrapper(fault_schema());
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
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "integer"}}},
            {"timestamp", {{"type", "string"}}},
            {"level", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"message", {{"type", "string"}}},
            {"function", {{"type", "string"}}},
            {"file", {{"type", "string"}}},
            {"line", {{"type", "integer"}}}}},
          {"required", {"id", "timestamp", "level", "message"}}};
}

nlohmann::json SchemaBuilder::health_schema() {
  return {{"type", "object"},
          {"properties", {{"status", {{"type", "string"}}}, {"timestamp", {{"type", "integer"}}}}},
          {"required", {"status"}}};
}

nlohmann::json SchemaBuilder::version_info_schema() {
  return {{"type", "object"},
          {"properties",
           {{"sovd_info",
             {{"type", "array"},
              {"items",
               {{"type", "object"},
                {"properties",
                 {{"version", {{"type", "string"}}},
                  {"base_uri", {{"type", "string"}}},
                  {"vendor_info", {{"type", "object"}}}}},
                {"required", {"version", "base_uri"}}}}}}}},
          {"required", {"sovd_info"}}};
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
