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

#pragma once

#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

/// Wraps JsonSerializer::get_schema() with OpenAPI-specific formatting,
/// graceful degradation, and static schema factories for SOVD types.
class SchemaBuilder {
 public:
  /// Constructs SchemaBuilder with its own JsonSerializer instance.
  /// JsonSerializer is stateless and thread-safe - no sharing needed.
  SchemaBuilder();

  // Runtime schema generation from ROS 2 types

  /// Generate JSON Schema for a ROS 2 message type.
  /// Returns a generic object schema with x-medkit-schema-unavailable on failure.
  nlohmann::json from_ros_msg(const std::string & type_string) const;

  /// Generate JSON Schema for a ROS 2 service request type.
  /// @param srv_type Service type string, e.g. "std_srvs/srv/SetBool"
  nlohmann::json from_ros_srv_request(const std::string & srv_type) const;

  /// Generate JSON Schema for a ROS 2 service response type.
  /// @param srv_type Service type string, e.g. "std_srvs/srv/SetBool"
  nlohmann::json from_ros_srv_response(const std::string & srv_type) const;

  // Static SOVD schemas

  /// SOVD GenericError schema (7.4.2)
  static nlohmann::json generic_error();

  /// Fault list item schema (flat format from FaultManager::fault_to_json)
  static nlohmann::json fault_list_item_schema();

  /// Fault detail schema (SOVD nested format from FaultHandlers::build_sovd_fault_response)
  static nlohmann::json fault_detail_schema();

  /// Fault list response schema (items wrapper around fault_list_item_schema)
  static nlohmann::json fault_list_schema();

  /// Single entity detail schema
  static nlohmann::json entity_detail_schema();

  /// Entity list response schema (items wrapper around entity_detail_schema)
  static nlohmann::json entity_list_schema();

  /// Wrap an item schema in a SOVD collection response: {"items": [item_schema]}
  static nlohmann::json items_wrapper(const nlohmann::json & item_schema);

  /// Configuration metadata schema (list endpoint - SOVD ConfigurationMetaData)
  static nlohmann::json configuration_metadata_schema();

  /// Configuration read value schema (GET detail response - SOVD ReadValue)
  static nlohmann::json configuration_read_value_schema();

  /// Configuration write value schema (PUT request body - only data field)
  static nlohmann::json configuration_write_value_schema();

  /// Log entry schema
  static nlohmann::json log_entry_schema();

  /// Health endpoint response schema
  static nlohmann::json health_schema();

  /// Version-info endpoint response schema (SOVD 7.4.1)
  static nlohmann::json version_info_schema();

  /// API root overview response schema (GET /)
  static nlohmann::json root_overview_schema();

  /// Data item in collection list
  static nlohmann::json data_item_schema();

  /// Generic object schema (for dynamic ROS 2 message payloads)
  static nlohmann::json generic_object_schema();

  /// Binary content schema (for file downloads)
  static nlohmann::json binary_schema();

  /// Operation item in collection list
  static nlohmann::json operation_item_schema();

  /// Operation detail schema (wraps OperationItem in {item: ...})
  static nlohmann::json operation_detail_schema();

  /// Operation execution status
  static nlohmann::json operation_execution_schema();

  /// Trigger schema (CRUD responses)
  static nlohmann::json trigger_schema();

  /// Cyclic subscription schema (CRUD responses)
  static nlohmann::json cyclic_subscription_schema();

  /// Lock schema (CRUD responses)
  static nlohmann::json lock_schema();

  /// Lock acquire request schema (POST /locks)
  static nlohmann::json acquire_lock_request_schema();

  /// Lock extend request schema (PUT /locks/{id})
  static nlohmann::json extend_lock_request_schema();

  /// Script metadata schema (list/get)
  static nlohmann::json script_metadata_schema();

  /// Script execution status schema
  static nlohmann::json script_execution_schema();

  /// Bulk-data category list schema (items are bare strings)
  static nlohmann::json bulk_data_category_list_schema();

  /// Bulk-data descriptor schema
  static nlohmann::json bulk_data_descriptor_schema();

  /// Script upload response schema (minimal: id + name)
  static nlohmann::json script_upload_response_schema();

  /// Trigger condition sub-schema (shared by trigger response and create request)
  static nlohmann::json trigger_condition_schema();

  /// Trigger update request schema (only mutable fields)
  static nlohmann::json trigger_update_request_schema();

  /// Trigger create request schema (client-supplied fields only)
  static nlohmann::json trigger_create_request_schema();

  /// Configuration delete-all multi-status response schema (207)
  static nlohmann::json configuration_delete_multi_status_schema();

  /// Cyclic subscription create request schema
  static nlohmann::json cyclic_subscription_create_request_schema();

  /// Software update list schema (items: [string])
  static nlohmann::json update_list_schema();

  /// Software update status schema
  static nlohmann::json update_status_schema();

  /// Log configuration schema (GET/PUT)
  static nlohmann::json log_configuration_schema();

  /// Data write request schema (PUT /data/{id})
  static nlohmann::json data_write_request_schema();

  /// Execution update request schema (PUT /operations/{id}/executions/{id})
  static nlohmann::json execution_update_request_schema();

  /// Script control request schema (PUT /scripts/{id}/executions/{id})
  static nlohmann::json script_control_request_schema();

  /// Auth token response schema
  static nlohmann::json auth_token_response_schema();

  /// Auth credentials request body schema
  static nlohmann::json auth_credentials_schema();

  /// Returns a $ref JSON object pointing to a named component schema.
  static nlohmann::json ref(const std::string & schema_name);

  /// Wraps a $ref in an items collection.
  static nlohmann::json items_wrapper_ref(const std::string & schema_name);

  /// Returns all named schemas for registration in components/schemas.
  /// Key = schema name (becomes the type name in generated clients).
  /// List-type schemas use internal $ref to avoid duplicate types.
  static const std::map<std::string, nlohmann::json> & component_schemas();

 private:
  ros2_medkit_serialization::JsonSerializer serializer_;  // Owns its own instance
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
