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

#include "ros2_medkit_gateway/dto/registry.hpp"

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
  return dto::SchemaWriter<dto::GenericError>::schema();
}

nlohmann::json SchemaBuilder::items_wrapper(const nlohmann::json & item_schema) {
  return {{"type", "object"},
          {"properties", {{"items", {{"type", "array"}, {"items", item_schema}}}}},
          {"required", {"items"}}};
}

nlohmann::json SchemaBuilder::generic_object_schema() {
  return {{"type", "object"}};
}

nlohmann::json SchemaBuilder::binary_schema() {
  return {{"type", "string"}, {"format", "binary"}};
}

nlohmann::json SchemaBuilder::ref(const std::string & schema_name) {
  return {{"$ref", "#/components/schemas/" + schema_name}};
}

nlohmann::json SchemaBuilder::items_wrapper_ref(const std::string & schema_name) {
  return {{"type", "object"},
          {"properties", {{"items", {{"type", "array"}, {"items", ref(schema_name)}}}}},
          {"required", {"items"}}};
}

const std::map<std::string, nlohmann::json> & SchemaBuilder::component_schemas() {
  static const std::map<std::string, nlohmann::json> schemas = []() {
    std::map<std::string, nlohmann::json> m = {
        // Core types
        {"GenericError", generic_error()},
        // Logs - LogEntry, LogEntryList, LogConfiguration, LogContext, LogListXMedkit
        // now come from DTO (dto/logs.hpp).
        // Health / Root - HealthStatus, VersionInfo, RootOverview and sub-DTOs
        // now come from DTO (dto/health.hpp).
        // Operations - OperationItem, OperationDetail, OperationExecution,
        // ExecutionUpdateRequest now come from DTO (dto/operations.hpp).
        // OperationExecutionList is kept here as a thin wrapper over the DTO type.
        {"OperationExecutionList", items_wrapper_ref("OperationExecution")},
        // Triggers - Trigger, TriggerList, TriggerCreateRequest, TriggerUpdateRequest
        // now come from DTO (dto/triggers.hpp).
        // Subscriptions - CyclicSubscription, CyclicSubscriptionList, CyclicSubscriptionCreateRequest,
        // CyclicSubscriptionUpdateRequest now come from DTO (dto/cyclic_subscriptions.hpp).
        // Locking - Lock, LockList, AcquireLockRequest, ExtendLockRequest now come from DTO (dto/locks.hpp).
        // Scripts - ScriptMetadata, ScriptMetadataList, ScriptExecution, ScriptUploadResponse,
        // ScriptControlRequest now come from DTO (dto/scripts.hpp).
        // Bulk Data - BulkDataCategoryList, BulkDataDescriptor, BulkDataDescriptorList
        // now come from DTO (dto/bulkdata.hpp).
        // Updates - UpdateList, UpdateSubProgress, XMedkitUpdate, UpdateStatus
        // now come from DTO (dto/updates.hpp).
        // Auth - AuthCredentials, AuthTokenResponse now come from DTO (dto/auth.hpp).
    };
    // DTO-contract schemas, merged on top of the hand-written factories. The DTO
    // version wins on a name collision (currently only "GenericError"). Each
    // domain migration task removes its now-redundant factory call later.
    auto dto_schemas = dto::collect_component_schemas();
    for (auto & [name, schema] : dto_schemas.items()) {
      m[name] = schema;
    }
    return m;
  }();
  return schemas;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
