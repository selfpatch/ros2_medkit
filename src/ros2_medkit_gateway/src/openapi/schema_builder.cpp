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
                                      {"data", {{"description", "Snapshot data"}}},
                                      {"bulk_data_uri", {{"type", "string"}}},
                                      {"size_bytes", {{"type", "integer"}}},
                                      {"duration_sec", {{"type", "number"}}},
                                      {"format", {{"type", "string"}}},
                                      {"x-medkit", {{"type", "object"}, {"additionalProperties", true}}}}}};

  nlohmann::json env_data_schema = {{"type", "object"},
                                    {"properties",
                                     {{"extended_data_records",
                                       {{"type", "object"},
                                        {"properties",
                                         {{"first_occurrence", {{"type", "string"}, {"format", "date-time"}}},
                                          {"last_occurrence", {{"type", "string"}, {"format", "date-time"}}}}}}},
                                      {"snapshots", {{"type", "array"}, {"items", snapshot_schema}}}}}};

  nlohmann::json x_medkit_schema = {{"type", "object"},
                                    {"additionalProperties", true},
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
  // Aggregation provenance surfaced on every merged entity (x-medkit vendor
  // extension). Present only when non-empty; ordering is stable ("local"
  // first when present, then "peer:<name>" entries alphabetically) so that
  // clients and snapshot tests can rely on it. See design/aggregation.rst.
  nlohmann::json x_medkit_schema = {
      {"type", "object"},
      {"properties",
       {{"contributors",
         {{"type", "array"},
          {"items", {{"type", "string"}}},
          {"description",
           "Aggregation provenance: 'local' and/or 'peer:<name>' entries naming the sources that "
           "contributed to this merged entity. Sorted with 'local' first and 'peer:*' entries "
           "alphabetically. Present only when aggregation is active and the entity has at least "
           "one known source."}}}}},
      {"additionalProperties", true}};

  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"type", {{"type", "string"}}},
            {"uri", {{"type", "string"}}},
            {"x-medkit", x_medkit_schema}}},
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

nlohmann::json SchemaBuilder::configuration_metadata_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}, {"description", "Configuration parameter ID"}}},
            {"name", {{"type", "string"}, {"description", "Parameter name"}}},
            {"type", {{"type", "string"}, {"description", "Parameter type (e.g. 'parameter')"}}}}},
          {"required", {"id", "name", "type"}}};
}

nlohmann::json SchemaBuilder::configuration_read_value_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}, {"description", "Configuration parameter ID"}}},
            {"data", {{"description", "Configuration value (type varies by parameter)"}}}}},
          {"required", {"id", "data"}}};
}

nlohmann::json SchemaBuilder::configuration_write_value_schema() {
  return {{"type", "object"},
          {"properties", {{"data", {{"description", "Configuration value to set (type varies by parameter)"}}}}},
          {"required", {"data"}}};
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
  nlohmann::json linking_schema = {{"type", "object"},
                                   {"properties",
                                    {{"linked_count", {{"type", "integer"}}},
                                     {"orphan_count", {{"type", "integer"}}},
                                     {"binding_conflicts", {{"type", "array"}, {"items", {{"type", "string"}}}}},
                                     {"warnings", {{"type", "array"}, {"items", {{"type", "string"}}}}}}}};

  nlohmann::json discovery_schema = {{"type", "object"},
                                     {"properties",
                                      {{"mode", {{"type", "string"}}},
                                       {"strategy", {{"type", "string"}}},
                                       {"pipeline", {{"type", "object"}}},
                                       {"linking", linking_schema}}},
                                     {"description", "Discovery subsystem status"}};

  nlohmann::json peer_status_schema = {{"type", "object"}, {"additionalProperties", true}};

  nlohmann::json aggregation_warning_schema = {
      {"type", "object"},
      {"description",
       "Operator-actionable aggregation warning. Codes are documented in "
       "docs/api/warning_codes.rst and stable across releases."},
      {"properties",
       {{"code",
         {{"type", "string"}, {"description", "Stable machine-readable identifier, e.g. 'leaf_id_collision'."}}},
        {"message", {{"type", "string"}, {"description", "Human-readable description including remediation hints."}}},
        {"entity_ids",
         {{"type", "array"},
          {"items", {{"type", "string"}}},
          {"description", "SOVD entity IDs affected by the warning."}}},
        {"peer_names",
         {{"type", "array"},
          {"items", {{"type", "string"}}},
          {"description", "Aggregation peers involved in the anomaly."}}}}},
      {"required", {"code", "message", "entity_ids", "peer_names"}}};

  return {
      {"type", "object"},
      {"properties",
       {{"status", {{"type", "string"}}},
        {"timestamp", {{"type", "integer"}}},
        {"discovery", discovery_schema},
        {"peers",
         {{"type", "array"},
          {"items", peer_status_schema},
          {"description", "Aggregation peer status (x-medkit extension; present only when aggregation is enabled)."}}},
        {"warnings",
         {{"type", "array"},
          {"items", aggregation_warning_schema},
          {"description",
           "Operator-actionable aggregation warnings (x-medkit extension; always an array when "
           "aggregation is enabled, empty when there are no active warnings)."}}}}},
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

nlohmann::json SchemaBuilder::root_overview_schema() {
  nlohmann::json capabilities_schema = {{"type", "object"},
                                        {"properties",
                                         {{"discovery", {{"type", "boolean"}}},
                                          {"data_access", {{"type", "boolean"}}},
                                          {"operations", {{"type", "boolean"}}},
                                          {"async_actions", {{"type", "boolean"}}},
                                          {"configurations", {{"type", "boolean"}}},
                                          {"faults", {{"type", "boolean"}}},
                                          {"logs", {{"type", "boolean"}}},
                                          {"bulk_data", {{"type", "boolean"}}},
                                          {"cyclic_subscriptions", {{"type", "boolean"}}},
                                          {"locking", {{"type", "boolean"}}},
                                          {"triggers", {{"type", "boolean"}}},
                                          {"updates", {{"type", "boolean"}}},
                                          {"authentication", {{"type", "boolean"}}},
                                          {"tls", {{"type", "boolean"}}},
                                          {"scripts", {{"type", "boolean"}}},
                                          {"vendor_extensions", {{"type", "boolean"}}}}}};

  nlohmann::json auth_schema = {{"type", "object"},
                                {"properties",
                                 {{"enabled", {{"type", "boolean"}}},
                                  {"algorithm", {{"type", "string"}}},
                                  {"require_auth_for", {{"type", "string"}}}}}};

  nlohmann::json tls_schema = {
      {"type", "object"}, {"properties", {{"enabled", {{"type", "boolean"}}}, {"min_version", {{"type", "string"}}}}}};

  return {{"type", "object"},
          {"properties",
           {{"name", {{"type", "string"}}},
            {"version", {{"type", "string"}}},
            {"api_base", {{"type", "string"}}},
            {"endpoints", {{"type", "array"}, {"items", {{"type", "string"}}}}},
            {"capabilities", capabilities_schema},
            {"auth", auth_schema},
            {"tls", tls_schema}}},
          {"required", {"name", "version", "api_base", "endpoints", "capabilities"}}};
}

nlohmann::json SchemaBuilder::data_item_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"category", {{"type", "string"}}},
            {"x-medkit", {{"type", "object"}, {"additionalProperties", true}}}}},
          {"required", {"id", "name"}}};
}

nlohmann::json SchemaBuilder::generic_object_schema() {
  return {{"type", "object"}};
}

nlohmann::json SchemaBuilder::binary_schema() {
  return {{"type", "string"}, {"format", "binary"}};
}

nlohmann::json SchemaBuilder::operation_item_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"proximity_proof_required", {{"type", "boolean"}, {"description", "Whether proximity proof is needed"}}},
            {"asynchronous_execution", {{"type", "boolean"}, {"description", "Whether operation runs asynchronously"}}},
            {"x-medkit", {{"type", "object"}, {"additionalProperties", true}}}}},
          {"required", {"id", "name"}}};
}

nlohmann::json SchemaBuilder::operation_detail_schema() {
  return {{"type", "object"}, {"properties", {{"item", ref("OperationItem")}}}, {"required", {"item"}}};
}

nlohmann::json SchemaBuilder::operation_execution_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"status", {{"type", "string"}, {"enum", {"pending", "running", "completed", "failed"}}}},
            {"progress", {{"type", "number"}}},
            {"result", {{"type", "object"}}}}},
          {"required", {"id", "status"}}};
}

nlohmann::json SchemaBuilder::trigger_condition_schema() {
  return {{"type", "object"},
          {"properties", {{"condition_type", {{"type", "string"}}}}},
          {"required", {"condition_type"}},
          {"additionalProperties", true}};
}

nlohmann::json SchemaBuilder::trigger_schema() {
  auto condition_schema = trigger_condition_schema();

  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"status", {{"type", "string"}, {"enum", {"active", "terminated"}}}},
            {"observed_resource", {{"type", "string"}, {"description", "Resource URI being observed"}}},
            {"event_source", {{"type", "string"}, {"description", "Server-generated event source URI"}}},
            {"protocol", {{"type", "string"}, {"description", "Transport protocol"}}},
            {"trigger_condition", condition_schema},
            {"multishot", {{"type", "boolean"}, {"description", "Whether trigger fires multiple times"}}},
            {"persistent", {{"type", "boolean"}, {"description", "Whether trigger survives server restarts"}}},
            {"lifetime", {{"type", "number"}, {"description", "Trigger lifetime in seconds"}}},
            {"path", {{"type", "string"}, {"description", "Notification delivery path"}}},
            {"log_settings", {{"type", "object"}}}}},
          {"required", {"id", "status", "observed_resource", "event_source", "protocol", "trigger_condition"}}};
}

nlohmann::json SchemaBuilder::cyclic_subscription_schema() {
  return {
      {"type", "object"},
      {"properties",
       {{"id", {{"type", "string"}}},
        {"observed_resource", {{"type", "string"}, {"description", "Resource URI being observed"}}},
        {"event_source", {{"type", "string"}, {"description", "Server-generated event source URI"}}},
        {"protocol", {{"type", "string"}, {"description", "Transport protocol"}}},
        {"interval", {{"type", "string"}, {"enum", {"fast", "normal", "slow"}}, {"description", "Polling interval"}}}}},
      {"required", {"id", "observed_resource", "event_source", "protocol", "interval"}}};
}

nlohmann::json SchemaBuilder::lock_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"owned", {{"type", "boolean"}}},
            {"scopes", {{"type", "array"}, {"items", {{"type", "string"}}}}},
            {"lock_expiration", {{"type", "string"}, {"format", "date-time"}}}}},
          {"required", {"id", "owned", "lock_expiration"}}};
}

nlohmann::json SchemaBuilder::acquire_lock_request_schema() {
  return {{"type", "object"},
          {"properties",
           {{"lock_expiration",
             {{"type", "integer"}, {"minimum", 1}, {"example", 300}, {"description", "Lock duration in seconds"}}},
            {"scopes",
             {{"type", "array"},
              {"items", {{"type", "string"}}},
              {"description", "Lock scopes (e.g. 'configurations', 'operations')"}}},
            {"break_lock",
             {{"type", "boolean"}, {"description", "Force-acquire by breaking an existing lock (default: false)"}}}}},
          {"required", {"lock_expiration"}}};
}

nlohmann::json SchemaBuilder::extend_lock_request_schema() {
  return {{"type", "object"},
          {"properties",
           {{"lock_expiration",
             {{"type", "integer"},
              {"minimum", 1},
              {"example", 120},
              {"description", "Additional seconds to extend the lock"}}}}},
          {"required", {"lock_expiration"}}};
}

nlohmann::json SchemaBuilder::script_metadata_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"description", {{"type", "string"}}},
            {"href", {{"type", "string"}}},
            {"managed", {{"type", "boolean"}}},
            {"proximity_proof_required", {{"type", "boolean"}}},
            {"parameters_schema", {{"type", {"object", "null"}}}}}},
          {"required", {"id", "name"}}};
}

nlohmann::json SchemaBuilder::script_execution_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"status", {{"type", "string"}}},
            {"progress", {{"type", "number"}}},
            {"started_at", {{"type", "string"}}},
            {"completed_at", {{"type", "string"}}},
            {"parameters", {{"type", "object"}}},
            {"error", {{"type", "object"}}}}},
          {"required", {"id", "status"}}};
}

nlohmann::json SchemaBuilder::script_upload_response_schema() {
  return {{"type", "object"},
          {"properties", {{"id", {{"type", "string"}}}, {"name", {{"type", "string"}}}}},
          {"required", {"id", "name"}}};
}

nlohmann::json SchemaBuilder::trigger_update_request_schema() {
  return {
      {"type", "object"},
      {"properties", {{"lifetime", {{"type", "integer"}, {"minimum", 1}, {"description", "New lifetime in seconds"}}}}},
      {"required", {"lifetime"}}};
}

nlohmann::json SchemaBuilder::trigger_create_request_schema() {
  auto condition_schema = trigger_condition_schema();

  return {{"type", "object"},
          {"properties",
           {{"resource", {{"type", "string"}, {"description", "Resource URI to observe"}}},
            {"trigger_condition", condition_schema},
            {"protocol", {{"type", "string"}, {"description", "Transport protocol (default: sse)"}}},
            {"multishot", {{"type", "boolean"}}},
            {"persistent", {{"type", "boolean"}}},
            {"lifetime", {{"type", "integer"}, {"minimum", 1}}},
            {"path", {{"type", "string"}}},
            {"log_settings", {{"type", "object"}}}}},
          {"required", {"resource", "trigger_condition"}}};
}

nlohmann::json SchemaBuilder::configuration_delete_multi_status_schema() {
  nlohmann::json result_entry = {{"type", "object"},
                                 {"properties",
                                  {{"node", {{"type", "string"}}},
                                   {"app_id", {{"type", "string"}}},
                                   {"success", {{"type", "boolean"}}},
                                   {"error", {{"type", "string"}}}}}};

  return {
      {"type", "object"},
      {"properties", {{"entity_id", {{"type", "string"}}}, {"results", {{"type", "array"}, {"items", result_entry}}}}},
      {"required", {"entity_id", "results"}}};
}

nlohmann::json SchemaBuilder::cyclic_subscription_create_request_schema() {
  return {{"type", "object"},
          {"properties",
           {{"resource", {{"type", "string"}, {"description", "Resource URI to subscribe to"}}},
            {"interval", {{"type", "string"}, {"enum", {"fast", "normal", "slow"}}}},
            {"duration", {{"type", "integer"}, {"minimum", 1}, {"description", "Subscription duration in seconds"}}},
            {"protocol", {{"type", "string"}, {"description", "Transport protocol (default: sse)"}}}}},
          {"required", {"resource", "interval", "duration"}}};
}

nlohmann::json SchemaBuilder::bulk_data_category_list_schema() {
  return items_wrapper({{"type", "string"}});
}

nlohmann::json SchemaBuilder::bulk_data_descriptor_schema() {
  return {{"type", "object"},
          {"properties",
           {{"id", {{"type", "string"}}},
            {"name", {{"type", "string"}}},
            {"size", {{"type", "integer"}}},
            {"mimetype", {{"type", "string"}, {"description", "MIME type of the file"}}},
            {"creation_date",
             {{"type", "string"}, {"format", "date-time"}, {"description", "ISO 8601 creation timestamp"}}},
            {"description", {{"type", "string"}, {"description", "Human-readable description"}}},
            {"x-medkit", {{"type", "object"}, {"additionalProperties", true}}}}},
          {"required", {"id", "name"}}};
}

nlohmann::json SchemaBuilder::update_list_schema() {
  return items_wrapper({{"type", "string"}});
}

nlohmann::json SchemaBuilder::update_status_schema() {
  nlohmann::json sub_progress_schema = {
      {"type", "object"},
      {"properties", {{"name", {{"type", "string"}}}, {"progress", {{"type", "number"}}}}},
      {"required", {"name", "progress"}}};

  return {{"type", "object"},
          {"properties",
           {{"status", {{"type", "string"}, {"enum", {"pending", "inProgress", "completed", "failed"}}}},
            {"progress", {{"type", "number"}}},
            {"sub_progress", {{"type", "array"}, {"items", sub_progress_schema}}},
            {"error", {{"type", "string"}}}}},
          {"required", {"status"}}};
}

nlohmann::json SchemaBuilder::log_configuration_schema() {
  return {{"type", "object"},
          {"properties",
           {{"severity_filter", {{"type", "string"}, {"enum", {"debug", "info", "warning", "error", "fatal"}}}},
            {"max_entries", {{"type", "integer"}, {"minimum", 1}, {"maximum", 10000}}}}}};
}

nlohmann::json SchemaBuilder::data_write_request_schema() {
  return {{"type", "object"},
          {"properties",
           {{"type", {{"type", "string"}, {"description", "ROS 2 message type (e.g. 'std_msgs/msg/Float32')"}}},
            {"data", {{"description", "Message value to publish"}}}}},
          {"required", {"type", "data"}}};
}

nlohmann::json SchemaBuilder::execution_update_request_schema() {
  return {{"type", "object"},
          {"properties",
           {{"capability",
             {{"type", "string"},
              {"enum", {"stop", "execute", "freeze", "reset"}},
              {"description", "Control command for the running execution"}}}}},
          {"required", {"capability"}}};
}

nlohmann::json SchemaBuilder::script_control_request_schema() {
  return {{"type", "object"},
          {"properties",
           {{"action",
             {{"type", "string"},
              {"enum", {"stop", "forced_termination"}},
              {"description", "Control action for the running script execution"}}}}},
          {"required", {"action"}}};
}

nlohmann::json SchemaBuilder::auth_token_response_schema() {
  return {{"type", "object"},
          {"properties",
           {{"access_token", {{"type", "string"}}},
            {"token_type", {{"type", "string"}}},
            {"expires_in", {{"type", "integer"}}},
            {"scope", {{"type", "string"}}},
            {"refresh_token", {{"type", "string"}}}}},
          {"required", {"access_token", "token_type", "expires_in"}}};
}

nlohmann::json SchemaBuilder::auth_credentials_schema() {
  return {{"type", "object"},
          {"properties", {{"username", {{"type", "string"}}}, {"password", {{"type", "string"}}}}},
          {"required", {"username", "password"}}};
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
  static const std::map<std::string, nlohmann::json> schemas = {
      // Core types
      {"GenericError", generic_error()},
      {"EntityDetail", entity_detail_schema()},
      {"EntityList", items_wrapper_ref("EntityDetail")},
      // Faults
      {"FaultListItem", fault_list_item_schema()},
      {"FaultDetail", fault_detail_schema()},
      {"FaultList", items_wrapper_ref("FaultListItem")},
      // Configuration
      {"ConfigurationMetaData", configuration_metadata_schema()},
      {"ConfigurationMetaDataList", items_wrapper_ref("ConfigurationMetaData")},
      {"ConfigurationReadValue", configuration_read_value_schema()},
      {"ConfigurationWriteValue", configuration_write_value_schema()},
      {"ConfigurationDeleteMultiStatus", configuration_delete_multi_status_schema()},
      // Logs
      {"LogEntry", log_entry_schema()},
      {"LogEntryList", items_wrapper_ref("LogEntry")},
      {"LogConfiguration", log_configuration_schema()},
      // Server
      {"HealthStatus", health_schema()},
      {"VersionInfo", version_info_schema()},
      {"RootOverview", root_overview_schema()},
      // Data
      {"DataItem", data_item_schema()},
      {"DataItemList", items_wrapper_ref("DataItem")},
      {"DataWriteRequest", data_write_request_schema()},
      // Operations
      {"OperationItem", operation_item_schema()},
      {"OperationItemList", items_wrapper_ref("OperationItem")},
      {"OperationDetail", operation_detail_schema()},
      {"OperationExecution", operation_execution_schema()},
      {"OperationExecutionList", items_wrapper_ref("OperationExecution")},
      {"ExecutionUpdateRequest", execution_update_request_schema()},
      // Triggers
      {"Trigger", trigger_schema()},
      {"TriggerList", items_wrapper_ref("Trigger")},
      {"TriggerUpdateRequest", trigger_update_request_schema()},
      {"TriggerCreateRequest", trigger_create_request_schema()},
      // Subscriptions
      {"CyclicSubscription", cyclic_subscription_schema()},
      {"CyclicSubscriptionList", items_wrapper_ref("CyclicSubscription")},
      {"CyclicSubscriptionCreateRequest", cyclic_subscription_create_request_schema()},
      // Locking
      {"Lock", lock_schema()},
      {"LockList", items_wrapper_ref("Lock")},
      {"AcquireLockRequest", acquire_lock_request_schema()},
      {"ExtendLockRequest", extend_lock_request_schema()},
      // Scripts
      {"ScriptMetadata", script_metadata_schema()},
      {"ScriptMetadataList", items_wrapper_ref("ScriptMetadata")},
      {"ScriptUploadResponse", script_upload_response_schema()},
      {"ScriptExecution", script_execution_schema()},
      {"ScriptControlRequest", script_control_request_schema()},
      // Bulk Data
      {"BulkDataCategoryList", bulk_data_category_list_schema()},
      {"BulkDataDescriptor", bulk_data_descriptor_schema()},
      {"BulkDataDescriptorList", items_wrapper_ref("BulkDataDescriptor")},
      // Updates
      {"UpdateList", update_list_schema()},
      {"UpdateStatus", update_status_schema()},
      // Auth
      {"AuthTokenResponse", auth_token_response_schema()},
      {"AuthCredentials", auth_credentials_schema()},
  };
  return schemas;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
