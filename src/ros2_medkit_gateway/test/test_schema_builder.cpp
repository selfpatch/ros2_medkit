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

#include <gtest/gtest.h>

#include <regex>
#include <set>
#include <string>
#include <vector>

#include "../src/openapi/schema_builder.hpp"

using ros2_medkit_gateway::openapi::SchemaBuilder;

// =============================================================================
// Static schema tests - no ROS 2 type libraries needed
// =============================================================================

TEST(SchemaBuilderStaticTest, GenericErrorSchema) {
  auto schema = SchemaBuilder::generic_error();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("error_code"));
  EXPECT_TRUE(schema["properties"].contains("message"));
  EXPECT_TRUE(schema["properties"].contains("parameters"));
  EXPECT_EQ(schema["properties"]["error_code"]["type"], "string");
  EXPECT_EQ(schema["properties"]["message"]["type"], "string");
  EXPECT_EQ(schema["properties"]["parameters"]["type"], "object");

  // Required fields
  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "error_code"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "message"), required.end());
}

TEST(SchemaBuilderStaticTest, FaultListItemSchema) {
  auto schema = SchemaBuilder::fault_list_item_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("fault_code"));
  EXPECT_TRUE(schema["properties"].contains("severity"));
  EXPECT_TRUE(schema["properties"].contains("severity_label"));
  EXPECT_TRUE(schema["properties"].contains("description"));
  EXPECT_TRUE(schema["properties"].contains("first_occurred"));
  EXPECT_TRUE(schema["properties"].contains("last_occurred"));
  EXPECT_TRUE(schema["properties"].contains("occurrence_count"));
  EXPECT_TRUE(schema["properties"].contains("status"));
  EXPECT_TRUE(schema["properties"].contains("reporting_sources"));
  EXPECT_EQ(schema["properties"]["fault_code"]["type"], "string");
  EXPECT_EQ(schema["properties"]["severity"]["type"], "integer");
  EXPECT_EQ(schema["properties"]["status"]["type"], "string");
  EXPECT_EQ(schema["properties"]["reporting_sources"]["type"], "array");
}

TEST(SchemaBuilderStaticTest, FaultDetailSchema) {
  auto schema = SchemaBuilder::fault_detail_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  // SOVD nested structure
  EXPECT_TRUE(schema["properties"].contains("item"));
  EXPECT_TRUE(schema["properties"].contains("environment_data"));
  EXPECT_TRUE(schema["properties"].contains("x-medkit"));

  // item subfields
  auto & item = schema["properties"]["item"];
  EXPECT_EQ(item["type"], "object");
  EXPECT_TRUE(item["properties"].contains("code"));
  EXPECT_TRUE(item["properties"].contains("fault_name"));
  EXPECT_TRUE(item["properties"].contains("severity"));
  EXPECT_TRUE(item["properties"].contains("status"));
  EXPECT_EQ(item["properties"]["status"]["type"], "object");
  EXPECT_TRUE(item["properties"]["status"]["properties"].contains("aggregatedStatus"));

  // environment_data subfields
  auto & env = schema["properties"]["environment_data"];
  EXPECT_EQ(env["type"], "object");
  EXPECT_TRUE(env["properties"].contains("extended_data_records"));
  EXPECT_TRUE(env["properties"].contains("snapshots"));

  // x-medkit subfields
  auto & xmedkit = schema["properties"]["x-medkit"];
  EXPECT_EQ(xmedkit["type"], "object");
  EXPECT_TRUE(xmedkit["properties"].contains("occurrence_count"));
  EXPECT_TRUE(xmedkit["properties"].contains("reporting_sources"));
  EXPECT_TRUE(xmedkit["properties"].contains("severity_label"));
  EXPECT_TRUE(xmedkit["properties"].contains("status_raw"));
}

TEST(SchemaBuilderStaticTest, FaultListSchema) {
  auto schema = SchemaBuilder::fault_list_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");

  // Items should contain the fault list item schema
  auto & item_schema = schema["properties"]["items"]["items"];
  EXPECT_EQ(item_schema["type"], "object");
  EXPECT_TRUE(item_schema["properties"].contains("fault_code"));
}

TEST(SchemaBuilderStaticTest, EntityDetailSchema) {
  auto schema = SchemaBuilder::entity_detail_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("id"));
  EXPECT_TRUE(schema["properties"].contains("name"));
  EXPECT_TRUE(schema["properties"].contains("type"));
  EXPECT_TRUE(schema["properties"].contains("uri"));
  EXPECT_EQ(schema["properties"]["id"]["type"], "string");
  EXPECT_EQ(schema["properties"]["name"]["type"], "string");
}

TEST(SchemaBuilderStaticTest, EntityListSchema) {
  auto schema = SchemaBuilder::entity_list_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");

  // Items should contain the entity detail schema
  auto & item_schema = schema["properties"]["items"]["items"];
  EXPECT_EQ(item_schema["type"], "object");
  EXPECT_TRUE(item_schema["properties"].contains("id"));
  EXPECT_TRUE(item_schema["properties"].contains("name"));
}

TEST(SchemaBuilderStaticTest, ItemsWrapper) {
  nlohmann::json inner = {{"type", "string"}};
  auto schema = SchemaBuilder::items_wrapper(inner);
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");
  EXPECT_EQ(schema["properties"]["items"]["items"]["type"], "string");

  // Required
  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "items"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ConfigurationMetaDataSchema) {
  auto schema = SchemaBuilder::configuration_metadata_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("id"));
  EXPECT_TRUE(schema["properties"].contains("name"));
  EXPECT_TRUE(schema["properties"].contains("type"));
  EXPECT_FALSE(schema["properties"].contains("value"));
  EXPECT_EQ(schema["properties"]["id"]["type"], "string");
  EXPECT_EQ(schema["properties"]["name"]["type"], "string");
  EXPECT_EQ(schema["properties"]["type"]["type"], "string");

  // Required: id, name, type (no value)
  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "id"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "name"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "type"), required.end());
  EXPECT_EQ(std::find(required.begin(), required.end(), "value"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ConfigurationReadValueSchema) {
  auto schema = SchemaBuilder::configuration_read_value_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("id"));
  EXPECT_TRUE(schema["properties"].contains("data"));
  EXPECT_FALSE(schema["properties"].contains("name"));
  EXPECT_FALSE(schema["properties"].contains("value"));
  EXPECT_EQ(schema["properties"]["id"]["type"], "string");

  // Required: id, data
  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "id"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "data"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, OperationDetailSchema) {
  auto schema = SchemaBuilder::operation_detail_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("item"));
  // item references OperationItem via $ref
  EXPECT_TRUE(schema["properties"]["item"].contains("$ref"));

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "item"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ConfigurationWriteValueSchema) {
  auto schema = SchemaBuilder::configuration_write_value_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("data"));
  EXPECT_FALSE(schema["properties"].contains("id"));

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "data"), required.end());
  EXPECT_EQ(std::find(required.begin(), required.end(), "id"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ScriptUploadResponseSchema) {
  auto schema = SchemaBuilder::script_upload_response_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("id"));
  EXPECT_TRUE(schema["properties"].contains("name"));
  EXPECT_EQ(schema["properties"]["id"]["type"], "string");
  EXPECT_EQ(schema["properties"]["name"]["type"], "string");

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "id"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "name"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, TriggerUpdateRequestSchema) {
  auto schema = SchemaBuilder::trigger_update_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("lifetime"));
  EXPECT_EQ(schema["properties"]["lifetime"]["type"], "integer");

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "lifetime"), required.end());

  EXPECT_EQ(schema["properties"]["lifetime"]["minimum"], 1);
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, BulkDataCategoryListSchema) {
  auto schema = SchemaBuilder::bulk_data_category_list_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");
  EXPECT_EQ(schema["properties"]["items"]["items"]["type"], "string");
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, BulkDataDescriptorSchema) {
  auto schema = SchemaBuilder::bulk_data_descriptor_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("id"));
  EXPECT_TRUE(schema["properties"].contains("name"));
  EXPECT_TRUE(schema["properties"].contains("mimetype"));
  EXPECT_TRUE(schema["properties"].contains("creation_date"));
  EXPECT_TRUE(schema["properties"].contains("description"));
  EXPECT_TRUE(schema["properties"].contains("x-medkit"));
  EXPECT_FALSE(schema["properties"].contains("content_type"));
  EXPECT_FALSE(schema["properties"].contains("created_at"));

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "id"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "name"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, CyclicSubscriptionCreateRequestSchema) {
  auto schema = SchemaBuilder::cyclic_subscription_create_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("resource"));
  EXPECT_TRUE(schema["properties"].contains("interval"));
  EXPECT_TRUE(schema["properties"].contains("duration"));
  EXPECT_TRUE(schema["properties"].contains("protocol"));
  EXPECT_FALSE(schema["properties"].contains("id"));
  EXPECT_FALSE(schema["properties"].contains("event_source"));

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "resource"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "interval"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "duration"), required.end());
  EXPECT_EQ(std::find(required.begin(), required.end(), "id"), required.end());

  // Verify interval enum constraint
  EXPECT_EQ(schema["properties"]["interval"]["type"], "string");
  ASSERT_TRUE(schema["properties"]["interval"].contains("enum"));
  auto enum_vals = schema["properties"]["interval"]["enum"].get<std::vector<std::string>>();
  EXPECT_EQ(enum_vals.size(), 3u);

  // Verify duration type and minimum
  EXPECT_EQ(schema["properties"]["duration"]["type"], "integer");
  EXPECT_EQ(schema["properties"]["duration"]["minimum"], 1);
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, TriggerCreateRequestSchema) {
  auto schema = SchemaBuilder::trigger_create_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("resource"));
  EXPECT_TRUE(schema["properties"].contains("trigger_condition"));
  EXPECT_FALSE(schema["properties"].contains("id"));
  EXPECT_FALSE(schema["properties"].contains("status"));
  EXPECT_FALSE(schema["properties"].contains("event_source"));

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "resource"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "trigger_condition"), required.end());
  EXPECT_EQ(std::find(required.begin(), required.end(), "id"), required.end());
}

TEST(SchemaBuilderStaticTest, LogEntrySchema) {
  auto schema = SchemaBuilder::log_entry_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("id"));
  EXPECT_TRUE(schema["properties"].contains("timestamp"));
  EXPECT_TRUE(schema["properties"].contains("severity"));
  EXPECT_TRUE(schema["properties"].contains("message"));
  EXPECT_TRUE(schema["properties"].contains("context"));
  EXPECT_EQ(schema["properties"]["id"]["type"], "string");
  EXPECT_EQ(schema["properties"]["severity"]["type"], "string");
  EXPECT_EQ(schema["properties"]["context"]["type"], "object");
  EXPECT_TRUE(schema["properties"]["context"]["properties"].contains("node"));
}

TEST(SchemaBuilderStaticTest, HealthSchema) {
  auto schema = SchemaBuilder::health_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("status"));
  EXPECT_TRUE(schema["properties"].contains("timestamp"));
  EXPECT_TRUE(schema["properties"].contains("discovery"));
  EXPECT_EQ(schema["properties"]["status"]["type"], "string");
  EXPECT_EQ(schema["properties"]["timestamp"]["type"], "integer");

  // Discovery subfields
  auto & discovery = schema["properties"]["discovery"];
  EXPECT_EQ(discovery["type"], "object");
  EXPECT_TRUE(discovery["properties"].contains("mode"));
  EXPECT_TRUE(discovery["properties"].contains("strategy"));
  EXPECT_EQ(discovery["properties"]["mode"]["type"], "string");
  EXPECT_EQ(discovery["properties"]["strategy"]["type"], "string");

  // Required
  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "status"), required.end());
}

TEST(SchemaBuilderStaticTest, VersionInfoSchema) {
  auto schema = SchemaBuilder::version_info_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");

  // Items should have version, base_uri, and vendor_info
  auto & item_schema = schema["properties"]["items"]["items"];
  EXPECT_EQ(item_schema["type"], "object");
  EXPECT_TRUE(item_schema["properties"].contains("version"));
  EXPECT_TRUE(item_schema["properties"].contains("base_uri"));
  EXPECT_TRUE(item_schema["properties"].contains("vendor_info"));

  // vendor_info should have version and name
  auto & vendor_schema = item_schema["properties"]["vendor_info"];
  EXPECT_EQ(vendor_schema["type"], "object");
  EXPECT_TRUE(vendor_schema["properties"].contains("version"));
  EXPECT_TRUE(vendor_schema["properties"].contains("name"));
}

// =============================================================================
// Runtime schema tests - require ROS 2 type libraries (ament_index)
// =============================================================================

TEST(SchemaBuilderRuntimeTest, FromRosMsgKnownType) {
  // @verifies REQ_INTEROP_002
  SchemaBuilder builder;
  auto schema = builder.from_ros_msg("std_msgs/msg/String");

  // Should return valid JSON Schema with type: object and properties.data
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("data"));
}

TEST(SchemaBuilderRuntimeTest, FromRosMsgUnknownType) {
  // @verifies REQ_INTEROP_002
  SchemaBuilder builder;
  auto schema = builder.from_ros_msg("nonexistent/msg/Type");

  // Should gracefully degrade to generic object
  EXPECT_EQ(schema["type"], "object");
  EXPECT_TRUE(schema.contains("x-medkit-schema-unavailable"));
  EXPECT_TRUE(schema["x-medkit-schema-unavailable"].get<bool>());
}

TEST(SchemaBuilderRuntimeTest, FromRosSrvRequest) {
  SchemaBuilder builder;
  auto schema = builder.from_ros_srv_request("std_srvs/srv/SetBool");

  // SetBool_Request has a "data" field (bool)
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("data"));
}

TEST(SchemaBuilderRuntimeTest, FromRosSrvResponse) {
  SchemaBuilder builder;
  auto schema = builder.from_ros_srv_response("std_srvs/srv/SetBool");

  // SetBool_Response has "success" and "message" fields
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("success"));
  EXPECT_TRUE(schema["properties"].contains("message"));
}

TEST(SchemaBuilderRuntimeTest, FromRosSrvRequestUnknown) {
  SchemaBuilder builder;
  auto schema = builder.from_ros_srv_request("nonexistent/srv/Foo");

  // Should gracefully degrade
  EXPECT_EQ(schema["type"], "object");
  EXPECT_TRUE(schema.contains("x-medkit-schema-unavailable"));
  EXPECT_TRUE(schema["x-medkit-schema-unavailable"].get<bool>());
}

TEST(SchemaBuilderRuntimeTest, FromRosSrvResponseUnknown) {
  SchemaBuilder builder;
  auto schema = builder.from_ros_srv_response("nonexistent/srv/Foo");

  // Should gracefully degrade
  EXPECT_EQ(schema["type"], "object");
  EXPECT_TRUE(schema.contains("x-medkit-schema-unavailable"));
  EXPECT_TRUE(schema["x-medkit-schema-unavailable"].get<bool>());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, AcquireLockRequestSchema) {
  auto schema = SchemaBuilder::acquire_lock_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("lock_expiration"));
  EXPECT_TRUE(schema["properties"].contains("scopes"));
  EXPECT_TRUE(schema["properties"].contains("break_lock"));
  EXPECT_EQ(schema["properties"]["lock_expiration"]["type"], "integer");
  EXPECT_EQ(schema["properties"]["lock_expiration"]["minimum"], 1);
  EXPECT_EQ(schema["properties"]["scopes"]["type"], "array");
  EXPECT_EQ(schema["properties"]["break_lock"]["type"], "boolean");

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "lock_expiration"), required.end());
  EXPECT_EQ(required.size(), 1u);
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ExtendLockRequestSchema) {
  auto schema = SchemaBuilder::extend_lock_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("lock_expiration"));
  EXPECT_EQ(schema["properties"]["lock_expiration"]["type"], "integer");
  EXPECT_EQ(schema["properties"]["lock_expiration"]["minimum"], 1);

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "lock_expiration"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, DataWriteRequestSchema) {
  auto schema = SchemaBuilder::data_write_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("type"));
  EXPECT_TRUE(schema["properties"].contains("data"));
  EXPECT_EQ(schema["properties"]["type"]["type"], "string");

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "type"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "data"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ExecutionUpdateRequestSchema) {
  auto schema = SchemaBuilder::execution_update_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("capability"));
  EXPECT_EQ(schema["properties"]["capability"]["type"], "string");
  ASSERT_TRUE(schema["properties"]["capability"].contains("enum"));
  auto enum_vals = schema["properties"]["capability"]["enum"].get<std::vector<std::string>>();
  EXPECT_EQ(enum_vals.size(), 4u);
  EXPECT_NE(std::find(enum_vals.begin(), enum_vals.end(), "stop"), enum_vals.end());

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "capability"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, ScriptControlRequestSchema) {
  auto schema = SchemaBuilder::script_control_request_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("action"));
  EXPECT_EQ(schema["properties"]["action"]["type"], "string");
  ASSERT_TRUE(schema["properties"]["action"].contains("enum"));
  auto enum_vals = schema["properties"]["action"]["enum"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(enum_vals.begin(), enum_vals.end(), "stop"), enum_vals.end());
  EXPECT_NE(std::find(enum_vals.begin(), enum_vals.end(), "forced_termination"), enum_vals.end());

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "action"), required.end());
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, LogConfigurationSchemaFieldsOptional) {
  auto schema = SchemaBuilder::log_configuration_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("severity_filter"));
  EXPECT_TRUE(schema["properties"].contains("max_entries"));

  // Both fields are optional - no required array
  EXPECT_FALSE(schema.contains("required"));

  // severity_filter has enum constraint
  ASSERT_TRUE(schema["properties"]["severity_filter"].contains("enum"));
  auto enum_vals = schema["properties"]["severity_filter"]["enum"].get<std::vector<std::string>>();
  EXPECT_EQ(enum_vals.size(), 5u);

  // max_entries has bounds
  EXPECT_EQ(schema["properties"]["max_entries"]["minimum"], 1);
  EXPECT_EQ(schema["properties"]["max_entries"]["maximum"], 10000);
}

// @verifies REQ_INTEROP_002
TEST(SchemaBuilderStaticTest, TriggerConditionSchemaShared) {
  auto schema = SchemaBuilder::trigger_condition_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("condition_type"));
  EXPECT_TRUE(schema["additionalProperties"].get<bool>());

  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "condition_type"), required.end());
}

// =============================================================================
// Schema registry consistency tests
// Validates that all $ref references resolve and schemas are internally consistent.
// This catches mismatches between schema definitions and route registrations.
// =============================================================================

namespace {

// Recursively collect all $ref targets from a JSON schema
void collect_refs(const nlohmann::json & schema, std::set<std::string> & refs) {
  if (schema.is_object()) {
    if (schema.contains("$ref")) {
      auto ref_str = schema["$ref"].get<std::string>();
      // Extract schema name from "#/components/schemas/SchemaName"
      std::regex ref_regex(R"(^#/components/schemas/(.+)$)");
      std::smatch match;
      if (std::regex_match(ref_str, match, ref_regex)) {
        refs.insert(match[1].str());
      }
    }
    for (auto & [key, val] : schema.items()) {
      collect_refs(val, refs);
    }
  } else if (schema.is_array()) {
    for (const auto & item : schema) {
      collect_refs(item, refs);
    }
  }
}

}  // namespace

// @verifies REQ_INTEROP_002
TEST(SchemaConsistencyTest, AllRefsResolveToRegisteredSchemas) {
  const auto & schemas = SchemaBuilder::component_schemas();

  // Collect all $ref targets across all schemas
  std::set<std::string> all_refs;
  for (const auto & [name, schema] : schemas) {
    collect_refs(schema, all_refs);
  }

  // Verify every $ref target exists in component_schemas
  for (const auto & ref_name : all_refs) {
    EXPECT_TRUE(schemas.count(ref_name) > 0)
        << "Schema $ref '#/components/schemas/" << ref_name << "' does not resolve to any registered schema";
  }
}

// @verifies REQ_INTEROP_002
TEST(SchemaConsistencyTest, ListSchemasReferenceExistingItemSchemas) {
  const auto & schemas = SchemaBuilder::component_schemas();

  // Every schema named *List should reference an existing item schema via $ref
  for (const auto & [name, schema] : schemas) {
    if (name.size() > 4 && name.substr(name.size() - 4) == "List") {
      SCOPED_TRACE("Checking list schema: " + name);
      ASSERT_TRUE(schema.contains("properties")) << name << " should have properties";
      ASSERT_TRUE(schema["properties"].contains("items")) << name << " should have 'items' property";

      auto items_prop = schema["properties"]["items"];
      ASSERT_TRUE(items_prop.contains("items")) << name << " items property should define array items";

      auto item_schema = items_prop["items"];
      if (item_schema.contains("$ref")) {
        auto ref_str = item_schema["$ref"].get<std::string>();
        std::regex ref_regex(R"(^#/components/schemas/(.+)$)");
        std::smatch match;
        ASSERT_TRUE(std::regex_match(ref_str, match, ref_regex)) << name << " has malformed $ref: " << ref_str;
        EXPECT_TRUE(schemas.count(match[1].str()) > 0) << name << " references non-existent schema: " << match[1].str();
      } else {
        // Inline items (e.g., BulkDataCategoryList with string items) must have a type
        EXPECT_TRUE(item_schema.contains("type")) << name << " has inline items without a type field";
      }
    }
  }
}

// @verifies REQ_INTEROP_002
TEST(SchemaConsistencyTest, RequiredFieldsExistInProperties) {
  const auto & schemas = SchemaBuilder::component_schemas();

  for (const auto & [name, schema] : schemas) {
    if (schema.contains("required") && !schema.contains("properties")) {
      ADD_FAILURE() << "Schema '" << name << "' has 'required' but no 'properties'";
      continue;
    }
    if (!schema.contains("required") || !schema.contains("properties")) {
      continue;
    }
    SCOPED_TRACE("Checking schema: " + name);
    auto required = schema["required"].get<std::vector<std::string>>();
    for (const auto & field : required) {
      EXPECT_TRUE(schema["properties"].contains(field))
          << "Schema '" << name << "' has required field '" << field << "' not present in properties";
    }
  }
}
