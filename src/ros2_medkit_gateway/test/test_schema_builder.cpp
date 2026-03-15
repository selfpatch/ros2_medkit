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

TEST(SchemaBuilderStaticTest, FaultSchema) {
  auto schema = SchemaBuilder::fault_schema();
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

TEST(SchemaBuilderStaticTest, FaultListSchema) {
  auto schema = SchemaBuilder::fault_list_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");

  // Items should contain the fault schema
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

TEST(SchemaBuilderStaticTest, ConfigurationParamSchema) {
  auto schema = SchemaBuilder::configuration_param_schema();
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("name"));
  EXPECT_TRUE(schema["properties"].contains("value"));
  EXPECT_TRUE(schema["properties"].contains("type"));
  EXPECT_EQ(schema["properties"]["name"]["type"], "string");

  // Required
  ASSERT_TRUE(schema.contains("required"));
  auto required = schema["required"].get<std::vector<std::string>>();
  EXPECT_NE(std::find(required.begin(), required.end(), "name"), required.end());
  EXPECT_NE(std::find(required.begin(), required.end(), "value"), required.end());
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
  EXPECT_EQ(schema["properties"]["status"]["type"], "string");
  EXPECT_EQ(schema["properties"]["timestamp"]["type"], "integer");

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
