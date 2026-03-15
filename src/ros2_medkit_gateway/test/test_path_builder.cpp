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

#include <string>
#include <vector>

#include "../src/openapi/path_builder.hpp"
#include "../src/openapi/schema_builder.hpp"

using ros2_medkit_gateway::ActionInfo;
using ros2_medkit_gateway::AggregatedOperations;
using ros2_medkit_gateway::ServiceInfo;
using ros2_medkit_gateway::TopicData;
using ros2_medkit_gateway::openapi::PathBuilder;
using ros2_medkit_gateway::openapi::SchemaBuilder;

class PathBuilderTest : public ::testing::Test {
 protected:
  SchemaBuilder schema_builder_;
  PathBuilder path_builder_{schema_builder_};
};

// =============================================================================
// Entity collection tests
// =============================================================================

TEST_F(PathBuilderTest, EntityCollectionHasGet) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_entity_collection("apps");
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"].contains("summary"));
  EXPECT_TRUE(result["get"].contains("responses"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, EntityCollectionHasItemsSchema) {
  auto result = path_builder_.build_entity_collection("components");
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  EXPECT_EQ(schema["properties"]["items"]["type"], "array");
}

TEST_F(PathBuilderTest, EntityCollectionHasQueryParams) {
  auto result = path_builder_.build_entity_collection("areas");
  ASSERT_TRUE(result["get"].contains("parameters"));
  auto & params = result["get"]["parameters"];
  ASSERT_GE(params.size(), 2u);

  // Check limit and offset parameters exist
  bool has_limit = false;
  bool has_offset = false;
  for (const auto & p : params) {
    if (p["name"] == "limit") {
      has_limit = true;
    }
    if (p["name"] == "offset") {
      has_offset = true;
    }
  }
  EXPECT_TRUE(has_limit);
  EXPECT_TRUE(has_offset);
}

TEST_F(PathBuilderTest, EntityCollectionHasErrorResponses) {
  auto result = path_builder_.build_entity_collection("apps");
  EXPECT_TRUE(result["get"]["responses"].contains("400"));
  EXPECT_TRUE(result["get"]["responses"].contains("404"));
  EXPECT_TRUE(result["get"]["responses"].contains("500"));
}

// =============================================================================
// Entity detail tests
// =============================================================================

TEST_F(PathBuilderTest, EntityDetailHasGet) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_entity_detail("apps");
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"].contains("summary"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, EntityDetailHasPathParam) {
  auto result = path_builder_.build_entity_detail("components");
  ASSERT_TRUE(result["get"].contains("parameters"));
  auto & params = result["get"]["parameters"];
  ASSERT_GE(params.size(), 1u);
  EXPECT_EQ(params[0]["in"], "path");
  EXPECT_TRUE(params[0]["required"].get<bool>());
}

// =============================================================================
// Data collection tests
// =============================================================================

TEST_F(PathBuilderTest, DataCollectionHasGet) {
  // @verifies REQ_INTEROP_002
  std::vector<TopicData> topics = {{"temperature", "std_msgs/msg/Float32", "publish"},
                                   {"command", "std_msgs/msg/String", "subscribe"}};
  auto result = path_builder_.build_data_collection("apps/sensor", topics);
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, DataCollectionHasSovdExtension) {
  std::vector<TopicData> topics;
  auto result = path_builder_.build_data_collection("apps/sensor", topics);
  EXPECT_TRUE(result.contains("x-sovd-data-category"));
  EXPECT_EQ(result["x-sovd-data-category"], "currentData");
}

// =============================================================================
// Data item tests
// =============================================================================

TEST_F(PathBuilderTest, DataItemGetAlwaysPresent) {
  // @verifies REQ_INTEROP_002
  TopicData topic{"temperature", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, DataItemPutForSubscribeTopic) {
  // @verifies REQ_INTEROP_002
  TopicData topic{"command", "std_msgs/msg/String", "subscribe"};
  auto result = path_builder_.build_data_item("apps/actuator", topic);
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("put"));
  EXPECT_TRUE(result["put"].contains("requestBody"));
  EXPECT_TRUE(result["put"]["requestBody"]["required"].get<bool>());
}

TEST_F(PathBuilderTest, DataItemPutForBothDirection) {
  TopicData topic{"sensor_data", "std_msgs/msg/Float32", "both"};
  auto result = path_builder_.build_data_item("apps/node", topic);
  ASSERT_TRUE(result.contains("put"));
}

TEST_F(PathBuilderTest, DataItemNoPutForPublishOnly) {
  TopicData topic{"output", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  ASSERT_TRUE(result.contains("get"));
  EXPECT_FALSE(result.contains("put"));
}

TEST_F(PathBuilderTest, DataItemHasSovdExtensions) {
  TopicData topic{"temperature", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  EXPECT_EQ(result["x-sovd-data-category"], "currentData");
  EXPECT_TRUE(result["x-sovd-cyclic-subscription-supported"].get<bool>());
  EXPECT_EQ(result["x-sovd-name"], "temperature");
}

TEST_F(PathBuilderTest, DataItemSchemaFromRosType) {
  TopicData topic{"temperature", "std_msgs/msg/Float32", "publish"};
  auto result = path_builder_.build_data_item("apps/sensor", topic);
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  // std_msgs/msg/Float32 has a "data" field
  EXPECT_EQ(schema["type"], "object");
  EXPECT_TRUE(schema.contains("properties"));
}

// =============================================================================
// Operations collection tests
// =============================================================================

TEST_F(PathBuilderTest, OperationsCollectionHasGet) {
  // @verifies REQ_INTEROP_002
  AggregatedOperations ops;
  ops.services.push_back({"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt});
  auto result = path_builder_.build_operations_collection("apps/engine", ops);
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, OperationsCollectionResponseHasItems) {
  AggregatedOperations ops;
  auto result = path_builder_.build_operations_collection("apps/engine", ops);
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
}

// =============================================================================
// Operation item (service) tests
// =============================================================================

TEST_F(PathBuilderTest, ServiceOperationHasGetAndPost) {
  // @verifies REQ_INTEROP_002
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("post"));
}

TEST_F(PathBuilderTest, ServiceOperationPostHasRequestBody) {
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  ASSERT_TRUE(result["post"].contains("requestBody"));
  EXPECT_TRUE(result["post"]["requestBody"]["required"].get<bool>());
}

TEST_F(PathBuilderTest, ServiceOperationHasSovdName) {
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  EXPECT_EQ(result["x-sovd-name"], "calibrate");
}

TEST_F(PathBuilderTest, ServiceOperationNotAsynchronous) {
  ServiceInfo service{"calibrate", "/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/engine", service);
  EXPECT_FALSE(result.contains("x-sovd-asynchronous-execution"));
}

// =============================================================================
// Operation item (action) tests
// =============================================================================

TEST_F(PathBuilderTest, ActionOperationHasGetAndPost) {
  // @verifies REQ_INTEROP_002
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("post"));
}

TEST_F(PathBuilderTest, ActionOperationIsAsynchronous) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  ASSERT_TRUE(result.contains("x-sovd-asynchronous-execution"));
  EXPECT_TRUE(result["x-sovd-asynchronous-execution"].get<bool>());
}

TEST_F(PathBuilderTest, ActionOperationPostReturns202) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  EXPECT_TRUE(result["post"]["responses"].contains("202"));
}

TEST_F(PathBuilderTest, ActionOperationHasSovdName) {
  ActionInfo action{"navigate", "/nav/navigate", "nav2_msgs/action/NavigateToPose", std::nullopt};
  auto result = path_builder_.build_operation_item("apps/navigation", action);
  EXPECT_EQ(result["x-sovd-name"], "navigate");
}

// =============================================================================
// Configurations collection tests
// =============================================================================

TEST_F(PathBuilderTest, ConfigurationsHasGetPutDelete) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_configurations_collection("apps/sensor");
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("put"));
  ASSERT_TRUE(result.contains("delete"));
}

TEST_F(PathBuilderTest, ConfigurationsGetReturnsItemsWrapper) {
  auto result = path_builder_.build_configurations_collection("apps/sensor");
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
}

TEST_F(PathBuilderTest, ConfigurationsPutHasRequestBody) {
  auto result = path_builder_.build_configurations_collection("apps/sensor");
  ASSERT_TRUE(result["put"].contains("requestBody"));
  EXPECT_TRUE(result["put"]["requestBody"]["required"].get<bool>());
  auto req_schema = result["put"]["requestBody"]["content"]["application/json"]["schema"];
  // Should be configuration_param_schema
  EXPECT_EQ(req_schema["type"], "object");
  EXPECT_TRUE(req_schema["properties"].contains("name"));
  EXPECT_TRUE(req_schema["properties"].contains("value"));
}

// =============================================================================
// Faults collection tests
// =============================================================================

TEST_F(PathBuilderTest, FaultsHasGetAndPut) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_faults_collection("apps/engine");
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("put"));
}

TEST_F(PathBuilderTest, FaultsGetReturnsFaultList) {
  auto result = path_builder_.build_faults_collection("apps/engine");
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  EXPECT_EQ(schema["type"], "object");
  ASSERT_TRUE(schema.contains("properties"));
  ASSERT_TRUE(schema["properties"].contains("items"));
  // Items should be fault objects
  auto & item_schema = schema["properties"]["items"]["items"];
  EXPECT_TRUE(item_schema["properties"].contains("fault_code"));
}

TEST_F(PathBuilderTest, FaultsPutHasStatusRequestBody) {
  auto result = path_builder_.build_faults_collection("apps/engine");
  ASSERT_TRUE(result["put"].contains("requestBody"));
  auto req_schema = result["put"]["requestBody"]["content"]["application/json"]["schema"];
  EXPECT_TRUE(req_schema["properties"].contains("status"));
}

// =============================================================================
// Logs collection tests
// =============================================================================

TEST_F(PathBuilderTest, LogsHasGet) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_logs_collection("apps/sensor");
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

TEST_F(PathBuilderTest, LogsHasLevelQueryParam) {
  auto result = path_builder_.build_logs_collection("apps/sensor");
  auto & params = result["get"]["parameters"];
  bool has_level = false;
  for (const auto & p : params) {
    if (p["name"] == "level") {
      has_level = true;
    }
  }
  EXPECT_TRUE(has_level);
}

TEST_F(PathBuilderTest, LogsReturnsLogEntryItems) {
  auto result = path_builder_.build_logs_collection("apps/sensor");
  auto schema = result["get"]["responses"]["200"]["content"]["application/json"]["schema"];
  auto & item_schema = schema["properties"]["items"]["items"];
  EXPECT_TRUE(item_schema["properties"].contains("timestamp"));
  EXPECT_TRUE(item_schema["properties"].contains("level"));
  EXPECT_TRUE(item_schema["properties"].contains("message"));
}

// =============================================================================
// Bulk data collection tests
// =============================================================================

TEST_F(PathBuilderTest, BulkDataHasGet) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_bulk_data_collection("apps/sensor");
  ASSERT_TRUE(result.contains("get"));
  EXPECT_TRUE(result["get"]["responses"].contains("200"));
}

// =============================================================================
// Cyclic subscriptions collection tests
// =============================================================================

TEST_F(PathBuilderTest, CyclicSubscriptionsHasGetAndPost) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_cyclic_subscriptions_collection("apps/sensor");
  ASSERT_TRUE(result.contains("get"));
  ASSERT_TRUE(result.contains("post"));
}

TEST_F(PathBuilderTest, CyclicSubscriptionsPostHasRequestBody) {
  auto result = path_builder_.build_cyclic_subscriptions_collection("apps/sensor");
  ASSERT_TRUE(result["post"].contains("requestBody"));
  auto req_schema = result["post"]["requestBody"]["content"]["application/json"]["schema"];
  EXPECT_TRUE(req_schema["properties"].contains("topic"));
}

TEST_F(PathBuilderTest, CyclicSubscriptionsPostReturns201) {
  auto result = path_builder_.build_cyclic_subscriptions_collection("apps/sensor");
  EXPECT_TRUE(result["post"]["responses"].contains("201"));
}

// =============================================================================
// SSE endpoint tests
// =============================================================================

TEST_F(PathBuilderTest, SseEndpointHasGet) {
  // @verifies REQ_INTEROP_002
  auto result = path_builder_.build_sse_endpoint("/events/faults", "Fault event stream");
  ASSERT_TRUE(result.contains("get"));
}

TEST_F(PathBuilderTest, SseEndpointHasEventStreamContentType) {
  auto result = path_builder_.build_sse_endpoint("/events/faults", "Fault event stream");
  ASSERT_TRUE(result["get"]["responses"].contains("200"));
  auto & content = result["get"]["responses"]["200"]["content"];
  ASSERT_TRUE(content.contains("text/event-stream"));
}

TEST_F(PathBuilderTest, SseEndpointHasDescription) {
  auto result = path_builder_.build_sse_endpoint("/events/faults", "Fault event stream");
  EXPECT_EQ(result["get"]["summary"], "Fault event stream");
}

// =============================================================================
// Error responses tests
// =============================================================================

TEST_F(PathBuilderTest, ErrorResponsesWithoutAuth) {
  // @verifies REQ_INTEROP_002
  auto errors = path_builder_.error_responses();
  EXPECT_TRUE(errors.contains("400"));
  EXPECT_TRUE(errors.contains("404"));
  EXPECT_TRUE(errors.contains("500"));
  EXPECT_FALSE(errors.contains("401"));
  EXPECT_FALSE(errors.contains("403"));
}

TEST_F(PathBuilderTest, ErrorResponsesWithAuth) {
  PathBuilder auth_builder(schema_builder_, true);
  auto errors = auth_builder.error_responses();
  EXPECT_TRUE(errors.contains("400"));
  EXPECT_TRUE(errors.contains("404"));
  EXPECT_TRUE(errors.contains("500"));
  EXPECT_TRUE(errors.contains("401"));
  EXPECT_TRUE(errors.contains("403"));
}

TEST_F(PathBuilderTest, ErrorResponsesUseGenericErrorSchema) {
  auto errors = path_builder_.error_responses();
  for (const auto & [code, resp] : errors.items()) {
    ASSERT_TRUE(resp.contains("content")) << "Error " << code << " missing content";
    auto schema = resp["content"]["application/json"]["schema"];
    EXPECT_EQ(schema["type"], "object") << "Error " << code << " schema type mismatch";
    EXPECT_TRUE(schema["properties"].contains("error_code")) << "Error " << code << " missing error_code";
    EXPECT_TRUE(schema["properties"].contains("message")) << "Error " << code << " missing message";
  }
}
