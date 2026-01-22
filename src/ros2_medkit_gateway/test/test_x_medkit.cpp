// Copyright 2025 bburda, mfaferek93
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

#include "ros2_medkit_gateway/http/x_medkit.hpp"

using ros2_medkit_gateway::XMedkit;
using json = nlohmann::json;

class XMedkitTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }
};

// ==================== Basic functionality tests ====================

TEST_F(XMedkitTest, EmptyWhenNoFieldsSet) {
  XMedkit ext;
  EXPECT_TRUE(ext.empty());
  EXPECT_TRUE(ext.build().empty());
}

TEST_F(XMedkitTest, NotEmptyAfterSettingRos2Field) {
  XMedkit ext;
  ext.ros2_node("/test_node");
  EXPECT_FALSE(ext.empty());
}

TEST_F(XMedkitTest, NotEmptyAfterSettingOtherField) {
  XMedkit ext;
  ext.source("heuristic");
  EXPECT_FALSE(ext.empty());
}

// ==================== ROS2 metadata tests ====================

TEST_F(XMedkitTest, BuildsRos2NodeCorrectly) {
  XMedkit ext;
  ext.ros2_node("/sensors/temp_sensor");

  auto result = ext.build();
  EXPECT_TRUE(result.contains("ros2"));
  EXPECT_EQ(result["ros2"]["node"], "/sensors/temp_sensor");
}

TEST_F(XMedkitTest, BuildsRos2NamespaceCorrectly) {
  XMedkit ext;
  ext.ros2_namespace("/sensors");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["namespace"], "/sensors");
}

TEST_F(XMedkitTest, BuildsRos2TypeCorrectly) {
  XMedkit ext;
  ext.ros2_type("sensor_msgs/msg/Temperature");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["type"], "sensor_msgs/msg/Temperature");
}

TEST_F(XMedkitTest, BuildsRos2TopicCorrectly) {
  XMedkit ext;
  ext.ros2_topic("/sensors/temperature");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["topic"], "/sensors/temperature");
}

TEST_F(XMedkitTest, BuildsRos2ServiceCorrectly) {
  XMedkit ext;
  ext.ros2_service("/calibration/start");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["service"], "/calibration/start");
}

TEST_F(XMedkitTest, BuildsRos2ActionCorrectly) {
  XMedkit ext;
  ext.ros2_action("/navigate_to_pose");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["action"], "/navigate_to_pose");
}

TEST_F(XMedkitTest, BuildsRos2KindCorrectly) {
  XMedkit ext;
  ext.ros2_kind("service");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["kind"], "service");
}

// ==================== Discovery metadata tests ====================

TEST_F(XMedkitTest, BuildsSourceCorrectly) {
  XMedkit ext;
  ext.source("heuristic");

  auto result = ext.build();
  EXPECT_EQ(result["source"], "heuristic");
}

TEST_F(XMedkitTest, BuildsIsOnlineCorrectly) {
  XMedkit ext;
  ext.is_online(true);

  auto result = ext.build();
  EXPECT_EQ(result["is_online"], true);

  XMedkit ext2;
  ext2.is_online(false);
  auto result2 = ext2.build();
  EXPECT_EQ(result2["is_online"], false);
}

TEST_F(XMedkitTest, BuildsComponentIdCorrectly) {
  XMedkit ext;
  ext.component_id("powertrain_component");

  auto result = ext.build();
  EXPECT_EQ(result["component_id"], "powertrain_component");
}

TEST_F(XMedkitTest, BuildsEntityIdCorrectly) {
  XMedkit ext;
  ext.entity_id("temp_sensor");

  auto result = ext.build();
  EXPECT_EQ(result["entity_id"], "temp_sensor");
}

// ==================== Type introspection tests ====================

TEST_F(XMedkitTest, BuildsTypeInfoCorrectly) {
  XMedkit ext;
  json field1 = {{"name", "temperature"}, {"type", "float64"}};
  json type_info = {{"fields", json::array({field1})}};
  ext.type_info(type_info);

  auto result = ext.build();
  EXPECT_TRUE(result.contains("type_info"));
  EXPECT_EQ(result["type_info"]["fields"][0]["name"], "temperature");
}

TEST_F(XMedkitTest, BuildsTypeSchemaCorrectly) {
  XMedkit ext;
  // ROS2 IDL-derived type schema (distinct from SOVD OpenAPI schema)
  json schema = {{"type", "object"}, {"properties", {{"data", {{"type", "string"}}}}}};
  ext.type_schema(schema);

  auto result = ext.build();
  EXPECT_TRUE(result.contains("type_schema"));
  EXPECT_EQ(result["type_schema"]["type"], "object");
  EXPECT_EQ(result["type_schema"]["properties"]["data"]["type"], "string");
}

// ==================== Execution tracking tests ====================

TEST_F(XMedkitTest, BuildsGoalIdCorrectly) {
  XMedkit ext;
  ext.goal_id("abc123-uuid-456");

  auto result = ext.build();
  EXPECT_EQ(result["goal_id"], "abc123-uuid-456");
}

TEST_F(XMedkitTest, BuildsGoalStatusCorrectly) {
  XMedkit ext;
  ext.goal_status("executing");

  auto result = ext.build();
  EXPECT_EQ(result["goal_status"], "executing");
}

TEST_F(XMedkitTest, BuildsLastFeedbackCorrectly) {
  XMedkit ext;
  json feedback = {{"progress", 75}, {"message", "Processing..."}};
  ext.last_feedback(feedback);

  auto result = ext.build();
  EXPECT_EQ(result["last_feedback"]["progress"], 75);
}

// ==================== Generic methods tests ====================

TEST_F(XMedkitTest, AddCustomFieldCorrectly) {
  XMedkit ext;
  ext.add("custom_field", "custom_value");

  auto result = ext.build();
  EXPECT_EQ(result["custom_field"], "custom_value");
}

TEST_F(XMedkitTest, AddRos2CustomFieldCorrectly) {
  XMedkit ext;
  ext.add_ros2("custom_ros2_field", 42);

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["custom_ros2_field"], 42);
}

// ==================== Fluent builder tests ====================

TEST_F(XMedkitTest, FluentBuilderChains) {
  XMedkit ext;
  ext.ros2_node("/my_node")
      .ros2_type("std_msgs/msg/String")
      .ros2_namespace("/test")
      .source("heuristic")
      .is_online(true);

  auto result = ext.build();

  // Verify all fields are set
  EXPECT_EQ(result["ros2"]["node"], "/my_node");
  EXPECT_EQ(result["ros2"]["type"], "std_msgs/msg/String");
  EXPECT_EQ(result["ros2"]["namespace"], "/test");
  EXPECT_EQ(result["source"], "heuristic");
  EXPECT_EQ(result["is_online"], true);
}

TEST_F(XMedkitTest, BuildsCorrectStructure) {
  XMedkit ext;
  ext.ros2_node("/sensors/temp_sensor")
      .ros2_type("sensor_msgs/msg/Temperature")
      .ros2_topic("/temperature")
      .source("heuristic")
      .is_online(true)
      .component_id("sensors_component");

  auto result = ext.build();

  // Verify ROS2 section exists and contains expected fields
  EXPECT_TRUE(result.contains("ros2"));
  EXPECT_EQ(result["ros2"]["node"], "/sensors/temp_sensor");
  EXPECT_EQ(result["ros2"]["type"], "sensor_msgs/msg/Temperature");
  EXPECT_EQ(result["ros2"]["topic"], "/temperature");

  // Verify top-level extension fields
  EXPECT_EQ(result["source"], "heuristic");
  EXPECT_EQ(result["is_online"], true);
  EXPECT_EQ(result["component_id"], "sensors_component");

  // Verify no unexpected nesting
  EXPECT_FALSE(result["ros2"].contains("source"));
  EXPECT_FALSE(result["ros2"].contains("is_online"));
}

TEST_F(XMedkitTest, MultipleCallsOverwritePreviousValues) {
  XMedkit ext;
  ext.ros2_node("/first_node");
  ext.ros2_node("/second_node");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["node"], "/second_node");
}

// ==================== Edge cases ====================

TEST_F(XMedkitTest, HandlesEmptyStrings) {
  XMedkit ext;
  ext.ros2_node("");
  ext.source("");

  auto result = ext.build();
  EXPECT_EQ(result["ros2"]["node"], "");
  EXPECT_EQ(result["source"], "");
  EXPECT_FALSE(ext.empty());  // Empty strings still count as set
}

TEST_F(XMedkitTest, HandlesJsonArrays) {
  XMedkit ext;
  json arr = json::array({"item1", "item2", "item3"});
  ext.add("items", arr);

  auto result = ext.build();
  EXPECT_TRUE(result["items"].is_array());
  EXPECT_EQ(result["items"].size(), 3);
}

TEST_F(XMedkitTest, HandlesNestedJsonObjects) {
  XMedkit ext;
  json nested = {{"level1", {{"level2", {{"level3", "deep_value"}}}}}};
  ext.add("nested", nested);

  auto result = ext.build();
  EXPECT_EQ(result["nested"]["level1"]["level2"]["level3"], "deep_value");
}
