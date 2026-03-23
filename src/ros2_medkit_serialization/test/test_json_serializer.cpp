// Copyright 2026 Selfpatch
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

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_serialization {

class JsonSerializerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    TypeCache::instance().clear();
  }

  JsonSerializer serializer_;
};

// YAML ↔ JSON conversion tests

TEST_F(JsonSerializerTest, YamlToJsonNull) {
  YAML::Node yaml;
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_null());
}

TEST_F(JsonSerializerTest, YamlToJsonBool) {
  YAML::Node yaml;
  yaml = true;
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_boolean());
  EXPECT_TRUE(json.get<bool>());
}

TEST_F(JsonSerializerTest, YamlToJsonInteger) {
  YAML::Node yaml;
  yaml = 42;
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_number_integer());
  EXPECT_EQ(json.get<int>(), 42);
}

TEST_F(JsonSerializerTest, YamlToJsonFloat) {
  YAML::Node yaml;
  yaml = 3.14;
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_number_float());
  EXPECT_NEAR(json.get<double>(), 3.14, 0.001);
}

TEST_F(JsonSerializerTest, YamlToJsonString) {
  YAML::Node yaml;
  yaml = "hello";
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_string());
  EXPECT_EQ(json.get<std::string>(), "hello");
}

TEST_F(JsonSerializerTest, YamlToJsonArray) {
  YAML::Node yaml;
  yaml.push_back(1);
  yaml.push_back(2);
  yaml.push_back(3);
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_array());
  EXPECT_EQ(json.size(), 3U);
  EXPECT_EQ(json[0].get<int>(), 1);
  EXPECT_EQ(json[1].get<int>(), 2);
  EXPECT_EQ(json[2].get<int>(), 3);
}

TEST_F(JsonSerializerTest, YamlToJsonObject) {
  YAML::Node yaml;
  yaml["name"] = "test";
  yaml["value"] = 123;
  auto json = JsonSerializer::yaml_to_json(yaml);
  EXPECT_TRUE(json.is_object());
  EXPECT_EQ(json["name"].get<std::string>(), "test");
  EXPECT_EQ(json["value"].get<int>(), 123);
}

TEST_F(JsonSerializerTest, JsonToYamlNull) {
  nlohmann::json json = nullptr;
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsNull());
}

TEST_F(JsonSerializerTest, JsonToYamlBool) {
  nlohmann::json json = true;
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsScalar());
  EXPECT_TRUE(yaml.as<bool>());
}

TEST_F(JsonSerializerTest, JsonToYamlInteger) {
  nlohmann::json json = 42;
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsScalar());
  EXPECT_EQ(yaml.as<int>(), 42);
}

TEST_F(JsonSerializerTest, JsonToYamlFloat) {
  nlohmann::json json = 3.14;
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsScalar());
  EXPECT_NEAR(yaml.as<double>(), 3.14, 0.001);
}

TEST_F(JsonSerializerTest, JsonToYamlString) {
  nlohmann::json json = "hello";
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsScalar());
  EXPECT_EQ(yaml.as<std::string>(), "hello");
}

TEST_F(JsonSerializerTest, JsonToYamlArray) {
  nlohmann::json json = {1, 2, 3};
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsSequence());
  EXPECT_EQ(yaml.size(), 3U);
}

TEST_F(JsonSerializerTest, JsonToYamlObject) {
  nlohmann::json json = {{"name", "test"}, {"value", 123}};
  auto yaml = JsonSerializer::json_to_yaml(json);
  EXPECT_TRUE(yaml.IsMap());
  EXPECT_EQ(yaml["name"].as<std::string>(), "test");
  EXPECT_EQ(yaml["value"].as<int>(), 123);
}

// Message serialization tests

TEST_F(JsonSerializerTest, ToJsonStdMsgsString) {
  std_msgs::msg::String msg;
  msg.data = "hello world";

  auto type_info = TypeCache::instance().get_message_type_info("std_msgs", "String");
  ASSERT_NE(type_info, nullptr);

  auto json = serializer_.to_json(type_info, &msg);
  EXPECT_TRUE(json.is_object());
  EXPECT_EQ(json["data"].get<std::string>(), "hello world");
}

TEST_F(JsonSerializerTest, ToJsonStdMsgsInt32) {
  std_msgs::msg::Int32 msg;
  msg.data = 42;

  auto type_info = TypeCache::instance().get_message_type_info("std_msgs", "Int32");
  ASSERT_NE(type_info, nullptr);

  auto json = serializer_.to_json(type_info, &msg);
  EXPECT_TRUE(json.is_object());
  EXPECT_EQ(json["data"].get<int>(), 42);
}

TEST_F(JsonSerializerTest, ToJsonGeometryMsgsPoint) {
  geometry_msgs::msg::Point msg;
  msg.x = 1.0;
  msg.y = 2.0;
  msg.z = 3.0;

  auto type_info = TypeCache::instance().get_message_type_info("geometry_msgs", "Point");
  ASSERT_NE(type_info, nullptr);

  auto json = serializer_.to_json(type_info, &msg);
  EXPECT_TRUE(json.is_object());
  EXPECT_NEAR(json["x"].get<double>(), 1.0, 0.001);
  EXPECT_NEAR(json["y"].get<double>(), 2.0, 0.001);
  EXPECT_NEAR(json["z"].get<double>(), 3.0, 0.001);
}

TEST_F(JsonSerializerTest, ToJsonWithTypeString) {
  std_msgs::msg::String msg;
  msg.data = "test";

  auto json = serializer_.to_json("std_msgs/msg/String", &msg);
  EXPECT_EQ(json["data"].get<std::string>(), "test");
}

TEST_F(JsonSerializerTest, FromJsonStdMsgsString) {
  nlohmann::json json = {{"data", "hello"}};

  auto type_info = TypeCache::instance().get_message_type_info("std_msgs", "String");
  ASSERT_NE(type_info, nullptr);

  auto ros_msg = serializer_.from_json(type_info, json);
  ASSERT_NE(ros_msg.data, nullptr);

  auto * msg = reinterpret_cast<std_msgs::msg::String *>(ros_msg.data);
  EXPECT_EQ(msg->data, "hello");

  // Clean up
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynmsg::cpp::ros_message_destroy_with_allocator(&ros_msg, &allocator);
}

TEST_F(JsonSerializerTest, FromJsonGeometryMsgsPoint) {
  nlohmann::json json = {{"x", 1.5}, {"y", 2.5}, {"z", 3.5}};

  auto ros_msg = serializer_.from_json("geometry_msgs/msg/Point", json);
  ASSERT_NE(ros_msg.data, nullptr);

  auto * msg = reinterpret_cast<geometry_msgs::msg::Point *>(ros_msg.data);
  EXPECT_NEAR(msg->x, 1.5, 0.001);
  EXPECT_NEAR(msg->y, 2.5, 0.001);
  EXPECT_NEAR(msg->z, 3.5, 0.001);

  // Clean up
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynmsg::cpp::ros_message_destroy_with_allocator(&ros_msg, &allocator);
}

TEST_F(JsonSerializerTest, FromJsonToExistingMessage) {
  nlohmann::json json = {{"data", "populated"}};
  std_msgs::msg::String msg;

  auto type_info = TypeCache::instance().get_message_type_info("std_msgs", "String");
  ASSERT_NE(type_info, nullptr);

  serializer_.from_json_to_message(type_info, json, &msg);
  EXPECT_EQ(msg.data, "populated");
}

// Schema tests

TEST_F(JsonSerializerTest, GetSchemaStdMsgsString) {
  auto schema = serializer_.get_schema("std_msgs/msg/String");
  EXPECT_EQ(schema["type"].get<std::string>(), "object");
  EXPECT_TRUE(schema.contains("properties"));
  EXPECT_TRUE(schema["properties"].contains("data"));
  EXPECT_EQ(schema["properties"]["data"]["type"].get<std::string>(), "string");
}

TEST_F(JsonSerializerTest, GetSchemaGeometryMsgsPoint) {
  auto schema = serializer_.get_schema("geometry_msgs/msg/Point");
  EXPECT_EQ(schema["type"].get<std::string>(), "object");
  EXPECT_TRUE(schema["properties"].contains("x"));
  EXPECT_TRUE(schema["properties"].contains("y"));
  EXPECT_TRUE(schema["properties"].contains("z"));
  EXPECT_EQ(schema["properties"]["x"]["type"].get<std::string>(), "number");
}

// Default values tests

TEST_F(JsonSerializerTest, GetDefaultsStdMsgsString) {
  auto defaults = serializer_.get_defaults("std_msgs/msg/String");
  EXPECT_TRUE(defaults.is_object());
  EXPECT_TRUE(defaults.contains("data"));
  EXPECT_EQ(defaults["data"].get<std::string>(), "");
}

TEST_F(JsonSerializerTest, GetDefaultsStdMsgsInt32) {
  auto defaults = serializer_.get_defaults("std_msgs/msg/Int32");
  EXPECT_TRUE(defaults.is_object());
  EXPECT_TRUE(defaults.contains("data"));
  EXPECT_EQ(defaults["data"].get<int>(), 0);
}

// Error handling tests

TEST_F(JsonSerializerTest, ToJsonNullTypeInfoThrows) {
  int dummy = 0;
  EXPECT_THROW(serializer_.to_json(nullptr, &dummy), JsonConversionError);
}

TEST_F(JsonSerializerTest, ToJsonNullDataThrows) {
  auto type_info = TypeCache::instance().get_message_type_info("std_msgs", "String");
  ASSERT_NE(type_info, nullptr);
  EXPECT_THROW(serializer_.to_json(type_info, nullptr), JsonConversionError);
}

TEST_F(JsonSerializerTest, FromJsonNullTypeInfoThrows) {
  nlohmann::json json = {{"data", "test"}};
  EXPECT_THROW(serializer_.from_json(nullptr, json), JsonConversionError);
}

TEST_F(JsonSerializerTest, InvalidTypeStringThrows) {
  std_msgs::msg::String msg;
  EXPECT_THROW(serializer_.to_json("invalid/type/Name", &msg), TypeNotFoundError);
}

// Round-trip tests

TEST_F(JsonSerializerTest, RoundTripStdMsgsString) {
  std_msgs::msg::String original;
  original.data = "round trip test";

  auto json = serializer_.to_json("std_msgs/msg/String", &original);
  auto ros_msg = serializer_.from_json("std_msgs/msg/String", json);

  auto * restored = reinterpret_cast<std_msgs::msg::String *>(ros_msg.data);
  EXPECT_EQ(restored->data, original.data);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynmsg::cpp::ros_message_destroy_with_allocator(&ros_msg, &allocator);
}

TEST_F(JsonSerializerTest, RoundTripGeometryMsgsPoint) {
  geometry_msgs::msg::Point original;
  original.x = 1.234;
  original.y = 5.678;
  original.z = 9.012;

  auto json = serializer_.to_json("geometry_msgs/msg/Point", &original);
  auto ros_msg = serializer_.from_json("geometry_msgs/msg/Point", json);

  auto * restored = reinterpret_cast<geometry_msgs::msg::Point *>(ros_msg.data);
  EXPECT_NEAR(restored->x, original.x, 0.0001);
  EXPECT_NEAR(restored->y, original.y, 0.0001);
  EXPECT_NEAR(restored->z, original.z, 0.0001);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynmsg::cpp::ros_message_destroy_with_allocator(&ros_msg, &allocator);
}

}  // namespace ros2_medkit_serialization

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
