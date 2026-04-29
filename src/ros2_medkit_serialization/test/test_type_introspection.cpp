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

#include <memory>

#include "ros2_medkit_serialization/type_introspection.hpp"

namespace ros2_medkit_serialization {

class TypeIntrospectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    introspection_ = std::make_unique<TypeIntrospection>("");
  }

  std::unique_ptr<TypeIntrospection> introspection_;
};

TEST_F(TypeIntrospectionTest, get_type_template_for_valid_type) {
  auto template_json = introspection_->get_type_template("std_msgs/msg/String");

  EXPECT_TRUE(template_json.is_object());
  EXPECT_TRUE(template_json.contains("data"));
}

TEST_F(TypeIntrospectionTest, get_type_template_for_float32) {
  auto template_json = introspection_->get_type_template("std_msgs/msg/Float32");

  EXPECT_TRUE(template_json.is_object());
  EXPECT_TRUE(template_json.contains("data"));
}

TEST_F(TypeIntrospectionTest, get_type_template_for_unknown_type_throws) {
  EXPECT_THROW(introspection_->get_type_template("nonexistent_pkg/msg/NonExistent"), std::runtime_error);
}

TEST_F(TypeIntrospectionTest, get_type_schema_for_valid_type) {
  auto schema = introspection_->get_type_schema("std_msgs/msg/String");

  EXPECT_TRUE(schema.is_object());
}

TEST_F(TypeIntrospectionTest, get_type_schema_for_unknown_type_throws) {
  EXPECT_THROW(introspection_->get_type_schema("nonexistent_pkg/msg/NonExistent"), std::runtime_error);
}

TEST_F(TypeIntrospectionTest, get_type_info_returns_complete_info) {
  auto info = introspection_->get_type_info("std_msgs/msg/String");

  EXPECT_EQ(info.name, "std_msgs/msg/String");
  EXPECT_TRUE(info.default_value.is_object());
  EXPECT_TRUE(info.schema.is_object());
}

TEST_F(TypeIntrospectionTest, get_type_info_caches_results) {
  // First call
  auto info1 = introspection_->get_type_info("std_msgs/msg/Float32");
  // Second call should return cached result
  auto info2 = introspection_->get_type_info("std_msgs/msg/Float32");

  EXPECT_EQ(info1.name, info2.name);
  EXPECT_EQ(info1.default_value, info2.default_value);
  EXPECT_EQ(info1.schema, info2.schema);
}

TEST_F(TypeIntrospectionTest, get_type_info_for_unknown_type_returns_empty) {
  // For unknown types, get_type_info should return empty objects instead of throwing
  auto info = introspection_->get_type_info("nonexistent_pkg/msg/NonExistent");

  EXPECT_EQ(info.name, "nonexistent_pkg/msg/NonExistent");
  EXPECT_TRUE(info.default_value.is_object());
  EXPECT_TRUE(info.schema.is_object());
}

}  // namespace ros2_medkit_serialization
