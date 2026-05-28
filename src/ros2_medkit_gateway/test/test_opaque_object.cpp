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

#include <algorithm>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace dto = ros2_medkit_gateway::dto;

namespace {
struct OpaqueSample {
  std::string label;
  nlohmann::json value;
};
}  // namespace

template <>
[[maybe_unused]] inline constexpr auto dto::dto_fields<OpaqueSample> =
    std::make_tuple(dto::field("label", &OpaqueSample::label), dto::opaque_object("value", &OpaqueSample::value));

template <>
[[maybe_unused]] inline constexpr std::string_view dto::dto_name<OpaqueSample> = "OpaqueSample";

// -----------------------------------------------------------------------------
// Detection trait
// -----------------------------------------------------------------------------

TEST(OpaqueObjectTrait, DetectsOpaqueObjectField) {
  constexpr auto opaque_f = dto::opaque_object("value", &OpaqueSample::value);
  using OpaqueT = std::decay_t<decltype(opaque_f)>;
  EXPECT_TRUE(dto::is_opaque_object_field_v<OpaqueT>);
}

TEST(OpaqueObjectTrait, RejectsRegularField) {
  constexpr auto regular_f = dto::field("label", &OpaqueSample::label);
  using RegularT = std::decay_t<decltype(regular_f)>;
  EXPECT_FALSE(dto::is_opaque_object_field_v<RegularT>);
}

// -----------------------------------------------------------------------------
// JsonWriter
// -----------------------------------------------------------------------------

TEST(OpaqueObjectJsonWriter, PassesArbitraryObjectThrough) {
  OpaqueSample s;
  s.label = "ros_msg";
  s.value = nlohmann::json{{"foo", 42}, {"bar", nlohmann::json::array({1, 2, 3})}};

  const auto j = dto::JsonWriter<OpaqueSample>::write(s);

  EXPECT_EQ(j.at("label"), "ros_msg");
  ASSERT_TRUE(j.contains("value"));
  EXPECT_TRUE(j.at("value").is_object());
  EXPECT_EQ(j.at("value").at("foo"), 42);
  ASSERT_TRUE(j.at("value").at("bar").is_array());
  EXPECT_EQ(j.at("value").at("bar").size(), 3u);
}

TEST(OpaqueObjectJsonWriter, EmptyObjectStillSerializedAsObject) {
  OpaqueSample s;
  s.label = "empty";
  s.value = nlohmann::json::object();

  const auto j = dto::JsonWriter<OpaqueSample>::write(s);
  ASSERT_TRUE(j.contains("value"));
  EXPECT_TRUE(j.at("value").is_object());
  EXPECT_TRUE(j.at("value").empty());
}

// -----------------------------------------------------------------------------
// JsonReader
// -----------------------------------------------------------------------------

TEST(OpaqueObjectJsonReader, AcceptsObjectValue) {
  const auto j = nlohmann::json{{"label", "x"}, {"value", {{"k", "v"}, {"n", 7}}}};
  const auto result = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->label, "x");
  ASSERT_TRUE(result->value.is_object());
  EXPECT_EQ(result->value.at("k"), "v");
  EXPECT_EQ(result->value.at("n"), 7);
}

TEST(OpaqueObjectJsonReader, AcceptsEmptyObjectValue) {
  const auto j = nlohmann::json{{"label", "x"}, {"value", nlohmann::json::object()}};
  const auto result = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->value.is_object());
  EXPECT_TRUE(result->value.empty());
}

TEST(OpaqueObjectJsonReader, RejectsScalarValue) {
  const auto j = nlohmann::json{{"label", "x"}, {"value", 42}};
  const auto result = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1u);
  EXPECT_EQ(result.error()[0].field, "value");
  EXPECT_EQ(result.error()[0].message, "expected object");
}

TEST(OpaqueObjectJsonReader, RejectsArrayValue) {
  const auto j = nlohmann::json{{"label", "x"}, {"value", nlohmann::json::array({1, 2, 3})}};
  const auto result = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1u);
  EXPECT_EQ(result.error()[0].field, "value");
  EXPECT_EQ(result.error()[0].message, "expected object");
}

TEST(OpaqueObjectJsonReader, AbsentValueLeavesDefault) {
  // Opaque object field is required by schema, but JsonReader's lenient policy
  // for opaque fields is to leave the member at its default (empty object)
  // when absent or null. Tighter validation is the schema's job.
  const auto j = nlohmann::json{{"label", "x"}};
  const auto result = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->label, "x");
}

TEST(OpaqueObjectJsonReader, NullValueLeavesDefault) {
  const auto j = nlohmann::json{{"label", "x"}, {"value", nullptr}};
  const auto result = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_TRUE(result.has_value());
}

// -----------------------------------------------------------------------------
// SchemaWriter
// -----------------------------------------------------------------------------

TEST(OpaqueObjectSchemaWriter, EmitsOpaqueObjectFragment) {
  const auto schema = dto::SchemaWriter<OpaqueSample>::schema();
  ASSERT_EQ(schema.at("type"), "object");
  ASSERT_TRUE(schema.at("properties").contains("value"));

  const auto & value_prop = schema.at("properties").at("value");
  EXPECT_EQ(value_prop.at("type"), "object");
  EXPECT_EQ(value_prop.at("additionalProperties"), true);
  ASSERT_TRUE(value_prop.contains("x-medkit-opaque"));
  EXPECT_EQ(value_prop.at("x-medkit-opaque"), true);
}

TEST(OpaqueObjectSchemaWriter, OpaqueFieldIsRequired) {
  const auto schema = dto::SchemaWriter<OpaqueSample>::schema();
  ASSERT_TRUE(schema.contains("required"));
  const auto & req = schema.at("required");
  EXPECT_NE(std::find(req.begin(), req.end(), "value"), req.end());
  EXPECT_NE(std::find(req.begin(), req.end(), "label"), req.end());
}

// -----------------------------------------------------------------------------
// Round-trip
// -----------------------------------------------------------------------------

TEST(OpaqueObjectRoundTrip, ArbitraryNestedJsonSurvivesWriteRead) {
  OpaqueSample s;
  s.label = "round_trip";
  s.value = nlohmann::json{
      {"scalar", 3.14}, {"nested", {{"a", 1}, {"b", "two"}}}, {"list", nlohmann::json::array({true, false})}};

  const auto j = dto::JsonWriter<OpaqueSample>::write(s);
  const auto back = dto::JsonReader<OpaqueSample>::read(j);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->label, "round_trip");
  EXPECT_EQ(back->value, s.value);
}
