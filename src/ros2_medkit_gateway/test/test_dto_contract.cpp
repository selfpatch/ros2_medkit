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
#include <optional>
#include <string>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace dto = ros2_medkit_gateway::dto;

namespace {
struct Sample {
  std::string id;
  std::optional<std::string> note;
};
inline constexpr std::string_view kSampleColors[] = {"red", "green"};
}  // namespace

template <>
inline constexpr auto dto::dto_fields<Sample> =
    std::make_tuple(dto::field("id", &Sample::id), dto::field("note", &Sample::note));

template <>
inline constexpr std::string_view dto::dto_name<Sample> = "Sample";

TEST(DtoContract, FieldFactoryDerivesPresenceFromOptional) {
  constexpr auto fields = dto::dto_fields<Sample>;
  EXPECT_EQ(std::get<0>(fields).presence, dto::Presence::kRequired);
  EXPECT_EQ(std::get<1>(fields).presence, dto::Presence::kOptional);
  EXPECT_EQ(std::get<0>(fields).key, "id");
}

TEST(DtoContract, IsDtoDetectsSpecialization) {
  EXPECT_TRUE(dto::is_dto_v<Sample>);
  EXPECT_FALSE(dto::is_dto_v<std::string>);
  EXPECT_FALSE(dto::is_dto_v<int>);
}

TEST(DtoContract, ForEachFieldVisitsEveryField) {
  int count = 0;
  dto::for_each_field<Sample>([&](const auto & /*f*/) {
    ++count;
  });
  EXPECT_EQ(count, 2);
}

TEST(DtoContract, FieldEnumPopulatesVocabularyAndDtoName) {
  constexpr auto f = dto::field_enum("color", &Sample::id, kSampleColors);
  EXPECT_EQ(f.enum_count, 2u);
  EXPECT_EQ(f.enum_values[0], "red");
  EXPECT_EQ(dto::dto_name<Sample>, "Sample");
}

TEST(SchemaWriter, BuildsObjectSchemaWithProperties) {
  const auto schema = dto::SchemaWriter<Sample>::schema();
  EXPECT_EQ(schema.at("type"), "object");
  EXPECT_TRUE(schema.at("properties").contains("id"));
  EXPECT_TRUE(schema.at("properties").contains("note"));
  EXPECT_EQ(schema.at("properties").at("id").at("type"), "string");
}

TEST(SchemaWriter, RequiredListExcludesOptionalFields) {
  const auto schema = dto::SchemaWriter<Sample>::schema();
  const auto & req = schema.at("required");
  EXPECT_NE(std::find(req.begin(), req.end(), "id"), req.end());
  EXPECT_EQ(std::find(req.begin(), req.end(), "note"), req.end());
}

TEST(JsonWriter, WritesRequiredFieldsAndSkipsEmptyOptional) {
  Sample s{"area_1", std::nullopt};
  const auto j = dto::JsonWriter<Sample>::write(s);
  EXPECT_EQ(j.at("id"), "area_1");
  EXPECT_FALSE(j.contains("note"));
}

TEST(JsonWriter, WritesPresentOptional) {
  Sample s{"area_1", std::string{"hello"}};
  const auto j = dto::JsonWriter<Sample>::write(s);
  EXPECT_EQ(j.at("note"), "hello");
}
