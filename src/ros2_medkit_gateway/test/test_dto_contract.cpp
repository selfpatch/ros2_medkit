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
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/errors.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/registry.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace dto = ros2_medkit_gateway::dto;

namespace {
struct Sample {
  std::string id;
  std::optional<std::string> note;
};
inline constexpr std::string_view kSampleColors[] = {"red", "green"};

struct ScalarSample {
  std::string id;
  bool active;
  int count;
};
}  // namespace

template <>
inline constexpr auto dto::dto_fields<Sample> =
    std::make_tuple(dto::field("id", &Sample::id), dto::field("note", &Sample::note));

template <>
inline constexpr std::string_view dto::dto_name<Sample> = "Sample";

template <>
inline constexpr auto dto::dto_fields<ScalarSample> =
    std::make_tuple(dto::field("id", &ScalarSample::id), dto::field("active", &ScalarSample::active),
                    dto::field("count", &ScalarSample::count));

template <>
inline constexpr std::string_view dto::dto_name<ScalarSample> = "ScalarSample";

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

TEST(JsonReader, DecodesValidObject) {
  const auto j = nlohmann::json{{"id", "x"}, {"note", "n"}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "x");
  ASSERT_TRUE(result->note.has_value());
  EXPECT_EQ(*result->note, "n");
}

TEST(JsonReader, ReportsMissingRequiredField) {
  const auto j = nlohmann::json{{"note", "n"}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error().size(), 1u);
  EXPECT_EQ(result.error()[0].field, "id");
}

TEST(JsonReader, ReportsWrongType) {
  const auto j = nlohmann::json{{"id", 123}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error()[0].field, "id");
}

TEST(JsonReader, IgnoresUnknownFields) {
  const auto j = nlohmann::json{{"id", "x"}, {"bogus", 1}};
  const auto result = dto::JsonReader<Sample>::read(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "x");
}

TEST(DtoSample, RoundTripsThroughWriterAndReader) {
  const Sample s = dto::make_sample<Sample>();
  const auto j = dto::JsonWriter<Sample>::write(s);
  const auto back = dto::JsonReader<Sample>::read(j);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->id, s.id);
}

TEST(DtoSample, SampleContainsEveryRequiredSchemaKey) {
  const Sample s = dto::make_sample<Sample>();
  const auto j = dto::JsonWriter<Sample>::write(s);
  const auto schema = dto::SchemaWriter<Sample>::schema();
  for (const auto & req : schema.at("required")) {
    EXPECT_TRUE(j.contains(req.get<std::string>())) << req;
  }
}

TEST(DtoSample, SynthesizesScalarMembers) {
  const ScalarSample s = dto::make_sample<ScalarSample>();
  const auto j = dto::JsonWriter<ScalarSample>::write(s);
  const auto back = dto::JsonReader<ScalarSample>::read(j);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->id, s.id);
  EXPECT_EQ(back->active, s.active);
  EXPECT_EQ(back->count, s.count);
}

TEST(DtoRegistry, CollectsNamedSchemas) {
  const auto schemas = dto::collect_component_schemas();
  EXPECT_TRUE(schemas.is_object());
}

TEST(DtoErrors, GenericErrorIsDtoWithCorrectFields) {
  EXPECT_TRUE(dto::is_dto_v<dto::GenericError>);
  EXPECT_EQ(dto::dto_name<dto::GenericError>, "GenericError");

  const auto schema = dto::SchemaWriter<dto::GenericError>::schema();
  EXPECT_EQ(schema.at("type"), "object");
  EXPECT_TRUE(schema.at("properties").contains("error_code"));
  EXPECT_TRUE(schema.at("properties").contains("message"));
  EXPECT_TRUE(schema.at("properties").contains("parameters"));

  // error_code and message are required; parameters is optional
  const auto & req = schema.at("required");
  EXPECT_NE(std::find(req.begin(), req.end(), "error_code"), req.end());
  EXPECT_NE(std::find(req.begin(), req.end(), "message"), req.end());
  EXPECT_EQ(std::find(req.begin(), req.end(), "parameters"), req.end());
}

TEST(DtoErrors, GenericErrorRoundTrips) {
  dto::GenericError e{"x-medkit-entity-not-found", "Entity not found", std::nullopt};
  const auto j = dto::JsonWriter<dto::GenericError>::write(e);
  EXPECT_EQ(j.at("error_code"), "x-medkit-entity-not-found");
  EXPECT_EQ(j.at("message"), "Entity not found");
  EXPECT_FALSE(j.contains("parameters"));
}

TEST(DtoEnums, EntityTypeVocabularyHasFourValues) {
  EXPECT_EQ(std::size(dto::kEntityTypeValues), 4u);
  EXPECT_EQ(dto::kEntityTypeValues[0], "area");
  EXPECT_EQ(dto::kEntityTypeValues[3], "function");
}

TEST(DtoEnums, CyclicSubscriptionIntervalHasThreeValues) {
  EXPECT_EQ(std::size(dto::kCyclicSubscriptionIntervalValues), 3u);
}

// =============================================================================
// XMedkit DTOs
// =============================================================================

TEST(XMedkitDtos, XMedkitRos2IsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitRos2>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitRos2>, "XMedkitRos2");
}

TEST(XMedkitDtos, XMedkitRos2NamespaceKeyIsMappedCorrectly) {
  // The C++ member is `ns` but the wire key must be "namespace".
  dto::XMedkitRos2 r2;
  r2.ns = "/sensors";
  const auto j = dto::JsonWriter<dto::XMedkitRos2>::write(r2);
  EXPECT_TRUE(j.contains("namespace"));
  EXPECT_EQ(j.at("namespace"), "/sensors");
  EXPECT_FALSE(j.contains("ns"));
}

TEST(XMedkitDtos, XMedkitAreaNestedRos2IsSerializedCorrectly) {
  dto::XMedkitArea area;
  dto::XMedkitRos2 r2;
  r2.ns = "/perception";
  area.ros2 = r2;
  area.contributors = std::vector<std::string>{"local", "peer:robot2"};

  const auto j = dto::JsonWriter<dto::XMedkitArea>::write(area);

  // Nested "ros2" object must be present with the "namespace" sub-key.
  ASSERT_TRUE(j.contains("ros2"));
  EXPECT_TRUE(j.at("ros2").contains("namespace"));
  EXPECT_EQ(j.at("ros2").at("namespace"), "/perception");

  // optional parent_area_id is absent when not set
  EXPECT_FALSE(j.contains("parent_area_id"));

  // contributors are serialized
  ASSERT_TRUE(j.contains("contributors"));
  EXPECT_EQ(j.at("contributors").size(), 2u);
}

TEST(XMedkitDtos, XMedkitAreaSchemaHasRos2RefProperty) {
  const auto schema = dto::SchemaWriter<dto::XMedkitArea>::schema();
  ASSERT_EQ(schema.at("type"), "object");
  ASSERT_TRUE(schema.at("properties").contains("ros2"));
  // Nested DTO renders as a $ref in schema.
  const auto & ros2_prop = schema.at("properties").at("ros2");
  EXPECT_TRUE(ros2_prop.contains("$ref"));
  EXPECT_EQ(ros2_prop.at("$ref"), "#/components/schemas/XMedkitRos2");
}

TEST(XMedkitDtos, XMedkitComponentIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitComponent>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitComponent>, "XMedkitComponent");
}

TEST(XMedkitDtos, XMedkitComponentCamelCaseWireKeys) {
  dto::XMedkitComponent comp;
  comp.parent_component_id = "parent_comp";
  comp.depends_on = std::vector<std::string>{"dep_a", "dep_b"};

  const auto j = dto::JsonWriter<dto::XMedkitComponent>::write(comp);

  EXPECT_TRUE(j.contains("parentComponentId"));
  EXPECT_EQ(j.at("parentComponentId"), "parent_comp");
  EXPECT_TRUE(j.contains("dependsOn"));
  EXPECT_EQ(j.at("dependsOn").size(), 2u);
}

TEST(XMedkitDtos, XMedkitAppIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitApp>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitApp>, "XMedkitApp");
}

TEST(XMedkitDtos, XMedkitAppRoundTrip) {
  dto::XMedkitApp app;
  dto::XMedkitRos2 r2;
  r2.node = "/sensors/camera";
  app.ros2 = r2;
  app.source = "runtime";
  app.is_online = true;
  app.component_id = "sensors_comp";

  const auto j = dto::JsonWriter<dto::XMedkitApp>::write(app);
  EXPECT_EQ(j.at("source"), "runtime");
  EXPECT_EQ(j.at("is_online"), true);
  EXPECT_EQ(j.at("component_id"), "sensors_comp");
  ASSERT_TRUE(j.contains("ros2"));
  EXPECT_EQ(j.at("ros2").at("node"), "/sensors/camera");
}

TEST(XMedkitDtos, XMedkitFunctionIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitFunction>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitFunction>, "XMedkitFunction");
}

TEST(XMedkitDtos, XMedkitFunctionHostsSerializedAsArray) {
  dto::XMedkitFunction func;
  func.source = "manifest";
  func.hosts = std::vector<std::string>{"app_a", "app_b"};
  func.description = "Navigation function";

  const auto j = dto::JsonWriter<dto::XMedkitFunction>::write(func);
  EXPECT_EQ(j.at("source"), "manifest");
  ASSERT_TRUE(j.contains("hosts"));
  EXPECT_EQ(j.at("hosts").size(), 2u);
  EXPECT_EQ(j.at("description"), "Navigation function");
}

TEST(XMedkitDtos, XMedkitCollectionIsDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::XMedkitCollection>);
  EXPECT_EQ(dto::dto_name<dto::XMedkitCollection>, "XMedkitCollection");
}

TEST(XMedkitDtos, XMedkitCollectionTotalCount) {
  dto::XMedkitCollection col;
  col.total_count = 42u;

  const auto j = dto::JsonWriter<dto::XMedkitCollection>::write(col);
  ASSERT_TRUE(j.contains("total_count"));
  EXPECT_EQ(j.at("total_count"), 42u);
  EXPECT_FALSE(j.contains("contributors"));
}

TEST(XMedkitDtos, AllXMedkitSchemasAreObjects) {
  const auto area_schema = dto::SchemaWriter<dto::XMedkitArea>::schema();
  const auto comp_schema = dto::SchemaWriter<dto::XMedkitComponent>::schema();
  const auto app_schema = dto::SchemaWriter<dto::XMedkitApp>::schema();
  const auto func_schema = dto::SchemaWriter<dto::XMedkitFunction>::schema();
  const auto coll_schema = dto::SchemaWriter<dto::XMedkitCollection>::schema();
  const auto ros2_schema = dto::SchemaWriter<dto::XMedkitRos2>::schema();

  EXPECT_EQ(area_schema.at("type"), "object");
  EXPECT_EQ(comp_schema.at("type"), "object");
  EXPECT_EQ(app_schema.at("type"), "object");
  EXPECT_EQ(func_schema.at("type"), "object");
  EXPECT_EQ(coll_schema.at("type"), "object");
  EXPECT_EQ(ros2_schema.at("type"), "object");
}

// =============================================================================
// All-DTO registry round-trip test
// =============================================================================

template <class Tuple, std::size_t I>
void check_one() {
  using D = std::tuple_element_t<I, Tuple>;
  EXPECT_FALSE(dto::dto_name<D>.empty()) << "DTO at index " << I;
  const auto schema = dto::SchemaWriter<D>::schema();
  EXPECT_EQ(schema.at("type"), "object") << dto::dto_name<D>;
  const D s = dto::make_sample<D>();
  const auto j = dto::JsonWriter<D>::write(s);
  EXPECT_TRUE(dto::JsonReader<D>::read(j).has_value()) << dto::dto_name<D>;
}

template <class Tuple, std::size_t... I>
void check_all(std::index_sequence<I...> /*seq*/) {
  (check_one<Tuple, I>(), ...);
}

TEST(DtoRegistry, EveryRegisteredDtoRoundTrips) {
  check_all<dto::AllDtos>(std::make_index_sequence<std::tuple_size_v<dto::AllDtos>>{});
}
