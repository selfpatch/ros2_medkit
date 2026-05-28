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

#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

#include "ros2_medkit_gateway/dto/aggregation.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/data.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/dto/json_reader.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace dto = ros2_medkit_gateway::dto;

namespace {

// Test DTO used as the item type in custom-XMedkit collection instantiations.
struct TestItem {
  std::string id;
};

// Custom x-medkit shape registered as a DTO so SchemaWriter emits a $ref to it.
struct CustomXMedkit {
  std::optional<std::string> tenant;
  std::optional<int> peer_count;
};

}  // namespace

template <>
[[maybe_unused]] inline constexpr auto dto::dto_fields<TestItem> = std::make_tuple(dto::field("id", &TestItem::id));

template <>
[[maybe_unused]] inline constexpr std::string_view dto::dto_name<TestItem> = "TestItem";

template <>
[[maybe_unused]] inline constexpr auto dto::dto_fields<CustomXMedkit> =
    std::make_tuple(dto::field("tenant", &CustomXMedkit::tenant), dto::field("peer_count", &CustomXMedkit::peer_count));

template <>
[[maybe_unused]] inline constexpr std::string_view dto::dto_name<CustomXMedkit> = "CustomXMedkit";

template <>
[[maybe_unused]] inline constexpr std::string_view dto::dto_name<dto::Collection<TestItem, CustomXMedkit>> =
    "TestItemListCustom";

// ---------------------------------------------------------------------------
// Collection<T, XMedkit> backward compatibility
// ---------------------------------------------------------------------------

TEST(CollectionTemplate, DefaultXMedkitIsXMedkitCollection) {
  // A 1-arg instantiation must be identical to an explicit 2-arg instantiation
  // with XMedkitCollection as the second parameter.
  using Default = dto::Collection<dto::DataItem>;
  using Explicit = dto::Collection<dto::DataItem, dto::XMedkitCollection>;
  EXPECT_TRUE((std::is_same_v<Default, Explicit>));
}

TEST(CollectionTemplate, DefaultCollectionEmitsXMedkitCollectionSchemaRef) {
  // The schema for a default Collection's x-medkit field must $ref the
  // XMedkitCollection schema, not any other XMedkit type. Because the field is
  // std::optional<XMedkitCollection>, SchemaWriter emits the OpenAPI 3.1
  // anyOf+null idiom; the $ref lives on the non-null branch.
  const auto schema = dto::SchemaWriter<dto::Collection<dto::DataItem>>::schema();
  ASSERT_TRUE(schema.at("properties").contains("x-medkit"));
  const auto & xm = schema.at("properties").at("x-medkit");
  ASSERT_TRUE(xm.contains("anyOf")) << xm.dump();
  EXPECT_EQ(xm.at("anyOf").at(0).at("$ref"), "#/components/schemas/XMedkitCollection");
  EXPECT_EQ(xm.at("anyOf").at(1).at("type"), "null");
}

TEST(CollectionTemplate, DefaultCollectionWireOutputUnchanged) {
  // Backward compatibility: a 1-arg Collection with no x-medkit set must
  // serialize to just {"items": []} (no other fields).
  dto::Collection<dto::DataItem> coll;
  const auto j = dto::JsonWriter<dto::Collection<dto::DataItem>>::write(coll);
  EXPECT_TRUE(j.contains("items"));
  EXPECT_TRUE(j.at("items").is_array());
  EXPECT_EQ(j.at("items").size(), 0u);
  EXPECT_FALSE(j.contains("x-medkit"));
  EXPECT_FALSE(j.contains("_links"));
}

// ---------------------------------------------------------------------------
// Collection<T, XMedkit> with custom x-medkit type
// ---------------------------------------------------------------------------

TEST(CollectionTemplate, CustomXMedkitSchemaUsesCustomRef) {
  // A Collection<T, CustomXMedkit> must emit a $ref to CustomXMedkit in its
  // x-medkit property, NOT to XMedkitCollection. Optional field -> anyOf+null
  // idiom; the $ref lives on the non-null branch.
  const auto schema = dto::SchemaWriter<dto::Collection<TestItem, CustomXMedkit>>::schema();
  ASSERT_TRUE(schema.at("properties").contains("x-medkit"));
  const auto & xm = schema.at("properties").at("x-medkit");
  ASSERT_TRUE(xm.contains("anyOf")) << xm.dump();
  EXPECT_EQ(xm.at("anyOf").at(0).at("$ref"), "#/components/schemas/CustomXMedkit");
  EXPECT_EQ(xm.at("anyOf").at(1).at("type"), "null");
}

TEST(CollectionTemplate, CustomXMedkitRoundTrips) {
  dto::Collection<TestItem, CustomXMedkit> coll;
  TestItem item{};
  item.id = "abc";
  coll.items.push_back(item);
  CustomXMedkit xm;
  xm.tenant = "tenant-a";
  xm.peer_count = 3;
  coll.x_medkit = xm;

  const auto j = dto::JsonWriter<dto::Collection<TestItem, CustomXMedkit>>::write(coll);
  ASSERT_TRUE(j.contains("items"));
  EXPECT_EQ(j.at("items").size(), 1u);
  EXPECT_EQ(j.at("items").at(0).at("id"), "abc");
  ASSERT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j.at("x-medkit").at("tenant"), "tenant-a");
  EXPECT_EQ(j.at("x-medkit").at("peer_count"), 3);

  const auto roundtrip = dto::JsonReader<dto::Collection<TestItem, CustomXMedkit>>::read(j);
  ASSERT_TRUE(roundtrip.has_value());
  ASSERT_EQ(roundtrip->items.size(), 1u);
  EXPECT_EQ(roundtrip->items[0].id, "abc");
  ASSERT_TRUE(roundtrip->x_medkit.has_value());
  EXPECT_EQ(roundtrip->x_medkit->tenant, "tenant-a");
  EXPECT_EQ(roundtrip->x_medkit->peer_count, 3);
}

// ---------------------------------------------------------------------------
// XMedkitCollection fan-out observability fields
// ---------------------------------------------------------------------------

TEST(XMedkitCollectionFanOut, PartialAndFailedPeersRoundTrip) {
  dto::XMedkitCollection xm;
  xm.partial = true;
  xm.failed_peers = std::vector<std::string>{"http://peer1", "http://peer2"};

  const auto j = dto::JsonWriter<dto::XMedkitCollection>::write(xm);
  EXPECT_EQ(j.at("partial"), true);
  ASSERT_TRUE(j.contains("failed_peers"));
  EXPECT_EQ(j.at("failed_peers").size(), 2u);
  EXPECT_EQ(j.at("failed_peers").at(0), "http://peer1");

  const auto roundtrip = dto::JsonReader<dto::XMedkitCollection>::read(j);
  ASSERT_TRUE(roundtrip.has_value());
  ASSERT_TRUE(roundtrip->partial.has_value());
  EXPECT_TRUE(*roundtrip->partial);
  ASSERT_TRUE(roundtrip->failed_peers.has_value());
  ASSERT_EQ(roundtrip->failed_peers->size(), 2u);
  EXPECT_EQ((*roundtrip->failed_peers)[0], "http://peer1");
}

TEST(XMedkitCollectionFanOut, OmittedFanOutFieldsStayOffTheWire) {
  // Backward compatibility: when none of the new fan-out fields are set, the
  // emitted JSON must not contain them (existing peers must not see new keys).
  dto::XMedkitCollection xm;
  xm.total_count = 5u;

  const auto j = dto::JsonWriter<dto::XMedkitCollection>::write(xm);
  EXPECT_TRUE(j.contains("total_count"));
  EXPECT_FALSE(j.contains("partial"));
  EXPECT_FALSE(j.contains("failed_peers"));
  EXPECT_FALSE(j.contains("peer_dropped_items"));
}

TEST(XMedkitCollectionFanOut, PeerDroppedItemsRoundTrip) {
  dto::DroppedItem d1;
  d1.peer = "http://peer1";
  d1.reason = "field missing: id";
  d1.source_id = "unknown";

  dto::XMedkitCollection xm;
  xm.peer_dropped_items = std::vector<dto::DroppedItem>{d1};

  const auto j = dto::JsonWriter<dto::XMedkitCollection>::write(xm);
  ASSERT_TRUE(j.contains("peer_dropped_items"));
  ASSERT_EQ(j.at("peer_dropped_items").size(), 1u);
  EXPECT_EQ(j.at("peer_dropped_items").at(0).at("peer"), "http://peer1");
  EXPECT_EQ(j.at("peer_dropped_items").at(0).at("reason"), "field missing: id");

  const auto roundtrip = dto::JsonReader<dto::XMedkitCollection>::read(j);
  ASSERT_TRUE(roundtrip.has_value());
  ASSERT_TRUE(roundtrip->peer_dropped_items.has_value());
  ASSERT_EQ(roundtrip->peer_dropped_items->size(), 1u);
  EXPECT_EQ((*roundtrip->peer_dropped_items)[0].peer, "http://peer1");
  EXPECT_EQ((*roundtrip->peer_dropped_items)[0].reason, "field missing: id");
  EXPECT_EQ((*roundtrip->peer_dropped_items)[0].source_id, "unknown");
}

// ---------------------------------------------------------------------------
// DroppedItem DTO
// ---------------------------------------------------------------------------

TEST(DroppedItemDto, IsRegisteredDto) {
  EXPECT_TRUE(dto::is_dto_v<dto::DroppedItem>);
  EXPECT_EQ(dto::dto_name<dto::DroppedItem>, "DroppedItem");
}

TEST(DroppedItemDto, RoundTrips) {
  dto::DroppedItem item;
  item.peer = "http://peer1";
  item.reason = "JsonReader: expected a string";
  item.source_id = "fault_42";

  const auto j = dto::JsonWriter<dto::DroppedItem>::write(item);
  EXPECT_EQ(j.at("peer"), "http://peer1");
  EXPECT_EQ(j.at("reason"), "JsonReader: expected a string");
  EXPECT_EQ(j.at("source_id"), "fault_42");

  const auto roundtrip = dto::JsonReader<dto::DroppedItem>::read(j);
  ASSERT_TRUE(roundtrip.has_value());
  EXPECT_EQ(roundtrip->peer, "http://peer1");
  EXPECT_EQ(roundtrip->reason, "JsonReader: expected a string");
  EXPECT_EQ(roundtrip->source_id, "fault_42");
}

TEST(DroppedItemDto, SchemaMarksAllFieldsRequired) {
  const auto schema = dto::SchemaWriter<dto::DroppedItem>::schema();
  EXPECT_EQ(schema.at("type"), "object");
  ASSERT_TRUE(schema.contains("required"));
  EXPECT_EQ(schema.at("required").size(), 3u);
}

// ---------------------------------------------------------------------------
// Per-domain XMedkit types carry the new optional fan-out fields
// ---------------------------------------------------------------------------

TEST(FaultListXMedkitFanOut, PeerDroppedItemsRoundTrip) {
  dto::DroppedItem d1;
  d1.peer = "http://peer1";
  d1.reason = "bad JSON";
  d1.source_id = "fault_99";

  dto::FaultListXMedkit xm;
  xm.count = 7;
  xm.peer_dropped_items = std::vector<dto::DroppedItem>{d1};

  const auto j = dto::JsonWriter<dto::FaultListXMedkit>::write(xm);
  ASSERT_TRUE(j.contains("peer_dropped_items"));
  EXPECT_EQ(j.at("peer_dropped_items").at(0).at("peer"), "http://peer1");

  const auto roundtrip = dto::JsonReader<dto::FaultListXMedkit>::read(j);
  ASSERT_TRUE(roundtrip.has_value());
  ASSERT_TRUE(roundtrip->peer_dropped_items.has_value());
  EXPECT_EQ((*roundtrip->peer_dropped_items)[0].peer, "http://peer1");
}

TEST(DataListXMedkitFanOut, NewFieldsPresentAndOptional) {
  dto::DataListXMedkit xm;
  // No fan-out fields set => none appear on the wire (backward compat).
  const auto j = dto::JsonWriter<dto::DataListXMedkit>::write(xm);
  EXPECT_FALSE(j.contains("partial"));
  EXPECT_FALSE(j.contains("failed_peers"));
  EXPECT_FALSE(j.contains("peer_dropped_items"));

  // With all three set => they appear and round-trip.
  xm.partial = true;
  xm.failed_peers = std::vector<std::string>{"http://p1"};
  dto::DroppedItem d;
  d.peer = "http://p1";
  d.reason = "missing field: id";
  d.source_id = "";
  xm.peer_dropped_items = std::vector<dto::DroppedItem>{d};

  const auto j2 = dto::JsonWriter<dto::DataListXMedkit>::write(xm);
  EXPECT_EQ(j2.at("partial"), true);
  EXPECT_EQ(j2.at("failed_peers").at(0), "http://p1");
  EXPECT_EQ(j2.at("peer_dropped_items").at(0).at("reason"), "missing field: id");

  const auto roundtrip = dto::JsonReader<dto::DataListXMedkit>::read(j2);
  ASSERT_TRUE(roundtrip.has_value());
  EXPECT_TRUE(*roundtrip->partial);
  ASSERT_EQ(roundtrip->failed_peers->size(), 1u);
  ASSERT_EQ(roundtrip->peer_dropped_items->size(), 1u);
}

// ---------------------------------------------------------------------------
// Existing Collection<X> instantiations remain unchanged
// ---------------------------------------------------------------------------

TEST(CollectionBackCompat, ExistingNamedCollectionsStillResolveDtoName) {
  // The pre-existing dto_name specializations (written with a single template
  // argument) must continue to resolve under the 2-parameter template.
  EXPECT_EQ(dto::dto_name<dto::Collection<dto::AreaListItem>>, "AreaList");
  EXPECT_EQ(dto::dto_name<dto::Collection<dto::ComponentListItem>>, "ComponentList");
  EXPECT_EQ(dto::dto_name<dto::Collection<dto::AppListItem>>, "AppList");
  EXPECT_EQ(dto::dto_name<dto::Collection<dto::FunctionListItem>>, "FunctionList");
}

TEST(CollectionBackCompat, DataItemCollectionSchemaUnchanged) {
  // The wire format of an existing Collection<DataItem> must be byte-identical
  // when no x-medkit / no links are set.
  dto::Collection<dto::DataItem> coll;
  dto::DataItem it;
  it.id = "x";
  it.name = "n";
  it.category = "currentData";
  coll.items.push_back(it);

  const auto j = dto::JsonWriter<dto::Collection<dto::DataItem>>::write(coll);
  EXPECT_EQ(j.at("items").size(), 1u);
  EXPECT_EQ(j.at("items").at(0).at("id"), "x");
  EXPECT_FALSE(j.contains("x-medkit"));
  EXPECT_FALSE(j.contains("_links"));
}
