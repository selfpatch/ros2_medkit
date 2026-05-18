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

#pragma once

#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/sample.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// XMedkitDataItem - x-medkit vendor extension emitted on data collection list
// items (handle_list_data per-topic), on data write responses
// (handle_put_data_item), and on data read responses (handle_get_data_item).
//
// Wire keys for list items:
//   ros2.topic     - ROS 2 topic path
//   ros2.direction - topic direction: "publish" | "subscribe" | "both"
//                    (maps to XMedkitRos2::direction)
//   ros2.type      - ROS 2 message type string
//   type_info      - dynamic type schema + default_value (free-form JSON;
//                    only present when type introspection succeeds)
//
// Additional keys for write responses (handle_put_data_item):
//   entity_id         - SOVD entity ID
//   status            - publish result status
//   publisher_created - true when a new publisher was created
//
// Additional keys for read responses (handle_get_data_item):
//   timestamp        - sample timestamp in nanoseconds since epoch (int64)
//   publisher_count  - number of publishers on the topic at sample time (int64)
//   subscriber_count - number of subscribers on the topic at sample time (int64)
// =============================================================================
struct XMedkitDataItem {
  std::optional<XMedkitRos2> ros2;
  std::optional<nlohmann::json> type_info;       // free-form: dynamic ROS IDL schema
  std::optional<std::string> entity_id;          // SOVD entity ID (write + read responses)
  std::optional<nlohmann::json> status;          // publish result / sample status
  std::optional<bool> publisher_created;         // true when publisher created (write responses)
  std::optional<std::int64_t> timestamp;         // sample timestamp in ns (read responses)
  std::optional<std::int64_t> publisher_count;   // publisher count at sample time (read responses)
  std::optional<std::int64_t> subscriber_count;  // subscriber count at sample time (read responses)
};

template <>
inline constexpr auto dto_fields<XMedkitDataItem> =
    std::make_tuple(field("ros2", &XMedkitDataItem::ros2), field("type_info", &XMedkitDataItem::type_info),
                    field("entity_id", &XMedkitDataItem::entity_id), field("status", &XMedkitDataItem::status),
                    field("publisher_created", &XMedkitDataItem::publisher_created),
                    field("timestamp", &XMedkitDataItem::timestamp),
                    field("publisher_count", &XMedkitDataItem::publisher_count),
                    field("subscriber_count", &XMedkitDataItem::subscriber_count));

template <>
inline constexpr std::string_view dto_name<XMedkitDataItem> = "XMedkitDataItem";

// =============================================================================
// DataItem - single item emitted in handle_list_data "items" array.
//
// Wire keys (from data_handlers.cpp handle_list_data per-topic construction):
//   id       - ROS 2 topic path (used as round-trip ID for GET/PUT)
//   name     - same as id (topic path)
//   category - always "currentData"
//   x-medkit - optional vendor extension with ros2 topology + type info
// =============================================================================
struct DataItem {
  std::string id;
  std::string name;
  std::string category;                     // always "currentData"
  std::optional<XMedkitDataItem> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<DataItem> =
    std::make_tuple(field("id", &DataItem::id), field("name", &DataItem::name), field("category", &DataItem::category),
                    field("x-medkit", &DataItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<DataItem> = "DataItem";

// =============================================================================
// DataListXMedkit - x-medkit vendor extension on the data collection list
// response (the top-level response object from handle_list_data).
//
// Wire keys (from data_handlers.cpp handle_list_data response-level XMedkit):
//   entity_id          - SOVD entity ID being queried
//   aggregated         - true when cache reports aggregated data
//   aggregation_sources - list of source IDs when aggregated
//   aggregation_level  - aggregation level string when aggregated
//   total_count        - total number of items in the response
// =============================================================================
struct DataListXMedkit {
  std::optional<std::string> entity_id;
  std::optional<bool> aggregated;
  std::optional<std::vector<std::string>> aggregation_sources;
  std::optional<std::string> aggregation_level;
  std::optional<std::size_t> total_count;
};

template <>
inline constexpr auto dto_fields<DataListXMedkit> =
    std::make_tuple(field("entity_id", &DataListXMedkit::entity_id), field("aggregated", &DataListXMedkit::aggregated),
                    field("aggregation_sources", &DataListXMedkit::aggregation_sources),
                    field("aggregation_level", &DataListXMedkit::aggregation_level),
                    field("total_count", &DataListXMedkit::total_count));

template <>
inline constexpr std::string_view dto_name<DataListXMedkit> = "DataListXMedkit";

// =============================================================================
// DataWriteRequest - PUT request body for /{entity}/data/{id}.
// Parsed by handle_put_data_item via parse_body<DataWriteRequest>.
//
// Wire keys (SOVD convention, from data_handlers.cpp handle_put_data_item):
//   type  - ROS 2 message type string (e.g. "std_msgs/msg/Float32"), required
//   data  - message value to publish (free-form: any JSON object), required
// =============================================================================
struct DataWriteRequest {
  std::string type;
  nlohmann::json data;  // free-form: message payload
};

template <>
inline constexpr auto dto_fields<DataWriteRequest> =
    std::make_tuple(field("type", &DataWriteRequest::type), field("data", &DataWriteRequest::data));

template <>
inline constexpr std::string_view dto_name<DataWriteRequest> = "DataWriteRequest";

// =============================================================================
// Collection<DataItem> - named "DataList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<DataItem>> = "DataList";

// =============================================================================
// dto_sample specialization for DataWriteRequest.
//
// The generic sample path produces nlohmann::json{} (null) for bare json
// fields, which the round-trip reader treats as missing (null == absent for
// required fields).  Provide an explicit sample with a non-null value to
// ensure EveryRegisteredDtoRoundTrips passes.
// =============================================================================
template <>
struct dto_sample<DataWriteRequest> {
  static DataWriteRequest make() {
    DataWriteRequest obj;
    obj.type = "std_msgs/msg/Float32";
    obj.data = nlohmann::json{{"data", 0.0}};  // non-null: representative message value
    return obj;
  }
};

}  // namespace dto
}  // namespace ros2_medkit_gateway
