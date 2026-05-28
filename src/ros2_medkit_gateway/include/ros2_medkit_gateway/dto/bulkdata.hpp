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

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// BulkDataCategoryList - response for GET /{entity}/bulk-data
//
// Wire shape: {"items": ["rosbags", "cat1", ...]}
// The items array contains bare strings (category names).
// =============================================================================
struct BulkDataCategoryList {
  std::vector<std::string> items;
};

template <>
inline constexpr auto dto_fields<BulkDataCategoryList> = std::make_tuple(field("items", &BulkDataCategoryList::items));

template <>
inline constexpr std::string_view dto_name<BulkDataCategoryList> = "BulkDataCategoryList";

// =============================================================================
// BulkDataDescriptor - one downloadable file descriptor.
//
// Emitted by:
//   list_descriptors  -> array items inside {"items": [...]}
//   upload (201)      -> single descriptor response
//
// Wire keys (from bulkdata_handlers.cpp):
//   id              - unique file identifier (required)
//   name            - human-readable filename / label (required)
//   mimetype        - MIME type of the file (required)
//   size            - byte count (required)
//   creation_date   - ISO 8601 timestamp string (required)
//   description     - optional human-readable description
//   x-medkit        - optional open vendor extension object; for rosbags:
//                     {fault_code, duration_sec, format}; for user uploads:
//                     arbitrary metadata JSON object set by the uploader.
// =============================================================================
struct BulkDataDescriptor {
  std::string id;
  std::string name;
  std::string mimetype;
  uint64_t size{0};
  std::string creation_date;
  std::optional<std::string> description;
  std::optional<nlohmann::json> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<BulkDataDescriptor> =
    std::make_tuple(field("id", &BulkDataDescriptor::id), field("name", &BulkDataDescriptor::name),
                    field("mimetype", &BulkDataDescriptor::mimetype), field("size", &BulkDataDescriptor::size),
                    field("creation_date", &BulkDataDescriptor::creation_date),
                    field("description", &BulkDataDescriptor::description),
                    field("x-medkit", &BulkDataDescriptor::x_medkit));

template <>
inline constexpr std::string_view dto_name<BulkDataDescriptor> = "BulkDataDescriptor";

// =============================================================================
// Collection<BulkDataDescriptor> - named "BulkDataDescriptorList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<BulkDataDescriptor>> = "BulkDataDescriptorList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
