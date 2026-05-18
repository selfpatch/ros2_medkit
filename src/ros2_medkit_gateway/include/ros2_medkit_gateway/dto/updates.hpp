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

#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// UpdateList - response for GET /updates
//
// Wire shape: {"items": ["update_id_1", "update_id_2", ...]}
// The items array contains bare strings (update package IDs).
// =============================================================================
struct UpdateList {
  std::vector<std::string> items;
};

template <>
inline constexpr auto dto_fields<UpdateList> = std::make_tuple(field("items", &UpdateList::items));

template <>
inline constexpr std::string_view dto_name<UpdateList> = "UpdateList";

// =============================================================================
// UpdateSubProgress - a single sub-step progress entry.
//
// Wire shape (from update_status_to_json in update_types.hpp):
//   name     - sub-step name (required)
//   progress - sub-step progress percentage 0-100 (required)
//
// Nested inside UpdateStatus::sub_progress array.
// =============================================================================
struct UpdateSubProgress {
  std::string name;
  int progress{0};
};

template <>
inline constexpr auto dto_fields<UpdateSubProgress> =
    std::make_tuple(field("name", &UpdateSubProgress::name), field("progress", &UpdateSubProgress::progress));

template <>
inline constexpr std::string_view dto_name<UpdateSubProgress> = "UpdateSubProgress";

// =============================================================================
// XMedkitUpdate - typed x-medkit vendor extension on update status responses.
//
// Emitted by handle_get_status (and the SSE sampler via update_status_to_json).
// Wire keys (from update_status_to_json in update_types.hpp):
//
//   phase - internal lifecycle phase, distinguishes prepare-completed from
//           execute-completed (required; always emitted by update_status_to_json)
//           enum: none | preparing | prepared | executing | executed | failed | deleting
//
// Uses field_enum: phase is a RESPONSE-side field; the handler does NOT perform
// bespoke validation of the phase value in any request.
// =============================================================================
struct XMedkitUpdate {
  std::string phase;  // enum: kUpdatePhaseValues
};

template <>
inline constexpr auto dto_fields<XMedkitUpdate> =
    std::make_tuple(field_enum("phase", &XMedkitUpdate::phase, kUpdatePhaseValues));

template <>
inline constexpr std::string_view dto_name<XMedkitUpdate> = "XMedkitUpdate";

// =============================================================================
// UpdateStatus - response for GET /updates/{update_id}/status
//
// Wire shape (from update_status_to_json in update_types.hpp):
//   status       - SOVD update status enum (required)
//                  enum: pending | inProgress | completed | failed
//   progress     - optional overall progress percentage (0-100)
//   sub_progress - optional array of per-step progress entries
//   error        - optional error message string (set when status == failed)
//   x-medkit     - typed vendor extension (required; always emitted)
//
// status uses field_enum (response DTO, no bespoke handler-side range check).
// x-medkit is required: update_status_to_json always sets j["x-medkit"] unconditionally.
// =============================================================================
struct UpdateStatus {
  std::string status;           // enum: kUpdateStatusValues
  std::optional<int> progress;  // 0-100
  std::optional<std::vector<UpdateSubProgress>> sub_progress;
  std::optional<std::string> error;
  XMedkitUpdate x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<UpdateStatus> =
    std::make_tuple(field_enum("status", &UpdateStatus::status, kUpdateStatusValues),
                    field("progress", &UpdateStatus::progress), field("sub_progress", &UpdateStatus::sub_progress),
                    field("error", &UpdateStatus::error), field("x-medkit", &UpdateStatus::x_medkit));

template <>
inline constexpr std::string_view dto_name<UpdateStatus> = "UpdateStatus";

}  // namespace dto
}  // namespace ros2_medkit_gateway
