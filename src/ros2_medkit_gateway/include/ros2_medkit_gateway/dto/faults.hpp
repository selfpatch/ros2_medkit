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
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// FaultListItem - flat fault list item emitted by fault_msg_conversions.cpp
// (fault_to_json shape), wrapped in Collection<FaultListItem> for list endpoints.
//
// Wire keys (exact, from fault_msg_conversions.cpp):
//   fault_code, severity, description, first_occurred, last_occurred,
//   occurrence_count, status, reporting_sources, severity_label
// =============================================================================
struct FaultListItem {
  std::string fault_code;
  int64_t severity{0};
  std::optional<std::string> description;
  std::optional<double> first_occurred;
  std::optional<double> last_occurred;
  std::optional<int64_t> occurrence_count;
  std::string status;
  std::optional<std::vector<std::string>> reporting_sources;
  std::optional<std::string> severity_label;  // enum: INFO|WARN|ERROR|CRITICAL|UNKNOWN
};

template <>
inline constexpr auto dto_fields<FaultListItem> = std::make_tuple(
    field("fault_code", &FaultListItem::fault_code), field("severity", &FaultListItem::severity),
    field("description", &FaultListItem::description), field("first_occurred", &FaultListItem::first_occurred),
    field("last_occurred", &FaultListItem::last_occurred), field("occurrence_count", &FaultListItem::occurrence_count),
    field("status", &FaultListItem::status), field("reporting_sources", &FaultListItem::reporting_sources),
    field("severity_label", &FaultListItem::severity_label));

template <>
inline constexpr std::string_view dto_name<FaultListItem> = "FaultListItem";

// =============================================================================
// FaultStatus - SOVD status sub-object inside FaultDetail.item
//
// Wire keys (from build_status_object in fault_handlers.cpp):
//   aggregatedStatus (required, enum), testFailed, confirmedDTC, pendingDTC
// =============================================================================
struct FaultStatus {
  std::string aggregated_status;             // wire key: "aggregatedStatus"
  std::optional<std::string> test_failed;    // wire key: "testFailed"
  std::optional<std::string> confirmed_dtc;  // wire key: "confirmedDTC"
  std::optional<std::string> pending_dtc;    // wire key: "pendingDTC"
};

template <>
inline constexpr auto dto_fields<FaultStatus> =
    std::make_tuple(field_enum("aggregatedStatus", &FaultStatus::aggregated_status, kFaultAggregatedStatusValues),
                    field("testFailed", &FaultStatus::test_failed), field("confirmedDTC", &FaultStatus::confirmed_dtc),
                    field("pendingDTC", &FaultStatus::pending_dtc));

template <>
inline constexpr std::string_view dto_name<FaultStatus> = "FaultStatus";

// =============================================================================
// FaultItem - SOVD "item" sub-object inside FaultDetail
//
// Wire keys (from build_sovd_fault_response):
//   code (required), fault_name (optional), severity (required), status (required)
// =============================================================================
struct FaultItem {
  std::string code;
  std::optional<std::string> fault_name;
  int64_t severity{0};
  FaultStatus status;
};

template <>
inline constexpr auto dto_fields<FaultItem> =
    std::make_tuple(field("code", &FaultItem::code), field("fault_name", &FaultItem::fault_name),
                    field("severity", &FaultItem::severity), field("status", &FaultItem::status));

template <>
inline constexpr std::string_view dto_name<FaultItem> = "FaultItem";

// =============================================================================
// FaultEnvironmentData - SOVD "environment_data" sub-object inside FaultDetail
//
// Wire keys (from build_sovd_fault_response):
//   extended_data_records (free-form JSON object, optional),
//   snapshots (free-form JSON array - discriminated freeze_frame|rosbag, optional)
//
// Both fields are genuinely free-form: snapshots carry a runtime type
// discriminator ("type"/"snapshot_type") with per-variant optional fields.
// =============================================================================
struct FaultEnvironmentData {
  std::optional<nlohmann::json> extended_data_records;
  std::optional<nlohmann::json> snapshots;
};

template <>
inline constexpr auto dto_fields<FaultEnvironmentData> =
    std::make_tuple(field("extended_data_records", &FaultEnvironmentData::extended_data_records),
                    field("snapshots", &FaultEnvironmentData::snapshots));

template <>
inline constexpr std::string_view dto_name<FaultEnvironmentData> = "FaultEnvironmentData";

// =============================================================================
// FaultXMedkit - x-medkit vendor extension inside FaultDetail
//
// Wire keys (from build_sovd_fault_response):
//   occurrence_count, reporting_sources, severity_label, status_raw
// =============================================================================
struct FaultXMedkit {
  std::optional<int64_t> occurrence_count;
  std::optional<std::vector<std::string>> reporting_sources;
  std::optional<std::string> severity_label;
  std::optional<std::string> status_raw;
};

template <>
inline constexpr auto dto_fields<FaultXMedkit> =
    std::make_tuple(field("occurrence_count", &FaultXMedkit::occurrence_count),
                    field("reporting_sources", &FaultXMedkit::reporting_sources),
                    field("severity_label", &FaultXMedkit::severity_label),
                    field("status_raw", &FaultXMedkit::status_raw));

template <>
inline constexpr std::string_view dto_name<FaultXMedkit> = "FaultXMedkit";

// =============================================================================
// FaultDetail - SOVD nested fault detail response
// Emitted by FaultHandlers::handle_get_fault via build_sovd_fault_response.
//
// Wire keys:
//   item (required), environment_data (required), x-medkit (optional)
// =============================================================================
struct FaultDetail {
  FaultItem item;
  FaultEnvironmentData environment_data;
  std::optional<FaultXMedkit> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FaultDetail> =
    std::make_tuple(field("item", &FaultDetail::item), field("environment_data", &FaultDetail::environment_data),
                    field("x-medkit", &FaultDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<FaultDetail> = "FaultDetail";

// =============================================================================
// Collection<FaultListItem> - named "FaultList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<FaultListItem>> = "FaultList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
