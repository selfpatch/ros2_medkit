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

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// XMedkitOperationItem - x-medkit vendor extension on OperationItem responses
// (handle_list_operations / handle_get_operation).
//
//   ros2        - nested ROS 2 metadata sub-object (service|action path, type,
//                 kind)
//   entity_id   - SOVD entity the operation belongs to
//   source      - always "ros2_medkit_gateway"
//   type_info   - optional dynamic ROS IDL schema JSON (request/response for
//                 services; goal/result/feedback for actions); kept as
//                 nlohmann::json because the structure is runtime-determined
//                 by type introspection and cannot be statically typed
// =============================================================================
struct XMedkitOperationItem {
  std::optional<XMedkitRos2> ros2;
  std::optional<std::string> entity_id;
  std::optional<std::string> source;
  std::optional<nlohmann::json> type_info;  // free-form: dynamic ROS IDL schemas
};

template <>
inline constexpr auto dto_fields<XMedkitOperationItem> =
    std::make_tuple(field("ros2", &XMedkitOperationItem::ros2), field("entity_id", &XMedkitOperationItem::entity_id),
                    field("source", &XMedkitOperationItem::source),
                    field("type_info", &XMedkitOperationItem::type_info));

template <>
inline constexpr std::string_view dto_name<XMedkitOperationItem> = "XMedkitOperationItem";

// =============================================================================
// XMedkitOperationExecution - x-medkit vendor extension on OperationExecution
// responses (handle_get_execution).
//
//   goal_id     - UUID of the tracked ROS 2 action goal
//   ros2_status - raw ROS 2 goal status string (accepted|executing|canceling|
//                 succeeded|canceled|aborted|unknown)
//   ros2        - nested ROS 2 metadata sub-object (action path + type)
// =============================================================================
struct XMedkitOperationExecution {
  std::string goal_id;
  std::optional<std::string> ros2_status;  // raw ROS 2 enum string
  std::optional<XMedkitRos2> ros2;
};

template <>
inline constexpr auto dto_fields<XMedkitOperationExecution> =
    std::make_tuple(field("goal_id", &XMedkitOperationExecution::goal_id),
                    field("ros2_status", &XMedkitOperationExecution::ros2_status),
                    field("ros2", &XMedkitOperationExecution::ros2));

template <>
inline constexpr std::string_view dto_name<XMedkitOperationExecution> = "XMedkitOperationExecution";

// =============================================================================
// OperationItem - single item emitted in handle_list_operations "items" array
// and wrapped inside OperationDetail.
//
// Wire keys (from handle_list_operations / handle_get_operation):
//   id                     - operation name (required)
//   name                   - operation name (required)
//   proximity_proof_required - bool (required, always false for ROS 2)
//   asynchronous_execution - bool (required; false for services, true for actions)
//   x-medkit               - typed vendor extension; carries ros2.{service|action,
//                            type,kind}, entity_id, source, and optional type_info
// =============================================================================
struct OperationItem {
  std::string id;
  std::string name;
  bool proximity_proof_required{false};
  bool asynchronous_execution{false};
  std::optional<XMedkitOperationItem> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<OperationItem> =
    std::make_tuple(field("id", &OperationItem::id), field("name", &OperationItem::name),
                    field("proximity_proof_required", &OperationItem::proximity_proof_required),
                    field("asynchronous_execution", &OperationItem::asynchronous_execution),
                    field("x-medkit", &OperationItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<OperationItem> = "OperationItem";

// =============================================================================
// OperationDetail - response shape for GET /{entity}/{id}/operations/{op_id}.
//
// Wire keys (from handle_get_operation):
//   item - required; the full OperationItem for this operation
// =============================================================================
struct OperationDetail {
  OperationItem item;
};

template <>
inline constexpr auto dto_fields<OperationDetail> = std::make_tuple(field("item", &OperationDetail::item));

template <>
inline constexpr std::string_view dto_name<OperationDetail> = "OperationDetail";

// =============================================================================
// OperationExecution - execution status response shape.
//
// Used by:
//   - GET  /{entity}/{id}/operations/{op_id}/executions/{exec_id}
//     (handle_get_execution): status + capability + optional parameters +
//     optional x-medkit (goal_id / ros2_status / ros2.action / ros2.type)
//   - POST /{entity}/{id}/operations/{op_id}/executions (202 for actions):
//     id + status
//   - PUT  /{entity}/{id}/operations/{op_id}/executions/{exec_id} (202 stop):
//     id + status
//
// Wire keys:
//   id          - execution / goal UUID (optional; set on 202 responses)
//   status      - execution status enum (required)
//   capability  - control capability in context (optional; "execute" on GET)
//   parameters  - dynamic ROS payload (optional; last_feedback on GET)
//   x-medkit    - vendor extension (optional; goal tracking data on GET)
// =============================================================================
struct OperationExecution {
  std::optional<std::string> id;
  std::string status;  // enum: pending|running|completed|failed
  std::optional<std::string> capability;
  std::optional<nlohmann::json> parameters;           // free-form: last_feedback / result
  std::optional<XMedkitOperationExecution> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<OperationExecution> =
    std::make_tuple(field("id", &OperationExecution::id),
                    field_enum("status", &OperationExecution::status, kOperationExecutionStatusValues),
                    field("capability", &OperationExecution::capability),
                    field("parameters", &OperationExecution::parameters),
                    field("x-medkit", &OperationExecution::x_medkit));

template <>
inline constexpr std::string_view dto_name<OperationExecution> = "OperationExecution";

// =============================================================================
// ExecutionUpdateRequest - PUT request body for execution control.
//
// Wire keys (from handle_update_execution / execution_update_request_schema):
//   capability - required; one of: stop | execute | freeze | reset
// =============================================================================
struct ExecutionUpdateRequest {
  std::string capability;  // enum: stop|execute|freeze|reset
};

template <>
inline constexpr auto dto_fields<ExecutionUpdateRequest> =
    std::make_tuple(field_enum("capability", &ExecutionUpdateRequest::capability, kExecutionCapabilityValues));

template <>
inline constexpr std::string_view dto_name<ExecutionUpdateRequest> = "ExecutionUpdateRequest";

// =============================================================================
// Collection<OperationItem> - named "OperationList"
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<OperationItem>> = "OperationList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
