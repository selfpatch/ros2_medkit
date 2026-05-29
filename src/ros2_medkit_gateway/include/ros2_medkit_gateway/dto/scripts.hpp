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
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// HateoasLinks - typed sub-struct for the script-list `_links` envelope.
//
// Wire shape (from the legacy raw-JSON `response["_links"] = {{"self", ...},
// {"parent", ...}}` in handle_list_scripts):
//   self    - canonical URI of the listing endpoint (required when emitted)
//   parent  - canonical URI of the owning entity (optional)
//
// Used as the typed `_links` field on `dto::ScriptList`. Distinct from the
// free-form `Collection<T>::links` (kept for discovery aggregations whose
// `_links` shape is genuinely dynamic) so that script listings travel through
// the typed serializer end-to-end.
// =============================================================================
struct HateoasLinks {
  std::string self;
  std::optional<std::string> parent;
};

template <>
inline constexpr auto dto_fields<HateoasLinks> =
    std::make_tuple(field("self", &HateoasLinks::self), field("parent", &HateoasLinks::parent));

template <>
inline constexpr std::string_view dto_name<HateoasLinks> = "HateoasLinks";

// =============================================================================
// ScriptMetadata - single item in the script list / GET script response.
//
// Wire shape (from script_metadata_schema() + script_info_to_json()):
//   id                     - script ID (required)
//   name                   - script name (required)
//   description            - human-readable description (optional)
//   href                   - canonical resource URI (optional)
//   managed                - true if managed by the system (optional)
//   proximity_proof_required - true if proximity proof is required (optional)
//   parameters_schema      - optional free-form JSON schema for execution params
//                            (null when absent, kept as nlohmann::json to allow
//                             any JSON structure)
// =============================================================================
struct ScriptMetadata {
  std::string id;
  std::string name;
  std::optional<std::string> description;
  std::optional<std::string> href;
  std::optional<bool> managed;
  std::optional<bool> proximity_proof_required;
  std::optional<nlohmann::json> parameters_schema;  // free-form: runtime-determined schema
};

template <>
inline constexpr auto dto_fields<ScriptMetadata> =
    std::make_tuple(field("id", &ScriptMetadata::id), field("name", &ScriptMetadata::name),
                    field("description", &ScriptMetadata::description), field("href", &ScriptMetadata::href),
                    field("managed", &ScriptMetadata::managed),
                    field("proximity_proof_required", &ScriptMetadata::proximity_proof_required),
                    field("parameters_schema", &ScriptMetadata::parameters_schema));

template <>
inline constexpr std::string_view dto_name<ScriptMetadata> = "ScriptMetadata";

// =============================================================================
// Collection<ScriptMetadata> - named "ScriptMetadataList".
//
// Wire shape: {"items": [<ScriptMetadata>, ...]}
//
// Retained in the registry because the generic Collection<T> visitor stamps it
// onto the OpenAPI schema. The wire-facing list endpoint actually emits the
// `ScriptList` wrapper below so the `_links` envelope is typed instead of
// free-form JSON.
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<ScriptMetadata>> = "ScriptMetadataList";

// =============================================================================
// ScriptList - GET /{entity}/scripts response with typed HATEOAS envelope.
//
// Wire shape (matches the legacy handle_list_scripts output byte-for-byte):
//   items   - array of ScriptMetadata (required, may be empty)
//   _links  - typed HateoasLinks block (optional; current handler always emits
//             it but the field is optional so future endpoints that share this
//             wrapper can omit it without re-shaping the wire body)
//
// The wrapper exists because `Collection<T>::links` is free-form
// `nlohmann::json`. Migrating that to a typed sub-struct on the generic
// Collection would touch every list endpoint; for the script-list case we
// instead use a domain-specific wrapper so the typed surface is end-to-end.
// =============================================================================
struct ScriptList {
  std::vector<ScriptMetadata> items;
  std::optional<HateoasLinks> links;  // wire key: "_links"
};

template <>
inline constexpr auto dto_fields<ScriptList> =
    std::make_tuple(field("items", &ScriptList::items), field("_links", &ScriptList::links));

template <>
inline constexpr std::string_view dto_name<ScriptList> = "ScriptList";

// =============================================================================
// ScriptExecution - execution status response.
//
// Used by:
//   - POST /{entity}/scripts/{script_id}/executions (202 - execution started)
//   - GET  /{entity}/scripts/{script_id}/executions/{execution_id}
//   - PUT  /{entity}/scripts/{script_id}/executions/{execution_id} (200 - control)
//
// Wire shape (from script_execution_schema() + execution_info_to_json()):
//   id           - execution ID (required)
//   status       - execution status string (required)
//   progress     - optional integer progress value (0-100; matches ExecutionInfo)
//   started_at   - optional ISO 8601 start timestamp string
//   completed_at - optional ISO 8601 completion timestamp string
//   parameters   - optional free-form output parameters JSON object
//   error        - optional free-form error detail JSON object
//
// status is NOT field_enum here: the handler passes the value through from
// the backend without bespoke range-checking - enum is informational only.
// parameters and error are kept as nlohmann::json because they carry
// runtime-determined structures from the script backend.
// =============================================================================
struct ScriptExecution {
  std::string id;
  std::string status;
  std::optional<int> progress;
  std::optional<std::string> started_at;
  std::optional<std::string> completed_at;
  std::optional<nlohmann::json> parameters;  // free-form: runtime output parameters
  std::optional<nlohmann::json> error;       // free-form: backend error detail
};

template <>
inline constexpr auto dto_fields<ScriptExecution> =
    std::make_tuple(field("id", &ScriptExecution::id), field("status", &ScriptExecution::status),
                    field("progress", &ScriptExecution::progress), field("started_at", &ScriptExecution::started_at),
                    field("completed_at", &ScriptExecution::completed_at),
                    field("parameters", &ScriptExecution::parameters), field("error", &ScriptExecution::error));

template <>
inline constexpr std::string_view dto_name<ScriptExecution> = "ScriptExecution";

// =============================================================================
// ScriptUploadResponse - 201 response body for POST /{entity}/scripts.
//
// Wire shape (from script_upload_response_schema() + handle_upload_script()):
//   id   - assigned script ID (required)
//   name - script name (required)
// =============================================================================
struct ScriptUploadResponse {
  std::string id;
  std::string name;
};

template <>
inline constexpr auto dto_fields<ScriptUploadResponse> =
    std::make_tuple(field("id", &ScriptUploadResponse::id), field("name", &ScriptUploadResponse::name));

template <>
inline constexpr std::string_view dto_name<ScriptUploadResponse> = "ScriptUploadResponse";

// =============================================================================
// ScriptControlRequest - PUT request body for execution control.
//
// Wire shape (from script_control_request_schema() + handle_control_execution()):
//   action - control action to apply (required)
//            built-in backend: "stop" | "forced_termination"
//
// Uses plain field() (NOT field_enum): control_execution forwards `action`
// verbatim to the ScriptProvider, which may be a plugin backend supporting
// actions beyond the built-in stop/forced_termination (e.g. pause/resume).
// Constraining the value at parse time would block those plugins at the
// gateway. The handler validates presence (ERR_INVALID_REQUEST when missing);
// the provider validates the value. (Same reasoning as
// ExecutionUpdateRequest.capability.)
// =============================================================================
struct ScriptControlRequest {
  std::string action;  // built-in backend: stop | forced_termination; plugins may extend
};

template <>
inline constexpr auto dto_fields<ScriptControlRequest> =
    std::make_tuple(field("action", &ScriptControlRequest::action));

template <>
inline constexpr std::string_view dto_name<ScriptControlRequest> = "ScriptControlRequest";

}  // namespace dto
}  // namespace ros2_medkit_gateway
