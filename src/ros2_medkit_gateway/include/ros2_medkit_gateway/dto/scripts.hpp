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

namespace ros2_medkit_gateway {
namespace dto {

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
// =============================================================================
template <>
inline constexpr std::string_view dto_name<Collection<ScriptMetadata>> = "ScriptMetadataList";

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
//   progress     - optional float progress (0.0-1.0)
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
  std::optional<double> progress;
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
//            enum: "stop" | "forced_termination"
//
// Uses field_enum: the control handler validates only that "action" is present
// and non-empty (ERR_INVALID_REQUEST for missing). It does NOT perform bespoke
// value-range validation with ERR_INVALID_PARAMETER; parse_body provides the
// enum check instead.
// =============================================================================
struct ScriptControlRequest {
  std::string action;  // enum: stop | forced_termination
};

template <>
inline constexpr auto dto_fields<ScriptControlRequest> =
    std::make_tuple(field_enum("action", &ScriptControlRequest::action, kScriptControlActionValues));

template <>
inline constexpr std::string_view dto_name<ScriptControlRequest> = "ScriptControlRequest";

}  // namespace dto
}  // namespace ros2_medkit_gateway
