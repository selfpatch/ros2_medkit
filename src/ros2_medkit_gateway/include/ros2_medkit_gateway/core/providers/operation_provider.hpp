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
#include <string>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/operations.hpp"

namespace ros2_medkit_gateway {

enum class OperationProviderError {
  EntityNotFound,
  OperationNotFound,
  InvalidParameters,
  TransportError,
  Timeout,
  Rejected,
  Internal
};

struct OperationProviderErrorInfo {
  OperationProviderError code;
  std::string message;
  int http_status{500};  ///< Suggested HTTP status code
};

/**
 * @brief Provider interface for entity operations
 *
 * Typed provider interface for plugins that serve SOVD operations
 * (GET /{entity_type}/{id}/operations, POST /{entity_type}/{id}/operations/{name}).
 * Per-entity routing like DataProvider.
 *
 * @par Thread safety
 * All methods may be called concurrently. Implementations must synchronize.
 *
 * @see GatewayPlugin for the base class all plugins must also implement
 * @see DataProvider for the data counterpart
 */
class OperationProvider {
 public:
  virtual ~OperationProvider() = default;

  /// List available operations for an entity.
  ///
  /// Returns a typed `Collection<OperationItem>`. The wire shape is unchanged
  /// from the pre-typed ABI (`{"items": [...]}` plus optional `x-medkit` /
  /// `_links`); JsonWriter<Collection<OperationItem>> emits identical bytes.
  ///
  /// @param entity_id SOVD entity ID
  virtual tl::expected<dto::Collection<dto::OperationItem>, OperationProviderErrorInfo>
  list_operations(const std::string & entity_id) = 0;

  /// Get a specific operation by name.
  ///
  /// Returns the bare `OperationItem` (without the `{"item": ...}` envelope);
  /// the gateway handler wraps it into the SOVD `OperationDetail` response on
  /// the wire.
  ///
  /// @param entity_id SOVD entity ID
  /// @param operation_name Operation name
  /// @note Default implementation calls list_operations + linear scan.
  ///       Override for O(1) lookup in plugins with many operations.
  virtual tl::expected<dto::OperationItem, OperationProviderErrorInfo>
  get_operation(const std::string & entity_id, const std::string & operation_name) {
    auto result = list_operations(entity_id);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    for (const auto & item : result->items) {
      if (item.id == operation_name) {
        return item;
      }
    }
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::OperationNotFound, "Operation not found", 404});
  }

  /// Execute an operation.
  ///
  /// Returns a typed `OperationExecutionResult` envelope around the
  /// plugin-defined response object. JsonWriter emits `content` verbatim, so
  /// the wire bytes are byte-identical to the pre-typed `nlohmann::json` ABI.
  /// The payload shape is plugin-determined (ROS service / action result,
  /// OPC-UA method result, ...) and the OpenAPI schema is opaque
  /// (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID
  /// @param operation_name Operation name (e.g., "session_control")
  /// @param parameters JSON parameters from request body
  virtual tl::expected<dto::OperationExecutionResult, OperationProviderErrorInfo>
  execute_operation(const std::string & entity_id, const std::string & operation_name,
                    const nlohmann::json & parameters) = 0;
};

}  // namespace ros2_medkit_gateway
