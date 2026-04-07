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

  /// List available operations for an entity
  /// @param entity_id SOVD entity ID
  /// @return JSON with {"items": [...]} array of operation descriptors
  virtual tl::expected<nlohmann::json, OperationProviderErrorInfo> list_operations(const std::string & entity_id) = 0;

  /// Get a specific operation by name
  /// @param entity_id SOVD entity ID
  /// @param operation_name Operation name
  /// @return JSON with operation detail, or OperationNotFound error
  /// @note Default implementation calls list_operations + linear scan.
  ///       Override for O(1) lookup in plugins with many operations.
  virtual tl::expected<nlohmann::json, OperationProviderErrorInfo> get_operation(const std::string & entity_id,
                                                                                 const std::string & operation_name) {
    auto result = list_operations(entity_id);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    if (result->contains("items") && (*result)["items"].is_array()) {
      for (const auto & item : (*result)["items"]) {
        if (item.value("id", "") == operation_name) {
          return item;
        }
      }
    }
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::OperationNotFound, "Operation not found", 404});
  }

  /// Execute an operation
  /// @param entity_id SOVD entity ID
  /// @param operation_name Operation name (e.g., "session_control")
  /// @param parameters JSON parameters from request body
  /// @return JSON response body with operation result
  virtual tl::expected<nlohmann::json, OperationProviderErrorInfo>
  execute_operation(const std::string & entity_id, const std::string & operation_name,
                    const nlohmann::json & parameters) = 0;
};

}  // namespace ros2_medkit_gateway
