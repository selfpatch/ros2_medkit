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

#include <string>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/dto/faults.hpp"

namespace ros2_medkit_gateway {

enum class FaultProviderError { EntityNotFound, FaultNotFound, TransportError, Timeout, Internal };

struct FaultProviderErrorInfo {
  FaultProviderError code;
  std::string message;
  int http_status{500};  ///< Suggested HTTP status code
};

/**
 * @brief Provider interface for entity fault/DTC resources
 *
 * Typed provider interface for plugins that serve SOVD faults
 * (GET /{entity_type}/{id}/faults, GET /{entity_type}/{id}/faults/{code}).
 * Per-entity routing like DataProvider.
 *
 * Plugins implementing this interface query their backend (e.g., UDS
 * ReadDTCInformation 0x19) on demand and return faults in SOVD format.
 *
 * @par Thread safety
 * All methods may be called concurrently. Implementations must synchronize.
 *
 * @see GatewayPlugin for the base class all plugins must also implement
 * @see DataProvider for the data counterpart
 */
class FaultProvider {
 public:
  virtual ~FaultProvider() = default;

  /// List faults for an entity.
  ///
  /// Returns a typed `FaultListResult` envelope around the plugin-defined
  /// response body. JsonWriter emits `content` verbatim, so the wire bytes are
  /// byte-identical to the pre-typed `nlohmann::json` ABI. The payload shape
  /// is plugin-determined (typically `{"items": [...]}` with per-item fields
  /// varying across backends - UDS DTC records, OPC-UA alarm metadata, vendor
  /// extensions, ...). The OpenAPI schema is opaque
  /// (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID
  virtual tl::expected<dto::FaultListResult, FaultProviderErrorInfo> list_faults(const std::string & entity_id) = 0;

  /// Get a specific fault with environment data.
  ///
  /// Returns a typed `FaultDetailResult` envelope around the plugin-defined
  /// response body. JsonWriter emits `content` verbatim, so the wire bytes are
  /// byte-identical to the pre-typed `nlohmann::json` ABI. The payload shape
  /// is runtime-dependent (UDS environment records, OPC-UA condition state,
  /// vendor extended status) and the OpenAPI schema is opaque
  /// (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID
  /// @param fault_code Fault code (e.g., DTC identifier)
  virtual tl::expected<dto::FaultDetailResult, FaultProviderErrorInfo> get_fault(const std::string & entity_id,
                                                                                 const std::string & fault_code) = 0;

  /// Clear a fault.
  ///
  /// Returns a typed `FaultClearResult` envelope around the plugin-defined
  /// response body. JsonWriter emits `content` verbatim, so the wire bytes are
  /// byte-identical to the pre-typed `nlohmann::json` ABI. The payload shape
  /// is plugin-determined (typically `{"code": ..., "cleared": true}` or
  /// `{"status": "ok"}`) and the OpenAPI schema is opaque
  /// (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID
  /// @param fault_code Fault code to clear
  virtual tl::expected<dto::FaultClearResult, FaultProviderErrorInfo> clear_fault(const std::string & entity_id,
                                                                                  const std::string & fault_code) = 0;
};

}  // namespace ros2_medkit_gateway
