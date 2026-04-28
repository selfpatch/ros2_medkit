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

  /// List faults for an entity
  /// @param entity_id SOVD entity ID
  /// @return JSON with {"items": [...]} array of fault descriptors
  virtual tl::expected<nlohmann::json, FaultProviderErrorInfo> list_faults(const std::string & entity_id) = 0;

  /// Get a specific fault with environment data
  /// @param entity_id SOVD entity ID
  /// @param fault_code Fault code (e.g., DTC identifier)
  /// @return JSON response with fault detail + environment data
  virtual tl::expected<nlohmann::json, FaultProviderErrorInfo> get_fault(const std::string & entity_id,
                                                                         const std::string & fault_code) = 0;

  /// Clear a fault
  /// @param entity_id SOVD entity ID
  /// @param fault_code Fault code to clear
  /// @return JSON response confirming the clear
  virtual tl::expected<nlohmann::json, FaultProviderErrorInfo> clear_fault(const std::string & entity_id,
                                                                           const std::string & fault_code) = 0;
};

}  // namespace ros2_medkit_gateway
