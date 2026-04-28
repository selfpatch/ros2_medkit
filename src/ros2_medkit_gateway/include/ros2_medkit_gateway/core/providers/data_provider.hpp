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

enum class DataProviderError {
  EntityNotFound,
  ResourceNotFound,
  ReadOnly,
  WriteOnly,
  TransportError,
  Timeout,
  InvalidValue,
  Internal
};

struct DataProviderErrorInfo {
  DataProviderError code;
  std::string message;
  int http_status{500};  ///< Suggested HTTP status code
};

/**
 * @brief Provider interface for entity data resources
 *
 * Typed provider interface for plugins that serve SOVD data resources
 * (GET /{entity_type}/{id}/data, GET /{entity_type}/{id}/data/{name}).
 * Unlike LogProvider/ScriptProvider (singletons), multiple DataProvider
 * plugins can coexist - each handles its own set of entities.
 *
 * Entity ownership is determined by IntrospectionProvider: entities
 * created by a plugin's introspect() are routed to that plugin's
 * DataProvider.
 *
 * @par Thread safety
 * All methods may be called from multiple HTTP handler threads concurrently.
 * Implementations must provide their own synchronization.
 *
 * @see GatewayPlugin for the base class all plugins must also implement
 * @see OperationProvider for the operations counterpart
 */
class DataProvider {
 public:
  virtual ~DataProvider() = default;

  /// List available data resources for an entity
  /// @param entity_id SOVD entity ID (e.g., "openbsw_demo_ecu")
  /// @return JSON with {"items": [...]} array of data resource descriptors
  virtual tl::expected<nlohmann::json, DataProviderErrorInfo> list_data(const std::string & entity_id) = 0;

  /// Read a specific data resource
  /// @param entity_id SOVD entity ID
  /// @param resource_name Data resource name (e.g., "hardcoded_data")
  /// @return JSON response body for the data resource
  virtual tl::expected<nlohmann::json, DataProviderErrorInfo> read_data(const std::string & entity_id,
                                                                        const std::string & resource_name) = 0;

  /// Write a data resource value
  /// @param entity_id SOVD entity ID
  /// @param resource_name Data resource name
  /// @param value JSON value to write
  /// @return JSON response body confirming the write
  virtual tl::expected<nlohmann::json, DataProviderErrorInfo>
  write_data(const std::string & entity_id, const std::string & resource_name, const nlohmann::json & value) = 0;
};

}  // namespace ros2_medkit_gateway
