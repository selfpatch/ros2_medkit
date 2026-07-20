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

#include "ros2_medkit_gateway/dto/data.hpp"

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

  /// List available data resources for an entity.
  ///
  /// Returns a typed `DataListResult` envelope around the plugin-defined
  /// response body. JsonWriter emits `content` verbatim, so the wire bytes are
  /// byte-identical to the pre-typed `nlohmann::json` ABI. The payload shape
  /// is plugin-determined (typically `{"items": [...]}` with per-item fields
  /// varying across backends - ROS topic metadata, OPC-UA node attributes,
  /// UDS DID descriptors, ...). The OpenAPI schema is opaque
  /// (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID (e.g., "openbsw_demo_ecu")
  virtual tl::expected<dto::DataListResult, DataProviderErrorInfo> list_data(const std::string & entity_id) = 0;

  /// Read a specific data resource.
  ///
  /// Returns a typed `DataValue` envelope around the plugin-defined response
  /// body. JsonWriter emits `content` verbatim, so the wire bytes are
  /// byte-identical to the pre-typed `nlohmann::json` ABI. The payload shape
  /// is runtime-dependent (live ROS message, OPC-UA value with metadata, UDS
  /// DID payload, ...) and the OpenAPI schema is opaque
  /// (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID
  /// @param resource_name Data resource name (e.g., "hardcoded_data")
  virtual tl::expected<dto::DataValue, DataProviderErrorInfo> read_data(const std::string & entity_id,
                                                                        const std::string & resource_name) = 0;

  /// Write a data resource value.
  ///
  /// Returns a typed `DataWriteResult` envelope around the plugin-defined
  /// response body. JsonWriter emits `content` verbatim, so the wire bytes are
  /// byte-identical to the pre-typed `nlohmann::json` ABI. The payload shape
  /// is plugin-determined (typically `{"status": "ok"}` or similar) and the
  /// OpenAPI schema is opaque (`x-medkit-opaque:true`).
  ///
  /// @param entity_id SOVD entity ID
  /// @param resource_name Data resource name
  /// @param value JSON value to write
  virtual tl::expected<dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & entity_id, const std::string & resource_name, const nlohmann::json & value) = 0;

  /// Whether this provider actually has data for the given entity.
  ///
  /// A plugin implements one DataProvider for every entity it owns, but not
  /// every owned entity is data-bearing (e.g. a grouping Component, or an
  /// App that only surfaces faults). Override this so PluginManager stops
  /// routing - and the gateway stops advertising the `data` capability - for
  /// entities this provider cannot actually serve. Defaults to true, which
  /// preserves current behavior for providers whose owned entities are all
  /// data-bearing.
  virtual bool has_data(const std::string & /*entity_id*/) const {
    return true;
  }
};

}  // namespace ros2_medkit_gateway
