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

#include <variant>

#include "ros2_medkit_gateway/dto/config.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for configuration (parameter) REST API endpoints.
 *
 * Provides handlers for:
 * - GET    /{entity-path}/configurations              - List parameters
 * - GET    /{entity-path}/configurations/{param_name} - Read parameter value
 * - PUT    /{entity-path}/configurations/{param_name} - Set parameter value
 * - DELETE /{entity-path}/configurations/{param_name} - Reset parameter to default
 * - DELETE /{entity-path}/configurations              - Reset all parameters
 *
 * PR-403 commit 26: 5 config routes migrated to the typed RouteRegistry API.
 * The list endpoint uses the typed `fan_out_collection<ConfigurationMetaData>`
 * (commit 7) for peer aggregation: per-item wire shape is now enforced by
 * `JsonReader<ConfigurationMetaData>` on every contribution, closing the
 * issue #338 gap on this endpoint by lifting the per-item schema enforcement
 * from "spec only" to "compile-time + runtime contract".
 *
 * The delete-all endpoint uses `del_alternates<NoContent,
 * ConfigurationDeleteMultiStatus>` so the framework picks 204 on full success
 * or 207 on partial success based on which variant alternative the handler
 * returned. Wire format is byte-identical for both branches.
 */
class ConfigHandlers {
 public:
  /// Construct configuration handlers with shared context.
  explicit ConfigHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /// GET /{entity-path}/configurations - list all parameters with typed
  /// per-item parsing and typed fan-out to peers.
  http::Result<dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit>>
  list_configurations(const http::TypedRequest & req);

  /// GET /{entity-path}/configurations/{param_name} - read a single parameter.
  http::Result<dto::ConfigurationReadValue> get_configuration(const http::TypedRequest & req);

  /// PUT /{entity-path}/configurations/{param_name} - set a single parameter.
  http::Result<dto::ConfigurationReadValue> set_configuration(const http::TypedRequest & req,
                                                              dto::ConfigurationWriteRequest body);

  /// DELETE /{entity-path}/configurations/{param_name} - reset a single
  /// parameter to its default value, returning 204 No Content on success.
  http::Result<http::NoContent> delete_configuration(const http::TypedRequest & req);

  /// DELETE /{entity-path}/configurations - reset all parameters. Returns
  /// `NoContent` (204) on full success, or `ConfigurationDeleteMultiStatus`
  /// (207) when one or more per-node reset operations failed.
  http::Result<std::variant<http::NoContent, dto::ConfigurationDeleteMultiStatus>>
  delete_all_configurations(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
