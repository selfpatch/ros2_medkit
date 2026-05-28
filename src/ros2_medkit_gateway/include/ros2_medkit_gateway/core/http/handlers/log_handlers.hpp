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

#include "ros2_medkit_gateway/dto/logs.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for communication log REST API endpoints.
 *
 * Provides handlers for:
 * - GET /{entity-path}/logs              - Query log entries for an entity
 * - GET /{entity-path}/logs/configuration - Get log configuration for an entity
 * - PUT /{entity-path}/logs/configuration - Update log configuration for an entity
 *
 * Supported entity types: components, apps, areas, functions.
 *
 * Log entries are sourced from the /rosout ring buffer by default.
 * If a LogProvider plugin is registered, queries are delegated to it.
 *
 * @par Component vs App semantics
 * Components and areas use prefix matching: all nodes whose FQN starts
 * with the entity namespace are included. Apps use exact FQN matching.
 * Functions aggregate logs from all hosted apps.
 *
 * PR-403 commit 23: 3 log routes migrated to typed `Result<TResponse>` shape.
 * The list endpoint uses the typed `fan_out_collection<LogEntry>` from commit 7
 * for peer aggregation instead of the legacy `merge_peer_items` raw-JSON mutator.
 */
class LogHandlers {
 public:
  /// Construct log handlers with shared context.
  explicit LogHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /// GET /{entity-path}/logs - query log entries for an entity.
  ///
  /// Query parameters:
  ///   severity  - Optional minimum severity filter (debug/info/warning/error/fatal).
  ///               Stricter of this and entity config severity_filter is applied.
  ///   context   - Optional substring filter applied to the log entry's node name.
  http::Result<dto::Collection<dto::LogEntry, dto::LogListXMedkit>> get_logs(const http::TypedRequest & req);

  /// GET /{entity-path}/logs/configuration - get log configuration.
  http::Result<dto::LogConfiguration> get_logs_configuration(const http::TypedRequest & req);

  /// PUT /{entity-path}/logs/configuration - update log configuration.
  ///
  /// Body (JSON, all fields optional):
  ///   severity_filter  - Minimum severity to return in query results (debug/info/warning/error/fatal)
  ///   max_entries      - Maximum number of log entries to return per query (> 0)
  http::Result<http::NoContent> put_logs_configuration(const http::TypedRequest & req, dto::LogConfiguration body);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
