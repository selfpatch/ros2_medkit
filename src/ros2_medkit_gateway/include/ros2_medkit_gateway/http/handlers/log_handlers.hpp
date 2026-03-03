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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

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
 * Supported entity types: components, apps.
 *
 * Log entries are sourced from the /rosout ring buffer by default.
 * If a LogProvider plugin is registered, queries are delegated to it.
 *
 * @par Component vs App semantics
 * Components use prefix matching: all nodes whose FQN starts with the
 * component namespace are included. Apps use exact FQN matching.
 */
class LogHandlers {
 public:
  /**
   * @brief Construct log handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit LogHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /{entity-path}/logs - query log entries for an entity.
   *
   * Query parameters:
   *   severity  - Optional minimum severity filter (debug/info/warning/error/fatal).
   *               Stricter of this and entity config severity_filter is applied.
   *   context   - Optional substring filter applied to the log entry's node name.
   */
  void handle_get_logs(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /{entity-path}/logs/configuration - get log configuration.
   */
  void handle_get_logs_configuration(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle PUT /{entity-path}/logs/configuration - update log configuration.
   *
   * Body (JSON, all fields optional):
   *   severity_filter  - Minimum severity to return in query results (debug/info/warning/error/fatal)
   *   max_entries      - Maximum number of log entries to return per query (> 0)
   */
  void handle_put_logs_configuration(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
