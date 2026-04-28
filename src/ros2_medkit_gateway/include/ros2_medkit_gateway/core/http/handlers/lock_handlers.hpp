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

#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for SOVD entity locking REST API endpoints.
 *
 * Provides handlers for:
 * - POST   /{entity_type}/{entity_id}/locks                - Acquire lock
 * - GET    /{entity_type}/{entity_id}/locks                - List locks
 * - GET    /{entity_type}/{entity_id}/locks/{lock_id}      - Get lock
 * - PUT    /{entity_type}/{entity_id}/locks/{lock_id}      - Extend lock
 * - DELETE /{entity_type}/{entity_id}/locks/{lock_id}      - Release lock
 *
 * Locking is supported for components and apps only (per SOVD spec).
 */
class LockHandlers {
 public:
  /**
   * @brief Construct lock handlers with shared context and lock manager.
   * @param ctx The shared handler context
   * @param lock_manager Pointer to the LockManager (may be nullptr if locking disabled)
   */
  LockHandlers(HandlerContext & ctx, LockManager * lock_manager);

  /**
   * @brief Handle POST /{entity_type}/{entity_id}/locks - acquire a lock.
   *
   * Request body: {"scopes": [...], "lock_expiration": 300, "break_lock": false}
   * Requires X-Client-Id header.
   * Returns 201 with lock info on success.
   */
  void handle_acquire_lock(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /{entity_type}/{entity_id}/locks - list locks on entity.
   *
   * X-Client-Id header is optional (used to determine "owned" field).
   * Returns 200 with {"items": [...]}.
   */
  void handle_list_locks(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /{entity_type}/{entity_id}/locks/{lock_id} - get lock details.
   *
   * X-Client-Id header is optional (used to determine "owned" field).
   * Returns 200 with lock info, or 404 if not found.
   */
  void handle_get_lock(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle PUT /{entity_type}/{entity_id}/locks/{lock_id} - extend lock.
   *
   * Request body: {"lock_expiration": 300}
   * Requires X-Client-Id header.
   * Returns 204 on success, 403 if not owner, 404 if not found.
   */
  void handle_extend_lock(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /{entity_type}/{entity_id}/locks/{lock_id} - release lock.
   *
   * Requires X-Client-Id header.
   * Returns 204 on success, 403 if not owner, 404 if not found.
   */
  void handle_release_lock(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Format a LockInfo as SOVD-compliant JSON
   * @param lock Lock information
   * @param client_id Optional client ID for "owned" field
   * @return JSON object with lock details
   */
  static nlohmann::json lock_to_json(const LockInfo & lock, const std::string & client_id = "");

 private:
  HandlerContext & ctx_;
  LockManager * lock_manager_;

  /// Check that locking is enabled, send 501 if not. Returns true if OK.
  bool check_locking_enabled(httplib::Response & res);

  /// Extract and validate X-Client-Id header. Returns client_id or empty on error (error sent).
  std::optional<std::string> require_client_id(const httplib::Request & req, httplib::Response & res);

  /// Format a time_point as ISO 8601 UTC string
  static std::string format_expiration(std::chrono::steady_clock::time_point expires_at);
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
