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

#include "ros2_medkit_gateway/http/handlers/lock_handlers.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

LockHandlers::LockHandlers(HandlerContext & ctx, LockManager * lock_manager) : ctx_(ctx), lock_manager_(lock_manager) {
}

// ============================================================================
// Private helpers
// ============================================================================

bool LockHandlers::check_locking_enabled(httplib::Response & res) {
  if (!lock_manager_) {
    HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Locking is not enabled on this gateway");
    return false;
  }
  return true;
}

std::optional<std::string> LockHandlers::require_client_id(const httplib::Request & req, httplib::Response & res) {
  auto client_id = req.get_header_value("X-Client-Id");
  if (client_id.empty()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing required X-Client-Id header",
                               json{{"details", "X-Client-Id header is required for lock operations"}});
    return std::nullopt;
  }
  return client_id;
}

std::string LockHandlers::format_expiration(std::chrono::steady_clock::time_point expires_at) {
  // Convert steady_clock to system_clock for wall-clock time formatting.
  // Compute the offset between the two clocks at the current instant,
  // then apply it to the steady_clock time_point.
  auto now_steady = std::chrono::steady_clock::now();
  auto now_system = std::chrono::system_clock::now();
  auto offset = expires_at - now_steady;
  auto system_time = now_system + std::chrono::duration_cast<std::chrono::system_clock::duration>(offset);

  auto time_t_val = std::chrono::system_clock::to_time_t(system_time);
  std::tm tm_val{};
  gmtime_r(&time_t_val, &tm_val);

  std::ostringstream oss;
  oss << std::put_time(&tm_val, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

/**
 * @brief Map internal LockManager error codes to SOVD standard error codes
 *
 * The LockManager uses descriptive internal codes (lock-conflict, lock-not-owner, etc.)
 * but SOVD spec requires standard error codes (invalid-request, forbidden, etc.).
 */
static std::string to_sovd_error_code(const std::string & lock_code) {
  if (lock_code == "lock-conflict" || lock_code == "lock-not-breakable") {
    return ERR_INVALID_REQUEST;
  }
  if (lock_code == "lock-not-owner") {
    return ERR_FORBIDDEN;
  }
  if (lock_code == "lock-not-found") {
    return ERR_RESOURCE_NOT_FOUND;
  }
  if (lock_code == "invalid-scope" || lock_code == "invalid-expiration") {
    return ERR_INVALID_PARAMETER;
  }
  return lock_code;  // Pass through if no mapping
}

json LockHandlers::lock_to_json(const LockInfo & lock, const std::string & client_id) {
  json result;
  result["id"] = lock.lock_id;
  result["owned"] = !client_id.empty() && lock.client_id == client_id;

  // scopes field is conditional - only present when lock has specific scopes
  if (!lock.scopes.empty()) {
    result["scopes"] = lock.scopes;
  }

  result["lock_expiration"] = format_expiration(lock.expires_at);
  return result;
}

// ============================================================================
// Handler implementations
// ============================================================================

void LockHandlers::handle_acquire_lock(const httplib::Request & req, httplib::Response & res) {
  if (!check_locking_enabled(res)) {
    return;
  }

  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Require X-Client-Id for acquire
    auto client_id_opt = require_client_id(req, res);
    if (!client_id_opt) {
      return;
    }
    const auto & client_id = *client_id_opt;

    // Validate entity exists and matches route type
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;
    }

    // Parse request body
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON in request body",
                                 json{{"details", e.what()}});
      return;
    }

    // Extract lock_expiration (required)
    if (!body.contains("lock_expiration") || !body["lock_expiration"].is_number_integer()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid lock_expiration",
                                 json{{"details", "lock_expiration must be a positive integer (seconds)"}});
      return;
    }
    int expiration_seconds = body["lock_expiration"].get<int>();
    if (expiration_seconds <= 0) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid lock_expiration",
                                 json{{"details", "lock_expiration must be a positive integer (seconds)"}});
      return;
    }

    // Extract scopes (optional)
    std::vector<std::string> scopes;
    if (body.contains("scopes") && body["scopes"].is_array()) {
      for (const auto & scope : body["scopes"]) {
        if (!scope.is_string()) {
          HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid scope value",
                                     json{{"details", "Each scope must be a string"}});
          return;
        }
        auto scope_str = scope.get<std::string>();
        if (valid_lock_scopes().find(scope_str) == valid_lock_scopes().end()) {
          HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Unknown lock scope: " + scope_str,
                                     json{{"details",
                                           "Valid scopes: data, operations, configurations, faults, modes, "
                                           "scripts, bulk-data"},
                                          {"invalid_scope", scope_str}});
          return;
        }
        scopes.push_back(scope_str);
      }
    }

    // Extract break_lock (optional, default false)
    bool break_lock = false;
    if (body.contains("break_lock") && body["break_lock"].is_boolean()) {
      break_lock = body["break_lock"].get<bool>();
    }

    // Acquire the lock
    auto result = lock_manager_->acquire(entity_id, client_id, scopes, expiration_seconds, break_lock);

    if (result.has_value()) {
      auto response = lock_to_json(*result, client_id);
      res.status = 201;
      res.set_content(response.dump(2), "application/json");
    } else {
      const auto & err = result.error();
      HandlerContext::send_error(res, err.status_code, to_sovd_error_code(err.code), err.message,
                                 err.existing_lock_id ? json{{"existing_lock_id", *err.existing_lock_id}} : json{});
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to acquire lock",
                               json{{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_acquire_lock for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void LockHandlers::handle_list_locks(const httplib::Request & req, httplib::Response & res) {
  if (!check_locking_enabled(res)) {
    return;
  }

  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Validate entity exists and matches route type
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;
    }

    // X-Client-Id is optional for list (used for "owned" field)
    auto client_id = req.get_header_value("X-Client-Id");

    // Get lock for this entity
    auto lock = lock_manager_->get_lock(entity_id);

    json response;
    response["items"] = json::array();

    if (lock) {
      response["items"].push_back(lock_to_json(*lock, client_id));
    }

    res.status = 200;
    HandlerContext::send_json(res, response);

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to list locks",
                               json{{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_locks for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void LockHandlers::handle_get_lock(const httplib::Request & req, httplib::Response & res) {
  if (!check_locking_enabled(res)) {
    return;
  }

  std::string entity_id;
  std::string lock_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    lock_id = req.matches[2];

    // Validate entity exists and matches route type
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;
    }

    // X-Client-Id is optional for get (used for "owned" field)
    auto client_id = req.get_header_value("X-Client-Id");

    // Look up lock by ID
    auto lock = lock_manager_->get_lock_by_id(lock_id);
    if (!lock || lock->entity_id != entity_id) {
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Lock not found",
                                 json{{"lock_id", lock_id}, {"entity_id", entity_id}});
      return;
    }

    res.status = 200;
    HandlerContext::send_json(res, lock_to_json(*lock, client_id));

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to get lock",
                               json{{"details", e.what()}, {"entity_id", entity_id}, {"lock_id", lock_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_lock for entity '%s', lock '%s': %s", entity_id.c_str(),
                 lock_id.c_str(), e.what());
  }
}

void LockHandlers::handle_extend_lock(const httplib::Request & req, httplib::Response & res) {
  if (!check_locking_enabled(res)) {
    return;
  }

  std::string entity_id;
  std::string lock_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    lock_id = req.matches[2];

    // Require X-Client-Id for extend
    auto client_id_opt = require_client_id(req, res);
    if (!client_id_opt) {
      return;
    }
    const auto & client_id = *client_id_opt;

    // Validate entity exists and matches route type
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;
    }

    // Verify lock exists and belongs to this entity
    auto lock = lock_manager_->get_lock_by_id(lock_id);
    if (!lock || lock->entity_id != entity_id) {
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Lock not found",
                                 json{{"lock_id", lock_id}, {"entity_id", entity_id}});
      return;
    }

    // Parse request body for new expiration
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON in request body",
                                 json{{"details", e.what()}});
      return;
    }

    if (!body.contains("lock_expiration") || !body["lock_expiration"].is_number_integer()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid lock_expiration",
                                 json{{"details", "lock_expiration must be a positive integer (seconds)"}});
      return;
    }
    int additional_seconds = body["lock_expiration"].get<int>();
    if (additional_seconds <= 0) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid lock_expiration",
                                 json{{"details", "lock_expiration must be a positive integer (seconds)"}});
      return;
    }

    // Extend the lock
    auto result = lock_manager_->extend(entity_id, client_id, additional_seconds);

    if (result.has_value()) {
      res.status = 204;
    } else {
      const auto & err = result.error();
      HandlerContext::send_error(res, err.status_code, to_sovd_error_code(err.code), err.message);
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to extend lock",
                               json{{"details", e.what()}, {"entity_id", entity_id}, {"lock_id", lock_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_extend_lock for entity '%s', lock '%s': %s",
                 entity_id.c_str(), lock_id.c_str(), e.what());
  }
}

void LockHandlers::handle_release_lock(const httplib::Request & req, httplib::Response & res) {
  if (!check_locking_enabled(res)) {
    return;
  }

  std::string entity_id;
  std::string lock_id;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    lock_id = req.matches[2];

    // Require X-Client-Id for release
    auto client_id_opt = require_client_id(req, res);
    if (!client_id_opt) {
      return;
    }
    const auto & client_id = *client_id_opt;

    // Validate entity exists and matches route type
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;
    }

    // Verify lock exists and belongs to this entity
    auto lock = lock_manager_->get_lock_by_id(lock_id);
    if (!lock || lock->entity_id != entity_id) {
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Lock not found",
                                 json{{"lock_id", lock_id}, {"entity_id", entity_id}});
      return;
    }

    // Release the lock
    auto result = lock_manager_->release(entity_id, client_id);

    if (result.has_value()) {
      res.status = 204;
    } else {
      const auto & err = result.error();
      HandlerContext::send_error(res, err.status_code, to_sovd_error_code(err.code), err.message);
    }

  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to release lock",
                               json{{"details", e.what()}, {"entity_id", entity_id}, {"lock_id", lock_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_release_lock for entity '%s', lock '%s': %s",
                 entity_id.c_str(), lock_id.c_str(), e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
