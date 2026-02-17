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

#include <httplib.h>

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/subscription_manager.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief HTTP handlers for cyclic subscription CRUD and SSE streaming.
 *
 * Implements SOVD cyclic subscription endpoints:
 * - POST /{entity}/cyclic-subscriptions — create subscription
 * - GET /{entity}/cyclic-subscriptions — list subscriptions
 * - GET /{entity}/cyclic-subscriptions/{id} — get subscription
 * - PUT /{entity}/cyclic-subscriptions/{id} — update subscription
 * - DELETE /{entity}/cyclic-subscriptions/{id} — delete subscription
 * - GET /{entity}/cyclic-subscriptions/{id}/events — SSE stream
 */
class CyclicSubscriptionHandlers {
 public:
  CyclicSubscriptionHandlers(HandlerContext & ctx, SubscriptionManager & sub_mgr);

  /// POST /{entity}/cyclic-subscriptions — create subscription
  void handle_create(const httplib::Request & req, httplib::Response & res);

  /// GET /{entity}/cyclic-subscriptions — list all subscriptions for entity
  void handle_list(const httplib::Request & req, httplib::Response & res);

  /// GET /{entity}/cyclic-subscriptions/{id} — get single subscription
  void handle_get(const httplib::Request & req, httplib::Response & res);

  /// PUT /{entity}/cyclic-subscriptions/{id} — update subscription
  void handle_update(const httplib::Request & req, httplib::Response & res);

  /// DELETE /{entity}/cyclic-subscriptions/{id} — delete subscription
  void handle_delete(const httplib::Request & req, httplib::Response & res);

  /// GET /{entity}/cyclic-subscriptions/{id}/events — SSE event stream
  void handle_events(const httplib::Request & req, httplib::Response & res);

  /// Convert subscription info to JSON response
  static nlohmann::json subscription_to_json(const CyclicSubscriptionInfo & info, const std::string & event_source);

 private:
  /// Build event_source URI from subscription info
  static std::string build_event_source(const CyclicSubscriptionInfo & info);

  /// Extract entity type string ("apps" or "components") from request path
  static std::string extract_entity_type(const httplib::Request & req);

  /// Parse resource URI to extract topic name. Returns topic or error.
  static tl::expected<std::string, std::string> parse_resource_uri(const std::string & resource);

  HandlerContext & ctx_;
  SubscriptionManager & sub_mgr_;

  /// Keepalive interval for SSE streams
  static constexpr int kKeepaliveIntervalSec = 15;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
