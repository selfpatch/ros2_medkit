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

#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/core/subscription_transport.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// Result of parsing a SOVD resource URI for subscription
struct ParsedResourceUri {
  std::string entity_type;  // "apps" or "components"
  std::string entity_id;
  std::string collection;     // "data", "faults", "x-medkit-metrics", etc.
  std::string resource_path;  // path within collection (may be empty)
};

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
  CyclicSubscriptionHandlers(HandlerContext & ctx, SubscriptionManager & sub_mgr,
                             ResourceSamplerRegistry & sampler_registry, TransportRegistry & transport_registry,
                             int max_duration_sec);

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

  /// Parse resource URI to extract entity type, entity id, collection, and resource path.
  static tl::expected<ParsedResourceUri, std::string> parse_resource_uri(const std::string & resource);

 private:
  /// Build event_source URI from subscription info
  static std::string build_event_source(const CyclicSubscriptionInfo & info);

  /// Extract entity type string ("apps" or "components") from request path
  static std::string extract_entity_type(const httplib::Request & req);

  HandlerContext & ctx_;
  SubscriptionManager & sub_mgr_;
  ResourceSamplerRegistry & sampler_registry_;
  TransportRegistry & transport_registry_;
  int max_duration_sec_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
