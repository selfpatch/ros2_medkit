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

#include <atomic>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <tl/expected.hpp>
#include <vector>

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Collection of entities fetched from a peer gateway
 */
struct PeerEntities {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;
};

/**
 * @brief HTTP client for communicating with a peer gateway instance
 *
 * PeerClient wraps cpp-httplib to provide typed access to a peer gateway's
 * REST API. It supports health checking, entity fetching, transparent request
 * forwarding (proxy), and JSON fan-out queries.
 *
 * Thread safety: The healthy_ flag is atomic. Client creation is lazy and
 * guarded by a mutex. All public methods are safe to call from any thread.
 */
class PeerClient {
 public:
  /**
   * @brief Construct a PeerClient for a peer gateway
   * @param url Base URL of the peer (e.g., "http://localhost:8081")
   * @param name Human-readable peer name (e.g., "subsystem_b")
   * @param timeout_ms Connection and read timeout in milliseconds
   * @param forward_auth Whether to forward Authorization headers to this peer
   */
  PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth = false);

  /// Get the peer base URL
  const std::string & url() const;

  /// Get the peer name
  const std::string & name() const;

  /// Check if the peer was healthy at last health check
  bool is_healthy() const;

  /**
   * @brief Perform a health check against the peer
   *
   * GETs /api/v1/health on the peer. Sets the internal healthy_ flag
   * based on whether a 200 response was received.
   */
  void check_health();

  /**
   * @brief Fetch all entity collections from the peer
   *
   * GETs /api/v1/areas, /api/v1/components, /api/v1/apps, /api/v1/functions
   * and parses the items[] arrays. Each entity's source is set to "peer:<name>".
   *
   * @return PeerEntities on success, error message on failure
   */
  tl::expected<PeerEntities, std::string> fetch_entities();

  /**
   * @brief Forward an HTTP request transparently to the peer (proxy)
   *
   * Copies method, path, body, and Content-Type from the incoming request.
   * Forwards the Authorization header only if forward_auth is enabled.
   * Copies the peer's response status, headers, and body back to the
   * outgoing response.
   *
   * On connection failure, returns 502 with x-medkit-peer-unavailable error.
   *
   * @param req Incoming HTTP request to forward
   * @param res Outgoing HTTP response to populate
   */
  void forward_request(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Forward a request and parse the JSON response
   *
   * Used for fan-out merge scenarios where the aggregator needs to
   * combine JSON results from multiple peers.
   *
   * @param method HTTP method (e.g., "GET")
   * @param path Request path (e.g., "/api/v1/components/abc/data")
   * @param auth_header Authorization header value (empty to omit)
   * @return Parsed JSON on success, error message on failure
   */
  tl::expected<nlohmann::json, std::string> forward_and_get_json(const std::string & method, const std::string & path,
                                                                 const std::string & auth_header = "",
                                                                 const httplib::Headers & extra_headers = {});

 private:
  /**
   * @brief Ensure the underlying HTTP client exists (lazy initialization)
   *
   * Must be called under client_mutex_. Creates the client if it doesn't exist.
   */
  void ensure_client();

  std::string url_;
  std::string name_;
  int timeout_ms_;
  bool forward_auth_;
  std::atomic<bool> healthy_{false};

  std::mutex client_mutex_;
  std::unique_ptr<httplib::Client> client_;
};

}  // namespace ros2_medkit_gateway
