// Copyright 2025 mfaferek93
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

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <tl/expected.hpp>
#include <vector>

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/http/handlers/handlers.hpp"
#include "ros2_medkit_gateway/http/http_server.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;

/**
 * @brief REST API server for ROS 2 Medkit Gateway.
 *
 * Provides a RESTful interface to ROS 2 functionality including:
 * - Discovery (areas, components)
 * - Data access (topic read/write)
 * - Operations (service calls, action goals)
 * - Configuration (parameter management)
 * - Fault management
 * - Authentication (OAuth2-like flow)
 *
 * The server delegates request handling to specialized handler classes
 * organized by domain (health, areas, components, operations, config, faults, auth).
 */
class RESTServer {
 public:
  RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config,
             const AuthConfig & auth_config, const TlsConfig & tls_config = TlsConfig{});
  ~RESTServer();

  void start();
  void stop();

  /// Check if TLS/HTTPS is enabled
  bool is_tls_enabled() const {
    return http_server_ && http_server_->is_tls_enabled();
  }

 private:
  void setup_routes();
  void setup_pre_routing_handler();
  void setup_global_error_handlers();

  // CORS helper methods
  void set_cors_headers(httplib::Response & res, const std::string & origin) const;
  bool is_origin_allowed(const std::string & origin) const;

  GatewayNode * node_;
  std::string host_;
  int port_;
  CorsConfig cors_config_;
  AuthConfig auth_config_;
  TlsConfig tls_config_;
  std::unique_ptr<AuthManager> auth_manager_;
  std::unique_ptr<AuthMiddleware> auth_middleware_;

  // HTTP/HTTPS server manager
  std::unique_ptr<HttpServerManager> http_server_;

  // Handler context and domain-specific handlers
  std::unique_ptr<handlers::HandlerContext> handler_ctx_;
  std::unique_ptr<handlers::HealthHandlers> health_handlers_;
  std::unique_ptr<handlers::DiscoveryHandlers> discovery_handlers_;
  std::unique_ptr<handlers::DataHandlers> data_handlers_;
  std::unique_ptr<handlers::OperationHandlers> operation_handlers_;
  std::unique_ptr<handlers::ConfigHandlers> config_handlers_;
  std::unique_ptr<handlers::FaultHandlers> fault_handlers_;
  std::unique_ptr<handlers::AuthHandlers> auth_handlers_;
  std::unique_ptr<handlers::SSEFaultHandler> sse_fault_handler_;
  std::unique_ptr<handlers::BulkDataHandlers> bulkdata_handlers_;
};

}  // namespace ros2_medkit_gateway
