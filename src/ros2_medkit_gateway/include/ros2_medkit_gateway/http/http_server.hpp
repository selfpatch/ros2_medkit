// Copyright 2025 bburda
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
#include <string>

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/config.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Manages HTTP/HTTPS server instances with TLS support
 *
 * This class abstracts the creation and management of cpp-httplib Server
 * or SSLServer instances based on TLS configuration. It provides a unified
 * interface to get the active server pointer, eliminating code duplication.
 */
class HttpServerManager {
 public:
  /**
   * @brief Construct HTTP server manager
   * @param tls_config TLS configuration (if enabled, creates SSLServer)
   * @throws std::runtime_error if TLS is requested but SSL server creation fails
   */
  explicit HttpServerManager(const TlsConfig & tls_config);

  ~HttpServerManager() = default;

  // Non-copyable
  HttpServerManager(const HttpServerManager &) = delete;
  HttpServerManager & operator=(const HttpServerManager &) = delete;

  // Movable
  HttpServerManager(HttpServerManager &&) = default;
  HttpServerManager & operator=(HttpServerManager &&) = default;

  /**
   * @brief Get pointer to the active server instance
   * @return Pointer to httplib::Server (or SSLServer cast to Server*)
   * @note Returns nullptr if no server was successfully created
   */
  httplib::Server * get_server();

  /**
   * @brief Check if TLS is enabled
   */
  bool is_tls_enabled() const {
    return tls_config_.enabled;
  }

  /**
   * @brief Start listening on the specified host and port
   * @param host Host address to bind to
   * @param port Port number to bind to
   */
  void listen(const std::string & host, int port);

  /**
   * @brief Stop the server if running
   */
  void stop();

  /**
   * @brief Check if server is currently running
   */
  bool is_running() const;

 private:
  /**
   * @brief Configure TLS settings on SSL server
   */
  void configure_tls();

  TlsConfig tls_config_;

  // HTTP server (used when TLS is disabled)
  std::unique_ptr<httplib::Server> server_;

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  // HTTPS server (used when TLS is enabled)
  std::unique_ptr<httplib::SSLServer> ssl_server_;
#endif
};

}  // namespace ros2_medkit_gateway
