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

#include <cstddef>
#include <ctime>
#include <memory>
#include <string>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/config.hpp"

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
   * @param thread_pool_size Number of worker threads for the cpp-httplib request
   *        pool. When > 0, the server's task queue is bounded to a fixed-size
   *        ThreadPool of this many threads (issue #440). When 0, the cpp-httplib
   *        default (max(8, hardware_concurrency - 1)) is left untouched.
   * @param keep_alive_timeout_sec How long cpp-httplib pins a request-pool worker
   *        on an idle keep-alive connection before freeing it (issue #440). When
   *        > 0, overrides the cpp-httplib default (5s); a smaller value lets a
   *        small bounded pool recover workers faster. When 0, the library default
   *        is left untouched.
   * @throws std::runtime_error if TLS is requested but SSL server creation fails
   */
  explicit HttpServerManager(const TlsConfig & tls_config, std::size_t thread_pool_size = 0,
                             std::time_t keep_alive_timeout_sec = 0);

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

  /**
   * @brief Bound the server's request thread pool to thread_pool_size_ workers.
   *
   * No-op when thread_pool_size_ is 0 (keeps the cpp-httplib default). Must be
   * called before listen(), as new_task_queue is consumed when the server
   * starts accepting connections.
   */
  void apply_thread_pool(httplib::Server & srv) const;

  /**
   * @brief Override the server's keep-alive idle timeout.
   *
   * No-op when keep_alive_timeout_sec_ is 0 (keeps the cpp-httplib default of
   * 5s). Bounds how long a request-pool worker stays parked on an idle
   * keep-alive connection, so a small fixed pool cannot be starved by a burst of
   * short-lived client connections (issue #440). Must be called before listen().
   */
  void apply_keep_alive(httplib::Server & srv) const;

  TlsConfig tls_config_;

  // Fixed cpp-httplib worker-pool size (0 = leave the library default).
  std::size_t thread_pool_size_;

  // cpp-httplib keep-alive idle timeout in seconds (0 = leave the library default).
  std::time_t keep_alive_timeout_sec_;

  // HTTP server (used when TLS is disabled)
  std::unique_ptr<httplib::Server> server_;

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  // HTTPS server (used when TLS is enabled)
  std::unique_ptr<httplib::SSLServer> ssl_server_;
#endif
};

}  // namespace ros2_medkit_gateway
