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

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/version.hpp"

namespace ros2_medkit_gateway {

/// How the server names itself to clients. The defaults describe the ROS 2 node.
struct ServerIdentity {
  /// `name` on `GET /` and the title of the OpenAPI documents.
  std::string server_name{"ROS 2 Medkit Gateway"};
  /// `vendor_info.name` on `GET /version-info`.
  std::string vendor_name{"ros2_medkit"};
  /// `version` on `GET /`, `vendor_info.version` and the OpenAPI document version.
  std::string server_version{kGatewayVersion};
};

/// Raw configuration values the REST layer reads. Clamping happens where the value is used.
/// A program that hosts the core sets the `ServerIdentity` fields to its own values.
struct RestConfig : ServerIdentity {
  int64_t http_thread_pool_size{2};
  int64_t keep_alive_timeout_sec{2};
  int64_t sse_keepalive_interval_sec{30};
  int64_t sse_max_duration_sec{3600};
  bool docs_enabled{true};
  std::string server_host{"localhost"};
  int64_t server_port{8080};
  /// Logger name the REST server reports its warnings under.
  std::string logger_name{"rest_server"};
};

/**
 * @brief TLS/HTTPS configuration settings
 *
 * Enables encrypted communication using OpenSSL.
 * When enabled, the gateway will start an HTTPS server instead of HTTP.
 */
struct TlsConfig {
  /// Whether TLS is enabled (default: false for backward compatibility)
  bool enabled{false};

  /// Path to PEM-encoded certificate file
  std::string cert_file;

  /// Path to PEM-encoded private key file
  std::string key_file;

  /// Optional: Path to CA certificate file for client verification (mutual TLS)
  std::string ca_file;

  /// Minimum TLS version: "1.2" or "1.3" (default: "1.2")
  std::string min_version{"1.2"};

  /// Validate the configuration
  /// @return Empty string if valid, error message otherwise
  [[nodiscard]] std::string validate() const;
};

/**
 * @brief Builder for TlsConfig with fluent interface and validation
 *
 * Usage:
 *   auto config = TlsConfigBuilder()
 *       .with_enabled(true)
 *       .with_cert_file("/path/to/cert.pem")
 *       .with_key_file("/path/to/key.pem")
 *       .with_min_version("1.3")
 *       .build();
 *
 * @throws std::invalid_argument if configuration is invalid when build() is called
 */
class TlsConfigBuilder {
 public:
  TlsConfigBuilder & with_enabled(bool enabled);
  TlsConfigBuilder & with_cert_file(const std::string & cert_file);
  TlsConfigBuilder & with_key_file(const std::string & key_file);
  TlsConfigBuilder & with_ca_file(const std::string & ca_file);
  TlsConfigBuilder & with_min_version(const std::string & min_version);

  /// Build and validate the configuration
  /// @throws std::invalid_argument if configuration is invalid
  TlsConfig build();

 private:
  TlsConfig config_;
};

/**
 * @brief CORS (Cross-Origin Resource Sharing) configuration settings
 */
struct CorsConfig {
  bool enabled{false};
  std::vector<std::string> allowed_origins;
  std::vector<std::string> allowed_methods;
  std::vector<std::string> allowed_headers;
  bool allow_credentials{false};
  int max_age_seconds{86400};

  // Pre-built header values for performance
  std::string methods_header;
  std::string headers_header;
};

/**
 * @brief Builder for CorsConfig with fluent interface
 *
 * Usage:
 *   auto config = CorsConfigBuilder()
 *       .with_origins({"http://localhost:5173"})
 *       .with_methods({"GET", "PUT", "OPTIONS"})
 *       .with_headers({"Content-Type", "Accept"})
 *       .with_credentials(true)
 *       .with_max_age(3600)
 *       .build();
 */
class CorsConfigBuilder {
 public:
  CorsConfigBuilder & with_origins(std::vector<std::string> origins);
  CorsConfigBuilder & with_methods(std::vector<std::string> methods);
  CorsConfigBuilder & with_headers(std::vector<std::string> headers);
  CorsConfigBuilder & with_credentials(bool credentials);
  CorsConfigBuilder & with_max_age(int seconds);
  CorsConfig build();

 private:
  CorsConfig config_;
};

}  // namespace ros2_medkit_gateway
