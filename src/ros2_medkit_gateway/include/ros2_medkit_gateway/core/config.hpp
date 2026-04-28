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

#include <string>
#include <vector>

namespace ros2_medkit_gateway {

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
