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

#include "ros2_medkit_gateway/http/http_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace ros2_medkit_gateway {

HttpServerManager::HttpServerManager(const TlsConfig & tls_config) : tls_config_(tls_config) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled) {
    // Create SSL server with certificate and key
    ssl_server_ = std::make_unique<httplib::SSLServer>(tls_config_.cert_file.c_str(), tls_config_.key_file.c_str());

    if (!ssl_server_->is_valid()) {
      throw std::runtime_error(
          "Failed to create SSL server. Check certificate and key files: " + tls_config_.cert_file +
          " (key configured: " + (tls_config_.key_file.empty() ? "no" : "yes") + ")");
    }

    // Configure additional TLS settings
    configure_tls();

    RCLCPP_INFO(rclcpp::get_logger("http_server"), "TLS/HTTPS enabled - cert: %s, min_version: %s",
                tls_config_.cert_file.c_str(), tls_config_.min_version.c_str());
    // Note: key_file path intentionally not logged for security reasons
  } else {
    server_ = std::make_unique<httplib::Server>();
    RCLCPP_DEBUG(rclcpp::get_logger("http_server"), "TLS/HTTPS disabled - using plain HTTP");
  }
#else
  if (tls_config_.enabled) {
    throw std::runtime_error(
        "TLS/HTTPS support requested but cpp-httplib was not compiled with OpenSSL support. "
        "Ensure CPPHTTPLIB_OPENSSL_SUPPORT is defined.");
  }
  server_ = std::make_unique<httplib::Server>();
#endif
}

httplib::Server * HttpServerManager::get_server() {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    return ssl_server_.get();
  }
#endif
  return server_.get();
}

void HttpServerManager::listen(const std::string & host, int port) {
  std::string protocol = tls_config_.enabled ? "HTTPS" : "HTTP";
  RCLCPP_INFO(rclcpp::get_logger("http_server"), "Starting %s server on %s:%d...", protocol.c_str(), host.c_str(),
              port);

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    ssl_server_->listen(host.c_str(), port);
    return;
  }
#endif
  if (server_) {
    server_->listen(host.c_str(), port);
  }
}

void HttpServerManager::stop() {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_ && ssl_server_->is_running()) {
    RCLCPP_INFO(rclcpp::get_logger("http_server"), "Stopping HTTPS server...");
    ssl_server_->stop();
    return;
  }
#endif
  if (server_ && server_->is_running()) {
    RCLCPP_INFO(rclcpp::get_logger("http_server"), "Stopping HTTP server...");
    server_->stop();
  }
}

bool HttpServerManager::is_running() const {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    return ssl_server_->is_running();
  }
#endif
  return server_ && server_->is_running();
}

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
void HttpServerManager::configure_tls() {
  if (!ssl_server_) {
    return;
  }

  // YAGNI Decision: min_version field exists in TlsConfig for future extensibility
  // but is not fully implemented.
  //
  // Rationale:
  // - cpp-httplib's SSLServer doesn't expose SSL_CTX for min_version configuration
  // - Modern OpenSSL (1.1.1+) defaults to TLS 1.2+ which is secure
  //
  // Future implementation options:
  // 1. Fork cpp-httplib to expose SSL_CTX for SSL_CTX_set_min_proto_version()
  // 2. Use OpenSSL system-wide configuration (/etc/ssl/openssl.cnf)
  // 3. Replace cpp-httplib with Boost.Beast or another library with full SSL control
  //
  // TODO(future): Add mutual TLS support - requires cpp-httplib modifications
  // to expose SSL_CTX for SSL_CTX_set_verify() with SSL_VERIFY_PEER

  if (tls_config_.min_version != "1.2") {
    RCLCPP_WARN(rclcpp::get_logger("http_server"),
                "min_version='%s' requested but cpp-httplib uses OpenSSL defaults (TLS 1.2+). "
                "Custom min_version not enforced.",
                tls_config_.min_version.c_str());
  }

  // Log TLS handshake failures for debugging
  ssl_server_->set_logger([](const httplib::Request & req, const httplib::Response & res) {
    if (res.status >= 400) {
      RCLCPP_DEBUG(rclcpp::get_logger("http_server"), "Request %s %s -> %d", req.method.c_str(), req.path.c_str(),
                   res.status);
    }
  });
}
#else
void HttpServerManager::configure_tls() {
  // No-op when OpenSSL support is not available
}
#endif

}  // namespace ros2_medkit_gateway
