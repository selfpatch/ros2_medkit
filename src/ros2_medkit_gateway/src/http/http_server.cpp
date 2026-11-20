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

#include "ros2_medkit_gateway/core/http/http_server.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/logging.hpp"

namespace ros2_medkit_gateway {

namespace {

using Address = std::array<std::uint8_t, 16>;

// SO_REUSEADDR only: it lets a restarted gateway rebind over TIME_WAIT. Never SO_REUSEPORT,
// which would let a second process bind the same address:port while this one serves it.
void reuse_address_only(httplib::Server & srv) {
  srv.set_socket_options([](socket_t sock) {
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const void *>(&yes), sizeof(yes));
  });
}

Address from_v4(const in_addr & a) {
  Address out{};
  out[10] = 0xff;
  out[11] = 0xff;
  std::memcpy(out.data() + 12, &a.s_addr, 4);
  return out;
}

bool is_v4_mapped(const Address & a) {
  static const std::array<std::uint8_t, 12> prefix{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff};
  return std::equal(prefix.begin(), prefix.end(), a.begin());
}

bool is_wildcard(const Address & a) {
  Address any{};
  if (a == any) {
    return true;
  }
  return is_v4_mapped(a) && a[12] == 0 && a[13] == 0 && a[14] == 0 && a[15] == 0;
}

// Two bound addresses collide when they are equal, or one is a wildcard that covers the other:
// 0.0.0.0 covers every IPv4 address, :: covers every address.
bool addresses_collide(const Address & ours, const Address & theirs) {
  if (ours == theirs) {
    return true;
  }
  for (const auto & [a, b] : {std::pair{ours, theirs}, std::pair{theirs, ours}}) {
    if (!is_wildcard(a)) {
      continue;
    }
    if (is_v4_mapped(a)) {
      if (is_v4_mapped(b)) {
        return true;
      }
    } else {
      return true;
    }
  }
  return false;
}

// Reads LISTEN sockets on `port` from /proc/net/tcp and /proc/net/tcp6. The kernel table lists every
// listener of the network namespace, whatever its user and whatever options it bound with, which a
// connect probe cannot promise (a listener may be unreachable or not accepting).
std::vector<Address> listeners_on_port(int port) {
  std::vector<Address> found;
  for (const char * table : {"/proc/net/tcp", "/proc/net/tcp6"}) {
    std::ifstream in(table);
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
      std::istringstream row(line);
      std::string slot, local, remote, state;
      row >> slot >> local >> remote >> state;
      const auto colon = local.find(':');
      if (state != "0A" || colon == std::string::npos) {
        continue;
      }
      if (std::stoi(local.substr(colon + 1), nullptr, 16) != port) {
        continue;
      }
      const std::string hex = local.substr(0, colon);
      Address addr{};
      if (hex.size() == 8) {
        in_addr a{};
        a.s_addr = static_cast<std::uint32_t>(std::stoul(hex, nullptr, 16));
        addr = from_v4(a);
      } else if (hex.size() == 32) {
        // Four 32-bit words, each printed in host (little-endian) order.
        for (std::size_t w = 0; w < 4; ++w) {
          const auto word = static_cast<std::uint32_t>(std::stoul(hex.substr(w * 8, 8), nullptr, 16));
          std::memcpy(addr.data() + w * 4, &word, 4);
        }
      } else {
        continue;
      }
      found.push_back(addr);
    }
  }
  return found;
}

// The addresses `host` names. Empty when it does not resolve; `gai_error` then holds the reason.
std::vector<Address> resolve(const std::string & host, std::string & gai_error) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;
  addrinfo * res = nullptr;
  const int rc = getaddrinfo(host.empty() ? nullptr : host.c_str(), nullptr, &hints, &res);
  if (rc != 0) {
    gai_error = gai_strerror(rc);
    return {};
  }
  std::vector<Address> out;
  for (addrinfo * ai = res; ai != nullptr; ai = ai->ai_next) {
    if (ai->ai_family == AF_INET) {
      out.push_back(from_v4(reinterpret_cast<sockaddr_in *>(ai->ai_addr)->sin_addr));
    } else if (ai->ai_family == AF_INET6) {
      Address a{};
      std::memcpy(a.data(), &reinterpret_cast<sockaddr_in6 *>(ai->ai_addr)->sin6_addr, 16);
      out.push_back(a);
    }
  }
  freeaddrinfo(res);
  return out;
}

// True when a live listener may hold the address:port `host`:`port` names.
bool live_listener_holds(const std::string & host, int port) {
  std::string unused;
  const auto ours = resolve(host, unused);
  for (const auto & theirs : listeners_on_port(port)) {
    for (const auto & mine : ours) {
      if (addresses_collide(mine, theirs)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

HttpServerManager::HttpServerManager(const TlsConfig & tls_config, std::size_t thread_pool_size,
                                     std::time_t keep_alive_timeout_sec)
  : tls_config_(tls_config), thread_pool_size_(thread_pool_size), keep_alive_timeout_sec_(keep_alive_timeout_sec) {
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
    apply_thread_pool(*ssl_server_);
    reuse_address_only(*ssl_server_);
    apply_keep_alive(*ssl_server_);

    MEDKIT_LOG_INFO(log::Logger("http_server"), "TLS/HTTPS enabled - cert: %s, min_version: %s",
                    tls_config_.cert_file.c_str(), tls_config_.min_version.c_str());
    // Note: key_file path intentionally not logged for security reasons
  } else {
    server_ = std::make_unique<httplib::Server>();
    reuse_address_only(*server_);
    apply_thread_pool(*server_);
    apply_keep_alive(*server_);
    MEDKIT_LOG_DEBUG(log::Logger("http_server"), "TLS/HTTPS disabled - using plain HTTP");
  }
#else
  if (tls_config_.enabled) {
    throw std::runtime_error(
        "TLS/HTTPS support requested but cpp-httplib was not compiled with OpenSSL support. "
        "Ensure CPPHTTPLIB_OPENSSL_SUPPORT is defined.");
  }
  server_ = std::make_unique<httplib::Server>();
  reuse_address_only(*server_);
  apply_thread_pool(*server_);
  apply_keep_alive(*server_);
#endif
}

void HttpServerManager::apply_thread_pool(httplib::Server & srv) const {
  if (thread_pool_size_ == 0) {
    return;  // Keep the cpp-httplib default task queue.
  }
  // new_task_queue is invoked once when the server starts accepting connections.
  // Returning a fixed-size ThreadPool bounds the request worker count regardless
  // of host CPU count (issue #440). cpp-httplib owns and deletes the returned
  // TaskQueue.
  const std::size_t n = thread_pool_size_;
  srv.new_task_queue = [n] {
    return new httplib::ThreadPool(n);
  };
}

void HttpServerManager::apply_keep_alive(httplib::Server & srv) const {
  if (keep_alive_timeout_sec_ <= 0) {
    return;  // Keep the cpp-httplib default (5s).
  }
  // Bound how long a request-pool worker stays parked on an idle keep-alive
  // connection. With a small fixed pool, the library default (5s) lets a burst
  // of short-lived client connections pin every worker, stalling ordinary
  // requests for up to one timeout per cycle (issue #440).
  srv.set_keep_alive_timeout(keep_alive_timeout_sec_);
}

httplib::Server * HttpServerManager::get_server() {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    return ssl_server_.get();
  }
#endif
  return server_.get();
}

bool HttpServerManager::bind(const std::string & host, int port, int64_t wait_sec, std::string & error) {
  std::string protocol = tls_config_.enabled ? "HTTPS" : "HTTP";
  MEDKIT_LOG_INFO(log::Logger("http_server"), "Starting %s server on %s:%d...", protocol.c_str(), host.c_str(), port);

  httplib::Server * srv = get_server();
  if (srv == nullptr) {
    error = "no server instance";
    return false;
  }
  errno = 0;
  if (srv->bind_to_port(host, port)) {
    return true;
  }
  int bind_errno = errno;
  std::string gai_error;
  if (resolve(host, gai_error).empty()) {
    error = "cannot resolve the host: " + gai_error;
    return false;
  }
  error = bind_errno != 0 ? std::strerror(bind_errno) : "bind failed";
  if (bind_errno != EADDRINUSE || live_listener_holds(host, port)) {
    return false;
  }
  if (wait_sec <= 0) {
    error += " (no live listener holds it; server.bind_wait_sec is 0, so no wait)";
    return false;
  }

  // EADDRINUSE with no live listener: connections an earlier process closed are still in
  // TIME_WAIT (a release that bound with SO_REUSEPORT blocks a SO_REUSEADDR bind). They end
  // within the kernel's TIME_WAIT period; retry the same bind until then.
  MEDKIT_LOG_INFO(log::Logger("http_server"),
                  "%s:%d is held by closing connections of an earlier process; waiting up to %" PRId64
                  " s for them to end (server.bind_wait_sec)",
                  host.c_str(), port, wait_sec);
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::seconds(wait_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (live_listener_holds(host, port)) {
      error = std::strerror(EADDRINUSE);
      return false;
    }
    errno = 0;
    if (srv->bind_to_port(host, port)) {
      const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      MEDKIT_LOG_INFO(log::Logger("http_server"), "%s:%d bound after waiting %.1f s", host.c_str(), port, waited);
      return true;
    }
    bind_errno = errno;
    if (bind_errno != EADDRINUSE) {
      error = bind_errno != 0 ? std::strerror(bind_errno) : "bind failed";
      return false;
    }
  }
  error = std::string(std::strerror(EADDRINUSE)) + " (waited " + std::to_string(wait_sec) +
          " s for closing connections of an earlier process; server.bind_wait_sec)";
  return false;
}

void HttpServerManager::listen_bound() {
  if (httplib::Server * srv = get_server()) {
    srv->listen_after_bind();
  }
}

void HttpServerManager::stop() {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_ && ssl_server_->is_running()) {
    MEDKIT_LOG_INFO(log::Logger("http_server"), "Stopping HTTPS server...");
    ssl_server_->stop();
    return;
  }
#endif
  if (server_ && server_->is_running()) {
    MEDKIT_LOG_INFO(log::Logger("http_server"), "Stopping HTTP server...");
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
    MEDKIT_LOG_WARN(log::Logger("http_server"),
                    "min_version='%s' requested but cpp-httplib uses OpenSSL defaults (TLS 1.2+). "
                    "Custom min_version not enforced.",
                    tls_config_.min_version.c_str());
  }

  // Log TLS handshake failures for debugging
  ssl_server_->set_logger([](const httplib::Request & req, const httplib::Response & res) {
    if (res.status >= 400) {
      MEDKIT_LOG_DEBUG(log::Logger("http_server"), "Request %s %s -> %d", req.method.c_str(), req.path.c_str(),
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
