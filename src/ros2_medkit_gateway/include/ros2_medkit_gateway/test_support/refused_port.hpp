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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

/// Support for this package's tests and the tests of packages that extend it.
/// Not part of the gateway's supported API.
namespace ros2_medkit_gateway::test_support {

/**
 * @brief A loopback TCP port that refuses every connection while this object lives.
 *
 * The socket is bound to 127.0.0.1 on a kernel-chosen port and never listens. A connect to
 * it gets ECONNREFUSED at once, and no other socket can bind the port until it is closed.
 */
class RefusedPort {
 public:
  RefusedPort() {
    fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd_ < 0) {
      throw std::runtime_error(std::string("RefusedPort: socket failed: ") + std::strerror(errno));
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    socklen_t len = sizeof(addr);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
        ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
      const int error = errno;
      ::close(fd_);
      throw std::runtime_error(std::string("RefusedPort: bind failed: ") + std::strerror(error));
    }
    port_ = ntohs(addr.sin_port);
  }

  ~RefusedPort() {
    ::close(fd_);
  }

  RefusedPort(const RefusedPort &) = delete;
  RefusedPort & operator=(const RefusedPort &) = delete;
  RefusedPort(RefusedPort &&) = delete;
  RefusedPort & operator=(RefusedPort &&) = delete;

  int port() const {
    return port_;
  }

  /// "http://127.0.0.1:<port>"
  std::string http_url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }

 private:
  int fd_{-1};
  int port_{0};
};

}  // namespace ros2_medkit_gateway::test_support
