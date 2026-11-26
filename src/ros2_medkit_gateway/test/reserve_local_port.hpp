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

namespace ros2_medkit_gateway::test_support {

/// A free loopback TCP port chosen by the kernel, or 0 on failure.
/// Tests run in parallel, so a gateway in a test binds this port, never the default 8080.
inline int reserve_local_port() {
  const int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  socklen_t addr_len = sizeof(addr);
  const bool ok = bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0 &&
                  getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) == 0;
  close(sock);
  return ok ? ntohs(addr.sin_port) : 0;
}

}  // namespace ros2_medkit_gateway::test_support
