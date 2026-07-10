// Copyright 2026 mfaferek93
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

// Real network I/O for the discovery orchestrator: a bounded POSIX TCP connect
// probe and a read-only OPC-UA GetEndpoints identify via open62541pp. Split
// from network_discovery.cpp so the pure merge / dedup logic (and its unit
// tests) do not link against open62541 or open POSIX sockets.

#include "ros2_medkit_opcua/network_discovery.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

#include <open62541pp/client.hpp>
#include <open62541pp/types_composed.hpp>

namespace ros2_medkit_gateway {

bool tcp_connect_probe(const std::string & ip, uint16_t port, int timeout_ms) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return false;
  }

  // Non-blocking connect + poll so a filtered / silent host times out at
  // ``timeout_ms`` instead of the kernel default (tens of seconds). Bail out
  // on either fcntl() failing: leaving the socket blocking would let a single
  // filtered host stall the probe for the kernel's default connect timeout,
  // defeating the bounded-concurrency scan.
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags == -1 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    ::close(fd);
    return false;
  }

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    ::close(fd);
    return false;
  }

  bool connected = false;
  const int rc = ::connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  if (rc == 0) {
    connected = true;
  } else if (errno == EINPROGRESS) {
    // poll() rather than select(): select()'s fd_set is capped at FD_SETSIZE
    // (1024) and FD_SET() on a higher fd is POSIX undefined behaviour (writes
    // past the stack fd_set, a hard abort under _FORTIFY_SOURCE). A long-running
    // gateway running many concurrent probes can allocate fds past 1024; poll()
    // has no such limit.
    struct pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLOUT;
    if (::poll(&pfd, 1, timeout_ms) > 0) {
      int so_error = 0;
      socklen_t len = sizeof(so_error);
      if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0 && so_error == 0) {
        connected = true;
      }
    }
  }

  ::close(fd);
  return connected;
}

IdentifyResult opcua_getendpoints_identify(const std::string & url, int timeout_ms) {
  IdentifyResult out;
  try {
    opcua::Client client;
    client.config().setTimeout(static_cast<uint32_t>(timeout_ms));

    const std::vector<opcua::EndpointDescription> eps = client.getEndpoints(url);
    if (eps.empty()) {
      out.error = "no endpoints returned";
      return out;
    }

    for (const auto & ep : eps) {
      const std::string policy_uri(ep.getSecurityPolicyUri());
      const auto hash = policy_uri.rfind('#');
      const std::string policy_name = hash == std::string::npos ? policy_uri : policy_uri.substr(hash + 1);
      const int mode = static_cast<int>(ep.getSecurityMode());  // 1=None 2=Sign 3=SignAndEncrypt
      out.security_policies.push_back({policy_name, mode});
      // Channel security (None/None) and user authentication are independent in
      // OPC-UA: a None/None endpoint can still accept only Username/Certificate
      // tokens. Only mark it anonymous-usable when it also advertises an
      // Anonymous UserIdentityToken policy; otherwise the anonymous connect
      // fails at session activation (BadIdentityTokenRejected).
      if (policy_name == "None" && mode == 1 /* UA_MESSAGESECURITYMODE_NONE */) {
        for (const auto & token : ep.getUserIdentityTokens()) {
          if (token.getTokenType() == opcua::UserTokenType::Anonymous) {
            out.anonymous_none_available = true;
            break;
          }
        }
      }
    }

    const auto server = eps.front().getServer();
    out.ok = true;
    out.advertised_url = std::string(eps.front().getEndpointUrl());
    out.application_uri = std::string(server.getApplicationUri());
    out.product_uri = std::string(server.getProductUri());
    out.application_name = std::string(server.getApplicationName().getText());
    out.application_type = static_cast<int>(server.getApplicationType());
  } catch (const std::exception & e) {
    out.ok = false;
    out.error = e.what();
  }
  return out;
}

}  // namespace ros2_medkit_gateway
