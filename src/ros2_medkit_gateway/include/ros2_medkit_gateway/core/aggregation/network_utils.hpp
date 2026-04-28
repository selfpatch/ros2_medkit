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
#include <ifaddrs.h>
#include <netinet/in.h>

#include <array>
#include <string>
#include <unordered_set>
#include <utility>

namespace ros2_medkit_gateway {

/**
 * @brief Collect all IPv4 and IPv6 addresses assigned to local network interfaces.
 *
 * Always includes 127.0.0.1 and ::1 (loopback). Used for self-discovery filtering
 * so the gateway never forwards to itself even if an mDNS response spoofs the name.
 *
 * @return Set of IP address strings (e.g. "192.168.1.5", "::1")
 */
inline std::unordered_set<std::string> collect_local_addresses() {
  std::unordered_set<std::string> addrs;
  addrs.insert("127.0.0.1");
  addrs.insert("::1");

  struct ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1) {
    return addrs;
  }

  for (struct ifaddrs * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr) {
      continue;
    }
    std::array<char, INET6_ADDRSTRLEN> buf{};
    if (ifa->ifa_addr->sa_family == AF_INET) {
      auto * addr4 = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
      inet_ntop(AF_INET, &addr4->sin_addr, buf.data(), buf.size());
      addrs.insert(buf.data());
    } else if (ifa->ifa_addr->sa_family == AF_INET6) {
      auto * addr6 = reinterpret_cast<struct sockaddr_in6 *>(ifa->ifa_addr);
      inet_ntop(AF_INET6, &addr6->sin6_addr, buf.data(), buf.size());
      addrs.insert(buf.data());
    }
  }

  freeifaddrs(ifaddr);
  return addrs;
}

/**
 * @brief Parse host and port from a URL of the form "scheme://host:port[/path]".
 *
 * Handles IPv6 bracket notation (e.g. "http://[::1]:8080/api").
 *
 * @param url Full URL string
 * @return Pair of {host, port}. Returns {"", -1} if parsing fails entirely.
 *         Returns {host, -1} if port is missing.
 */
inline std::pair<std::string, int> parse_url_host_port(const std::string & url) {
  // Find "://"
  auto scheme_end = url.find("://");
  if (scheme_end == std::string::npos) {
    return {"", -1};
  }
  std::string rest = url.substr(scheme_end + 3);

  // Strip any path
  auto path_pos = rest.find('/');
  if (path_pos != std::string::npos) {
    rest = rest.substr(0, path_pos);
  }

  std::string host;
  std::string port_str;

  if (!rest.empty() && rest[0] == '[') {
    // IPv6 bracket notation: [addr]:port
    auto bracket_end = rest.find(']');
    if (bracket_end == std::string::npos) {
      return {"", -1};
    }
    host = rest.substr(1, bracket_end - 1);
    if (bracket_end + 1 < rest.size() && rest[bracket_end + 1] == ':') {
      port_str = rest.substr(bracket_end + 2);
    }
  } else {
    // IPv4 or hostname: host:port
    auto colon_pos = rest.rfind(':');
    if (colon_pos == std::string::npos) {
      return {rest, -1};
    }
    host = rest.substr(0, colon_pos);
    port_str = rest.substr(colon_pos + 1);
  }

  int port = -1;
  if (!port_str.empty()) {
    try {
      port = std::stoi(port_str);
    } catch (...) {
      return {host, -1};
    }
  }

  return {host, port};
}

}  // namespace ros2_medkit_gateway
