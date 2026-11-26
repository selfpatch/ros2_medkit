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

#include "ros2_medkit_gateway/core/net/bind_address.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>

#include <algorithm>
#include <cctype>
#include <cstdint>

namespace ros2_medkit_gateway {

namespace {

bool is_loopback_v4(std::uint32_t address_host_order) {
  return (address_host_order >> 24) == 127;
}

}  // namespace

bool is_loopback_bind_host(const std::string & host) {
  std::string name = host;
  if (name.size() >= 2 && name.front() == '[' && name.back() == ']') {
    name = name.substr(1, name.size() - 2);
  }
  if (name.empty() || std::any_of(name.begin(), name.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
      })) {
    return false;
  }

  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (lower == "localhost") {
    return true;
  }

  in6_addr v6{};
  if (name.find(':') != std::string::npos) {
    if (inet_pton(AF_INET6, name.c_str(), &v6) != 1) {
      return false;
    }
    if (IN6_IS_ADDR_LOOPBACK(&v6)) {
      return true;
    }
    if (IN6_IS_ADDR_V4MAPPED(&v6)) {
      std::uint32_t v4 = 0;
      for (int i = 12; i < 16; ++i) {
        v4 = (v4 << 8) | v6.s6_addr[i];
      }
      return is_loopback_v4(v4);
    }
    return false;
  }

  // inet_aton takes the short forms the resolver takes when it binds ("127.1").
  in_addr v4{};
  if (inet_aton(name.c_str(), &v4) == 0) {
    return false;
  }
  return is_loopback_v4(ntohl(v4.s_addr));
}

}  // namespace ros2_medkit_gateway
