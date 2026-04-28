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

#include <gtest/gtest.h>

#include <string>

#include "ros2_medkit_gateway/core/aggregation/network_utils.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// parse_url_host_port tests
// =============================================================================

TEST(NetworkUtils, parse_ipv4_url) {
  auto [host, port] = parse_url_host_port("http://192.168.1.5:8080");
  EXPECT_EQ(host, "192.168.1.5");
  EXPECT_EQ(port, 8080);
}

TEST(NetworkUtils, parse_ipv4_url_with_path) {
  auto [host, port] = parse_url_host_port("http://10.0.0.1:9090/api/v1");
  EXPECT_EQ(host, "10.0.0.1");
  EXPECT_EQ(port, 9090);
}

TEST(NetworkUtils, parse_https_url) {
  auto [host, port] = parse_url_host_port("https://192.168.1.5:8443");
  EXPECT_EQ(host, "192.168.1.5");
  EXPECT_EQ(port, 8443);
}

TEST(NetworkUtils, parse_ipv6_url) {
  auto [host, port] = parse_url_host_port("http://[::1]:8080");
  EXPECT_EQ(host, "::1");
  EXPECT_EQ(port, 8080);
}

TEST(NetworkUtils, parse_ipv6_full_address) {
  auto [host, port] = parse_url_host_port("http://[fe80::1%25eth0]:8080");
  EXPECT_EQ(host, "fe80::1%25eth0");
  EXPECT_EQ(port, 8080);
}

TEST(NetworkUtils, parse_ipv6_with_path) {
  auto [host, port] = parse_url_host_port("http://[::1]:9090/api/v1");
  EXPECT_EQ(host, "::1");
  EXPECT_EQ(port, 9090);
}

TEST(NetworkUtils, parse_localhost) {
  auto [host, port] = parse_url_host_port("http://127.0.0.1:8080");
  EXPECT_EQ(host, "127.0.0.1");
  EXPECT_EQ(port, 8080);
}

TEST(NetworkUtils, parse_no_scheme_returns_invalid) {
  auto [host, port] = parse_url_host_port("192.168.1.5:8080");
  EXPECT_EQ(host, "");
  EXPECT_EQ(port, -1);
}

TEST(NetworkUtils, parse_empty_string_returns_invalid) {
  auto [host, port] = parse_url_host_port("");
  EXPECT_EQ(host, "");
  EXPECT_EQ(port, -1);
}

TEST(NetworkUtils, parse_no_port_returns_host_only) {
  auto [host, port] = parse_url_host_port("http://example.com");
  EXPECT_EQ(host, "example.com");
  EXPECT_EQ(port, -1);
}

TEST(NetworkUtils, parse_hostname_with_port) {
  auto [host, port] = parse_url_host_port("http://my-gateway:8080");
  EXPECT_EQ(host, "my-gateway");
  EXPECT_EQ(port, 8080);
}

TEST(NetworkUtils, parse_invalid_port_returns_negative) {
  auto [host, port] = parse_url_host_port("http://192.168.1.5:notaport");
  EXPECT_EQ(host, "192.168.1.5");
  EXPECT_EQ(port, -1);
}

TEST(NetworkUtils, parse_ipv6_missing_closing_bracket) {
  auto [host, port] = parse_url_host_port("http://[::1:8080");
  EXPECT_EQ(host, "");
  EXPECT_EQ(port, -1);
}

TEST(NetworkUtils, parse_ipv6_no_port) {
  auto [host, port] = parse_url_host_port("http://[::1]");
  EXPECT_EQ(host, "::1");
  EXPECT_EQ(port, -1);
}

// =============================================================================
// collect_local_addresses tests
// =============================================================================

TEST(NetworkUtils, local_addresses_include_loopback) {
  auto addrs = collect_local_addresses();
  EXPECT_TRUE(addrs.count("127.0.0.1") > 0) << "Should always include IPv4 loopback";
  EXPECT_TRUE(addrs.count("::1") > 0) << "Should always include IPv6 loopback";
}

TEST(NetworkUtils, local_addresses_not_empty) {
  auto addrs = collect_local_addresses();
  // At minimum: 127.0.0.1 and ::1
  EXPECT_GE(addrs.size(), 2u);
}

// =============================================================================
// Self-discovery filter scenario tests
// =============================================================================

TEST(NetworkUtils, spoofed_mdns_response_detected_ipv4) {
  // Simulate: attacker sends mDNS response with different name but local IP:port
  auto local_addrs = collect_local_addresses();
  int self_port = 8080;
  std::string spoofed_url = "http://127.0.0.1:8080";
  std::string spoofed_name = "attacker-gateway";

  auto [peer_host, peer_port] = parse_url_host_port(spoofed_url);

  // The IP-based filter should catch this
  bool is_self = (peer_port == self_port && local_addrs.count(peer_host) > 0);
  EXPECT_TRUE(is_self) << "Should detect loopback address as self";
}

TEST(NetworkUtils, spoofed_mdns_response_detected_ipv6) {
  auto local_addrs = collect_local_addresses();
  int self_port = 8080;
  std::string spoofed_url = "http://[::1]:8080";

  auto [peer_host, peer_port] = parse_url_host_port(spoofed_url);

  bool is_self = (peer_port == self_port && local_addrs.count(peer_host) > 0);
  EXPECT_TRUE(is_self) << "Should detect IPv6 loopback as self";
}

TEST(NetworkUtils, legitimate_peer_not_rejected) {
  auto local_addrs = collect_local_addresses();
  int self_port = 8080;
  // Use an IP that is almost certainly not a local interface
  std::string peer_url = "http://198.51.100.42:8080";

  auto [peer_host, peer_port] = parse_url_host_port(peer_url);

  bool is_self = (peer_port == self_port && local_addrs.count(peer_host) > 0);
  EXPECT_FALSE(is_self) << "Should not reject non-local addresses (TEST-NET-2 198.51.100.0/24)";
}

TEST(NetworkUtils, different_port_not_rejected) {
  auto local_addrs = collect_local_addresses();
  int self_port = 8080;
  // Same local IP but different port - this is a different service, not self
  std::string peer_url = "http://127.0.0.1:9090";

  auto [peer_host, peer_port] = parse_url_host_port(peer_url);

  bool is_self = (peer_port == self_port && local_addrs.count(peer_host) > 0);
  EXPECT_FALSE(is_self) << "Should not reject local address with different port";
}
