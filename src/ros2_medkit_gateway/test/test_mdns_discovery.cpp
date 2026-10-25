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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include "mdns.h"
#pragma GCC diagnostic pop

#include "ros2_medkit_gateway/core/aggregation/mdns_discovery.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Config / construction tests (no network sockets opened)
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, default_config_values) {
  MdnsDiscovery::Config config;

  EXPECT_FALSE(config.announce);
  EXPECT_FALSE(config.discover);
  EXPECT_EQ(config.service, "_medkit._tcp.local");
  EXPECT_EQ(config.port, 8080);
  EXPECT_TRUE(config.name.empty());
  EXPECT_FALSE(config.on_error);  // No error callback by default
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, respects_announce_flag) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;  // Don't try to open sockets

  MdnsDiscovery discovery(config);

  // Start with both flags off - no threads should launch
  bool found_called = false;
  bool removed_called = false;

  discovery.start(
      [&](const std::string & /*url*/, const std::string & /*name*/) {
        found_called = true;
      },
      [&](const std::string & /*name*/) {
        removed_called = true;
      });

  // Neither thread should be running since both flags are off
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());

  discovery.stop();

  EXPECT_FALSE(found_called);
  EXPECT_FALSE(removed_called);
}

TEST(MdnsDiscovery, respects_discover_flag) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});

  EXPECT_FALSE(discovery.is_discovering());
  EXPECT_FALSE(discovery.is_announcing());

  discovery.stop();
}

TEST(MdnsDiscovery, stop_is_idempotent) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  // Start then stop multiple times - should not crash or hang
  discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});

  discovery.stop();
  discovery.stop();  // Second stop should be a no-op
  discovery.stop();  // Third stop should also be safe

  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());
}

TEST(MdnsDiscovery, destructor_stops_cleanly) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  {
    MdnsDiscovery discovery(config);
    discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});
    // Destructor should call stop() without hanging
  }

  // If we get here without hanging, the test passes
  SUCCEED();
}

TEST(MdnsDiscovery, custom_config_values) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;
  config.service = "_custom._tcp.local";
  config.port = 9090;
  config.name = "test-gateway";

  MdnsDiscovery discovery(config);

  // Construction with custom values should succeed
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());
}

TEST(MdnsDiscovery, start_without_callbacks_is_safe) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  // Passing nullptr-equivalent callbacks should not crash
  discovery.start(nullptr, nullptr);
  discovery.stop();
}

TEST(MdnsDiscovery, not_running_before_start) {
  MdnsDiscovery::Config config;
  config.announce = true;
  config.discover = true;

  MdnsDiscovery discovery(config);

  // Before start(), nothing should be running
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());
}

TEST(MdnsDiscovery, error_callback_is_stored_in_config) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_called = false;
  config.on_error = [&error_called](const std::string & /*msg*/) {
    error_called = true;
  };

  MdnsDiscovery discovery(config);

  // The callback is stored but not invoked when no sockets are opened
  EXPECT_FALSE(error_called);
}

// =============================================================================
// Error callback wiring tests
// =============================================================================

TEST(MdnsDiscovery, error_callback_not_invoked_without_sockets) {
  // Verify the on_error callback is not spuriously triggered when announce/discover are off
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_received = false;
  std::string error_message;
  config.on_error = [&](const std::string & msg) {
    error_received = true;
    error_message = msg;
  };

  MdnsDiscovery mdns(config);

  // Start and stop with both flags off - no socket operations
  mdns.start([](const std::string &, const std::string &) {}, [](const std::string &) {});
  mdns.stop();

  // Error callback should NOT have been invoked since no network activity occurred
  EXPECT_FALSE(error_received);
  EXPECT_TRUE(error_message.empty());
}

TEST(MdnsDiscovery, default_announce_and_discover_are_false) {
  MdnsDiscovery::Config config;
  EXPECT_FALSE(config.announce);
  EXPECT_FALSE(config.discover);
}

TEST(MdnsDiscovery, default_service_type) {
  MdnsDiscovery::Config config;
  EXPECT_EQ(config.service, "_medkit._tcp.local");
}

TEST(MdnsDiscovery, default_port_is_8080) {
  MdnsDiscovery::Config config;
  EXPECT_EQ(config.port, 8080);
}

TEST(MdnsDiscovery, default_name_is_empty) {
  MdnsDiscovery::Config config;
  EXPECT_TRUE(config.name.empty());
}

TEST(MdnsDiscovery, default_on_error_is_null) {
  MdnsDiscovery::Config config;
  EXPECT_FALSE(config.on_error);
}

TEST(MdnsDiscovery, start_stop_lifecycle_with_callbacks) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  bool found_called = false;
  bool removed_called = false;

  discovery.start(
      [&](const std::string & /*url*/, const std::string & /*name*/) {
        found_called = true;
      },
      [&](const std::string & /*name*/) {
        removed_called = true;
      });

  // With both flags off, callbacks should never fire
  EXPECT_FALSE(found_called);
  EXPECT_FALSE(removed_called);
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());

  discovery.stop();

  EXPECT_FALSE(found_called);
  EXPECT_FALSE(removed_called);
}

// =============================================================================
// instance_name() tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, instance_name_returns_explicit_name) {
  MdnsDiscovery::Config config;
  config.name = "my-gateway";
  MdnsDiscovery discovery(config);
  EXPECT_EQ(discovery.instance_name(), "my-gateway");
}

TEST(MdnsDiscovery, instance_name_defaults_to_hostname_when_empty) {
  MdnsDiscovery::Config config;
  // config.name is empty by default - constructor resolves to gethostname()
  MdnsDiscovery discovery(config);
  EXPECT_FALSE(discovery.instance_name().empty());
}

TEST(MdnsDiscovery, instance_name_preserves_set_value_across_lifecycle) {
  MdnsDiscovery::Config config;
  config.name = "perception-ecu";
  MdnsDiscovery discovery(config);
  EXPECT_EQ(discovery.instance_name(), "perception-ecu");
  discovery.stop();
  EXPECT_EQ(discovery.instance_name(), "perception-ecu");
}

// =============================================================================
// Privileged port rejection tests (via AggregationManager integration)
// =============================================================================
// The browse_callback in mdns_discovery.cpp rejects SRV records with port 0
// or port < 1024 before calling the on_found callback. Since browse_callback
// is a static function in an anonymous namespace, we cannot unit test it
// directly. Instead, we verify via AggregationManager that URLs with
// privileged ports constructed outside the mDNS path are still subject to
// address validation (the port check is defense-in-depth at the mDNS layer).

TEST(MdnsDiscovery, privileged_port_documentation) {
  // This test documents the privileged port rejection behavior.
  // browse_callback rejects SRV records with:
  //   - port == 0 (unspecified)
  //   - port < 1024 (privileged, requires root, typically system services)
  // These checks happen BEFORE the on_found callback is invoked, so
  // AggregationManager::add_discovered_peer never sees these URLs.
  //
  // Verify that Config can express the port for announcement correctly
  MdnsDiscovery::Config config;
  config.port = 8080;  // Non-privileged port - valid
  EXPECT_GE(config.port, 1024);

  config.port = 443;  // Privileged port - would be rejected in browse_callback
  EXPECT_LT(config.port, 1024);
  // Note: The announcement port is for our own service and is not validated
  // (the operator intentionally configures it). The check is only in
  // browse_callback for incoming SRV records from peers.
  SUCCEED();
}

// =============================================================================
// Constructor validation tests
// =============================================================================

TEST(MdnsDiscovery, out_of_range_port_triggers_error_callback) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_called = false;
  std::string error_msg;
  config.on_error = [&](const std::string & msg) {
    error_called = true;
    error_msg = msg;
  };

  // Port out of valid uint16_t range should trigger error callback and reset to 8080
  config.port = 70000;
  MdnsDiscovery discovery(config);

  EXPECT_TRUE(error_called) << "Error callback should fire for out-of-range port";
  EXPECT_NE(error_msg.find("out of valid range"), std::string::npos);
}

TEST(MdnsDiscovery, negative_port_triggers_error_callback) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_called = false;
  config.on_error = [&](const std::string & /*msg*/) {
    error_called = true;
  };

  config.port = -1;
  MdnsDiscovery discovery(config);

  EXPECT_TRUE(error_called) << "Error callback should fire for negative port";
}

// =============================================================================
// Socket creation resilience tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, start_announce_on_ephemeral_port_does_not_crash) {
  // Exercise the socket creation path. In CI containers mDNS port 5353 may not
  // be available (requires CAP_NET_BIND_SERVICE or root). The announce_loop should
  // handle the failure gracefully via the error callback rather than crashing.
  MdnsDiscovery::Config config;
  config.announce = true;
  config.discover = false;
  config.port = 0;  // Ephemeral port for our service announcement
  config.name = "test-socket-resilience";

  bool error_reported = false;
  config.on_error = [&](const std::string & /*msg*/) {
    error_reported = true;
  };

  MdnsDiscovery discovery(config);
  discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});

  // Give the thread a moment to attempt socket creation
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Whether or not the socket opened, stop must not hang or crash
  discovery.stop();

  // Either the announce thread ran (socket opened) or the error callback fired
  // (socket failed). Both are acceptable outcomes in CI.
  if (!discovery.is_announcing()) {
    // Socket likely failed - this is expected in restricted CI environments.
    // The important thing is we did not crash.
    SUCCEED();
  }
}

namespace {

// Longer than the 10 s browse interval, so a query sent late still fails the test
constexpr auto kNetDeadline = std::chrono::seconds(15);

constexpr uint16_t kFlagsQuery = 0x0000;
constexpr uint16_t kFlagsResponse = 0x8400;  // QR + AA
constexpr uint16_t kTypePtr = 12;
constexpr uint16_t kTypeSrv = 33;
constexpr uint16_t kClassIn = 1;
constexpr uint16_t kClassInUnicast = 0x8001;  // QU bit

/// Minimal DNS packet builder, so tests control the header flags and sections.
class DnsPacket {
 public:
  explicit DnsPacket(uint16_t flags) : flags_(flags) {
  }

  DnsPacket & question(const std::string & name, uint16_t type, uint16_t qclass = kClassIn) {
    put_name(questions_, name);
    put_u16(questions_, type);
    put_u16(questions_, qclass);
    ++qdcount_;
    return *this;
  }

  DnsPacket & ptr(const std::string & service, const std::string & instance, uint32_t ttl = 120) {
    std::vector<uint8_t> rdata;
    put_name(rdata, instance);
    return record(service, kTypePtr, ttl, rdata);
  }

  DnsPacket & srv(const std::string & instance, uint16_t port, uint32_t ttl = 120) {
    std::vector<uint8_t> rdata;
    put_u16(rdata, 0);
    put_u16(rdata, 0);
    put_u16(rdata, port);
    put_name(rdata, "test-host.local");
    return record(instance, kTypeSrv, ttl, rdata);
  }

  std::vector<uint8_t> bytes() const {
    std::vector<uint8_t> out;
    put_u16(out, 0);
    put_u16(out, flags_);
    put_u16(out, qdcount_);
    put_u16(out, ancount_);
    put_u16(out, 0);
    put_u16(out, 0);
    out.insert(out.end(), questions_.begin(), questions_.end());
    out.insert(out.end(), answers_.begin(), answers_.end());
    return out;
  }

 private:
  DnsPacket & record(const std::string & name, uint16_t type, uint32_t ttl, const std::vector<uint8_t> & rdata) {
    put_name(answers_, name);
    put_u16(answers_, type);
    put_u16(answers_, kClassIn);
    put_u16(answers_, static_cast<uint16_t>(ttl >> 16U));
    put_u16(answers_, static_cast<uint16_t>(ttl & 0xFFFFU));
    put_u16(answers_, static_cast<uint16_t>(rdata.size()));
    answers_.insert(answers_.end(), rdata.begin(), rdata.end());
    ++ancount_;
    return *this;
  }

  static void put_u16(std::vector<uint8_t> & out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v >> 8U));
    out.push_back(static_cast<uint8_t>(v & 0xFFU));
  }

  static void put_name(std::vector<uint8_t> & out, const std::string & name) {
    size_t start = 0;
    while (start < name.size()) {
      size_t dot = name.find('.', start);
      if (dot == std::string::npos) {
        dot = name.size();
      }
      out.push_back(static_cast<uint8_t>(dot - start));
      out.insert(out.end(), name.begin() + static_cast<std::ptrdiff_t>(start),
                 name.begin() + static_cast<std::ptrdiff_t>(dot));
      start = dot + 1;
    }
    out.push_back(0);
  }

  uint16_t flags_;
  uint16_t qdcount_{0};
  uint16_t ancount_{0};
  std::vector<uint8_t> questions_;
  std::vector<uint8_t> answers_;
};

sockaddr_in ipv4(const char * ip, uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  inet_pton(AF_INET, ip, &addr.sin_addr);
  return addr;
}

bool send_packet(int sock, const std::vector<uint8_t> & packet, const sockaddr_in & to) {
  return sendto(sock, packet.data(), packet.size(), 0, reinterpret_cast<const sockaddr *>(&to), sizeof(to)) ==
         static_cast<ssize_t>(packet.size());
}

bool send_multicast(int sock, const std::vector<uint8_t> & packet) {
  return send_packet(sock, packet, ipv4("224.0.0.251", MDNS_PORT));
}

/// A shared udp/5353 socket in the mDNS group, like a peer's responder.
int open_mdns_responder() {
  sockaddr_in saddr{};
  saddr.sin_family = AF_INET;
  saddr.sin_addr.s_addr = INADDR_ANY;
  saddr.sin_port = htons(MDNS_PORT);
  return mdns_socket_open_ipv4(&saddr);
}

bool readable(int sock, std::chrono::milliseconds timeout) {
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sock, &readfds);
  struct timeval tv {};
  tv.tv_sec = timeout.count() / 1000;
  tv.tv_usec = (timeout.count() % 1000) * 1000;
  return select(sock + 1, &readfds, nullptr, nullptr, &tv) > 0;
}

/// Thread-safe record of what an MdnsDiscovery reported.
class PeerLog {
 public:
  void found(const std::string & url, const std::string & name) {
    std::lock_guard<std::mutex> lock(mutex_);
    found_[name] = url;
    cv_.notify_all();
  }
  void removed(const std::string & name) {
    std::lock_guard<std::mutex> lock(mutex_);
    removed_.insert(name);
    cv_.notify_all();
  }
  void log(const std::string & line) {
    std::lock_guard<std::mutex> lock(mutex_);
    logs_.push_back(line);
  }
  bool wait_found(const std::string & name, std::chrono::seconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      return found_.count(name) > 0;
    });
  }
  bool wait_removed(const std::string & name, std::chrono::seconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      return removed_.count(name) > 0;
    });
  }
  std::map<std::string, std::string> found_peers() {
    std::lock_guard<std::mutex> lock(mutex_);
    return found_;
  }
  std::set<std::string> removed_peers() {
    std::lock_guard<std::mutex> lock(mutex_);
    return removed_;
  }
  bool logged(const std::string & needle) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & line : logs_) {
      if (line.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::map<std::string, std::string> found_;
  std::set<std::string> removed_;
  std::vector<std::string> logs_;
};

/// Starts a browse-only MdnsDiscovery wired to a PeerLog, waits until it browses.
class Browser {
 public:
  Browser(const std::string & service, PeerLog & peers, bool announce = false, const std::string & name = "") {
    MdnsDiscovery::Config config;
    config.announce = announce;
    config.discover = true;
    config.service = service;
    config.port = 18080;
    config.name = name.empty() ? "browser-under-test" : name;
    config.on_log = [&peers](const std::string & line) {
      peers.log(line);
    };
    discovery_ = std::make_unique<MdnsDiscovery>(config);
    discovery_->start(
        [&peers](const std::string & url, const std::string & peer) {
          peers.found(url, peer);
        },
        [&peers](const std::string & peer) {
          peers.removed(peer);
        });
  }
  ~Browser() {
    discovery_->stop();
  }
  Browser(const Browser &) = delete;
  Browser & operator=(const Browser &) = delete;
  Browser(Browser &&) = delete;
  Browser & operator=(Browser &&) = delete;

  bool wait_ready(bool announce = false) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (discovery_->is_discovering() && (!announce || discovery_->is_announcing())) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

 private:
  std::unique_ptr<MdnsDiscovery> discovery_;
};

struct QueryCapture {
  std::string service;
  bool seen{false};
  bool unicast{false};
  sockaddr_in from{};
};

int capture_query(int /*sock*/, const struct sockaddr * from, size_t /*addrlen*/, mdns_entry_type_t entry,
                  uint16_t /*query_id*/, uint16_t rtype, uint16_t rclass, uint32_t /*ttl*/, const void * data,
                  size_t size, size_t name_offset, size_t /*name_length*/, size_t /*record_offset*/,
                  size_t /*record_length*/, void * user_data) {
  auto * capture = static_cast<QueryCapture *>(user_data);
  if (entry != MDNS_ENTRYTYPE_QUESTION || rtype != MDNS_RECORDTYPE_PTR || from->sa_family != AF_INET) {
    return 0;
  }
  std::array<char, 256> name_buf{};
  size_t offset = name_offset;
  mdns_string_t name = mdns_string_extract(data, size, &offset, name_buf.data(), name_buf.size());
  if (std::string(name.str, name.length) != capture->service + ".") {
    return 0;
  }
  capture->unicast = (rclass & MDNS_UNICAST_RESPONSE) != 0;
  capture->from = *reinterpret_cast<const sockaddr_in *>(from);
  capture->seen = true;
  return 0;
}

/// Reads mDNS packets on sock until a PTR query for service arrives or the deadline passes.
QueryCapture wait_for_query(int sock, const std::string & service, std::chrono::seconds timeout) {
  QueryCapture capture;
  capture.service = service;
  std::array<char, 2048> buffer{};
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!capture.seen && std::chrono::steady_clock::now() < deadline) {
    if (readable(sock, std::chrono::milliseconds(100))) {
      mdns_socket_listen(sock, buffer.data(), buffer.size(), capture_query, &capture);
    }
  }
  return capture;
}

}  // namespace

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, browse_query_asks_for_multicast_replies) {
  // A unicast-response query makes peers answer to the browser's source port,
  // which a host firewall that only allows the mdns service drops. The browse
  // query must leave the QU bit clear so peers answer on 5353.
  int listener = open_mdns_responder();
  ASSERT_GE(listener, 0) << "cannot bind udp/5353";

  const std::string service = "_medkit-qu-test._tcp.local";
  PeerLog peers;
  QueryCapture capture;
  {
    Browser browser(service, peers);
    capture = wait_for_query(listener, service, kNetDeadline);
  }
  mdns_socket_close(listener);

  ASSERT_TRUE(capture.seen) << "the browse query never arrived on udp/5353";
  EXPECT_FALSE(capture.unicast);
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, browse_finds_peer_answering_only_by_multicast) {
  // A peer behind a firewall that allows only the mdns service answers by
  // multicast to 5353, whatever the QU bit says. The browser must hear it.
  int responder = open_mdns_responder();
  ASSERT_GE(responder, 0) << "cannot bind udp/5353";

  const std::string service = "_medkit-mcast-test._tcp.local";
  PeerLog peers;
  bool found = false;
  {
    Browser browser(service, peers);
    const auto deadline = std::chrono::steady_clock::now() + kNetDeadline;
    while (!found && std::chrono::steady_clock::now() < deadline) {
      auto left = std::chrono::duration_cast<std::chrono::seconds>(deadline - std::chrono::steady_clock::now());
      QueryCapture query = wait_for_query(responder, service, std::max(left, std::chrono::seconds(1)));
      if (query.seen) {
        ASSERT_TRUE(send_multicast(responder, DnsPacket(kFlagsResponse)
                                                  .ptr(service, "mcast-peer." + service)
                                                  .srv("mcast-peer." + service, 18081)
                                                  .bytes()));
        found = peers.wait_found("mcast-peer", std::chrono::seconds(2));
      }
    }
  }
  mdns_socket_close(responder);

  ASSERT_TRUE(found) << "a peer answering only by multicast was not found";
  EXPECT_NE(peers.found_peers()["mcast-peer"].find(":18081"), std::string::npos);
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, browse_ignores_records_of_other_services) {
  // On 5353 the browser sees every mDNS packet on the LAN. SRV records of other
  // service types must neither add peers nor remove ours with a goodbye.
  int sender = open_mdns_responder();
  ASSERT_GE(sender, 0) << "cannot bind udp/5353";

  const std::string service = "_medkit-fleet-b._tcp.local";
  std::string service_upper = service;
  std::transform(service_upper.begin(), service_upper.end(), service_upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  PeerLog peers;
  bool found = false;
  bool removed = false;
  {
    Browser browser(service, peers);
    ASSERT_TRUE(browser.wait_ready());
    ASSERT_TRUE(send_multicast(sender, DnsPacket(kFlagsResponse)
                                           .srv("robot-a." + service, 18082)
                                           .srv("fleet-a._medkit-fleet-a._tcp.local", 18083)
                                           .srv("printer-web._http._tcp.local", 18084)
                                           .srv("spoof.x" + service, 18085)
                                           .srv(service, 18086)
                                           .srv("robot-a._http._tcp.local", 18087, 0)
                                           .bytes()));
    found = peers.wait_found("robot-a", kNetDeadline);
    // Same name, foreign service: must not remove our robot-a. Then a real goodbye
    // in our service, matched case-insensitively, proves goodbyes still work.
    ASSERT_TRUE(send_multicast(sender, DnsPacket(kFlagsResponse).srv("robot-a._http._tcp.local", 18087, 0).bytes()));
    ASSERT_TRUE(send_multicast(sender, DnsPacket(kFlagsResponse).srv("robot-c." + service_upper, 18088, 0).bytes()));
    removed = peers.wait_removed("robot-c", kNetDeadline);
  }
  mdns_socket_close(sender);

  ASSERT_TRUE(found);
  ASSERT_TRUE(removed);
  auto found_peers = peers.found_peers();
  EXPECT_EQ(found_peers.size(), 1U);
  EXPECT_EQ(found_peers.count("robot-a"), 1U);
  EXPECT_EQ(peers.removed_peers(), std::set<std::string>{"robot-c"});
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, browse_ignores_known_answers_in_queries) {
  // Other hosts' queries carry known-answer SRV records for our service. They
  // describe what the asker already knows, not a peer at the asker's address.
  int sender = open_mdns_responder();
  ASSERT_GE(sender, 0) << "cannot bind udp/5353";

  const std::string service = "_medkit-ka-test._tcp.local";
  PeerLog peers;
  bool found = false;
  {
    Browser browser(service, peers);
    ASSERT_TRUE(browser.wait_ready());
    ASSERT_TRUE(send_multicast(sender, DnsPacket(kFlagsQuery)
                                           .question(service, kTypePtr)
                                           .ptr(service, "ka-peer." + service)
                                           .srv("ka-peer." + service, 18090)
                                           .bytes()));
    ASSERT_TRUE(send_multicast(sender, DnsPacket(kFlagsResponse).srv("real-peer." + service, 18091).bytes()));
    found = peers.wait_found("real-peer", kNetDeadline);
  }
  mdns_socket_close(sender);

  ASSERT_TRUE(found);
  EXPECT_EQ(peers.found_peers().count("ka-peer"), 0U);
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, browse_leaves_unicast_to_5353_for_earlier_socket) {
  // avahi binds 5353 first with SO_REUSEADDR. Unicast datagrams to 5353 are
  // its own, and the browse socket must not take them over.
  int earlier = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(earlier, 0);
  const int one = 1;
  setsockopt(earlier, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  // Count only unicast: without this Linux also hands it the browser's group traffic
  const int zero = 0;
  ASSERT_EQ(setsockopt(earlier, IPPROTO_IP, IP_MULTICAST_ALL, &zero, sizeof(zero)), 0);
  sockaddr_in any = ipv4("0.0.0.0", MDNS_PORT);
  ASSERT_EQ(bind(earlier, reinterpret_cast<const sockaddr *>(&any), sizeof(any)), 0) << "cannot bind udp/5353";

  int client = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(client, 0);

  constexpr int kDatagrams = 50;
  int received = 0;
  PeerLog peers;
  {
    Browser browser("_medkit-uc-test._tcp.local", peers);
    ASSERT_TRUE(browser.wait_ready());
    const auto packet = DnsPacket(kFlagsResponse).srv("uc-peer._medkit-uc-test._tcp.local", 18092).bytes();
    for (int i = 0; i < kDatagrams; ++i) {
      ASSERT_TRUE(send_packet(client, packet, ipv4("127.0.0.1", MDNS_PORT)));
    }
    std::array<uint8_t, 2048> buffer{};
    while (received < kDatagrams && readable(earlier, std::chrono::milliseconds(1000))) {
      const ssize_t n = recv(earlier, buffer.data(), buffer.size(), 0);
      if (n == static_cast<ssize_t>(packet.size()) && std::equal(packet.begin(), packet.end(), buffer.begin())) {
        ++received;
      }
    }
  }
  close(client);
  close(earlier);

  ASSERT_TRUE(peers.logged("224.0.0.251:5353")) << "browse socket fell back to an ephemeral port";
  EXPECT_EQ(received, kDatagrams);
  EXPECT_EQ(peers.found_peers().count("uc-peer"), 0U);
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, announce_answers_every_unicast_query_while_browsing) {
  // With announce and browse both on 5353, every unicast QU query to the host
  // must reach the announce socket and get an answer.
  const std::string service = "_medkit-uq-test._tcp.local";
  PeerLog peers;
  constexpr int kQueries = 40;
  int answered = 0;
  {
    Browser browser(service, peers, true, "uq-gateway");
    ASSERT_TRUE(browser.wait_ready(true)) << "cannot bind udp/5353";
    const auto query = DnsPacket(kFlagsQuery).question(service, kTypePtr, kClassInUnicast).bytes();
    std::array<char, 2048> buffer{};
    for (int i = 0; i < kQueries; ++i) {
      // A new socket per query gives a new source port, so a reuseport hash
      // would spread the queries over every socket in the group.
      int client = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
      ASSERT_GE(client, 0);
      ASSERT_TRUE(send_packet(client, query, ipv4("127.0.0.1", MDNS_PORT)));
      if (readable(client, std::chrono::milliseconds(1000)) && recv(client, buffer.data(), buffer.size(), 0) > 0) {
        ++answered;
      }
      close(client);
    }
  }

  EXPECT_EQ(answered, kQueries);
}

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, browse_falls_back_to_ephemeral_port_when_5353_is_taken) {
  // A socket bound to 5353 without SO_REUSEADDR makes the group bind fail. The
  // browser must then query from an ephemeral port, ask for unicast replies,
  // and still find a peer that answers by unicast.
  int blocker = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(blocker, 0);
  ip_mreq req{};
  req.imr_multiaddr.s_addr = htonl((224U << 24U) | 251U);
  req.imr_interface.s_addr = htonl(INADDR_ANY);
  ASSERT_EQ(setsockopt(blocker, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req)), 0);
  sockaddr_in any = ipv4("0.0.0.0", MDNS_PORT);
  ASSERT_EQ(bind(blocker, reinterpret_cast<const sockaddr *>(&any), sizeof(any)), 0)
      << "udp/5353 must be free for this test";

  const std::string service = "_medkit-fallback-test._tcp.local";
  PeerLog peers;
  QueryCapture query;
  bool found = false;
  {
    Browser browser(service, peers);
    ASSERT_TRUE(browser.wait_ready());
    query = wait_for_query(blocker, service, kNetDeadline);
    if (query.seen) {
      ASSERT_TRUE(
          send_packet(blocker, DnsPacket(kFlagsResponse).srv("fallback-peer." + service, 18093).bytes(), query.from));
      found = peers.wait_found("fallback-peer", kNetDeadline);
    }
  }
  close(blocker);

  EXPECT_TRUE(peers.logged("an ephemeral port")) << "browse socket did not report the fallback mode";
  ASSERT_TRUE(query.seen) << "the fallback browse query never arrived";
  EXPECT_TRUE(query.unicast) << "from an ephemeral port the query must ask for unicast replies";
  EXPECT_NE(ntohs(query.from.sin_port), MDNS_PORT);
  EXPECT_TRUE(found) << "the peer answering by unicast to the fallback port was not found";
}
