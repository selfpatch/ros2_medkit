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

// Define MDNS_IMPLEMENTATION in exactly one translation unit to get the
// implementation of the mjansson/mdns header-only C library.
#define MDNS_IMPLEMENTATION

// Suppress compiler warnings for vendored C header
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
// The mdns.h header is C code with its own extern "C" guards
#include "mdns.h"
#pragma GCC diagnostic pop

#include "ros2_medkit_gateway/aggregation/mdns_discovery.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/discovery/host_info_provider.hpp"

namespace ros2_medkit_gateway {

namespace {

// Buffer size for mDNS packet assembly/parsing (must be 32-bit aligned)
constexpr size_t kMdnsBufferSize = 2048;

// How often the browse thread sends queries (seconds)
constexpr int kBrowseIntervalSec = 10;

// Poll interval for socket reads (milliseconds)
constexpr int kPollIntervalMs = 500;

/**
 * @brief Context passed to mDNS browse callback via user_data pointer
 */
struct BrowseContext {
  MdnsDiscovery::PeerFoundCallback * on_found;
  MdnsDiscovery::PeerRemovedCallback * on_removed;
  MdnsDiscovery::LogCallback * on_log;
  std::string service_name;
  std::string peer_scheme;
};

/**
 * @brief Context passed to mDNS announce callback via user_data pointer
 */
struct AnnounceContext {
  MdnsDiscovery::Config * config;
  MdnsDiscovery::LogCallback * on_log;
};

/**
 * @brief Get the local hostname
 * @return Hostname string, or "unknown" on failure
 */
std::string get_hostname() {
  std::array<char, 256> buf{};
  if (gethostname(buf.data(), buf.size()) == 0) {
    return std::string(buf.data());
  }
  return "unknown";
}

/**
 * @brief mDNS callback for browse/query responses
 *
 * Called by mdns_query_recv() for each record in a response. We look for
 * SRV records that match our service type and extract the hostname and port
 * to build a peer URL.
 */
int browse_callback(int /*sock*/, const struct sockaddr * from, size_t addrlen, mdns_entry_type_t entry,
                    uint16_t /*query_id*/, uint16_t rtype, uint16_t /*rclass*/, uint32_t ttl, const void * data,
                    size_t size, size_t name_offset, size_t /*name_length*/, size_t record_offset, size_t record_length,
                    void * user_data) {
  auto * ctx = static_cast<BrowseContext *>(user_data);

  // We only care about answer records with SRV type
  if (entry != MDNS_ENTRYTYPE_ANSWER && entry != MDNS_ENTRYTYPE_ADDITIONAL) {
    return 0;
  }

  if (rtype != MDNS_RECORDTYPE_SRV) {
    if (ctx->on_log && *ctx->on_log) {
      (*ctx->on_log)("browse: ignoring record type " + std::to_string(rtype) +
                     " (want SRV=" + std::to_string(MDNS_RECORDTYPE_SRV) + ")");
    }
    return 0;
  }

  std::array<char, 256> name_buf{};
  mdns_record_srv_t srv =
      mdns_record_parse_srv(data, size, record_offset, record_length, name_buf.data(), name_buf.size());

  // Extract the instance name from the record name
  std::array<char, 256> entry_name_buf{};
  size_t name_off = name_offset;
  mdns_string_t entry_name = mdns_string_extract(data, size, &name_off, entry_name_buf.data(), entry_name_buf.size());

  std::string instance_name(entry_name.str, entry_name.length);
  std::string target(srv.name.str, srv.name.length);

  // Build peer URL from the source address
  std::array<char, INET6_ADDRSTRLEN> addr_buf{};
  std::string addr_str;

  if (from->sa_family == AF_INET) {
    const auto * addr4 = reinterpret_cast<const struct sockaddr_in *>(from);
    inet_ntop(AF_INET, &addr4->sin_addr, addr_buf.data(), addr_buf.size());
    addr_str = addr_buf.data();
  } else if (from->sa_family == AF_INET6) {
    const auto * addr6 = reinterpret_cast<const struct sockaddr_in6 *>(from);
    inet_ntop(AF_INET6, &addr6->sin6_addr, addr_buf.data(), addr_buf.size());
    addr_str = "[" + std::string(addr_buf.data()) + "]";
  } else {
    (void)addrlen;  // Suppress unused parameter warning
    return 0;
  }
  (void)addrlen;  // Suppress unused parameter warning in non-early-return paths

  // Reject privileged ports (< 1024), which includes port 0 (unspecified).
  // Privileged ports require root and are typically system services (SSH, HTTP, DNS)
  // that should never be SOVD peer gateways discovered via mDNS.
  if (srv.port < 1024) {
    if (ctx->on_log && *ctx->on_log) {
      (*ctx->on_log)("browse: rejecting SRV record with port " + std::to_string(srv.port) +
                     " (must be >= 1024), instance='" + instance_name + "'");
    }
    return 0;
  }

  std::string url = ctx->peer_scheme + "://" + addr_str + ":" + std::to_string(srv.port);

  // Use the instance name (before the service type) as the peer name
  // Instance names look like "gateway-name._medkit._tcp.local"
  std::string peer_name = instance_name;
  auto dot_pos = peer_name.find('.');
  if (dot_pos != std::string::npos) {
    peer_name = peer_name.substr(0, dot_pos);
  }

  // Sanitize peer name to valid entity ID characters
  peer_name = HostInfoProvider::sanitize_entity_id(peer_name);
  if (peer_name.empty()) {
    if (ctx->on_log && *ctx->on_log) {
      (*ctx->on_log)("browse: peer name empty after sanitization, instance='" + instance_name + "'");
    }
    return 0;
  }

  // TTL=0 is an mDNS "goodbye" - the service is departing the network
  if (ttl == 0) {
    if (ctx->on_log && *ctx->on_log) {
      (*ctx->on_log)("browse: received goodbye (TTL=0) for peer '" + peer_name + "' (instance='" + instance_name +
                     "')");
    }
    if (ctx->on_removed && *ctx->on_removed) {
      (*ctx->on_removed)(peer_name);
    }
    return 0;
  }

  if (ctx->on_log && *ctx->on_log) {
    (*ctx->on_log)("browse: discovered peer '" + peer_name + "' at " + url + " (instance='" + instance_name +
                   "', target='" + target + "', port=" + std::to_string(srv.port) + ")");
  }

  if (ctx->on_found) {
    (*ctx->on_found)(url, peer_name);
  }

  return 0;
}

/**
 * @brief mDNS callback for announce/listen - responds to queries for our service
 */
int announce_callback(int sock, const struct sockaddr * from, size_t addrlen, mdns_entry_type_t entry,
                      uint16_t query_id, uint16_t rtype, uint16_t rclass, uint32_t /*ttl*/, const void * data,
                      size_t size, size_t name_offset, size_t /*name_length*/, size_t /*record_offset*/,
                      size_t /*record_length*/, void * user_data) {
  auto * ctx = static_cast<AnnounceContext *>(user_data);
  auto * config = ctx->config;

  // We only care about incoming questions
  if (entry != MDNS_ENTRYTYPE_QUESTION) {
    if (ctx->on_log && *ctx->on_log) {
      (*ctx->on_log)("announce: ignoring non-question entry type " + std::to_string(static_cast<int>(entry)));
    }
    return 0;
  }

  // Extract the queried name
  std::array<char, 256> name_buf{};
  size_t name_off = name_offset;
  mdns_string_t name = mdns_string_extract(data, size, &name_off, name_buf.data(), name_buf.size());
  std::string queried_name(name.str, name.length);

  // Check if the query matches our service type (case-insensitive per RFC 1035)
  std::string queried_lower = queried_name;
  std::string service_lower = config->service;
  std::transform(queried_lower.begin(), queried_lower.end(), queried_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::transform(service_lower.begin(), service_lower.end(), service_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (queried_lower.find(service_lower) == std::string::npos && rtype != MDNS_RECORDTYPE_ANY) {
    if (ctx->on_log && *ctx->on_log) {
      (*ctx->on_log)("announce: query name '" + queried_name + "' (rtype=" + std::to_string(rtype) +
                     ") does not match service '" + config->service + "', ignoring");
    }
    return 0;
  }

  if (ctx->on_log && *ctx->on_log) {
    std::array<char, INET6_ADDRSTRLEN> from_buf{};
    std::string from_str = "unknown";
    if (from->sa_family == AF_INET) {
      const auto * addr4 = reinterpret_cast<const struct sockaddr_in *>(from);
      inet_ntop(AF_INET, &addr4->sin_addr, from_buf.data(), from_buf.size());
      from_str = std::string(from_buf.data()) + ":" + std::to_string(ntohs(addr4->sin_port));
    }
    bool unicast = (rclass & MDNS_UNICAST_RESPONSE) != 0;
    (*ctx->on_log)("announce: received query for '" + queried_name + "' (rtype=" + std::to_string(rtype) +
                   ", unicast=" + (unicast ? "true" : "false") + ") from " + from_str + ", sending response");
  }

  // Build the response per DNS-SD (RFC 6763):
  // - Answer: PTR record (service type -> instance name)
  // - Additional: SRV record (instance name -> host:port)
  std::string hostname = get_hostname();
  std::string instance = config->name + "." + config->service;

  // PTR answer: maps service type to our instance name
  mdns_record_t answer{};
  answer.name.str = config->service.c_str();
  answer.name.length = config->service.size();
  answer.type = MDNS_RECORDTYPE_PTR;
  answer.data.ptr.name.str = instance.c_str();
  answer.data.ptr.name.length = instance.size();
  answer.rclass = 0;
  answer.ttl = 120;

  // SRV additional: maps instance name to hostname and port
  mdns_record_t additional{};
  additional.name.str = instance.c_str();
  additional.name.length = instance.size();
  additional.type = MDNS_RECORDTYPE_SRV;
  additional.data.srv.priority = 0;
  additional.data.srv.weight = 0;
  additional.data.srv.port = static_cast<uint16_t>(config->port);
  additional.data.srv.name.str = hostname.c_str();
  additional.data.srv.name.length = hostname.size();
  additional.rclass = 0;
  additional.ttl = 120;

  // Send the response
  std::array<char, kMdnsBufferSize> buffer{};
  bool unicast = (rclass & MDNS_UNICAST_RESPONSE) != 0;

  int send_result = 0;
  if (unicast) {
    send_result = mdns_query_answer_unicast(sock, from, addrlen, buffer.data(), buffer.size(), query_id,
                                            static_cast<mdns_record_type_t>(rtype), name.str, name.length, answer,
                                            nullptr, 0, &additional, 1);
  } else {
    send_result = mdns_query_answer_multicast(sock, buffer.data(), buffer.size(), answer, nullptr, 0, &additional, 1);
  }

  if (ctx->on_log && *ctx->on_log) {
    if (send_result < 0) {
      (*ctx->on_log)("announce: FAILED to send " + std::string(unicast ? "unicast" : "multicast") +
                     " response (result=" + std::to_string(send_result) + ", errno=" + std::to_string(errno) + ")");
    } else {
      (*ctx->on_log)("announce: sent " + std::string(unicast ? "unicast" : "multicast") + " response for instance '" +
                     instance + "' (port " + std::to_string(config->port) + ")");
    }
  }

  return 0;
}

}  // namespace

MdnsDiscovery::MdnsDiscovery(const Config & config) : config_(config) {
  if (config_.name.empty()) {
    config_.name = get_hostname();
  }
  // Validate port range for uint16_t safety
  if (config_.port < 0 || config_.port > 65535) {
    if (config_.on_error) {
      config_.on_error("mDNS config port " + std::to_string(config_.port) +
                       " is out of valid range (0-65535), using default 8080");
    }
    config_.port = 8080;
  }
}

MdnsDiscovery::~MdnsDiscovery() {
  stop();
}

void MdnsDiscovery::start(PeerFoundCallback on_found, PeerRemovedCallback on_removed) {
  if (running_.load()) {
    return;
  }

  on_found_ = std::move(on_found);
  on_removed_ = std::move(on_removed);
  running_.store(true);

  if (config_.announce) {
    announce_thread_ = std::thread([this]() {
      announce_loop();
    });
  }

  if (config_.discover) {
    browse_thread_ = std::thread([this]() {
      browse_loop();
    });
  }
}

void MdnsDiscovery::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  if (announce_thread_.joinable()) {
    announce_thread_.join();
  }
  announcing_.store(false);

  if (browse_thread_.joinable()) {
    browse_thread_.join();
  }
  discovering_.store(false);
}

bool MdnsDiscovery::is_announcing() const {
  return announcing_.load();
}

bool MdnsDiscovery::is_discovering() const {
  return discovering_.load();
}

void MdnsDiscovery::announce_loop() {
  // Open a socket on the mDNS port (5353) to listen for queries
  struct sockaddr_in saddr {};
  saddr.sin_family = AF_INET;
  saddr.sin_addr.s_addr = INADDR_ANY;
  saddr.sin_port = htons(MDNS_PORT);

  int sock = mdns_socket_open_ipv4(&saddr);
  if (sock < 0) {
    if (config_.on_error) {
      config_.on_error(
          "Failed to open mDNS announce socket on port 5353. "
          "Check permissions (CAP_NET_BIND_SERVICE) or use static peers.");
    }
    return;
  }

  announcing_.store(true);

  if (config_.on_log) {
    config_.on_log("announce_loop: socket opened on port 5353 (fd=" + std::to_string(sock) + ", name='" + config_.name +
                   "', service='" + config_.service + "')");
  }

  // Send an initial announcement
  {
    std::string hostname = get_hostname();
    std::string instance = config_.name + "." + config_.service;

    mdns_record_t answer{};
    answer.name.str = instance.c_str();
    answer.name.length = instance.size();
    answer.type = MDNS_RECORDTYPE_SRV;
    answer.data.srv.priority = 0;
    answer.data.srv.weight = 0;
    answer.data.srv.port = static_cast<uint16_t>(config_.port);
    answer.data.srv.name.str = hostname.c_str();
    answer.data.srv.name.length = hostname.size();
    answer.rclass = 0;
    answer.ttl = 120;

    std::array<char, kMdnsBufferSize> buffer{};
    int ann_result = mdns_announce_multicast(sock, buffer.data(), buffer.size(), answer, nullptr, 0, nullptr, 0);
    if (config_.on_log) {
      if (ann_result < 0) {
        config_.on_log("announce_loop: initial announcement FAILED (result=" + std::to_string(ann_result) +
                       ", errno=" + std::to_string(errno) + ")");
      } else {
        config_.on_log("announce_loop: initial announcement sent for '" + instance + "'");
      }
    }
  }

  // Listen for incoming queries and respond
  AnnounceContext ctx;
  ctx.config = &config_;
  ctx.on_log = &config_.on_log;

  std::array<char, kMdnsBufferSize> buffer{};
  size_t loop_count = 0;
  while (running_.load()) {
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = kPollIntervalMs * 1000;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);

    int ret = select(sock + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;  // Signal interrupted select(), retry
      }
      if (config_.on_log) {
        config_.on_log("announce_loop: select() error (errno=" + std::to_string(errno) + ")");
      }
      break;
    }
    if (ret > 0) {
      if (config_.on_log) {
        config_.on_log("announce_loop: select() returned data, calling mdns_socket_listen");
      }
      size_t records = mdns_socket_listen(sock, buffer.data(), buffer.size(), announce_callback, &ctx);
      if (config_.on_log) {
        config_.on_log("announce_loop: mdns_socket_listen processed " + std::to_string(records) + " records");
      }
    }

    ++loop_count;
    // Log heartbeat every ~30 seconds (60 iterations * 500ms)
    if (config_.on_log && (loop_count % 60) == 0) {
      config_.on_log("announce_loop: alive, " + std::to_string(loop_count) + " iterations");
    }
  }

  if (config_.on_log) {
    config_.on_log("announce_loop: exiting (running=" + std::string(running_.load() ? "true" : "false") + ")");
  }

  // Send goodbye announcement before closing
  {
    std::string hostname = get_hostname();
    std::string instance = config_.name + "." + config_.service;

    mdns_record_t answer{};
    answer.name.str = instance.c_str();
    answer.name.length = instance.size();
    answer.type = MDNS_RECORDTYPE_SRV;
    answer.data.srv.priority = 0;
    answer.data.srv.weight = 0;
    answer.data.srv.port = static_cast<uint16_t>(config_.port);
    answer.data.srv.name.str = hostname.c_str();
    answer.data.srv.name.length = hostname.size();
    answer.rclass = 0;
    answer.ttl = 0;  // TTL=0 for goodbye

    std::array<char, kMdnsBufferSize> buf{};
    mdns_goodbye_multicast(sock, buf.data(), buf.size(), answer, nullptr, 0, nullptr, 0);
  }

  mdns_socket_close(sock);
  announcing_.store(false);
}

void MdnsDiscovery::browse_loop() {
  // Open a socket on an ephemeral port for sending queries
  int sock = mdns_socket_open_ipv4(nullptr);
  if (sock < 0) {
    if (config_.on_error) {
      config_.on_error(
          "Failed to open mDNS browse socket. "
          "Check network availability or use static peers.");
    }
    return;
  }

  discovering_.store(true);

  if (config_.on_log) {
    config_.on_log("browse_loop: socket opened on ephemeral port (fd=" + std::to_string(sock) + ")");
  }

  BrowseContext ctx;
  ctx.on_found = &on_found_;
  ctx.on_removed = &on_removed_;
  ctx.on_log = &config_.on_log;
  ctx.service_name = config_.service;
  ctx.peer_scheme = config_.peer_scheme;

  std::array<char, kMdnsBufferSize> buffer{};
  auto last_query = std::chrono::steady_clock::now() - std::chrono::seconds(kBrowseIntervalSec);

  size_t loop_count = 0;
  while (running_.load()) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_query).count();

    // Send a query periodically
    if (elapsed >= kBrowseIntervalSec) {
      int query_result = mdns_query_send(sock, MDNS_RECORDTYPE_PTR, config_.service.c_str(), config_.service.size(),
                                         buffer.data(), buffer.size(), 0);
      if (config_.on_log) {
        if (query_result < 0) {
          config_.on_log("browse_loop: query_send FAILED (result=" + std::to_string(query_result) +
                         ", errno=" + std::to_string(errno) + ")");
        } else {
          config_.on_log("browse_loop: sent PTR query for '" + config_.service + "'");
        }
      }
      last_query = now;
    }

    // Check for responses with a short timeout
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = kPollIntervalMs * 1000;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);

    int ret = select(sock + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;  // Signal interrupted select(), retry
      }
      if (config_.on_log) {
        config_.on_log("browse_loop: select() error (errno=" + std::to_string(errno) + ")");
      }
      break;
    }
    if (ret > 0) {
      if (config_.on_log) {
        config_.on_log("browse_loop: select() returned data, calling mdns_query_recv");
      }
      size_t records = mdns_query_recv(sock, buffer.data(), buffer.size(), browse_callback, &ctx, 0);
      if (config_.on_log) {
        config_.on_log("browse_loop: mdns_query_recv processed " + std::to_string(records) + " records");
      }
    }

    ++loop_count;
    if (config_.on_log && (loop_count % 60) == 0) {
      config_.on_log("browse_loop: alive, " + std::to_string(loop_count) + " iterations");
    }
  }

  if (config_.on_log) {
    config_.on_log("browse_loop: exiting");
  }

  mdns_socket_close(sock);
  discovering_.store(false);
}

}  // namespace ros2_medkit_gateway
