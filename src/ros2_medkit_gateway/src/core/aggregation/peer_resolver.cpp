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

#include "ros2_medkit_gateway/core/aggregation/peer_resolver.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>

#include <algorithm>
#include <condition_variable>
#include <thread>
#include <utility>

namespace ros2_medkit_gateway {

/// One getaddrinfo call, shared by the thread that makes it and every caller
/// waiting for it. Whoever holds it last frees it, so a caller that stops
/// waiting leaves nothing behind for the thread to write into.
struct PeerResolver::Lookup {
  std::mutex mutex;
  std::condition_variable answered;
  bool done{false};
  bool canceled{false};
  int status{0};
  std::vector<std::string> addresses;
};

namespace {

bool is_ip_address(const std::string & host) {
  in6_addr v6{};
  in_addr v4{};
  return inet_pton(AF_INET, host.c_str(), &v4) == 1 || inet_pton(AF_INET6, host.c_str(), &v6) == 1;
}

}  // namespace

std::optional<PeerHostPort> peer_host_port(const std::string & url) {
  // The way cpp-httplib's Client reads `scheme://host:port`: an optional
  // scheme, then `[v6]` or a host up to `:`, `/`, `?` or `#`, then a port.
  std::string rest = url;
  int default_port = 80;
  if (const auto scheme_end = rest.find("://"); scheme_end != std::string::npos) {
    if (rest.compare(0, scheme_end, "https") == 0) {
      default_port = 443;
    }
    rest = rest.substr(scheme_end + 3);
  }
  PeerHostPort out;
  size_t after_host = 0;
  if (!rest.empty() && rest.front() == '[') {
    const auto close = rest.find(']');
    if (close == std::string::npos) {
      return std::nullopt;
    }
    out.host = rest.substr(1, close - 1);
    after_host = close + 1;
  } else {
    after_host = std::min(rest.find_first_of(":/?#"), rest.size());
    out.host = rest.substr(0, after_host);
  }
  if (out.host.empty()) {
    return std::nullopt;
  }
  out.port = default_port;
  if (after_host < rest.size() && rest[after_host] == ':') {
    const auto port_end = std::min(rest.find_first_of("/?#", after_host + 1), rest.size());
    const auto digits = rest.substr(after_host + 1, port_end - after_host - 1);
    if (!digits.empty() && std::all_of(digits.begin(), digits.end(), [](unsigned char c) {
          return c >= '0' && c <= '9';
        })) {
      out.port = std::stoi(digits);
    }
  }
  return out;
}

PeerResolver::PeerResolver(const std::string & url) {
  if (auto host_port = peer_host_port(url)) {
    host_ = std::move(host_port->host);
    port_ = std::to_string(host_port->port);
    needs_no_lookup_ = is_ip_address(host_);
  }
}

const std::string & PeerResolver::host() const {
  return host_;
}

PeerResolver::Result PeerResolver::resolve(std::chrono::milliseconds timeout) {
  if (needs_no_lookup_) {
    return {};
  }
  std::shared_ptr<Lookup> lookup;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (canceled_) {
      return {Outcome::kCanceled, {}};
    }
    bool start = pending_ == nullptr;
    if (!start) {
      std::lock_guard<std::mutex> pending_lock(pending_->mutex);
      start = pending_->done;
    }
    if (start) {
      pending_ = std::make_shared<Lookup>();
      // Detached on purpose: a lookup the resolver does not answer cannot be
      // interrupted, and joining it is the wait this class exists to bound.
      // The thread owns its share of the lookup and touches nothing else.
      std::thread([lookup = pending_, host = host_, port = port_]() {
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo * found = nullptr;
        const int status = getaddrinfo(host.c_str(), port.c_str(), &hints, &found);
        std::vector<std::string> addresses;
        for (auto * entry = found; status == 0 && entry != nullptr; entry = entry->ai_next) {
          char text[NI_MAXHOST];
          if (getnameinfo(entry->ai_addr, entry->ai_addrlen, text, sizeof(text), nullptr, 0, NI_NUMERICHOST) == 0 &&
              std::find(addresses.begin(), addresses.end(), text) == addresses.end()) {
            addresses.emplace_back(text);
          }
        }
        if (found != nullptr) {
          freeaddrinfo(found);
        }
        std::lock_guard<std::mutex> done_lock(lookup->mutex);
        lookup->status = status;
        lookup->addresses = std::move(addresses);
        lookup->done = true;
        lookup->answered.notify_all();
      }).detach();
    }
    lookup = pending_;
  }
  std::unique_lock<std::mutex> lock(lookup->mutex);
  lookup->answered.wait_for(lock, timeout, [&lookup]() {
    return lookup->done || lookup->canceled;
  });
  if (lookup->canceled) {
    return {Outcome::kCanceled, {}};
  }
  if (!lookup->done) {
    return {Outcome::kTimedOut, {}};
  }
  if (lookup->status != 0 || lookup->addresses.empty()) {
    return {Outcome::kFailed, {}};
  }
  return {Outcome::kResolved, lookup->addresses};
}

void PeerResolver::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_ = true;
  if (pending_) {
    std::lock_guard<std::mutex> pending_lock(pending_->mutex);
    pending_->canceled = true;
    pending_->answered.notify_all();
  }
}

}  // namespace ros2_medkit_gateway
