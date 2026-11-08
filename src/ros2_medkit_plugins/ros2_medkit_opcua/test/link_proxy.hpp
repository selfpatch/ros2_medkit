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

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace test_support {

/// Listening socket on 127.0.0.1 and an ephemeral port, written to ``port``.
/// Returns -1 when the bind or listen fails.
inline int listen_loopback(uint16_t * port) {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 || ::listen(fd, 16) != 0) {
    ::close(fd);
    return -1;
  }
  socklen_t len = sizeof(addr);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
  *port = ntohs(addr.sin_port);
  return fd;
}

/// TCP proxy in front of an OPC UA server. ``blackhole`` keeps every connection
/// open and forwards nothing (a dead switch port), ``cut`` closes them (a pulled
/// cable) and ``reset`` aborts them with an RST (a peer that drops the
/// connection). The server never notices any of them, so its session stays
/// alive.
class LinkProxy {
 public:
  enum class Mode { Up, Blackhole };

  bool start(uint16_t target_port) {
    target_port_ = target_port;
    listen_fd_ = listen_loopback(&port_);
    if (listen_fd_ < 0) {
      return false;
    }
    running_ = true;
    thread_ = std::thread([this] {
      loop();
    });
    return true;
  }

  uint16_t port() const {
    return port_;
  }

  /// Returns once the proxy runs a pass under ``mode``, so nothing sent
  /// after this call is forwarded under the old one.
  void set_mode(Mode mode) {
    mode_ = mode;
    const uint64_t wanted = ++mode_requests_;
    while (mode_applied_.load() < wanted) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void cut() {
    cut_requested_ = true;
    while (cut_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// Like cut(), but the client side gets an RST instead of a FIN.
  void reset() {
    abortive_ = true;
    cut();
  }

  LinkProxy() = default;
  // Owns the listening socket, every proxied pair and the thread serving them.
  LinkProxy(const LinkProxy &) = delete;
  LinkProxy & operator=(const LinkProxy &) = delete;
  LinkProxy(LinkProxy &&) = delete;
  LinkProxy & operator=(LinkProxy &&) = delete;

  ~LinkProxy() {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    for (auto & [a, b] : pairs_) {
      ::close(a);
      ::close(b);
    }
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
    }
  }

 private:
  void loop() {
    while (running_) {
      if (cut_requested_) {
        for (auto & [a, b] : pairs_) {
          if (abortive_) {
            linger lg{1, 0};
            ::setsockopt(a, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
          }
          ::close(a);
          ::close(b);
        }
        pairs_.clear();
        abortive_ = false;
        cut_requested_ = false;
      }
      // Request count first, mode second: a pass that saw a request also
      // sees the mode that came with it.
      const uint64_t requests = mode_requests_.load();
      const bool forward = mode_ == Mode::Up;
      mode_applied_ = requests;
      std::vector<pollfd> fds;
      fds.push_back({listen_fd_, POLLIN, 0});
      for (auto & [a, b] : pairs_) {
        fds.push_back({a, static_cast<short>(forward ? POLLIN : 0), 0});
        fds.push_back({b, static_cast<short>(forward ? POLLIN : 0), 0});
      }
      if (::poll(fds.data(), fds.size(), 20) <= 0) {
        continue;
      }
      const size_t polled = pairs_.size();
      if (fds[0].revents & POLLIN) {
        int client = ::accept(listen_fd_, nullptr, nullptr);
        int upstream = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(target_port_);
        if (::connect(upstream, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
          pairs_.emplace_back(client, upstream);
        } else {
          ::close(client);
          ::close(upstream);
        }
      }
      std::vector<size_t> dead;
      for (size_t i = 0; i < polled; ++i) {
        const auto & [a, b] = pairs_[i];
        for (size_t side = 0; side < 2; ++side) {
          const pollfd & p = fds[1 + 2 * i + side];
          if ((p.revents & (POLLIN | POLLHUP | POLLERR)) == 0) {
            continue;
          }
          char buf[16384];
          const int from = side == 0 ? a : b;
          const int to = side == 0 ? b : a;
          ssize_t n = ::read(from, buf, sizeof(buf));
          if (n <= 0 || ::send(to, buf, static_cast<size_t>(n), MSG_NOSIGNAL) != n) {
            dead.push_back(i);
            break;
          }
        }
      }
      for (auto it = dead.rbegin(); it != dead.rend(); ++it) {
        ::close(pairs_[*it].first);
        ::close(pairs_[*it].second);
        pairs_.erase(pairs_.begin() + static_cast<std::ptrdiff_t>(*it));
      }
    }
  }

  uint16_t target_port_{0};
  uint16_t port_{0};
  int listen_fd_{-1};
  std::atomic<bool> running_{false};
  std::atomic<Mode> mode_{Mode::Up};
  std::atomic<uint64_t> mode_requests_{0};
  std::atomic<uint64_t> mode_applied_{0};
  std::atomic<bool> cut_requested_{false};
  std::atomic<bool> abortive_{false};
  std::vector<std::pair<int, int>> pairs_;
  std::thread thread_;
};

}  // namespace test_support
}  // namespace ros2_medkit_gateway
