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

// OpcuaClient across a link that goes away and comes back, against the real
// test_alarm_server behind a TCP proxy the test can blackhole or cut. The
// server keeps the session alive the whole time, which is what a PLC behind a
// flapping switch port does and what open62541 then tries to re-activate.

#include "ros2_medkit_opcua/opcua_client.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ros2_medkit_gateway {
namespace {

using namespace std::chrono_literals;

int listen_loopback(uint16_t * port) {
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

uint16_t free_port() {
  uint16_t port = 0;
  int fd = listen_loopback(&port);
  ::close(fd);
  return port;
}

/// test_alarm_server as a child process, driven over its stdin.
class AlarmServer {
 public:
  bool start(const std::string & binary) {
    for (int attempt = 0; attempt < 3; ++attempt) {
      port_ = free_port();
      int in[2];
      int out[2];
      if (::pipe(in) != 0 || ::pipe(out) != 0) {
        return false;
      }
      pid_ = ::fork();
      if (pid_ == 0) {
        ::dup2(in[0], STDIN_FILENO);
        ::dup2(out[1], STDOUT_FILENO);
        ::close(in[1]);
        ::close(out[0]);
        const std::string port = std::to_string(port_);
        ::execl(binary.c_str(), binary.c_str(), "--port", port.c_str(), static_cast<char *>(nullptr));
        ::_exit(127);
      }
      ::close(in[0]);
      ::close(out[1]);
      stdin_ = ::fdopen(in[1], "w");
      stdout_fd_ = out[0];
      if (wait_for_line("READY", 15s)) {
        return true;
      }
      stop();
    }
    return false;
  }

  void command(const std::string & line) {
    std::fprintf(stdin_, "%s\n", line.c_str());
    std::fflush(stdin_);
  }

  uint16_t port() const {
    return port_;
  }

  uint16_t namespace_index() const {
    return namespace_index_;
  }

  void stop() {
    if (stdin_ != nullptr) {
      std::fclose(stdin_);
      stdin_ = nullptr;
    }
    if (pid_ > 0) {
      ::kill(pid_, SIGTERM);
      ::waitpid(pid_, nullptr, 0);
      pid_ = -1;
    }
    if (stdout_fd_ >= 0) {
      ::close(stdout_fd_);
      stdout_fd_ = -1;
    }
  }

  ~AlarmServer() {
    stop();
  }

 private:
  bool wait_for_line(const std::string & prefix, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::string buffer;
    while (std::chrono::steady_clock::now() < deadline) {
      pollfd p{stdout_fd_, POLLIN, 0};
      if (::poll(&p, 1, 100) <= 0) {
        continue;
      }
      char chunk[256];
      ssize_t n = ::read(stdout_fd_, chunk, sizeof(chunk));
      if (n <= 0) {
        return false;
      }
      buffer.append(chunk, static_cast<size_t>(n));
      const auto at = buffer.find(prefix);
      if (at != std::string::npos && buffer.find('\n', at) != std::string::npos) {
        const auto ns_at = buffer.find("namespace=", at);
        if (ns_at != std::string::npos) {
          namespace_index_ = static_cast<uint16_t>(std::stoi(buffer.substr(ns_at + 10)));
        }
        return true;
      }
    }
    return false;
  }

  pid_t pid_{-1};
  uint16_t port_{0};
  uint16_t namespace_index_{0};
  FILE * stdin_{nullptr};
  int stdout_fd_{-1};
};

/// TCP proxy in front of the server. ``blackhole`` keeps every connection open
/// and forwards nothing (a dead switch port), ``cut`` closes them (a pulled
/// cable). The server never notices either, so its session stays alive.
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

  void set_mode(Mode mode) {
    mode_ = mode;
  }

  void cut() {
    cut_requested_ = true;
    while (cut_requested_) {
      std::this_thread::sleep_for(10ms);
    }
  }

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
          ::close(a);
          ::close(b);
        }
        pairs_.clear();
        cut_requested_ = false;
      }
      std::vector<pollfd> fds;
      fds.push_back({listen_fd_, POLLIN, 0});
      const bool forward = mode_ == Mode::Up;
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
        for (int side = 0; side < 2; ++side) {
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
  std::atomic<bool> cut_requested_{false};
  std::vector<std::pair<int, int>> pairs_;
  std::thread thread_;
};

const opcua::NodeId kServerObject(0, UA_NS0ID_SERVER);
const opcua::NodeId kServerCurrentTime(0, UA_NS0ID_SERVER_SERVERSTATUS_CURRENTTIME);

class OpcuaClientReconnect : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(server_.start(TEST_ALARM_SERVER_BIN)) << "test_alarm_server did not start";
    ASSERT_TRUE(proxy_.start(server_.port()));
    config_.endpoint_url = "opc.tcp://127.0.0.1:" + std::to_string(proxy_.port());
    config_.connect_timeout = 1000ms;
    ASSERT_TRUE(client_.connect(config_));
  }

  /// Event subscription on the Server object, counting into ``counter``.
  void subscribe(std::atomic<int> & counter) {
    const uint32_t sub = client_.create_subscription(50.0, [](const std::string &, const OpcuaValue &) {});
    ASSERT_NE(sub, 0u);
    const uint32_t mi = client_.add_event_monitored_item(
        sub, kServerObject, {}, [&counter](const auto &, const auto &, const auto &, const auto &) {
          counter.fetch_add(1);
        });
    ASSERT_NE(mi, 0u);
  }

  bool pump_until(const std::function<bool()> & done, std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      client_.run_iterate(20);
      if (done()) {
        return true;
      }
    }
    return done();
  }

  /// Fires one server event and waits until ``counter`` moves past ``before``.
  bool fire_and_wait(std::atomic<int> & counter) {
    const int before = counter.load();
    server_.command("sysevent");
    return pump_until([&] {
      return counter.load() > before;
    });
  }

  // Members, not test locals: an item still held at the end of a test can
  // deliver while the client is torn down.
  std::atomic<int> old_events{0};
  std::atomic<int> new_events{0};
  std::atomic<int> round_events[5]{};

  AlarmServer server_;
  LinkProxy proxy_;
  OpcuaClientConfig config_;
  OpcuaClient client_;
};

// A read that times out on a blackholed link marks the client disconnected.
// The reconnect rebuilds the client while open62541 still holds the old
// session and its subscriptions; nothing of them may be touched after it
// freed them, and the old event item must not deliver on the new session.
TEST_F(OpcuaClientReconnect, BlackholedReadThenReconnect) {
  subscribe(old_events);
  ASSERT_TRUE(fire_and_wait(old_events));

  proxy_.set_mode(LinkProxy::Mode::Blackhole);
  EXPECT_FALSE(client_.read_value(kServerCurrentTime).good);
  ASSERT_FALSE(client_.is_connected());
  proxy_.set_mode(LinkProxy::Mode::Up);
  proxy_.cut();

  ASSERT_TRUE(client_.connect(config_));
  subscribe(new_events);
  const int old_before = old_events.load();
  ASSERT_TRUE(fire_and_wait(new_events));
  // A second round so a late delivery to the old item has had its chance.
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), old_before);
}

// A read the server answers with BadNotConnected marks the client
// disconnected while its channel and session stay up. The reconnect then finds
// the channel open, and the old subscription keeps publishing on that session:
// its event item must still be safe to deliver to, and must stay silent.
TEST_F(OpcuaClientReconnect, NotConnectedReadThenReconnect) {
  subscribe(old_events);
  ASSERT_TRUE(fire_and_wait(old_events));

  server_.command("notconnected StatusWord");
  const opcua::NodeId status_word(server_.namespace_index(), "StatusWord");
  ASSERT_TRUE(pump_until([&] {
    return !client_.read_value(status_word).good;
  }));
  ASSERT_FALSE(client_.is_connected());

  ASSERT_TRUE(client_.connect(config_));
  subscribe(new_events);
  const int old_before = old_events.load();
  ASSERT_TRUE(fire_and_wait(new_events));
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), old_before);
}

// Same, with the read failing on a link that was cut instead.
TEST_F(OpcuaClientReconnect, CutLinkReadThenReconnect) {
  subscribe(old_events);
  ASSERT_TRUE(fire_and_wait(old_events));

  proxy_.cut();
  EXPECT_FALSE(client_.read_value(kServerCurrentTime).good);
  ASSERT_FALSE(client_.is_connected());

  ASSERT_TRUE(client_.connect(config_));
  subscribe(new_events);
  const int old_before = old_events.load();
  ASSERT_TRUE(fire_and_wait(new_events));
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), old_before);
}

// The event pump sees the cut first (run_iterate), the reconnect follows.
TEST_F(OpcuaClientReconnect, CutLinkSeenByIterateThenReconnect) {
  subscribe(old_events);
  ASSERT_TRUE(fire_and_wait(old_events));

  proxy_.cut();
  ASSERT_TRUE(pump_until([&] {
    return !client_.is_connected();
  }));

  ASSERT_TRUE(client_.connect(config_));
  subscribe(new_events);
  const int old_before = old_events.load();
  ASSERT_TRUE(fire_and_wait(new_events));
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), old_before);
}

// Repeated flaps on one client, each followed by a reconnect and resubscribe,
// as the poller does on a link with a bad cable.
TEST_F(OpcuaClientReconnect, RepeatedFlaps) {
  for (int round = 0; round < 5; ++round) {
    auto & events = round_events[round];
    subscribe(events);
    ASSERT_TRUE(fire_and_wait(events)) << "round " << round;
    if (round % 2 == 0) {
      proxy_.set_mode(LinkProxy::Mode::Blackhole);
      EXPECT_FALSE(client_.read_value(kServerCurrentTime).good);
      proxy_.set_mode(LinkProxy::Mode::Up);
    }
    proxy_.cut();
    pump_until([&] {
      return !client_.is_connected();
    });
    ASSERT_FALSE(client_.is_connected()) << "round " << round;
    ASSERT_TRUE(client_.connect(config_)) << "round " << round;
  }
}

}  // namespace
}  // namespace ros2_medkit_gateway
