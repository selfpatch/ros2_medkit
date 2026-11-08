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

#include "link_proxy.hpp"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace {

using namespace std::chrono_literals;
using test_support::LinkProxy;
using test_support::listen_loopback;

uint16_t free_port() {
  uint16_t port = 0;
  int fd = listen_loopback(&port);
  ::close(fd);
  return port;
}

/// test_alarm_server as a child process, driven over its stdin. Its stdout,
/// command replies and open62541's log both, is drained for the child's whole
/// life so a full pipe can never stall the server.
class AlarmServer {
 public:
  bool start(const std::string & binary, const std::vector<std::string> & extra_args = {}) {
    for (int attempt = 0; attempt < 3; ++attempt) {
      port_ = free_port();
      int in[2];
      int out[2];
      if (::pipe(in) != 0) {
        return false;
      }
      if (::pipe(out) != 0) {
        ::close(in[0]);
        ::close(in[1]);
        return false;
      }
      ::fcntl(in[1], F_SETFD, FD_CLOEXEC);
      ::fcntl(out[0], F_SETFD, FD_CLOEXEC);
      pid_ = ::fork();
      if (pid_ == 0) {
        ::dup2(in[0], STDIN_FILENO);
        ::dup2(out[1], STDOUT_FILENO);
        const std::string port = std::to_string(port_);
        std::vector<char *> argv{const_cast<char *>(binary.c_str()), const_cast<char *>("--port"),
                                 const_cast<char *>(port.c_str())};
        for (const auto & arg : extra_args) {
          argv.push_back(const_cast<char *>(arg.c_str()));
        }
        argv.push_back(nullptr);
        ::execv(binary.c_str(), argv.data());
        ::_exit(127);
      }
      ::close(in[0]);
      ::close(out[1]);
      stdin_ = ::fdopen(in[1], "w");
      stdout_fd_ = out[0];
      {
        std::lock_guard<std::mutex> lock(output_mutex_);
        output_.clear();
        output_closed_ = false;
      }
      reader_ = std::thread([this] {
        drain();
      });
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
      if (!reap(5s)) {
        ::kill(pid_, SIGKILL);
        ::waitpid(pid_, nullptr, 0);
      }
      pid_ = -1;
    }
    if (reader_.joinable()) {
      reader_.join();
    }
    if (stdout_fd_ >= 0) {
      ::close(stdout_fd_);
      stdout_fd_ = -1;
    }
  }

  ~AlarmServer() {
    stop();
  }

  // Owns a child process, its pipes and the thread draining them.
  AlarmServer() = default;
  AlarmServer(const AlarmServer &) = delete;
  AlarmServer & operator=(const AlarmServer &) = delete;
  AlarmServer(AlarmServer &&) = delete;
  AlarmServer & operator=(AlarmServer &&) = delete;

 private:
  bool reap(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (::waitpid(pid_, nullptr, WNOHANG) == pid_) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  // Runs until the child closes its stdout, which it does when it exits.
  void drain() {
    char chunk[4096];
    for (;;) {
      ssize_t n = ::read(stdout_fd_, chunk, sizeof(chunk));
      if (n < 0 && errno == EINTR) {
        continue;
      }
      std::lock_guard<std::mutex> lock(output_mutex_);
      if (n <= 0) {
        output_closed_ = true;
        output_cv_.notify_all();
        return;
      }
      // Only the start-up banner is ever looked up; keep the head, drop the rest.
      if (output_.size() < 64 * 1024) {
        output_.append(chunk, static_cast<size_t>(n));
      }
      output_cv_.notify_all();
    }
  }

  bool wait_for_line(const std::string & prefix, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(output_mutex_);
    size_t at = std::string::npos;
    const bool found = output_cv_.wait_for(lock, timeout, [&] {
      at = output_.find(prefix);
      return (at != std::string::npos && output_.find('\n', at) != std::string::npos) || output_closed_;
    });
    if (!found || at == std::string::npos) {
      return false;
    }
    const auto ns_at = output_.find("namespace=", at);
    if (ns_at != std::string::npos) {
      namespace_index_ = static_cast<uint16_t>(std::stoi(output_.substr(ns_at + 10)));
    }
    return true;
  }

  pid_t pid_{-1};
  uint16_t port_{0};
  uint16_t namespace_index_{0};
  FILE * stdin_{nullptr};
  int stdout_fd_{-1};
  std::thread reader_;
  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::string output_;
  bool output_closed_{false};
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
    ASSERT_NE(add_counting_item(sub, kServerObject, counter), 0u);
  }

  /// Event item counting into ``counter``. The callback also holds ``held``,
  /// so a test can see when the client lets go of it.
  uint32_t add_counting_item(uint32_t sub, const opcua::NodeId & source, std::atomic<int> & counter,
                             std::shared_ptr<int> held = nullptr) {
    return client_.add_event_monitored_item(
        sub, source, {}, [&counter, held = std::move(held)](const auto &, const auto &, const auto &, const auto &) {
          counter.fetch_add(1);
        });
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
  std::atomic<int> round_events[6]{};
  std::atomic<int> old_changes{0};
  std::atomic<int> new_changes{0};

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

// A value whose own status is BadNotConnected (a gateway server reporting a
// tag whose device is offline) is not a lost link: the read fails, but the
// session, its event item and every other value stay up.
TEST_F(OpcuaClientReconnect, NotConnectedValueKeepsSession) {
  subscribe(old_events);
  ASSERT_TRUE(fire_and_wait(old_events));

  server_.command("notconnected StatusWord");
  const opcua::NodeId status_word(server_.namespace_index(), "StatusWord");
  ASSERT_TRUE(pump_until([&] {
    return !client_.read_value(status_word).good;
  }));
  EXPECT_TRUE(client_.is_connected());
  EXPECT_TRUE(client_.read_value(opcua::NodeId(server_.namespace_index(), "Tank.Level")).good);
  ASSERT_TRUE(fire_and_wait(old_events));
  EXPECT_TRUE(client_.is_connected());
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

// The peer resets the connection between two requests and nothing calls
// run_iterate, as in poll mode. open62541 closes the socket on the failed send
// and returns BadConnectionClosed, but its channel still reads OPEN and its
// session ACTIVATED, so the code alone must mark the client down.
TEST_F(OpcuaClientReconnect, ResetLinkPollOnly) {
  ASSERT_TRUE(client_.read_value(kServerCurrentTime).good);
  proxy_.reset();
  int failed = 0;
  for (int i = 0; i < 5; ++i) {
    failed += client_.read_value(kServerCurrentTime).good ? 0 : 1;
  }
  EXPECT_EQ(failed, 5);
  EXPECT_FALSE(client_.is_connected());
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
// as the poller does on a link with a bad cable. Every item of an earlier
// round must stay silent while the current one delivers.
TEST_F(OpcuaClientReconnect, RepeatedFlaps) {
  constexpr int kRounds = 5;
  auto retired_unchanged = [&](int round, const std::vector<int> & before) {
    for (int r = 0; r < round; ++r) {
      EXPECT_EQ(round_events[r].load(), before[static_cast<size_t>(r)]) << "item of round " << r << " delivered";
    }
  };
  for (int round = 0; round <= kRounds; ++round) {
    auto & events = round_events[round];
    subscribe(events);
    std::vector<int> before;
    before.reserve(static_cast<size_t>(round));
    for (int r = 0; r < round; ++r) {
      before.push_back(round_events[r].load());
    }
    ASSERT_TRUE(fire_and_wait(events)) << "round " << round;
    // A second event so a late delivery to a retired item has had its chance.
    ASSERT_TRUE(fire_and_wait(events)) << "round " << round;
    retired_unchanged(round, before);
    if (round == kRounds) {
      break;
    }
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

// Removing one live item of a subscription silences it and releases its
// callback right away, while its peer on the same subscription keeps
// delivering.
TEST_F(OpcuaClientReconnect, RemoveLiveItemKeepsPeer) {
  const uint32_t sub = client_.create_subscription(50.0, [](const std::string &, const OpcuaValue &) {});
  ASSERT_NE(sub, 0u);
  auto held = std::make_shared<int>(0);
  std::weak_ptr<int> watch = held;
  const uint32_t removed = add_counting_item(sub, kServerObject, old_events, std::move(held));
  ASSERT_NE(removed, 0u);
  ASSERT_NE(add_counting_item(sub, kServerObject, new_events), 0u);
  ASSERT_TRUE(fire_and_wait(old_events));
  ASSERT_TRUE(pump_until([&] {
    return new_events.load() > 0;
  }));

  ASSERT_TRUE(client_.remove_event_monitored_item(sub, removed));
  EXPECT_TRUE(watch.expired()) << "the removed item's callback is still held";
  EXPECT_FALSE(client_.remove_event_monitored_item(sub, removed)) << "removed twice";

  const int old_before = old_events.load();
  ASSERT_TRUE(fire_and_wait(new_events));
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), old_before);
}

// An item the client cannot create returns 0 and lets go of its callback,
// whether the client refuses it before any request (unknown subscription) or
// the server rejects it (unknown event source). A valid item created next
// still delivers.
TEST_F(OpcuaClientReconnect, FailedItemCreationReleasesCallback) {
  const uint32_t sub = client_.create_subscription(50.0, [](const std::string &, const OpcuaValue &) {});
  ASSERT_NE(sub, 0u);

  auto unknown_sub = std::make_shared<int>(0);
  std::weak_ptr<int> unknown_sub_watch = unknown_sub;
  EXPECT_EQ(add_counting_item(sub + 1000, kServerObject, old_events, std::move(unknown_sub)), 0u);
  EXPECT_TRUE(unknown_sub_watch.expired()) << "callback of an item on an unknown subscription is still held";

  auto unknown_source = std::make_shared<int>(0);
  std::weak_ptr<int> unknown_source_watch = unknown_source;
  EXPECT_EQ(add_counting_item(sub, opcua::NodeId(0, 999999u), old_events, std::move(unknown_source)), 0u);
  EXPECT_TRUE(unknown_source_watch.expired()) << "callback of an item the server rejected is still held";

  ASSERT_NE(add_counting_item(sub, kServerObject, new_events), 0u);
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), 0);
}

// Two subscriptions each get their own monitored item ids, so both of these
// items can carry the same id. Removing one must remove that one only.
TEST_F(OpcuaClientReconnect, SameItemIdInTwoSubscriptions) {
  const uint32_t sub_a = client_.create_subscription(50.0, [](const std::string &, const OpcuaValue &) {});
  const uint32_t sub_b = client_.create_subscription(50.0, [](const std::string &, const OpcuaValue &) {});
  ASSERT_NE(sub_a, 0u);
  ASSERT_NE(sub_b, 0u);
  const uint32_t item_a = add_counting_item(sub_a, kServerObject, old_events);
  const uint32_t item_b = add_counting_item(sub_b, kServerObject, new_events);
  ASSERT_NE(item_a, 0u);
  ASSERT_NE(item_b, 0u);
  ASSERT_TRUE(fire_and_wait(old_events));

  ASSERT_TRUE(client_.remove_event_monitored_item(sub_a, item_a));
  const int a_before = old_events.load();
  ASSERT_TRUE(fire_and_wait(new_events));
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_events.load(), a_before);
  EXPECT_TRUE(client_.remove_event_monitored_item(sub_b, item_b));
}

// Data-change items go the same way as event items: across a cut link and a
// reconnect, the old item stays silent and the one created on the new session
// delivers, next to an event item on the same subscription.
TEST_F(OpcuaClientReconnect, DataChangeItemAcrossReconnect) {
  const opcua::NodeId status_word(server_.namespace_index(), "StatusWord");
  auto data_item = [&](std::atomic<int> & changes, std::atomic<int> & events) {
    const uint32_t sub = client_.create_subscription(50.0, [&changes](const std::string &, const OpcuaValue &) {
      changes.fetch_add(1);
    });
    ASSERT_NE(sub, 0u);
    ASSERT_TRUE(client_.add_monitored_item(sub, status_word));
    ASSERT_NE(add_counting_item(sub, kServerObject, events), 0u);
  };
  auto change_and_wait = [&](std::atomic<int> & changes, int value) {
    const int before = changes.load();
    server_.command("set StatusWord " + std::to_string(value));
    return pump_until([&] {
      return changes.load() > before;
    });
  };

  data_item(old_changes, old_events);
  ASSERT_TRUE(change_and_wait(old_changes, 1));
  ASSERT_TRUE(fire_and_wait(old_events));

  proxy_.cut();
  ASSERT_TRUE(pump_until([&] {
    return !client_.is_connected();
  }));
  ASSERT_TRUE(client_.connect(config_));

  data_item(new_changes, new_events);
  const int old_changes_before = old_changes.load();
  const int old_events_before = old_events.load();
  ASSERT_TRUE(change_and_wait(new_changes, 2));
  ASSERT_TRUE(change_and_wait(new_changes, 3));
  ASSERT_TRUE(fire_and_wait(new_events));
  EXPECT_EQ(old_changes.load(), old_changes_before);
  EXPECT_EQ(old_events.load(), old_events_before);
}

#ifdef UA_ENABLE_ENCRYPTION
// Overwrites the stack below the caller with a pattern, where the frames of the
// calls it returned from (connect among them) lived.
__attribute__((noinline)) void scribble_dead_stack() {
  volatile unsigned char junk[256 * 1024];
  for (auto & byte : junk) {
    byte = 0xA5;
  }
}

// A secured client behind the proxy: Basic256Sha256 SignAndEncrypt and a user
// login, against the fixture started with --secure and certificates made for
// this run.
class SecuredOpcuaClientReconnect : public ::testing::Test {
 protected:
  void SetUp() override {
    char dir_template[] = "/tmp/opcua_secured_reconnect_XXXXXX";
    ASSERT_NE(::mkdtemp(dir_template), nullptr);
    cert_dir_ = dir_template;
    const std::string gen = std::string("bash '") + GEN_TEST_CERTS_SH + "' '" + cert_dir_ + "' " + kServerUri + " " +
                            kClientUri + " >/dev/null";
    ASSERT_EQ(std::system(gen.c_str()), 0) << "gen_test_certs.sh failed";
    ASSERT_TRUE(
        server_.start(TEST_ALARM_SERVER_BIN, {"--secure", "--cert", cert_dir_ + "/server_cert.der", "--key",
                                              cert_dir_ + "/server_key.pem", "--trust", cert_dir_ + "/client_cert.der",
                                              "--app-uri", kServerUri, "--username", "medkit", "--password", "secret"}))
        << "secured test_alarm_server did not start";
    ASSERT_TRUE(proxy_.start(server_.port()));
    config_.endpoint_url = "opc.tcp://127.0.0.1:" + std::to_string(proxy_.port());
    config_.connect_timeout = 2000ms;
    config_.security_policy = SecurityPolicy::Basic256Sha256;
    config_.security_mode = SecurityMode::SignAndEncrypt;
    config_.client_cert_path = cert_dir_ + "/client_cert.der";
    config_.client_key_path = cert_dir_ + "/client_key.pem";
    config_.application_uri = kClientUri;
    config_.trust_list_paths = {cert_dir_ + "/server_cert.der"};
    config_.user_auth_mode = UserAuthMode::UsernamePassword;
    config_.username = "medkit";
    config_.password = "secret";
  }

  void TearDown() override {
    if (!cert_dir_.empty()) {
      std::system(("rm -rf '" + cert_dir_ + "'").c_str());
    }
  }

  bool pump_until_down(std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && client_.is_connected()) {
      client_.run_iterate(20);
    }
    return !client_.is_connected();
  }

  static constexpr const char * kServerUri = "urn:test:alarms:server";
  static constexpr const char * kClientUri = "urn:selfpatch:medkit:opcua-client";

  std::string cert_dir_;
  AlarmServer server_;
  LinkProxy proxy_;
  OpcuaClientConfig config_;
  OpcuaClient client_;
};

// The security policies log when a secured channel closes, through a pointer to
// the logger of the config the client was built from. That config has to
// outlive the client: kept only for the connect call, the pointer led into a
// stack frame long gone, and the close after a dropped link crashed the
// gateway (SIGSEGV in UA_ChannelModule_Delete_Context).
TEST_F(SecuredOpcuaClientReconnect, ChannelClosesAfterConnectReturned) {
  ASSERT_TRUE(client_.connect(config_));
  ASSERT_TRUE(client_.read_value(kServerCurrentTime).good);
  scribble_dead_stack();

  proxy_.cut();
  ASSERT_TRUE(pump_until_down());

  ASSERT_TRUE(client_.connect(config_));
  ASSERT_TRUE(client_.read_value(kServerCurrentTime).good);
  scribble_dead_stack();
  client_.disconnect();
  EXPECT_FALSE(client_.is_connected());
}
#endif  // UA_ENABLE_ENCRYPTION

}  // namespace
}  // namespace ros2_medkit_gateway
