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

// Issue #440: the HTTP request pool must be bounded to a fixed worker count
// instead of scaling with the host CPU count. These tests start a real
// loopback server with a small pool and prove that no more than `pool_size`
// request handlers ever run concurrently, regardless of how many clients
// connect at once.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

#include <httplib.h>

#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/http/http_server.hpp"
#include "ros2_medkit_gateway/core/thread_pool_config.hpp"

using namespace ros2_medkit_gateway;
using namespace std::chrono_literals;

namespace {

// Observed peak concurrency of request handlers when `clients` clients hit a
// blocking endpoint at once, served by a pool bounded to `pool_size` workers.
// Returns the maximum number of handlers seen running simultaneously.
int observe_peak_concurrency(std::size_t pool_size, int clients) {
  HttpServerManager manager(TlsConfig{}, pool_size);
  httplib::Server * srv = manager.get_server();
  EXPECT_NE(srv, nullptr);

  std::atomic<int> active{0};
  std::atomic<int> peak{0};
  std::atomic<int> started{0};
  std::mutex m;
  std::condition_variable cv;
  bool release = false;

  srv->Get("/block", [&](const httplib::Request &, httplib::Response & res) {
    const int now = ++active;
    started.fetch_add(1);
    // Record the running peak.
    int prev = peak.load();
    while (now > prev && !peak.compare_exchange_weak(prev, now)) {
      // prev refreshed by compare_exchange_weak on failure.
    }
    {
      std::unique_lock<std::mutex> lk(m);
      // Bounded wait so a wedged test fails fast instead of hanging forever.
      cv.wait_for(lk, 10s, [&] {
        return release;
      });
    }
    --active;
    res.set_content("ok", "text/plain");
  });

  const int port = srv->bind_to_any_port("127.0.0.1");
  EXPECT_GT(port, 0);
  std::thread server_thread([srv] {
    srv->listen_after_bind();
  });
  srv->wait_until_ready();

  std::vector<std::thread> client_threads;
  client_threads.reserve(static_cast<std::size_t>(clients));
  for (int i = 0; i < clients; ++i) {
    client_threads.emplace_back([port] {
      httplib::Client cli("127.0.0.1", port);
      cli.set_connection_timeout(10s);
      cli.set_read_timeout(15s);
      cli.Get("/block");
    });
  }

  // Wait until the pool is saturated: exactly pool_size handlers should be able
  // to start; the rest queue. Generous timeout to avoid scheduler flakiness.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (started.load() < static_cast<int>(pool_size) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }

  // Give any (incorrectly unbounded) extra workers a moment to also start, so
  // an over-large pool would be caught as peak > pool_size.
  std::this_thread::sleep_for(200ms);

  const int observed_peak = peak.load();

  // Release all handlers and let the queued requests drain.
  {
    std::lock_guard<std::mutex> lk(m);
    release = true;
  }
  cv.notify_all();

  for (auto & t : client_threads) {
    t.join();
  }
  manager.stop();
  if (server_thread.joinable()) {
    server_thread.join();
  }
  return observed_peak;
}

}  // namespace

// A pool of 1 worker serializes requests: peak concurrency is exactly 1 even
// when several clients connect simultaneously.
TEST(HttpServerThreadPoolTest, single_worker_serializes_requests) {
  EXPECT_EQ(observe_peak_concurrency(/*pool_size=*/1, /*clients=*/3), 1);
}

// A pool of 2 workers caps concurrency at 2: with 4 simultaneous clients,
// exactly 2 handlers run at once and the other 2 queue.
TEST(HttpServerThreadPoolTest, pool_caps_concurrency_at_size) {
  EXPECT_EQ(observe_peak_concurrency(/*pool_size=*/2, /*clients=*/4), 2);
}

// clamp_thread_count is the shared resolver for both the executor and HTTP pool
// sizes (issue #440). It must coerce mis-set ROS parameters into the bounded
// range: zero/negative values floor to min_threads, oversized values cap to
// max_threads, and in-range values pass through unchanged.
TEST(ThreadPoolConfigTest, clamp_thread_count_bounds_the_value) {
  using ros2_medkit_gateway::clamp_thread_count;
  // executor range [1, 256]
  EXPECT_EQ(clamp_thread_count(0, 1, 256), 1u);         // zero -> floor
  EXPECT_EQ(clamp_thread_count(-5, 1, 256), 1u);        // negative -> floor
  EXPECT_EQ(clamp_thread_count(2, 1, 256), 2u);         // in range -> unchanged
  EXPECT_EQ(clamp_thread_count(100000, 1, 256), 256u);  // oversized -> cap
  // HTTP pool range [1, 1024]
  EXPECT_EQ(clamp_thread_count(3, 1, 1024), 3u);            // in range -> unchanged
  EXPECT_EQ(clamp_thread_count(1u << 20, 1, 1024), 1024u);  // oversized -> cap
}
