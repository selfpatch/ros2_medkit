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

// Standalone TU: overrides global operator new/delete, gated by an armed flag, to assert the cache layer
// performs ~zero net structural allocations during steady-state reconcile churn.
#include <atomic>
#include <cstdlib>
#include <gtest/gtest.h>
#include <new>
#include <string>
#include <vector>
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
namespace {
std::atomic<long> g_net{0};
std::atomic<bool> g_armed{false};
}  // namespace
void * operator new(std::size_t n) {
  if (g_armed.load(std::memory_order_relaxed)) {
    g_net.fetch_add(1, std::memory_order_relaxed);
  }
  void * p = std::malloc(n ? n : 1);
  if (!p) {
    throw std::bad_alloc();
  }
  return p;
}
void operator delete(void * p) noexcept {
  if (p && g_armed.load(std::memory_order_relaxed)) {
    g_net.fetch_sub(1, std::memory_order_relaxed);
  }
  std::free(p);
}
void operator delete(void * p, std::size_t /*n*/) noexcept {
  ::operator delete(p);
}
using namespace ros2_medkit_gateway;
static App mkapp(const std::string & id) {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = "host";
  return a;
}

TEST(EntityCacheAlloc, SteadyStateChurnNoNetStructuralGrowth) {
  ThreadSafeEntityCache c(256);
  // Pre-build ALL inputs BEFORE arming. Two alternating SSO id-sets (<=15 chars), recycled every other tick:
  // structural reconcile work (slot reuse, index reset/refill) must not allocate; key strings are SSO.
  const std::size_t kTicks = 200;
  std::vector<std::vector<App>> ticks(kTicks);
  for (std::size_t i = 0; i < kTicks; ++i) {
    for (int k = 0; k < 50; ++k) {
      ticks[i].push_back(
          mkapp("a" + std::to_string((i % 2) * 100 + static_cast<std::size_t>(k))));  // ids "a0".."a149", <=5 chars
    }
  }
  const std::vector<Component> host{};   // empty (Component churn not under test here)
  c.update_all({}, host, ticks[0], {});  // warm storage to high-water (built outside arming)
  c.update_all({}, host, ticks[1], {});
  g_net.store(0);
  g_armed.store(true);
  for (std::size_t i = 2; i < kTicks; ++i) {
    c.update_all({}, std::vector<Component>{}, std::move(ticks[i]), {});
  }
  g_armed.store(false);
  // Only update_all ran while armed; no copies, no container mutation, no getter calls.
  const long kSlack = 4;  // tiny; tolerates allocator bookkeeping noise only. Do NOT inflate.
  EXPECT_LE(g_net.load(), kSlack) << "cache layer allocated under steady-state churn (regression to rebuild?)";
}
