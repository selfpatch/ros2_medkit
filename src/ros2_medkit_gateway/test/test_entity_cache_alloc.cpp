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

// All allocation routes through malloc, all deallocation through free. The
// FULL overload set (scalar + array + nothrow) must be replaced consistently:
// under AddressSanitizer the default array/nothrow operators are ASan's own
// (not delegating to our scalar new), so a partial replacement would pair an
// ASan operator-new with our free() and trip alloc-dealloc-mismatch.
void * counted_new(std::size_t n) {
  if (g_armed.load(std::memory_order_relaxed)) {
    g_net.fetch_add(1, std::memory_order_relaxed);
  }
  void * p = std::malloc(n ? n : 1);
  if (!p) {
    throw std::bad_alloc();
  }
  return p;
}
void * counted_new_nothrow(std::size_t n) noexcept {
  if (g_armed.load(std::memory_order_relaxed)) {
    g_net.fetch_add(1, std::memory_order_relaxed);
  }
  return std::malloc(n ? n : 1);
}
void counted_delete(void * p) noexcept {
  if (p && g_armed.load(std::memory_order_relaxed)) {
    g_net.fetch_sub(1, std::memory_order_relaxed);
  }
  std::free(p);
}
}  // namespace
void * operator new(std::size_t n) {
  return counted_new(n);
}
void * operator new[](std::size_t n) {
  return counted_new(n);
}
void * operator new(std::size_t n, const std::nothrow_t & /*tag*/) noexcept {
  return counted_new_nothrow(n);
}
void * operator new[](std::size_t n, const std::nothrow_t & /*tag*/) noexcept {
  return counted_new_nothrow(n);
}
void operator delete(void * p) noexcept {
  counted_delete(p);
}
void operator delete[](void * p) noexcept {
  counted_delete(p);
}
void operator delete(void * p, std::size_t /*n*/) noexcept {
  counted_delete(p);
}
void operator delete[](void * p, std::size_t /*n*/) noexcept {
  counted_delete(p);
}
void operator delete(void * p, const std::nothrow_t & /*tag*/) noexcept {
  counted_delete(p);
}
void operator delete[](void * p, const std::nothrow_t & /*tag*/) noexcept {
  counted_delete(p);
}
using namespace ros2_medkit_gateway;

// Populate an App with a NON-SSO id and a service carrying a non-SSO full_path,
// so the reconcile + operation-index path copies real heap keys every tick. With
// short SSO ids and empty payloads nothing on that path would allocate, making
// the steady-state assertion below pass vacuously.
static App mkapp(const std::string & id) {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = "host";
  ServiceInfo svc;
  svc.name = "calibrate_with_a_deliberately_long_name";
  svc.full_path = id + "/calibrate_with_a_deliberately_long_name";  // non-SSO operation key
  svc.type = "std_srvs/srv/Trigger";
  a.services.push_back(std::move(svc));
  return a;
}

TEST(EntityCacheAlloc, AllocationInterposerIsActive) {
  // Positive control: prove the global new/delete interposer actually counts, so
  // a green steady-state assertion cannot be a vacuous "nothing was measured".
  g_net.store(0);
  g_armed.store(true);
  auto * probe = new std::string(64, 'z');  // heap: the object and its 64-char buffer
  const long during = g_net.load();
  g_armed.store(false);
  delete probe;
  EXPECT_GT(during, 0) << "global operator new interposer is not active";
}

TEST(EntityCacheAlloc, SteadyStateChurnNoNetStructuralGrowth) {
  ThreadSafeEntityCache c(256);
  // Pre-build ALL inputs BEFORE arming. Two alternating id-sets with NON-SSO ids
  // (and a service each), recycled every other tick: the structural reconcile
  // work (slot reuse, index reset/refill, operation-index rebuild) must reuse its
  // backing storage rather than reallocating it under churn.
  const std::size_t kTicks = 200;
  std::vector<std::vector<App>> ticks(kTicks);
  for (std::size_t i = 0; i < kTicks; ++i) {
    for (int k = 0; k < 50; ++k) {
      // ids like "application_entity_with_long_id_000" - well past the SSO limit.
      const std::size_t n = (i % 2) * 100 + static_cast<std::size_t>(k);
      ticks[i].push_back(mkapp("application_entity_with_long_id_" + std::to_string(n)));
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
  // Only the UPPER bound is meaningful here. The regression this guards against
  // is the old full rebuild, which allocates under churn -> a positive net. The
  // steady-state net is legitimately negative: every tick's payloads (incl. the
  // non-SSO id/full_path heap strings) are pre-built before arming, so their
  // frees during the armed window are counted while their allocations are not. A
  // symmetric lower bound would therefore false-fail without catching any growth
  // (a net that only ever frees cannot grow memory under churn).
  EXPECT_LE(g_net.load(), kSlack) << "cache layer allocated under steady-state churn (regression to rebuild?)";
}
