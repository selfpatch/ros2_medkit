// Copyright 2026 selfpatch GmbH
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

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_beacon_common/beacon_hint_store.hpp"

using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_beacon::BeaconHintStore;
using HintStatus = BeaconHintStore::HintStatus;

namespace {

BeaconHint make_hint(const std::string & entity_id, const std::string & transport_type = "ros2") {
  BeaconHint h;
  h.entity_id = entity_id;
  h.transport_type = transport_type;
  h.display_name = entity_id + "_name";
  h.received_at = std::chrono::steady_clock::now();
  return h;
}

}  // namespace

// ---------------------------------------------------------------------------
// InsertAndRetrieve
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, InsertAndRetrieve) {
  BeaconHintStore store;
  auto hint = make_hint("app_1");
  ASSERT_TRUE(store.update(hint));

  auto result = store.get("app_1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->hint.entity_id, "app_1");
  EXPECT_EQ(result->hint.transport_type, "ros2");
  EXPECT_EQ(result->status, HintStatus::ACTIVE);
}

// ---------------------------------------------------------------------------
// UpdateRefreshesTimestamp
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, UpdateRefreshesTimestamp) {
  BeaconHintStore store;
  auto hint = make_hint("app_1");
  ASSERT_TRUE(store.update(hint));

  auto before = store.get("app_1")->last_seen;

  // Sleep briefly to ensure steady_clock advances.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  auto refresh = make_hint("app_1");
  ASSERT_TRUE(store.update(refresh));

  auto after = store.get("app_1")->last_seen;
  EXPECT_GT(after, before);
}

// ---------------------------------------------------------------------------
// UpdateOverwritesFields
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, UpdateOverwritesFields) {
  BeaconHintStore store;
  ASSERT_TRUE(store.update(make_hint("app_1", "ros2")));

  BeaconHint updated;
  updated.entity_id = "app_1";
  updated.transport_type = "zenoh";
  updated.received_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(store.update(updated));

  auto result = store.get("app_1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->hint.transport_type, "zenoh");
}

// ---------------------------------------------------------------------------
// StaleHintReactivatedOnRefresh
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, StaleHintReactivatedOnRefresh) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 1.0;
  cfg.beacon_expiry_sec = 60.0;
  BeaconHintStore store(cfg);

  // Insert hint with last_seen aged just past TTL.
  BeaconHint hint = make_hint("app_1");
  hint.received_at = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  ASSERT_TRUE(store.update(hint));

  // Force last_seen to be in the past so status reads as STALE.
  // The update above sets last_seen = now(), but we need to simulate staleness.
  // Re-insert with a manipulated approach: use a very short TTL, sleep past it.
  BeaconHintStore::Config cfg2;
  cfg2.beacon_ttl_sec = 0.05;  // 50ms
  cfg2.beacon_expiry_sec = 60.0;
  BeaconHintStore store2(cfg2);

  ASSERT_TRUE(store2.update(make_hint("app_1")));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  {
    auto result = store2.get("app_1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->status, HintStatus::STALE);
  }

  // Refresh - last_seen should be updated, status should go back to ACTIVE.
  ASSERT_TRUE(store2.update(make_hint("app_1")));

  {
    auto result = store2.get("app_1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->status, HintStatus::ACTIVE);
  }
}

// ---------------------------------------------------------------------------
// TTLTransitionToStale
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, TTLTransitionToStale) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;  // 50ms
  cfg.beacon_expiry_sec = 60.0;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("app_1")));

  // Still ACTIVE immediately after insert.
  EXPECT_EQ(store.get("app_1")->status, HintStatus::ACTIVE);

  // After sleeping past TTL, should be STALE.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(store.get("app_1")->status, HintStatus::STALE);
}

// ---------------------------------------------------------------------------
// ExpiryRemovesHint
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, ExpiryRemovesHint) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;
  cfg.beacon_expiry_sec = 0.10;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("app_1")));

  // Sleep past expiry.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto snapshot = store.evict_and_snapshot();
  EXPECT_TRUE(snapshot.empty());
  EXPECT_EQ(store.size(), 0u);
}

// ---------------------------------------------------------------------------
// EvictAndSnapshotIsAtomic
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, EvictAndSnapshotIsAtomic) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;
  cfg.beacon_expiry_sec = 0.10;
  BeaconHintStore store(cfg);

  // app_1: fresh - should survive eviction.
  ASSERT_TRUE(store.update(make_hint("app_1")));
  // app_2: inserted then aged past expiry via short sleep.
  ASSERT_TRUE(store.update(make_hint("app_2")));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Insert app_3 fresh after the sleep - should survive.
  ASSERT_TRUE(store.update(make_hint("app_3")));

  auto snapshot = store.evict_and_snapshot();

  // Only app_3 should remain (app_1 and app_2 expired).
  ASSERT_EQ(snapshot.size(), 1u);
  EXPECT_EQ(snapshot[0].hint.entity_id, "app_3");
  EXPECT_EQ(store.size(), 1u);
}

// ---------------------------------------------------------------------------
// EvictOnEmptyStoreIsNoop
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, EvictOnEmptyStoreIsNoop) {
  BeaconHintStore store;
  auto snapshot = store.evict_and_snapshot();
  EXPECT_TRUE(snapshot.empty());
  EXPECT_EQ(store.size(), 0u);
}

// ---------------------------------------------------------------------------
// MixedStatesInSnapshot
//
// Strategy: use a single store with TTL=50ms, Expiry=500ms.
//   1. Insert active_app, stale_app, expired_app.
//   2. Sleep 100ms -> stale_app and expired_app both become STALE.
//   3. Refresh active_app so it stays ACTIVE.
//   4. Sleep 450ms more -> expired_app exceeds expiry (total ~550ms > 500ms).
//      stale_app total ~550ms also exceeds expiry... so we need to refresh
//      stale_app after step 2 to keep it STALE (not expired).
//   Better: Insert expired_app early, others later. Use a gap where expired_app
//   has aged past expiry but the others haven't.
//
// Final approach:
//   Expiry = 200ms, TTL = 50ms.
//   t=0:   insert expired_app.
//   t=100ms (sleep 100ms): insert active_app and stale_app.
//   t=250ms (sleep 150ms): expired_app age=250ms > expiry=200ms. EXPIRED.
//                           stale_app age=150ms > TTL=50ms but < expiry=200ms. STALE.
//                           active_app: refresh at t=250ms -> ACTIVE.
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, MixedStatesInSnapshot) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;     // 50ms
  cfg.beacon_expiry_sec = 0.20;  // 200ms
  BeaconHintStore store(cfg);

  // t=0: insert expired_app.
  ASSERT_TRUE(store.update(make_hint("expired_app")));

  // t=100ms: insert active_app and stale_app.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(store.update(make_hint("active_app")));
  ASSERT_TRUE(store.update(make_hint("stale_app")));

  // t=250ms: expired_app age=250ms > 200ms (EXPIRED). stale_app age=150ms (STALE).
  // Refresh active_app to keep it ACTIVE.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(store.update(make_hint("active_app")));

  // Snapshot: evict expired_app, return active_app (ACTIVE) + stale_app (STALE).
  auto snapshot = store.evict_and_snapshot();
  ASSERT_EQ(snapshot.size(), 2u);

  bool found_active = false;
  bool found_stale = false;
  for (const auto & s : snapshot) {
    if (s.hint.entity_id == "active_app") {
      EXPECT_EQ(s.status, HintStatus::ACTIVE);
      found_active = true;
    } else if (s.hint.entity_id == "stale_app") {
      EXPECT_EQ(s.status, HintStatus::STALE);
      found_stale = true;
    }
  }
  EXPECT_TRUE(found_active);
  EXPECT_TRUE(found_stale);
  EXPECT_EQ(store.size(), 2u);
}

// ---------------------------------------------------------------------------
// CapacityLimitRejectsNewEntity
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, CapacityLimitRejectsNewEntity) {
  BeaconHintStore::Config cfg;
  cfg.max_hints = 3;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("app_1")));
  ASSERT_TRUE(store.update(make_hint("app_2")));
  ASSERT_TRUE(store.update(make_hint("app_3")));

  // Store is full - new entity_id should be rejected.
  EXPECT_FALSE(store.update(make_hint("app_4")));
  EXPECT_EQ(store.size(), 3u);
}

// ---------------------------------------------------------------------------
// CapacityLimitAcceptsRefresh
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, CapacityLimitAcceptsRefresh) {
  BeaconHintStore::Config cfg;
  cfg.max_hints = 3;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("app_1")));
  ASSERT_TRUE(store.update(make_hint("app_2")));
  ASSERT_TRUE(store.update(make_hint("app_3")));

  // Updating an existing entity even when at capacity should succeed.
  EXPECT_TRUE(store.update(make_hint("app_1")));
  EXPECT_EQ(store.size(), 3u);
}

// ---------------------------------------------------------------------------
// ConcurrentUpdateAndSnapshot
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, ConcurrentUpdateAndSnapshot) {
  BeaconHintStore store;

  constexpr int kNumUpdaters = 4;
  constexpr int kIterations = 500;

  std::atomic<bool> start_flag{false};
  std::vector<std::thread> updaters;

  for (int t = 0; t < kNumUpdaters; ++t) {
    updaters.emplace_back([&, t]() {
      while (!start_flag.load()) {
        std::this_thread::yield();
      }
      for (int i = 0; i < kIterations; ++i) {
        auto hint = make_hint("app_" + std::to_string(t));
        store.update(hint);
      }
    });
  }

  std::thread snapshotter([&]() {
    while (!start_flag.load()) {
      std::this_thread::yield();
    }
    for (int i = 0; i < kIterations; ++i) {
      (void)store.evict_and_snapshot();
    }
  });

  start_flag.store(true);

  for (auto & th : updaters) {
    th.join();
  }
  snapshotter.join();

  // No crashes or deadlocks - success if we reach here.
  SUCCEED();
}

// ---------------------------------------------------------------------------
// MetadataReplacedOnRefresh
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, MetadataReplacedOnRefresh) {
  BeaconHintStore store;

  // Insert with initial metadata
  auto hint1 = make_hint("app_1");
  hint1.metadata = {{"firmware", "1.0"}, {"calibrated", "true"}};
  ASSERT_TRUE(store.update(hint1));

  {
    auto result = store.get("app_1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->hint.metadata.size(), 2u);
    EXPECT_EQ(result->hint.metadata["firmware"], "1.0");
    EXPECT_EQ(result->hint.metadata["calibrated"], "true");
  }

  // Refresh with different metadata - old keys should be gone
  auto hint2 = make_hint("app_1");
  hint2.metadata = {{"firmware", "2.0"}};
  ASSERT_TRUE(store.update(hint2));

  {
    auto result = store.get("app_1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->hint.metadata.size(), 1u);
    EXPECT_EQ(result->hint.metadata["firmware"], "2.0");
    EXPECT_EQ(result->hint.metadata.count("calibrated"), 0u);
  }
}

// ---------------------------------------------------------------------------
// EmptyMetadataPreservesExisting
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, EmptyMetadataPreservesExisting) {
  BeaconHintStore store;

  // Insert with metadata
  auto hint1 = make_hint("app_1");
  hint1.metadata = {{"firmware", "1.0"}};
  ASSERT_TRUE(store.update(hint1));

  // Refresh with empty metadata - existing metadata should be preserved
  auto hint2 = make_hint("app_1");
  // hint2.metadata is empty by default
  ASSERT_TRUE(store.update(hint2));

  {
    auto result = store.get("app_1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->hint.metadata.size(), 1u);
    EXPECT_EQ(result->hint.metadata["firmware"], "1.0");
  }
}

// ---------------------------------------------------------------------------
// ConcurrentGetDoesNotBlock
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, ConcurrentGetDoesNotBlock) {
  BeaconHintStore store;
  for (int i = 0; i < 10; ++i) {
    store.update(make_hint("app_" + std::to_string(i)));
  }

  constexpr int kNumReaders = 8;
  constexpr int kIterations = 1000;

  std::atomic<bool> start_flag{false};
  std::vector<std::thread> readers;

  for (int t = 0; t < kNumReaders; ++t) {
    readers.emplace_back([&, t]() {
      while (!start_flag.load()) {
        std::this_thread::yield();
      }
      for (int i = 0; i < kIterations; ++i) {
        (void)store.get("app_" + std::to_string(t % 10));
      }
    });
  }

  start_flag.store(true);

  for (auto & th : readers) {
    th.join();
  }

  // No crashes or deadlocks - success if we reach here.
  SUCCEED();
}
