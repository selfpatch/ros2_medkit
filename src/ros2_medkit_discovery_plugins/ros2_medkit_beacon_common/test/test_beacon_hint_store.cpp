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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
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
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

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
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, TTLTransitionToStale) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;  // 50ms
  cfg.beacon_expiry_sec = 60.0;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("app_1")));

  // Still ACTIVE immediately after insert.
  EXPECT_EQ(store.get("app_1")->status, HintStatus::ACTIVE);

  // After sleeping past TTL, should be STALE.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(store.get("app_1")->status, HintStatus::STALE);
}

// ---------------------------------------------------------------------------
// ExpiryRemovesHint
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, ExpiryRemovesHint) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;
  cfg.beacon_expiry_sec = 0.10;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("app_1")));

  // Sleep past expiry (4x margin for CI stability).
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  auto snapshot = store.evict_and_snapshot();
  EXPECT_TRUE(snapshot.empty());
  EXPECT_EQ(store.size(), 0u);
}

// ---------------------------------------------------------------------------
// EvictAndSnapshotIsAtomic
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, EvictAndSnapshotIsAtomic) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;
  cfg.beacon_expiry_sec = 0.10;
  BeaconHintStore store(cfg);

  // app_1: fresh - should survive eviction.
  ASSERT_TRUE(store.update(make_hint("app_1")));
  // app_2: inserted then aged past expiry via short sleep.
  ASSERT_TRUE(store.update(make_hint("app_2")));

  std::this_thread::sleep_for(std::chrono::milliseconds(400));

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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, MixedStatesInSnapshot) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.05;     // 50ms
  cfg.beacon_expiry_sec = 0.40;  // 400ms (wide margin for CI stability)
  BeaconHintStore store(cfg);

  // t=0: insert expired_app.
  ASSERT_TRUE(store.update(make_hint("expired_app")));

  // t=200ms: insert active_app and stale_app.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ASSERT_TRUE(store.update(make_hint("active_app")));
  ASSERT_TRUE(store.update(make_hint("stale_app")));

  // t=500ms: expired_app age=500ms > 400ms (EXPIRED). stale_app age=300ms (STALE, < 400ms).
  // Refresh active_app to keep it ACTIVE.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
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
// @verifies REQ_DISCO_BEACON_03
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

// ---------------------------------------------------------------------------
// ReceivedAtSetsLastSeen
// @verifies REQ_DISCO_BEACON_03
//
// A hint with received_at set to 5 seconds ago should produce a last_seen
// that reflects that age (approximately 5s old, not ~0s).
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, ReceivedAtSetsLastSeen) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 10.0;
  cfg.beacon_expiry_sec = 300.0;
  BeaconHintStore store(cfg);

  auto five_sec_ago = std::chrono::steady_clock::now() - std::chrono::seconds(5);

  BeaconHint hint;
  hint.entity_id = "app_stamped";
  hint.received_at = five_sec_ago;
  ASSERT_TRUE(store.update(hint));

  auto result = store.get("app_stamped");
  ASSERT_TRUE(result.has_value());

  // last_seen should be approximately 5 seconds ago, not now.
  auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - result->last_seen).count();
  EXPECT_GE(age, 4.5);  // At least 4.5s old
  EXPECT_LE(age, 6.0);  // Not more than 6s old (generous margin)
}

// ---------------------------------------------------------------------------
// DefaultReceivedAtUsesNow
// @verifies REQ_DISCO_BEACON_03
//
// A hint with default (zero-initialized) received_at should have last_seen
// set to approximately steady_clock::now() at insert time.
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, DefaultReceivedAtUsesNow) {
  BeaconHintStore store;

  BeaconHint hint;
  hint.entity_id = "app_no_stamp";
  // received_at is default-constructed (zero / epoch)
  ASSERT_TRUE(store.update(hint));

  auto result = store.get("app_no_stamp");
  ASSERT_TRUE(result.has_value());

  // last_seen should be approximately now (age ~0).
  auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - result->last_seen).count();
  EXPECT_LT(age, 1.0);  // Should be less than 1 second old
}

// ---------------------------------------------------------------------------
// ReceivedAtFarPastIsImmediatelyStale
// @verifies REQ_DISCO_BEACON_03
//
// A hint with received_at set far in the past should be immediately STALE
// if the age exceeds the TTL.
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, ReceivedAtFarPastIsImmediatelyStale) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 2.0;
  cfg.beacon_expiry_sec = 60.0;
  BeaconHintStore store(cfg);

  // Set received_at to 10 seconds ago - well past the 2s TTL.
  BeaconHint hint;
  hint.entity_id = "app_old_stamp";
  hint.received_at = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  ASSERT_TRUE(store.update(hint));

  auto result = store.get("app_old_stamp");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, HintStatus::STALE);
}

// ---------------------------------------------------------------------------
// TTLLifecycle
// @verifies REQ_DISCO_BEACON_03
//
// Verify the ACTIVE -> STALE -> EXPIRED lifecycle with short TTL and expiry.
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, TTLLifecycle) {
  BeaconHintStore::Config cfg;
  cfg.beacon_ttl_sec = 0.1;     // 100ms TTL
  cfg.beacon_expiry_sec = 2.0;  // 2s expiry (wide gap for CI stability)
  BeaconHintStore store(cfg);

  // Insert hint and verify ACTIVE immediately.
  ASSERT_TRUE(store.update(make_hint("lifecycle_app")));
  EXPECT_EQ(store.get("lifecycle_app")->status, HintStatus::ACTIVE);

  // Wait past TTL (0.4s > 0.1s TTL) but well before expiry (2s).
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_EQ(store.get("lifecycle_app")->status, HintStatus::STALE);

  // Wait past expiry (total ~2.9s > 2.0s expiry). get() returns nullopt for expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  EXPECT_FALSE(store.get("lifecycle_app").has_value());
  auto snapshot = store.evict_and_snapshot();
  EXPECT_TRUE(snapshot.empty());
  EXPECT_EQ(store.size(), 0u);
}

// ---------------------------------------------------------------------------
// MinimalHint
// @verifies REQ_DISCO_BEACON_03
//
// A hint with only entity_id set should store and retrieve correctly.
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, MinimalHint) {
  BeaconHintStore store;

  BeaconHint hint;
  hint.entity_id = "minimal_app";
  // All other fields are empty/default.
  ASSERT_TRUE(store.update(hint));

  auto result = store.get("minimal_app");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->hint.entity_id, "minimal_app");
  EXPECT_TRUE(result->hint.transport_type.empty());
  EXPECT_TRUE(result->hint.display_name.empty());
  EXPECT_EQ(result->hint.process_id, 0u);
  EXPECT_EQ(result->status, HintStatus::ACTIVE);
}

// ---------------------------------------------------------------------------
// GetReturnsNulloptForExpiredHint
// @verifies REQ_DISCO_BEACON_03
//
// After expiry, get() should return nullopt instead of serving stale data.
// ---------------------------------------------------------------------------
TEST(BeaconHintStore, GetReturnsNulloptForExpiredHint) {
  BeaconHintStore::Config config;
  config.beacon_ttl_sec = 0.05;    // 50ms TTL
  config.beacon_expiry_sec = 0.1;  // 100ms expiry
  BeaconHintStore store(config);

  BeaconHint hint;
  hint.entity_id = "test_entity";
  hint.received_at = std::chrono::steady_clock::now();
  store.update(hint);

  // Immediately: should be active
  auto result = store.get("test_entity");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, HintStatus::ACTIVE);

  // Wait past expiry (4x margin for CI stability)
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  // Should return nullopt for expired hint
  auto expired_result = store.get("test_entity");
  EXPECT_FALSE(expired_result.has_value());
}

// ---------------------------------------------------------------------------
// CapacityBehaviorThirdInsertRejected
// @verifies REQ_DISCO_BEACON_03
//
// A store with max_hints=2 should reject the 3rd unique entity insert.
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_03
TEST(BeaconHintStore, CapacityBehaviorThirdInsertRejected) {
  BeaconHintStore::Config cfg;
  cfg.max_hints = 2;
  BeaconHintStore store(cfg);

  ASSERT_TRUE(store.update(make_hint("entity_a")));
  ASSERT_TRUE(store.update(make_hint("entity_b")));
  EXPECT_EQ(store.size(), 2u);

  // Third insert with a different entity_id should be rejected.
  EXPECT_FALSE(store.update(make_hint("entity_c")));
  EXPECT_EQ(store.size(), 2u);

  // Verify only the first two are stored.
  EXPECT_TRUE(store.get("entity_a").has_value());
  EXPECT_TRUE(store.get("entity_b").has_value());
  EXPECT_FALSE(store.get("entity_c").has_value());
}
