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
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/subscription_manager.hpp"

using namespace ros2_medkit_gateway;

// --- Interval parsing edge cases ---

TEST(IntervalParsingTest, ParsesValidIntervals) {
  EXPECT_EQ(parse_interval("fast"), CyclicInterval::FAST);
  EXPECT_EQ(parse_interval("normal"), CyclicInterval::NORMAL);
  EXPECT_EQ(parse_interval("slow"), CyclicInterval::SLOW);
}

TEST(IntervalParsingTest, InvalidIntervalThrows) {
  EXPECT_THROW(parse_interval("turbo"), std::invalid_argument);
  EXPECT_THROW(parse_interval(""), std::invalid_argument);
  EXPECT_THROW(parse_interval("FAST"), std::invalid_argument);  // case-sensitive
}

TEST(IntervalParsingTest, IntervalRoundTrip) {
  for (auto interval : {CyclicInterval::FAST, CyclicInterval::NORMAL, CyclicInterval::SLOW}) {
    EXPECT_EQ(parse_interval(interval_to_string(interval)), interval);
  }
}

TEST(IntervalParsingTest, DurationsArePositive) {
  EXPECT_GT(interval_to_duration(CyclicInterval::FAST).count(), 0);
  EXPECT_GT(interval_to_duration(CyclicInterval::NORMAL).count(), 0);
  EXPECT_GT(interval_to_duration(CyclicInterval::SLOW).count(), 0);
  // fast < normal < slow
  EXPECT_LT(interval_to_duration(CyclicInterval::FAST), interval_to_duration(CyclicInterval::NORMAL));
  EXPECT_LT(interval_to_duration(CyclicInterval::NORMAL), interval_to_duration(CyclicInterval::SLOW));
}

// --- Capacity enforcement ---

class SubscriptionManagerTest : public ::testing::Test {
 protected:
  SubscriptionManager mgr_{5};  // small capacity for testing
};

TEST_F(SubscriptionManagerTest, CreateFailsAtCapacity) {
  for (int i = 0; i < 5; ++i) {
    auto r = mgr_.create("e", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 60);
    ASSERT_TRUE(r.has_value()) << "Subscription " << i << " should succeed";
  }
  auto r = mgr_.create("e", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 60);
  ASSERT_FALSE(r.has_value());
  EXPECT_NE(r.error().find("capacity"), std::string::npos);
}

TEST_F(SubscriptionManagerTest, CapacityFreedAfterRemove) {
  for (int i = 0; i < 5; ++i) {
    (void)mgr_.create("e", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 60);
  }
  auto list = mgr_.list("e");
  ASSERT_FALSE(list.empty());
  mgr_.remove(list[0].id);

  auto r = mgr_.create("e", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 60);
  EXPECT_TRUE(r.has_value()) << "Should succeed after freeing a slot";
}

// --- ID uniqueness ---

TEST_F(SubscriptionManagerTest, GeneratesUniqueIds) {
  auto r1 = mgr_.create("a", "apps", "/r1", "/t1", "sse", CyclicInterval::FAST, 60);
  auto r2 = mgr_.create("a", "apps", "/r2", "/t2", "sse", CyclicInterval::FAST, 60);
  ASSERT_TRUE(r1.has_value());
  ASSERT_TRUE(r2.has_value());
  EXPECT_NE(r1->id, r2->id);
}

// --- Expiry edge cases ---

TEST_F(SubscriptionManagerTest, CleanupExpiredRemovesOnlyExpired) {
  (void)mgr_.create("a", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 0);    // expired immediately
  (void)mgr_.create("b", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 300);  // still active

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  size_t removed = mgr_.cleanup_expired();
  EXPECT_EQ(removed, 1u);
  EXPECT_EQ(mgr_.active_count(), 1u);
  EXPECT_EQ(mgr_.list("b").size(), 1u);
}

TEST_F(SubscriptionManagerTest, IsActiveReturnsFalseAfterExpiry) {
  auto r = mgr_.create("a", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 0);
  ASSERT_TRUE(r.has_value());

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(mgr_.is_active(r->id));
}

TEST_F(SubscriptionManagerTest, IsActiveReturnsFalseForNonexistent) {
  EXPECT_FALSE(mgr_.is_active("nonexistent"));
}

// --- Update edge cases ---

TEST_F(SubscriptionManagerTest, UpdateOnlyIntervalLeavesOtherFieldsUnchanged) {
  auto created = mgr_.create("a", "apps", "/r", "/t", "sse", CyclicInterval::NORMAL, 300);
  ASSERT_TRUE(created.has_value());

  auto updated = mgr_.update(created->id, CyclicInterval::FAST, std::nullopt);
  ASSERT_TRUE(updated.has_value());
  EXPECT_EQ(updated->interval, CyclicInterval::FAST);
  EXPECT_EQ(updated->duration_sec, 300);
  EXPECT_EQ(updated->entity_id, "a");
  EXPECT_EQ(updated->resource_uri, "/r");
}

TEST_F(SubscriptionManagerTest, UpdateDurationExtendsExpiry) {
  auto created = mgr_.create("a", "apps", "/r", "/t", "sse", CyclicInterval::NORMAL, 10);
  ASSERT_TRUE(created.has_value());

  auto updated = mgr_.update(created->id, std::nullopt, 600);
  ASSERT_TRUE(updated.has_value());
  EXPECT_GT(updated->expires_at, created->expires_at);
}

TEST_F(SubscriptionManagerTest, UpdateNonexistentFails) {
  auto result = mgr_.update("nonexistent", CyclicInterval::FAST, std::nullopt);
  EXPECT_FALSE(result.has_value());
}

// --- Concurrent access ---

TEST(SubscriptionManagerConcurrencyTest, ConcurrentCreateIsThreadSafe) {
  SubscriptionManager mgr(100);
  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (int i = 0; i < 20; ++i) {
    threads.emplace_back([&mgr, &success_count, i]() {
      auto r = mgr.create("entity_" + std::to_string(i), "apps", "/r", "/t", "sse", CyclicInterval::FAST, 60);
      if (r.has_value()) {
        success_count.fetch_add(1);
      }
    });
  }
  for (auto & t : threads) {
    t.join();
  }
  EXPECT_EQ(success_count.load(), 20);
  EXPECT_EQ(mgr.active_count(), 20u);
}

TEST(SubscriptionManagerConcurrencyTest, ConcurrentCreateRespectsCapacity) {
  SubscriptionManager mgr(5);
  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (int i = 0; i < 20; ++i) {
    threads.emplace_back([&mgr, &success_count, i]() {
      auto r = mgr.create("e_" + std::to_string(i), "apps", "/r", "/t", "sse", CyclicInterval::FAST, 60);
      if (r.has_value()) {
        success_count.fetch_add(1);
      }
    });
  }
  for (auto & t : threads) {
    t.join();
  }
  EXPECT_EQ(success_count.load(), 5);
}

// --- Stream synchronization ---

TEST(SubscriptionManagerSyncTest, WaitForUpdateTimesOut) {
  SubscriptionManager mgr(10);
  auto created = mgr.create("a", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 300);
  ASSERT_TRUE(created.has_value());

  auto start = std::chrono::steady_clock::now();
  bool notified = mgr.wait_for_update(created->id, std::chrono::milliseconds(50));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(notified);
  EXPECT_GE(elapsed, std::chrono::milliseconds(40));
}

TEST(SubscriptionManagerSyncTest, WaitForUpdateWokenByNotify) {
  SubscriptionManager mgr(10);
  auto created = mgr.create("a", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 300);
  ASSERT_TRUE(created.has_value());

  std::thread notifier([&mgr, id = created->id]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    mgr.notify(id);
  });

  bool notified = mgr.wait_for_update(created->id, std::chrono::milliseconds(500));
  EXPECT_TRUE(notified);
  notifier.join();
}

TEST(SubscriptionManagerSyncTest, WaitForUpdateReturnsTrueIfRemoved) {
  SubscriptionManager mgr(10);
  auto created = mgr.create("a", "apps", "/r", "/t", "sse", CyclicInterval::FAST, 300);
  ASSERT_TRUE(created.has_value());

  std::thread remover([&mgr, id = created->id]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    mgr.remove(id);
  });

  bool notified = mgr.wait_for_update(created->id, std::chrono::milliseconds(500));
  EXPECT_TRUE(notified);
  remover.join();
}
