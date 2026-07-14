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

#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/util/bounded_lru_cache.hpp"

namespace ros2_medkit_gateway {
namespace {

TEST(BoundedLruCacheTest, FindMissReturnsNullptr) {
  BoundedLruCache<int, std::string> cache(4);
  EXPECT_EQ(cache.find(42), nullptr);
  EXPECT_EQ(cache.size(), 0u);
}

TEST(BoundedLruCacheTest, PutThenFindReturnsStoredValue) {
  BoundedLruCache<int, std::string> cache(4);
  cache.put(1, "one");
  auto * value = cache.find(1);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, "one");
}

TEST(BoundedLruCacheTest, PutReplaceUpdatesValueWithoutGrowing) {
  BoundedLruCache<int, std::string> cache(4);
  cache.put(1, "one");
  cache.put(1, "uno");
  EXPECT_EQ(cache.size(), 1u);
  auto * value = cache.find(1);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, "uno");
}

TEST(BoundedLruCacheTest, PutReturnsReferenceToStoredValue) {
  BoundedLruCache<int, int> cache(4);
  int & ref = cache.put(7, 70);
  EXPECT_EQ(ref, 70);
  ref = 71;  // mutate through the returned reference
  auto * value = cache.find(7);
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(*value, 71);
}

TEST(BoundedLruCacheTest, SizeNeverExceedsCapacity) {
  BoundedLruCache<int, int> cache(8);
  for (int i = 0; i < 100; ++i) {
    cache.put(i, i);
    EXPECT_LE(cache.size(), 8u);
  }
  EXPECT_EQ(cache.size(), 8u);
}

TEST(BoundedLruCacheTest, EvictsInsertionOrderWhenNothingAccessed) {
  BoundedLruCache<int, int> cache(2);
  cache.put(1, 1);
  cache.put(2, 2);
  cache.put(3, 3);  // over cap -> evict the oldest untouched entry (key 1)
  EXPECT_EQ(cache.find(1), nullptr);
  EXPECT_NE(cache.find(2), nullptr);
  EXPECT_NE(cache.find(3), nullptr);
}

TEST(BoundedLruCacheTest, FindRefreshesRecencySoHotEntrySurvives) {
  BoundedLruCache<int, int> cache(3);
  cache.put(1, 10);
  cache.put(2, 20);
  cache.put(3, 30);
  // Touch key 1, making key 2 the least-recently-used.
  ASSERT_NE(cache.find(1), nullptr);
  cache.put(4, 40);  // over cap -> evict the LRU (key 2), not the touched key 1
  EXPECT_EQ(cache.size(), 3u);
  EXPECT_NE(cache.find(1), nullptr);  // refreshed, survives
  EXPECT_EQ(cache.find(2), nullptr);  // least-recently-used, evicted
  EXPECT_NE(cache.find(3), nullptr);
  EXPECT_NE(cache.find(4), nullptr);  // just inserted, survives
}

TEST(BoundedLruCacheTest, ContainsDoesNotRefreshRecency) {
  BoundedLruCache<int, int> cache(2);
  cache.put(1, 1);
  cache.put(2, 2);
  EXPECT_TRUE(cache.contains(1));  // peek must NOT count as an access
  cache.put(3, 3);                 // key 1 is still the LRU despite the contains() peek -> evicted
  EXPECT_EQ(cache.find(1), nullptr);
  EXPECT_NE(cache.find(2), nullptr);
  EXPECT_NE(cache.find(3), nullptr);
}

TEST(BoundedLruCacheTest, ClearEmptiesCache) {
  BoundedLruCache<int, int> cache(4);
  cache.put(1, 1);
  cache.put(2, 2);
  cache.clear();
  EXPECT_EQ(cache.size(), 0u);
  EXPECT_EQ(cache.find(1), nullptr);
}

TEST(BoundedLruCacheTest, ClearThenReusePreservesBound) {
  BoundedLruCache<int, int> cache(2);
  cache.put(1, 1);
  cache.put(2, 2);
  cache.clear();
  for (int i = 10; i < 20; ++i) {
    cache.put(i, i);
  }
  EXPECT_EQ(cache.size(), 2u);
}

TEST(BoundedLruCacheTest, ZeroCapacityClampsToOne) {
  BoundedLruCache<int, int> cache(0);
  EXPECT_EQ(cache.max_size(), 1u);
  cache.put(1, 1);
  cache.put(2, 2);  // over cap(1) -> evict key 1
  EXPECT_EQ(cache.size(), 1u);
  EXPECT_EQ(cache.find(1), nullptr);
  EXPECT_NE(cache.find(2), nullptr);
}

TEST(BoundedLruCacheTest, PutOnExistingKeyRefreshesRecency) {
  BoundedLruCache<int, std::string> cache(2);
  cache.put(1, "one");
  cache.put(2, "two");
  cache.put(1, "updated");  // replace must re-stamp key 1 as most-recently-used
  cache.put(3, "three");    // over cap -> evict the LRU, which is now key 2, not key 1
  EXPECT_EQ(cache.size(), 2u);
  auto * one = cache.find(1);
  ASSERT_NE(one, nullptr);
  EXPECT_EQ(*one, "updated");
  EXPECT_EQ(cache.find(2), nullptr);  // least-recently-used after the replace, evicted
  EXPECT_NE(cache.find(3), nullptr);
}

TEST(BoundedLruCacheTest, OnEvictCallbackFiresWithEvictedEntry) {
  std::vector<std::pair<int, std::string>> evicted;
  BoundedLruCache<int, std::string> cache(2, [&evicted](const int & key, const std::string & value) {
    evicted.emplace_back(key, value);
  });
  cache.put(1, "one");
  cache.put(2, "two");
  EXPECT_TRUE(evicted.empty());  // still at capacity, nothing evicted yet
  cache.put(3, "three");         // over cap -> evict the least-recently-used (key 1)
  ASSERT_EQ(evicted.size(), 1u);
  EXPECT_EQ(evicted[0].first, 1);
  EXPECT_EQ(evicted[0].second, "one");
}

}  // namespace
}  // namespace ros2_medkit_gateway
