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
#include <vector>
#include "ros2_medkit_gateway/core/util/flat_hash_map.hpp"
using ros2_medkit_gateway::FlatHashMap;

TEST(FlatHashMap, InsertFindErase) {
  FlatHashMap<std::string, int> m(8);
  EXPECT_TRUE(m.insert_or_assign("a", 1));
  EXPECT_FALSE(m.insert_or_assign("a", 2));  // existing key -> not new
  ASSERT_NE(m.find("a"), nullptr);
  EXPECT_EQ(*m.find("a"), 2);
  EXPECT_EQ(m.find("b"), nullptr);
  EXPECT_TRUE(m.erase("a"));
  EXPECT_FALSE(m.erase("a"));
  EXPECT_EQ(m.find("a"), nullptr);
  EXPECT_EQ(m.size(), 0u);
}

TEST(FlatHashMap, GetOrCreateAndReset) {
  FlatHashMap<std::string, std::vector<uint32_t>> m(8);
  m.get_or_create("p").push_back(5);
  m.get_or_create("p").push_back(6);
  ASSERT_NE(m.find("p"), nullptr);
  EXPECT_EQ(m.find("p")->size(), 2u);
  m.reset();
  EXPECT_EQ(m.size(), 0u);
  EXPECT_EQ(m.find("p"), nullptr);
  m.get_or_create("p").push_back(9);  // reuse after reset
  EXPECT_EQ(m.find("p")->size(), 1u);
}

TEST(FlatHashMap, BoundedChurnVacuumsWithoutGrowing) {
  // Add/remove churn over a bounded live set (size stays <= 2) must reclaim
  // tombstones via an in-place same-capacity rehash, never growing the table.
  // This keeps grew() false so a steady-state index does not report growth.
  FlatHashMap<int, int> m(4);
  for (int i = 0; i < 1000; ++i) {
    m.insert_or_assign(i, i);
    m.erase(i - 1);
  }
  EXPECT_FALSE(m.grew());
  EXPECT_LE(m.size(), 2u);
  ASSERT_NE(m.find(999), nullptr);
  EXPECT_EQ(*m.find(999), 999);
}

TEST(FlatHashMap, LiveSetOutgrowingCapacityGrows) {
  // When the live (occupied) set genuinely exceeds the reserved capacity the
  // table must grow and report grew() == true.
  FlatHashMap<int, int> m(4);
  for (int i = 0; i < 1000; ++i) {
    m.insert_or_assign(i, i);  // never erased: live set grows past capacity
  }
  EXPECT_TRUE(m.grew());
  EXPECT_EQ(m.size(), 1000u);
  ASSERT_NE(m.find(999), nullptr);
  EXPECT_EQ(*m.find(999), 999);
}

TEST(FlatHashMap, ForEachVisitsOccupiedOnly) {
  FlatHashMap<std::string, int> m(8);
  m.insert_or_assign("x", 1);
  m.insert_or_assign("y", 2);
  m.erase("x");
  int sum = 0, count = 0;
  m.for_each([&](const std::string &, int v) {
    sum += v;
    ++count;
  });
  EXPECT_EQ(count, 1);
  EXPECT_EQ(sum, 2);
}
