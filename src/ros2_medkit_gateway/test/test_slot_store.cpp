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
#include "ros2_medkit_gateway/core/util/slot_store.hpp"
using ros2_medkit_gateway::SlotStore;

TEST(SlotStore, AllocAssignFreeReuse) {
  SlotStore<std::string> s(4);
  uint32_t a = s.alloc_slot();
  s.assign(a, std::string("alpha"));
  uint32_t b = s.alloc_slot();
  s.assign(b, std::string("beta"));
  EXPECT_EQ(s.live_count(), 2u);
  EXPECT_TRUE(s.is_live(a));
  EXPECT_EQ(s[a], "alpha");
  s.free(a);
  EXPECT_FALSE(s.is_live(a));
  EXPECT_EQ(s.live_count(), 1u);
  EXPECT_TRUE(s[a].empty());    // payload released
  uint32_t c = s.alloc_slot();  // reuses freed slot
  EXPECT_EQ(c, a);
  s.assign(c, std::string("gamma"));
  EXPECT_EQ(s.live_count(), 2u);
}

TEST(SlotStore, CollectLiveAndForEachSkipDead) {
  SlotStore<int> s(4);
  uint32_t a = s.alloc_slot();
  s.assign(a, 1);
  uint32_t b = s.alloc_slot();
  s.assign(b, 2);
  uint32_t c = s.alloc_slot();
  s.assign(c, 3);
  s.free(b);
  auto v = s.collect_live();
  EXPECT_EQ(v.size(), 2u);
  int sum = 0;
  s.for_each_live([&](uint32_t, const int & x) {
    sum += x;
  });
  EXPECT_EQ(sum, 4);
}

TEST(SlotStore, CapacityStableUnderChurnUntilOverflow) {
  SlotStore<int> s(8);
  size_t cap0 = s.capacity();
  for (int i = 0; i < 1000; ++i) {
    uint32_t k = s.alloc_slot();
    s.assign(k, i);
    s.free(k);
  }
  EXPECT_EQ(s.capacity(), cap0);  // bounded churn: no growth
  EXPECT_FALSE(s.grew());
}
