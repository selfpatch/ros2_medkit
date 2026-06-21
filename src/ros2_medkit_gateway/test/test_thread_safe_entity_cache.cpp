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

#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace ros2_medkit_gateway;

static App mkapp(std::string id) {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = "host";
  return a;
}
static Component mkcomp(std::string id) {
  Component c;
  c.id = id;
  c.name = id;
  return c;
}

TEST(EntityCacheIncremental, NoChangeDoesNotBumpGeneration) {
  ThreadSafeEntityCache c(64);
  std::vector<App> apps{mkapp("a"), mkapp("b")};
  c.update_all({}, {mkcomp("host")}, apps, {});
  uint64_t g = c.generation();
  c.update_all({}, {mkcomp("host")}, apps, {});  // identical
  EXPECT_EQ(c.generation(), g);                  // no bump
}

TEST(EntityCacheIncremental, AddRemoveUpdateReflected) {
  ThreadSafeEntityCache c(64);
  c.update_all({}, {mkcomp("host")}, {mkapp("a")}, {});
  uint64_t g = c.generation();
  auto a2 = mkapp("a");
  a2.name = "renamed";
  c.update_all({}, {mkcomp("host")}, {a2, mkapp("b")}, {});  // update a, add b
  EXPECT_GT(c.generation(), g);
  EXPECT_EQ(c.get_apps().size(), 2u);
  ASSERT_TRUE(c.get_app("a"));
  EXPECT_EQ(c.get_app("a")->name, "renamed");
  c.update_all({}, {mkcomp("host")}, {mkapp("b")}, {});  // remove a
  EXPECT_FALSE(c.get_app("a"));
  EXPECT_TRUE(c.get_app("b"));
  EXPECT_TRUE(c.validate().empty());
}

TEST(EntityCacheIncremental, AllRemovedBumpsGeneration) {
  ThreadSafeEntityCache c(64);
  c.update_all({}, {}, {mkapp("a")}, {});
  uint64_t g = c.generation();
  c.update_all({}, {}, {}, {});
  EXPECT_GT(c.generation(), g);
  EXPECT_EQ(c.get_apps().size(), 0u);
}

TEST(EntityCacheIncremental, DuplicateIdCollapsesLastWins) {
  ThreadSafeEntityCache c(64);
  auto a1 = mkapp("dup");
  a1.name = "first";
  auto a2 = mkapp("dup");
  a2.name = "second";
  c.update_all({}, {}, {a1, a2}, {});
  EXPECT_EQ(c.get_apps().size(), 1u);
  ASSERT_TRUE(c.get_app("dup"));
  EXPECT_EQ(c.get_app("dup")->name, "second");
}

TEST(EntityCacheIncremental, ChurnKeepsCapacityBounded) {
  ThreadSafeEntityCache c(64);
  for (int i = 0; i < 500; ++i) {
    std::vector<App> apps;
    for (int k = 0; k < 5; ++k) {
      apps.push_back(mkapp("n" + std::to_string(i * 5 + k)));
    }
    c.update_all({}, {mkcomp("host")}, apps, {});  // 5 fresh each tick (prev all removed)
  }
  auto s = c.get_stats();
  EXPECT_FALSE(s.grew);  // bounded set <= capacity -> no grow
  EXPECT_TRUE(c.validate().empty());
}

TEST(EntityCacheIncremental, OverflowGrowsAndFlags) {
  ThreadSafeEntityCache c(8);
  std::vector<App> apps;
  for (int i = 0; i < 64; ++i) {
    apps.push_back(mkapp("n" + std::to_string(i)));
  }
  c.update_all({}, {}, apps, {});
  EXPECT_EQ(c.get_apps().size(), 64u);
  EXPECT_TRUE(c.get_stats().grew);
  EXPECT_TRUE(c.validate().empty());
}

TEST(EntityCacheIncremental, TopicTypesAndNodeMapConditional) {
  ThreadSafeEntityCache c(64);
  c.update_all({}, {}, {mkapp("a")}, {});
  uint64_t g = c.generation();
  c.update_topic_types({{"/t", "std_msgs/msg/String"}});
  EXPECT_GT(c.generation(), g);
  g = c.generation();
  c.update_topic_types({{"/t", "std_msgs/msg/String"}});
  EXPECT_EQ(c.generation(), g);  // identical -> no bump
  EXPECT_EQ(c.get_topic_type("/t"), "std_msgs/msg/String");
}
