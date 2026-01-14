// Copyright 2026 Selfpatch
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

#include <thread>
#include <vector>

#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_serialization {

class TypeCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Clear cache before each test
    TypeCache::instance().clear();
  }
};

TEST_F(TypeCacheTest, SingletonInstance) {
  auto & cache1 = TypeCache::instance();
  auto & cache2 = TypeCache::instance();
  EXPECT_EQ(&cache1, &cache2);
}

TEST_F(TypeCacheTest, ParseTypeStringValid) {
  auto result = TypeCache::parse_type_string("std_msgs/msg/String");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<0>(*result), "std_msgs");
  EXPECT_EQ(std::get<1>(*result), "msg");
  EXPECT_EQ(std::get<2>(*result), "String");
}

TEST_F(TypeCacheTest, ParseTypeStringSrv) {
  auto result = TypeCache::parse_type_string("std_srvs/srv/SetBool");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<0>(*result), "std_srvs");
  EXPECT_EQ(std::get<1>(*result), "srv");
  EXPECT_EQ(std::get<2>(*result), "SetBool");
}

TEST_F(TypeCacheTest, ParseTypeStringAction) {
  auto result = TypeCache::parse_type_string("example_interfaces/action/Fibonacci");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<0>(*result), "example_interfaces");
  EXPECT_EQ(std::get<1>(*result), "action");
  EXPECT_EQ(std::get<2>(*result), "Fibonacci");
}

TEST_F(TypeCacheTest, ParseTypeStringInvalid) {
  EXPECT_FALSE(TypeCache::parse_type_string("invalid").has_value());
  EXPECT_FALSE(TypeCache::parse_type_string("").has_value());
  EXPECT_FALSE(TypeCache::parse_type_string("pkg/type").has_value());
  EXPECT_FALSE(TypeCache::parse_type_string("pkg/invalid/Type").has_value());
}

TEST_F(TypeCacheTest, GetMessageTypeInfoStdMsgsString) {
  const auto * type_info = TypeCache::instance().get_message_type_info("std_msgs", "String");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "String");
  EXPECT_STREQ(type_info->message_namespace_, "std_msgs::msg");
}

TEST_F(TypeCacheTest, GetMessageTypeInfoFromFullType) {
  const auto * type_info = TypeCache::instance().get_message_type_info("std_msgs/msg/String");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "String");
}

TEST_F(TypeCacheTest, CachingWorks) {
  auto & cache = TypeCache::instance();
  EXPECT_EQ(cache.size(), 0U);

  // First load
  const auto * type_info1 = cache.get_message_type_info("std_msgs", "String");
  ASSERT_NE(type_info1, nullptr);
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_TRUE(cache.is_cached("std_msgs", "String"));

  // Second load - should return cached
  const auto * type_info2 = cache.get_message_type_info("std_msgs", "String");
  EXPECT_EQ(type_info1, type_info2);
  EXPECT_EQ(cache.size(), 1U);
}

TEST_F(TypeCacheTest, ClearCache) {
  auto & cache = TypeCache::instance();
  cache.get_message_type_info("std_msgs", "String");
  EXPECT_GT(cache.size(), 0U);

  cache.clear();
  EXPECT_EQ(cache.size(), 0U);
  EXPECT_FALSE(cache.is_cached("std_msgs", "String"));
}

TEST_F(TypeCacheTest, NonExistentTypeReturnsNull) {
  const auto * type_info = TypeCache::instance().get_message_type_info("nonexistent_pkg", "FakeType");
  EXPECT_EQ(type_info, nullptr);
}

TEST_F(TypeCacheTest, GetServiceRequestType) {
  // Service request types use srv interface type
  const auto * type_info = TypeCache::instance().get_message_type_info("std_srvs/srv/Trigger_Request");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "Trigger_Request");
}

TEST_F(TypeCacheTest, GetServiceResponseType) {
  // Service response types use srv interface type
  const auto * type_info = TypeCache::instance().get_message_type_info("std_srvs/srv/Trigger_Response");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "Trigger_Response");
  // Trigger_Response has success (bool) and message (string)
  EXPECT_EQ(type_info->member_count_, 2U);
}

TEST_F(TypeCacheTest, GetActionGoalType) {
  // Action goal types use action interface type
  const auto * type_info = TypeCache::instance().get_message_type_info("example_interfaces/action/Fibonacci_Goal");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "Fibonacci_Goal");
  // Fibonacci_Goal has order (int32)
  EXPECT_EQ(type_info->member_count_, 1U);
}

TEST_F(TypeCacheTest, GetActionResultType) {
  const auto * type_info = TypeCache::instance().get_message_type_info("example_interfaces/action/Fibonacci_Result");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "Fibonacci_Result");
}

TEST_F(TypeCacheTest, GetActionFeedbackType) {
  const auto * type_info = TypeCache::instance().get_message_type_info("example_interfaces/action/Fibonacci_Feedback");
  ASSERT_NE(type_info, nullptr);
  EXPECT_STREQ(type_info->message_name_, "Fibonacci_Feedback");
}

TEST_F(TypeCacheTest, ThreadSafety) {
  auto & cache = TypeCache::instance();
  constexpr int NUM_THREADS = 10;
  constexpr int ITERATIONS = 100;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (int i = 0; i < NUM_THREADS; ++i) {
    threads.emplace_back([&cache, &success_count]() {
      for (int j = 0; j < ITERATIONS; ++j) {
        const auto * type_info = cache.get_message_type_info("std_msgs", "String");
        if (type_info != nullptr) {
          ++success_count;
        }
      }
    });
  }

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(success_count.load(), NUM_THREADS * ITERATIONS);
  // Cache should have exactly one entry despite concurrent access
  EXPECT_EQ(cache.size(), 1U);
}

}  // namespace ros2_medkit_serialization

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
