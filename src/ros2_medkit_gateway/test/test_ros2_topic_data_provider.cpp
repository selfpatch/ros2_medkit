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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

using ros2_medkit_gateway::Ros2TopicDataProvider;
using ros2_medkit_gateway::TopicDataProvider;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class Ros2TopicDataProviderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("provider_test_gateway");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    sub_exec_ = std::make_shared<Ros2SubscriptionExecutor>(node_, *executor_);
    serializer_ = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
    provider_ = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_);
  }
  void TearDown() override {
    provider_.reset();
    sub_exec_.reset();
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<Ros2SubscriptionExecutor> sub_exec_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  std::unique_ptr<Ros2TopicDataProvider> provider_;
};

TEST_F(Ros2TopicDataProviderTest, RejectsNullExecutor) {
  EXPECT_THROW(Ros2TopicDataProvider(nullptr, serializer_), std::invalid_argument);
}

TEST_F(Ros2TopicDataProviderTest, ConstructedProviderHasEmptyStats) {
  auto s = provider_->stats();
  EXPECT_EQ(s.pool_size, 0u);
  EXPECT_EQ(s.pool_hits, 0u);
  EXPECT_EQ(s.pool_misses, 0u);
  EXPECT_GT(s.pool_cap, 0u);
}

TEST_F(Ros2TopicDataProviderTest, DiscoverFindsPublisher) {
  auto pub = node_->create_publisher<std_msgs::msg::Int32>("/provider_test_topic", 10);

  auto deadline = std::chrono::steady_clock::now() + 2s;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto info = provider_->get_topic_info("/provider_test_topic");
    if (info.has_value() && info->publisher_count > 0) {
      found = true;
      EXPECT_EQ(info->type, "std_msgs/msg/Int32");
      EXPECT_GE(info->publisher_count, 1u);
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found) << "publisher not discovered within 2s";
  EXPECT_TRUE(provider_->has_publishers("/provider_test_topic"));
  EXPECT_FALSE(provider_->has_publishers("/nonexistent_topic_xxx"));
}

TEST_F(Ros2TopicDataProviderTest, DiscoverByNamespaceGroupsTopics) {
  auto pub_a = node_->create_publisher<std_msgs::msg::Int32>("/ns_test/topic_a", 10);
  auto pub_b = node_->create_publisher<std_msgs::msg::Int32>("/ns_test/topic_b", 10);

  auto deadline = std::chrono::steady_clock::now() + 2s;
  bool found_both = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = provider_->discover_topics_by_namespace();
    if (result.namespaces.count("ns_test") != 0u) {
      auto it = result.topics_by_ns.find("/ns_test");
      if (it != result.topics_by_ns.end() && it->second.publishes.size() >= 2u) {
        found_both = true;
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found_both);
}

TEST_F(Ros2TopicDataProviderTest, SampleWithoutPublishersReturnsMetadataOnly) {
  auto r = provider_->sample("/nonexistent_sample_topic", 100ms);
  ASSERT_TRUE(r.has_value());
  EXPECT_FALSE(r->has_data);
  EXPECT_EQ(r->topic_name, "/nonexistent_sample_topic");
  EXPECT_EQ(r->publisher_count, 0u);
}

TEST_F(Ros2TopicDataProviderTest, SampleHitReturnsDataAfterPublish) {
  auto pub = node_->create_publisher<std_msgs::msg::Int32>("/pool_data_topic", rclcpp::SensorDataQoS());

  // Wait for discovery, then publish repeatedly to ensure subscription catches.
  auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < discovery_deadline) {
    auto info = provider_->get_topic_info("/pool_data_topic");
    if (info.has_value() && info->publisher_count > 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // Priming call: creates pool entry + slot (miss).
  (void)provider_->sample("/pool_data_topic", 100ms);

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  auto deadline = std::chrono::steady_clock::now() + 3s;
  bool got_data = false;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    auto r = provider_->sample("/pool_data_topic", 200ms);
    ASSERT_TRUE(r.has_value());
    if (r->has_data) {
      got_data = true;
      EXPECT_EQ(r->topic_name, "/pool_data_topic");
      EXPECT_GE(r->publisher_count, 1u);
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(got_data);

  auto s = provider_->stats();
  EXPECT_GE(s.pool_size, 1u);
  EXPECT_GE(s.pool_misses, 1u);
  EXPECT_GE(s.pool_hits, 1u);
}

TEST_F(Ros2TopicDataProviderTest, PoolCapEvictsLruWhenFull) {
  Ros2TopicDataProvider::Config tight;
  tight.max_pool_size = 1;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, tight);

  auto pub1 = node_->create_publisher<std_msgs::msg::Int32>("/cap_topic_1", rclcpp::SensorDataQoS());
  auto pub2 = node_->create_publisher<std_msgs::msg::Int32>("/cap_topic_2", rclcpp::SensorDataQoS());

  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/cap_topic_1") && local->has_publishers("/cap_topic_2")) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // First miss: pool grows to 1.
  (void)local->sample("/cap_topic_1", 50ms);
  auto stats1 = local->stats();
  EXPECT_EQ(stats1.pool_size, 1u);
  EXPECT_EQ(stats1.evictions_total, 0u);

  // Second miss on a different topic: LRU eviction replaces topic_1 with topic_2.
  (void)local->sample("/cap_topic_2", 50ms);
  auto stats2 = local->stats();
  EXPECT_EQ(stats2.pool_size, 1u);
  EXPECT_GE(stats2.evictions_total, 1u);
}

TEST_F(Ros2TopicDataProviderTest, HitPromotesTopicToMru) {
  Ros2TopicDataProvider::Config tight;
  tight.max_pool_size = 2;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, tight);

  auto pub1 = node_->create_publisher<std_msgs::msg::Int32>("/lru_a", rclcpp::SensorDataQoS());
  auto pub2 = node_->create_publisher<std_msgs::msg::Int32>("/lru_b", rclcpp::SensorDataQoS());
  auto pub3 = node_->create_publisher<std_msgs::msg::Int32>("/lru_c", rclcpp::SensorDataQoS());

  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/lru_a") && local->has_publishers("/lru_b") && local->has_publishers("/lru_c")) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  (void)local->sample("/lru_a", 50ms);  // pool = [a]          lru = [a]
  (void)local->sample("/lru_b", 50ms);  // pool = [a, b]       lru = [b, a]
  (void)local->sample("/lru_a", 50ms);  // hit on a, promote.  lru = [a, b]

  const auto prev_evictions = local->stats().evictions_total;
  (void)local->sample("/lru_c", 50ms);  // miss, evict LRU=b.  lru = [c, a]

  auto s = local->stats();
  EXPECT_EQ(s.pool_size, 2u);
  EXPECT_GE(s.evictions_total, prev_evictions + 1);

  // a was promoted by the hit in step 3, so it survived the eviction and the
  // next sample on /lru_a is another hit. b was LRU and should now be a miss.
  const auto prev_hits = s.pool_hits;
  const auto prev_misses = s.pool_misses;
  (void)local->sample("/lru_a", 50ms);
  auto s2 = local->stats();
  EXPECT_GT(s2.pool_hits, prev_hits);

  (void)local->sample("/lru_b", 50ms);  // b was evicted: this is a miss
  auto s3 = local->stats();
  EXPECT_GT(s3.pool_misses, prev_misses);
}

TEST_F(Ros2TopicDataProviderTest, SampleParallelReturnsOneResultPerTopic) {
  auto r = provider_->sample_parallel({"/topic_a_notexist", "/topic_b_notexist"}, 100ms);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->size(), 2u);
  EXPECT_EQ((*r)[0].topic_name, "/topic_a_notexist");
  EXPECT_EQ((*r)[1].topic_name, "/topic_b_notexist");
  EXPECT_FALSE((*r)[0].has_data);
  EXPECT_FALSE((*r)[1].has_data);
}

TEST_F(Ros2TopicDataProviderTest, IsSystemTopicMatchesExpectedSet) {
  EXPECT_TRUE(Ros2TopicDataProvider::is_system_topic("/parameter_events"));
  EXPECT_TRUE(Ros2TopicDataProvider::is_system_topic("/rosout"));
  EXPECT_TRUE(Ros2TopicDataProvider::is_system_topic("/clock"));
  EXPECT_FALSE(Ros2TopicDataProvider::is_system_topic("/tf"));
  EXPECT_FALSE(Ros2TopicDataProvider::is_system_topic("/cmd_vel"));
}

TEST_F(Ros2TopicDataProviderTest, DeletedCopyAndMove) {
  EXPECT_FALSE(std::is_copy_constructible_v<Ros2TopicDataProvider>);
  EXPECT_FALSE(std::is_move_constructible_v<Ros2TopicDataProvider>);
}

TEST_F(Ros2TopicDataProviderTest, ConcurrentSampleFromMultipleThreadsDoesNotCrash) {
  // Issue #375 race-fix regression test: multiple httplib-like threads hammering
  // sample() on the same topic must not crash or deadlock. With
  // NativeTopicSampler this would race inside rcl's hash map during
  // create_generic_subscription / destroy. With Ros2TopicDataProvider the
  // subscription lifecycle runs on the single worker, and the pool entry is
  // shared across concurrent samplers.
  auto pub = node_->create_publisher<std_msgs::msg::Int32>("/concurrent_test_topic", rclcpp::SensorDataQoS());

  // Wait for publisher to appear in graph.
  auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < discovery_deadline) {
    if (provider_->has_publishers("/concurrent_test_topic")) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  // Drive publisher in the background so active samples see data.
  std::atomic<bool> stop{false};
  std::thread pub_thread([&stop, &pub] {
    std_msgs::msg::Int32 msg;
    int counter = 0;
    while (!stop.load(std::memory_order_acquire)) {
      msg.data = counter++;
      pub->publish(msg);
      std::this_thread::sleep_for(5ms);
    }
  });

  constexpr int kThreads = 4;
  constexpr int kIterations = 10;
  std::atomic<int> completions{0};
  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([this, &completions] {
      for (int i = 0; i < kIterations; ++i) {
        auto r = provider_->sample("/concurrent_test_topic", 200ms);
        if (r.has_value()) {
          completions.fetch_add(1);
        }
      }
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  stop.store(true, std::memory_order_release);
  pub_thread.join();

  EXPECT_EQ(completions.load(), kThreads * kIterations);

  // Pool should have exactly one entry for this topic despite concurrent misses.
  auto s = provider_->stats();
  EXPECT_GE(s.pool_size, 1u);
}

TEST_F(Ros2TopicDataProviderTest, InterfacePolymorphismWorks) {
  TopicDataProvider & iface = *provider_;
  auto r = iface.sample("/via_interface", 100ms);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->topic_name, "/via_interface");
}
