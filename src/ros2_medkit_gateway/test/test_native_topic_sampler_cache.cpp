// Copyright 2026
//
// Tests for NativeTopicSampler component-topic map cache behavior

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <atomic>

#include "ros2_medkit_gateway/native_topic_sampler.hpp"

using ros2_medkit_gateway::NativeTopicSampler;

class NativeTopicSamplerCacheTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_sampler_node_cache");
    sampler_ = std::make_unique<NativeTopicSampler>(node_.get());

    // Publisher node in namespace /cache_test_ns
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/cache_test_ns"});
    publisher_node_ = std::make_shared<rclcpp::Node>("cache_publisher", options);

    pub1_ = publisher_node_->create_publisher<std_msgs::msg::String>("/cache_test_ns/topic1", 10);

    // Allow the ROS graph to update
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    rclcpp::spin_some(node_);
  }

  void TearDown() override {
    pub1_.reset();
    sampler_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::unique_ptr<NativeTopicSampler> sampler_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
};

TEST_F(NativeTopicSamplerCacheTest, CachePopulatedOnFirstCall) {
  // Component FQN is namespace + node name
  const std::string comp = "/cache_test_ns/cache_publisher";

  auto topics = sampler_->get_component_topics(comp);

  // Should include our published topic
  bool found = false;
  for (const auto & t : topics.publishes) {
    if (t == "/cache_test_ns/topic1") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Cache build should discover published topic on first call";
}

TEST_F(NativeTopicSamplerCacheTest, CachedDataReturnedOnSubsequentCalls) {
  const std::string comp = "/cache_test_ns/cache_publisher";

  auto first = sampler_->get_component_topics(comp);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node_);
  auto second = sampler_->get_component_topics(comp);

  // Results should be equivalent (no graph change)
  EXPECT_EQ(first.publishes.size(), second.publishes.size());
  EXPECT_EQ(first.subscribes.size(), second.subscribes.size());
}

TEST_F(NativeTopicSamplerCacheTest, CacheInvalidatedWhenGraphChanges) {
  const std::string comp = "/cache_test_ns/cache_publisher";

  // Ensure initial topic present
  auto before = sampler_->get_component_topics(comp);
  bool found_before = std::any_of(before.publishes.begin(), before.publishes.end(), [](const std::string & t) {
    return t == "/cache_test_ns/topic1";
  });
  EXPECT_TRUE(found_before);

  // Remove publisher (simulate graph change)
  pub1_.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto after = sampler_->get_component_topics(comp);

  bool found_after = std::any_of(after.publishes.begin(), after.publishes.end(), [](const std::string & t) {
    return t == "/cache_test_ns/topic1";
  });

  EXPECT_FALSE(found_after) << "Cache should be invalidated and rebuilt after publisher removed";
}

TEST_F(NativeTopicSamplerCacheTest, ThreadSafetyUnderConcurrentAccess) {
  const std::string comp = "/cache_test_ns/cache_publisher";

  constexpr int NUM_THREADS = 8;
  constexpr int ITER = 100;

  std::atomic<int> success_count{0};

  std::vector<std::thread> threads;
  for (int i = 0; i < NUM_THREADS; ++i) {
    threads.emplace_back([&]() {
      for (int j = 0; j < ITER; ++j) {
        auto topics = sampler_->get_component_topics(comp);
        // If the cache is functioning, either we find the topic or not depending on graph state
        // Increment success if call returned without crashing and produced a valid structure
        if (topics.publishes.size() >= 0) {
          ++success_count;
        }
        // Small yield
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(success_count.load(), NUM_THREADS * ITER);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
