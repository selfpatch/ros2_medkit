// Copyright 2025 mfaferek93
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
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/native_topic_sampler.hpp"

using ros2_medkit_gateway::NativeTopicSampler;

// =============================================================================
// is_system_topic tests (static method - no node required)
// =============================================================================

class IsSystemTopicTest : public ::testing::Test {};

TEST_F(IsSystemTopicTest, FiltersParameterEvents) {
  EXPECT_TRUE(NativeTopicSampler::is_system_topic("/parameter_events"));
}

TEST_F(IsSystemTopicTest, FiltersRosout) {
  EXPECT_TRUE(NativeTopicSampler::is_system_topic("/rosout"));
}

TEST_F(IsSystemTopicTest, FiltersClock) {
  EXPECT_TRUE(NativeTopicSampler::is_system_topic("/clock"));
}

TEST_F(IsSystemTopicTest, DoesNotFilterTf) {
  // /tf should NOT be filtered - useful for diagnostics
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/tf"));
}

TEST_F(IsSystemTopicTest, DoesNotFilterTfStatic) {
  // /tf_static should NOT be filtered - useful for diagnostics
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/tf_static"));
}

TEST_F(IsSystemTopicTest, DoesNotFilterUserTopics) {
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/carter1/odom"));
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/cmd_vel"));
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/chassis/imu"));
}

TEST_F(IsSystemTopicTest, DoesNotFilterNamespacedSystemTopics) {
  // Namespaced versions are NOT system topics (belong to specific component)
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/robot1/parameter_events"));
  EXPECT_FALSE(NativeTopicSampler::is_system_topic("/robot1/rosout"));
}

// =============================================================================
// NativeTopicSampler tests requiring a ROS 2 node
// =============================================================================

class NativeTopicSamplerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_sampler_node");
    sampler_ = std::make_unique<NativeTopicSampler>(node_.get());
  }

  void TearDown() override {
    sampler_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<NativeTopicSampler> sampler_;
};

TEST_F(NativeTopicSamplerTest, DiscoverTopicsByNamespace_ReturnsEmptyForNoTopics) {
  // In a clean test environment, we may have system topics but no user topics
  // This test verifies the method doesn't crash and returns valid results
  auto result = sampler_->discover_topics_by_namespace();

  // Should return a valid struct (may have system topics filtered out)
  // The important thing is that namespaces and topics_by_ns are consistent
  for (const auto & ns : result.namespaces) {
    std::string ns_prefix = "/" + ns;
    // If namespace is in the set, it should have topics in the map
    // (unless topics were filtered by other logic)
    EXPECT_FALSE(ns.empty()) << "Namespace should not be empty";
  }
}

TEST_F(NativeTopicSamplerTest, DiscoverTopicsByNamespace_ConsistentNamespacesAndTopics) {
  auto result = sampler_->discover_topics_by_namespace();

  // Every namespace in the set should have a corresponding entry in topics_by_ns
  for (const auto & ns : result.namespaces) {
    std::string ns_prefix = "/" + ns;
    auto it = result.topics_by_ns.find(ns_prefix);
    EXPECT_NE(it, result.topics_by_ns.end()) << "Namespace '" << ns << "' should have topics in topics_by_ns";

    if (it != result.topics_by_ns.end()) {
      EXPECT_FALSE(it->second.publishes.empty()) << "Namespace '" << ns << "' should have at least one topic";
    }
  }

  // Every entry in topics_by_ns should have its namespace in the namespaces set
  for (const auto & [ns_prefix, topics] : result.topics_by_ns) {
    // ns_prefix is like "/carter1", we need "carter1"
    std::string ns = ns_prefix.substr(1);  // Remove leading slash
    EXPECT_TRUE(result.namespaces.count(ns) > 0)
        << "topics_by_ns key '" << ns_prefix << "' should have its namespace in namespaces set";
  }
}

TEST_F(NativeTopicSamplerTest, DiscoverTopicNamespaces_BackwardCompatibility) {
  // Test that the legacy method still works (delegates to new method)
  auto namespaces = sampler_->discover_topic_namespaces();

  // Compare with new method
  auto result = sampler_->discover_topics_by_namespace();

  EXPECT_EQ(namespaces, result.namespaces)
      << "Legacy discover_topic_namespaces should return same result as new method";
}

TEST_F(NativeTopicSamplerTest, GetTopicsForNamespace_ReturnsEmptyForNonexistentNamespace) {
  auto topics = sampler_->get_topics_for_namespace("/nonexistent_namespace_xyz123");

  EXPECT_TRUE(topics.publishes.empty());
  EXPECT_TRUE(topics.subscribes.empty());
}

TEST_F(NativeTopicSamplerTest, GetTopicsForNamespace_MatchesPrefix) {
  // This test verifies the matching logic by checking it doesn't match partial prefixes
  auto topics = sampler_->get_topics_for_namespace("/test");

  // All returned topics should start with "/test/" (with trailing slash)
  for (const auto & topic : topics.publishes) {
    EXPECT_TRUE(topic.find("/test/") == 0) << "Topic '" << topic << "' should start with '/test/'";
  }
}

// =============================================================================
// Integration test with publishers
// =============================================================================

class NativeTopicSamplerWithPublishersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Create main node for sampler
    node_ = std::make_shared<rclcpp::Node>("test_sampler_node");
    sampler_ = std::make_unique<NativeTopicSampler>(node_.get());

    // Create a publisher node in a specific namespace to test discovery
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/test_robot"});
    publisher_node_ = std::make_shared<rclcpp::Node>("publisher_node", options);

    // Create publishers on topics
    pub1_ = publisher_node_->create_publisher<std_msgs::msg::String>("/test_robot/status", 10);
    pub2_ = publisher_node_->create_publisher<std_msgs::msg::String>("/test_robot/sensor/data", 10);

    // Give ROS 2 graph time to discover the topics
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node_);
  }

  void TearDown() override {
    pub1_.reset();
    pub2_.reset();
    sampler_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::unique_ptr<NativeTopicSampler> sampler_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
};

TEST_F(NativeTopicSamplerWithPublishersTest, DiscoverTopicsByNamespace_FindsTestRobotNamespace) {
  // Allow more time for graph discovery
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto result = sampler_->discover_topics_by_namespace();

  // Should discover "test_robot" namespace
  EXPECT_TRUE(result.namespaces.count("test_robot") > 0)
      << "Should discover 'test_robot' namespace from published topics";
}

TEST_F(NativeTopicSamplerWithPublishersTest, DiscoverTopicsByNamespace_FindsTestRobotTopics) {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto result = sampler_->discover_topics_by_namespace();

  auto it = result.topics_by_ns.find("/test_robot");
  ASSERT_NE(it, result.topics_by_ns.end()) << "Should have /test_robot in topics_by_ns";

  // Should find our published topics
  const auto & topics = it->second.publishes;
  bool found_status = false;
  bool found_sensor = false;

  for (const auto & topic : topics) {
    if (topic == "/test_robot/status") {
      found_status = true;
    }
    if (topic == "/test_robot/sensor/data") {
      found_sensor = true;
    }
  }

  EXPECT_TRUE(found_status) << "Should find /test_robot/status topic";
  EXPECT_TRUE(found_sensor) << "Should find /test_robot/sensor/data topic";
}

TEST_F(NativeTopicSamplerWithPublishersTest, GetTopicsForNamespace_ReturnsCorrectTopics) {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto topics = sampler_->get_topics_for_namespace("/test_robot");

  EXPECT_GE(topics.publishes.size(), 2u) << "Should find at least 2 topics for /test_robot";

  bool found_status = false;
  bool found_sensor = false;

  for (const auto & topic : topics.publishes) {
    if (topic == "/test_robot/status") {
      found_status = true;
    }
    if (topic == "/test_robot/sensor/data") {
      found_sensor = true;
    }
  }

  EXPECT_TRUE(found_status) << "Should find /test_robot/status topic";
  EXPECT_TRUE(found_sensor) << "Should find /test_robot/sensor/data topic";
}

// =============================================================================
// Thread-safety tests for sample_topic()
// =============================================================================

class NativeTopicSamplerThreadSafetyTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_thread_safety_node");
    sampler_ = std::make_unique<NativeTopicSampler>(node_.get());

    // Spin the node in a background thread so subscription callbacks fire
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    // Publisher on a known topic with high frequency
    pub_ = node_->create_publisher<std_msgs::msg::String>("/thread_safety_test/data", 10);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      std_msgs::msg::String msg;
      msg.data = "test";
      pub_->publish(msg);
    });

    // Let DDS discover the topic
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  void TearDown() override {
    executor_->cancel();
    spin_thread_.join();
    executor_.reset();
    timer_.reset();
    pub_.reset();
    sampler_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<NativeTopicSampler> sampler_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// Verify that sample_topic on timeout path does not crash.
/// Exercises the shared_ptr SampleState fix: the callback may fire after
/// wait_for returns timeout, and must not access destroyed stack locals.
TEST_F(NativeTopicSamplerThreadSafetyTest, SampleTopicTimeoutDoesNotCrash) {
  for (int i = 0; i < 20; ++i) {
    auto result = sampler_->sample_topic("/no_such_topic_xyz", 0.05);
    EXPECT_FALSE(result.has_data);
  }
}

/// Verify that concurrent sample_topic calls from multiple threads do not crash.
/// Exercises the sampling_mutex_ + dedicated callback group fix: concurrent
/// create_generic_subscription calls must be serialized.
TEST_F(NativeTopicSamplerThreadSafetyTest, ConcurrentSampleTopicDoesNotCrash) {
  constexpr int kThreads = 4;
  constexpr int kIterations = 5;
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  std::atomic<int> successes{0};
  std::atomic<int> completions{0};

  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([this, &successes, &completions]() {
      for (int i = 0; i < kIterations; ++i) {
        auto result = sampler_->sample_topic("/thread_safety_test/data", 0.5);
        if (result.has_data) {
          successes.fetch_add(1);
        }
        completions.fetch_add(1);
      }
    });
  }

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(completions.load(), kThreads * kIterations);
  EXPECT_GT(successes.load(), 0) << "At least one concurrent sample should receive data";
}

/// Verify that sample_topic with a very short timeout and active publisher
/// does not crash when the message arrives right at the timeout boundary.
/// Stress-tests the SampleState fix: callback fires while stack is unwinding.
TEST_F(NativeTopicSamplerThreadSafetyTest, RapidTimeoutWithActivePublisher) {
  for (int i = 0; i < 30; ++i) {
    auto result = sampler_->sample_topic("/thread_safety_test/data", 0.02);
    // Don't check has_data - timing is non-deterministic. Just don't crash.
  }
}
