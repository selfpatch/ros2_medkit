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
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "ros2_medkit_gateway/ros2/trigger_topic_subscriber.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"

using namespace std::chrono_literals;
using ros2_medkit_gateway::TriggerTopicSubscriber;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;

namespace ros2_medkit_gateway {

// Friend shim: reach the private retry hook and the pending_/live_ maps so a
// test can drive the retry path deterministically (the retry timer would take
// kRetryIntervalSec seconds) and assert the pending -> live transition.
struct TriggerTopicSubscriberTestAccess {
  static void run_retry(TriggerTopicSubscriber & s) {
    s.retry_pending_subscriptions();
  }

  static std::size_t pending_count(TriggerTopicSubscriber & s) {
    std::lock_guard<std::mutex> lock(s.mutex_);
    return s.pending_.size();
  }
  static std::size_t live_count(TriggerTopicSubscriber & s) {
    std::lock_guard<std::mutex> lock(s.mutex_);
    return s.live_.size();
  }
  static Ros2SubscriptionExecutor * executor(TriggerTopicSubscriber & s) {
    return s.sub_exec_.load(std::memory_order_acquire);
  }
};

}  // namespace ros2_medkit_gateway

using ros2_medkit_gateway::TriggerTopicSubscriberTestAccess;

class TriggerTopicSubscriberTest : public ::testing::Test {
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
    node_ = std::make_shared<rclcpp::Node>("trigger_sub_test_gateway");
    // Publishers live on a separate node kept OFF the executor (same reason as
    // the data-provider test: a publisher-owning node iterated by a spinning
    // MultiThreadedExecutor races its internal hash map under TSan).
    publisher_node_ = std::make_shared<rclcpp::Node>("trigger_sub_test_publisher");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    sub_exec_ = std::make_shared<Ros2SubscriptionExecutor>(node_);
    subscriber_ = std::make_unique<TriggerTopicSubscriber>(node_.get());
  }

  void TearDown() override {
    // Drain the subscriber's slots on the still-live subscription worker before
    // the executor is destroyed (same order the gateway uses), then stop the
    // main executor and join, then tear down.
    subscriber_->shutdown();
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    subscriber_.reset();
    sub_exec_.reset();
    executor_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  // Wait until node_ has discovered a publisher for `topic` (so the retry path
  // can resolve its type from the graph). Returns false on timeout.
  bool wait_for_topic_discovered(const std::string & topic) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      auto names = node_->get_topic_names_and_types();
      if (names.count(topic) != 0) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<Ros2SubscriptionExecutor> sub_exec_;
  std::unique_ptr<TriggerTopicSubscriber> subscriber_;
};

// A subscribe() that arrives before the subscription executor is wired (the
// startup window: persistent restore in the ctor, or an early POST) must NOT
// throw. It is queued as pending, and once the executor is wired the retry
// path creates the real slot, which then delivers samples.
TEST_F(TriggerTopicSubscriberTest, DefersBeforeWiringThenResolvesAfterRetry) {
  const std::string topic = "/trigger_sub_test_topic";
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>(topic, 10);
  ASSERT_TRUE(wait_for_topic_discovered(topic));

  std::atomic<int> hits{0};
  auto cb = [&hits](const nlohmann::json & /*sample*/) {
    hits.fetch_add(1, std::memory_order_relaxed);
  };

  // Subscribe with the executor still unwired: must not throw, and the handle
  // is parked in pending_ (no live slot yet).
  ASSERT_NO_THROW(subscriber_->subscribe(topic, /*msg_type=*/"", "handle_1", cb));
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::pending_count(*subscriber_), 1u);
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::live_count(*subscriber_), 0u);

  // Wire the executor and run one retry tick: the pending handle resolves its
  // type from the graph and becomes a live slot.
  subscriber_->set_subscription_executor(sub_exec_.get());
  TriggerTopicSubscriberTestAccess::run_retry(*subscriber_);
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::pending_count(*subscriber_), 0u);
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::live_count(*subscriber_), 1u);

  // The live slot delivers: publish until the callback fires (bounded).
  std_msgs::msg::Int32 msg;
  msg.data = 42;
  for (int i = 0; i < 60 && hits.load(std::memory_order_relaxed) == 0; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GT(hits.load(std::memory_order_relaxed), 0);
}

// The executor is wired exactly once. A null argument is ignored, and a later
// re-set to a different executor is ignored (existing slots hold the first one
// and would be orphaned by a swap).
TEST_F(TriggerTopicSubscriberTest, SetSubscriptionExecutorIsSetOnceAndIgnoresNull) {
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::executor(*subscriber_), nullptr);

  subscriber_->set_subscription_executor(nullptr);
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::executor(*subscriber_), nullptr);

  subscriber_->set_subscription_executor(sub_exec_.get());
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::executor(*subscriber_), sub_exec_.get());

  // A second, different executor is rejected; the first stays wired. Built on
  // publisher_node_ so its internal subscription node does not collide with the
  // one sub_exec_ created on node_.
  auto other = std::make_shared<Ros2SubscriptionExecutor>(publisher_node_);
  subscriber_->set_subscription_executor(other.get());
  EXPECT_EQ(TriggerTopicSubscriberTestAccess::executor(*subscriber_), sub_exec_.get());
}
