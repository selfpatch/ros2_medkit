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
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"

using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionSlot;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class Ros2SubscriptionSlotTest : public ::testing::Test {
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
    node_ = std::make_shared<rclcpp::Node>("slot_test_gateway");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    sub_exec_ = std::make_unique<Ros2SubscriptionExecutor>(node_, *executor_);
  }

  void TearDown() override {
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
  std::unique_ptr<Ros2SubscriptionExecutor> sub_exec_;
};

TEST_F(Ros2SubscriptionSlotTest, CreateTypedSucceedsAndCallbackFires) {
  std::atomic<int> received{0};
  auto slot = Ros2SubscriptionSlot::create_typed<std_msgs::msg::Int32>(
      *sub_exec_, "/slot_typed", rclcpp::QoS(10), [&received](std::shared_ptr<const std_msgs::msg::Int32> msg) {
        received.store(msg->data);
      });
  ASSERT_TRUE(slot.has_value()) << slot.error();
  ASSERT_NE(*slot, nullptr);
  EXPECT_EQ((*slot)->topic(), "/slot_typed");

  auto pub = node_->create_publisher<std_msgs::msg::Int32>("/slot_typed", rclcpp::QoS(10));
  auto deadline = std::chrono::steady_clock::now() + 2s;
  std_msgs::msg::Int32 msg;
  msg.data = 42;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(received.load(), 42);
}

TEST_F(Ros2SubscriptionSlotTest, CreateGenericSucceedsAndCallbackFires) {
  std::atomic<int> received{0};
  auto slot =
      Ros2SubscriptionSlot::create_generic(*sub_exec_, "/slot_generic", "std_msgs/msg/String", rclcpp::QoS(10),
                                           [&received](std::shared_ptr<const rclcpp::SerializedMessage> /*msg*/) {
                                             received.fetch_add(1);
                                           });
  ASSERT_TRUE(slot.has_value()) << slot.error();
  EXPECT_EQ((*slot)->type_name(), "std_msgs/msg/String");

  auto pub = node_->create_publisher<std_msgs::msg::String>("/slot_generic", rclcpp::QoS(10));
  auto deadline = std::chrono::steady_clock::now() + 2s;
  std_msgs::msg::String msg;
  msg.data = "hello";
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GE(received.load(), 1);
}

TEST_F(Ros2SubscriptionSlotTest, DestructorRemovesSubscriberFromGraph) {
  auto slot = Ros2SubscriptionSlot::create_typed<std_msgs::msg::Int32>(
      *sub_exec_, "/slot_removed", rclcpp::QoS(10), [](std::shared_ptr<const std_msgs::msg::Int32>) {});
  ASSERT_TRUE(slot.has_value());

  // Verify graph has 1 subscriber for our topic.
  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (node_->count_subscribers("/slot_removed") == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_GE(node_->count_subscribers("/slot_removed"), 1u);

  slot->reset();  // destroy

  deadline = std::chrono::steady_clock::now() + 2s;
  while (node_->count_subscribers("/slot_removed") > 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(node_->count_subscribers("/slot_removed"), 0u);
}

TEST_F(Ros2SubscriptionSlotTest, DestructorOnShuttingDownExecutorDoesNotCrash) {
  auto slot = Ros2SubscriptionSlot::create_typed<std_msgs::msg::Int32>(
      *sub_exec_, "/slot_shutdown", rclcpp::QoS(10), [](std::shared_ptr<const std_msgs::msg::Int32>) {});
  ASSERT_TRUE(slot.has_value());

  // Order matters: tear down executor first (so slot sees is_shutting_down()).
  // Destroying the slot after should take the shutdown fast path and not crash.
  sub_exec_.reset();
  slot->reset();
  SUCCEED();
}

TEST_F(Ros2SubscriptionSlotTest, MultipleSlotsOnSameTopicBothReceive) {
  std::atomic<int> a{0};
  std::atomic<int> b{0};

  auto s1 = Ros2SubscriptionSlot::create_typed<std_msgs::msg::Int32>(
      *sub_exec_, "/slot_shared", rclcpp::QoS(10), [&a](std::shared_ptr<const std_msgs::msg::Int32> msg) {
        a.store(msg->data);
      });
  auto s2 = Ros2SubscriptionSlot::create_typed<std_msgs::msg::Int32>(
      *sub_exec_, "/slot_shared", rclcpp::QoS(10), [&b](std::shared_ptr<const std_msgs::msg::Int32> msg) {
        b.store(msg->data);
      });
  ASSERT_TRUE(s1.has_value());
  ASSERT_TRUE(s2.has_value());

  auto pub = node_->create_publisher<std_msgs::msg::Int32>("/slot_shared", rclcpp::QoS(10));
  auto deadline = std::chrono::steady_clock::now() + 2s;
  std_msgs::msg::Int32 msg;
  msg.data = 7;
  while ((a.load() == 0 || b.load() == 0) && std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(a.load(), 7);
  EXPECT_EQ(b.load(), 7);
}

TEST_F(Ros2SubscriptionSlotTest, DeletedCopyAndMove) {
  EXPECT_FALSE(std::is_copy_constructible_v<Ros2SubscriptionSlot>);
  EXPECT_FALSE(std::is_move_constructible_v<Ros2SubscriptionSlot>);
}
