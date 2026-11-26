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

#include "ros2_medkit_gateway/ros2/transports/ros2_log_source.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/log_types.hpp"

namespace ros2_medkit_gateway::ros2 {

class Ros2LogSourceTest : public ::testing::Test {
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
    node_ = std::make_shared<rclcpp::Node>("test_log_source_node");
    // Publishers live on a node the executor never spins, and the source's
    // subscription is created before the executor spins (see
    // Ros2SubscriptionSlotTest): entity creation on a node that a spin thread
    // iterates races on rcl's /rosout logger map under TSan.
    publisher_node_ = std::make_shared<rclcpp::Node>("test_log_source_publisher");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override {
    StopSpin();
    executor_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  void StartSpin() {
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
    // A cancel() that lands before spin() starts is lost, and join() would block.
    while (!executor_->is_spinning()) {
      std::this_thread::yield();
    }
  }

  void StopSpin() {
    if (spin_thread_.joinable()) {
      executor_->cancel();
      spin_thread_.join();
    }
  }

  /// Polls `condition` until it holds or the deadline passes.
  static bool WaitFor(const std::function<bool()> & condition) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!condition()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  }

  static rcl_interfaces::msg::Log make_log(const std::string & text) {
    rcl_interfaces::msg::Log msg;
    msg.level = rcl_interfaces::msg::Log::INFO;
    msg.name = "test";
    msg.msg = text;
    return msg;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr publisher_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// Regression guard: Ros2LogSource::stop() must guarantee the registered
// callback never fires again after stop() returns. The race was that the
// subscription lambda copied callback_ under the mutex, dropped the lock,
// then invoked the copy outside the lock - so stop() could acquire the
// mutex, null callback_, and return while a copy was still pending
// dispatch on the executor thread.
TEST_F(Ros2LogSourceTest, NoCallbackFiresAfterStopReturns) {
  std::atomic<int> callback_count{0};
  std::atomic<int> sentinel_count{0};
  Ros2LogSource source(node_.get());
  auto pub = publisher_node_->create_publisher<rcl_interfaces::msg::Log>("/rosout", 100);
  // A second subscriber on the spun node: it shows the executor handled the
  // messages published after stop().
  auto sentinel = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100), [&sentinel_count](const rcl_interfaces::msg::Log::ConstSharedPtr &) {
        ++sentinel_count;
      });

  source.start([&callback_count](const LogEntry &) {
    ++callback_count;
  });
  StartSpin();

  // Publish until the first delivery: a message sent before discovery matches is lost.
  ASSERT_TRUE(WaitFor([&]() {
    pub->publish(make_log("before stop"));
    return callback_count.load() > 0;
  }));

  const int count_before_stop = callback_count.load();
  source.stop();

  // After stop() returns, no further callbacks may fire even if more
  // /rosout messages arrive on the executor thread.
  const int sentinel_before = sentinel_count.load();
  ASSERT_TRUE(WaitFor([&]() {
    pub->publish(make_log("after stop"));
    return sentinel_count.load() >= sentinel_before + 3;
  }));

  EXPECT_EQ(callback_count.load(), count_before_stop)
      << "Ros2LogSource::stop() contract violated: callback fired after stop() returned";
}

// Regression guard: start() must clear the shutdown latch left behind by a
// prior stop(). Without this reset, the subscription lambda short-circuits
// forever after the first stop()/start() cycle, breaking the documented
// idempotent start/stop contract.
TEST_F(Ros2LogSourceTest, RestartResetsShutdownLatch) {
  std::atomic<int> callback_count{0};
  Ros2LogSource source(node_.get());
  auto pub = publisher_node_->create_publisher<rcl_interfaces::msg::Log>("/rosout", 100);

  source.start([&callback_count](const LogEntry &) {
    ++callback_count;
  });
  source.stop();

  source.start([&callback_count](const LogEntry &) {
    ++callback_count;
  });
  StartSpin();

  EXPECT_TRUE(WaitFor([&]() {
    pub->publish(make_log("after restart"));
    return callback_count.load() > 0;
  })) << "Ros2LogSource::start() did not reset shutdown_requested_ after a prior stop()";

  StopSpin();
  source.stop();
}

}  // namespace ros2_medkit_gateway::ros2
