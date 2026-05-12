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
#include <memory>
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
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// Regression guard: Ros2LogSource::stop() must guarantee the registered
// callback never fires again after stop() returns. The race was that the
// subscription lambda copied callback_ under the mutex, dropped the lock,
// then invoked the copy outside the lock — so stop() could acquire the
// mutex, null callback_, and return while a copy was still pending
// dispatch on the executor thread.
TEST_F(Ros2LogSourceTest, NoCallbackFiresAfterStopReturns) {
  std::atomic<int> callback_count{0};
  Ros2LogSource source(node_.get());

  source.start([&callback_count](const LogEntry &) {
    ++callback_count;
  });

  auto pub = node_->create_publisher<rcl_interfaces::msg::Log>("/rosout", 100);
  rcl_interfaces::msg::Log msg;
  msg.level = rcl_interfaces::msg::Log::INFO;
  msg.name = "test";
  msg.msg = "before stop";
  pub->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  const int count_before_stop = callback_count.load();

  source.stop();

  // After stop() returns, no further callbacks may fire even if more
  // /rosout messages arrive on the executor thread.
  msg.msg = "after stop";
  pub->publish(msg);
  pub->publish(msg);
  pub->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  EXPECT_EQ(callback_count.load(), count_before_stop)
      << "Ros2LogSource::stop() contract violated: callback fired after stop() returned";
}

}  // namespace ros2_medkit_gateway::ros2
