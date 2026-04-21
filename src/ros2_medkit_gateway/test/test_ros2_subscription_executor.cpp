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
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"

using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class Ros2SubscriptionExecutorTest : public ::testing::Test {
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
    node_ = std::make_shared<rclcpp::Node>("test_gateway_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });

    Ros2SubscriptionExecutor::Config cfg;
    cfg.max_queue_depth = 16;
    cfg.watchdog_threshold = 200ms;
    cfg.watchdog_tick = 50ms;
    cfg.graph_poll_tick = 50ms;
    sub_exec_ = std::make_unique<Ros2SubscriptionExecutor>(node_, *executor_, cfg);
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    sub_exec_.reset();
    executor_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<Ros2SubscriptionExecutor> sub_exec_;
};

TEST_F(Ros2SubscriptionExecutorTest, RunSyncReturnsResult) {
  auto r = sub_exec_->run_sync<int>([] {
    return 42;
  });
  ASSERT_TRUE(r.has_value()) << r.error();
  EXPECT_EQ(*r, 42);
}

TEST_F(Ros2SubscriptionExecutorTest, RunSyncPropagatesException) {
  auto r = sub_exec_->run_sync<int>([]() -> int {
    throw std::runtime_error("boom");
  });
  ASSERT_FALSE(r.has_value());
  EXPECT_NE(r.error().find("boom"), std::string::npos);
}

TEST_F(Ros2SubscriptionExecutorTest, RunSyncVoid) {
  std::atomic<int> counter{0};
  auto r = sub_exec_->run_sync<void>([&] {
    counter.fetch_add(1);
  });
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(counter.load(), 1);
}

TEST_F(Ros2SubscriptionExecutorTest, RunSyncDeadlineExceeded) {
  auto r = sub_exec_->run_sync<int>(
      [] {
        std::this_thread::sleep_for(500ms);
        return 1;
      },
      50ms);
  ASSERT_FALSE(r.has_value());
  EXPECT_NE(r.error().find("deadline"), std::string::npos);
}

TEST_F(Ros2SubscriptionExecutorTest, PostFireAndForget) {
  std::atomic<int> counter{0};
  EXPECT_TRUE(sub_exec_->post([&] {
    counter.fetch_add(1);
  }));
  auto deadline = std::chrono::steady_clock::now() + 1s;
  while (counter.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(counter.load(), 1);
}

TEST_F(Ros2SubscriptionExecutorTest, QueueFullReturnsBackpressure) {
  // Hold worker via a gate so we can fill the queue
  std::promise<void> gate_promise;
  std::shared_future<void> gate = gate_promise.get_future().share();
  EXPECT_TRUE(sub_exec_->post([gate] {
    gate.wait();
  }));

  int posted = 0;
  int rejected = 0;
  for (int i = 0; i < 64; ++i) {
    if (sub_exec_->post([] {})) {
      ++posted;
    } else {
      ++rejected;
    }
  }
  EXPECT_GT(rejected, 0);
  EXPECT_GE(posted, 15);  // at least max_queue_depth - 1 accepted
  gate_promise.set_value();
}

TEST_F(Ros2SubscriptionExecutorTest, NodeAccessibleWithSuffixedName) {
  auto * n = sub_exec_->node();
  ASSERT_NE(n, nullptr);
  EXPECT_EQ(std::string(n->get_name()), std::string("test_gateway_node_sub"));
}

TEST_F(Ros2SubscriptionExecutorTest, StatsTrackCompletedTasks) {
  for (int i = 0; i < 10; ++i) {
    auto r = sub_exec_->run_sync<int>([] {
      return 1;
    });
    ASSERT_TRUE(r.has_value());
  }
  auto s = sub_exec_->stats();
  EXPECT_TRUE(s.worker_alive);
  EXPECT_GE(s.tasks_completed, 10u);
  EXPECT_EQ(s.tasks_failed, 0u);
}

TEST_F(Ros2SubscriptionExecutorTest, IsShuttingDownFalseBeforeDtor) {
  EXPECT_FALSE(sub_exec_->is_shutting_down());
}

TEST_F(Ros2SubscriptionExecutorTest, ShutdownDrainsQueuedTasks) {
  // Queue up 5 tasks then destroy; they must all complete
  std::atomic<int> counter{0};
  for (int i = 0; i < 5; ++i) {
    sub_exec_->post([&counter] {
      std::this_thread::sleep_for(10ms);
      counter.fetch_add(1);
    });
  }
  sub_exec_.reset();  // dtor drains queue
  EXPECT_EQ(counter.load(), 5);
}

TEST_F(Ros2SubscriptionExecutorTest, GraphCallbackFiresOnExternalGraphChange) {
  std::atomic<int> fired{0};
  auto token = sub_exec_->on_graph_change([&] {
    fired.fetch_add(1);
  });
  EXPECT_LT(token, 16u);

  // Create publisher on a SEPARATE node to guarantee a graph event
  auto other_node = std::make_shared<rclcpp::Node>("other_node_for_graph_event");
  auto pub = other_node->create_publisher<std_msgs::msg::String>("/test_graph_topic", 10);
  executor_->add_node(other_node);

  auto deadline = std::chrono::steady_clock::now() + 3s;
  while (fired.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  executor_->remove_node(other_node);
  sub_exec_->remove_graph_change(token);

  EXPECT_GE(fired.load(), 1);
}

TEST_F(Ros2SubscriptionExecutorTest, RemoveGraphChangeStopsFiring) {
  std::atomic<int> fired{0};
  auto token = sub_exec_->on_graph_change([&] {
    fired.fetch_add(1);
  });
  sub_exec_->remove_graph_change(token);

  auto other_node = std::make_shared<rclcpp::Node>("other_node_removed_cb");
  auto pub = other_node->create_publisher<std_msgs::msg::String>("/removed_cb_topic", 10);
  executor_->add_node(other_node);

  std::this_thread::sleep_for(500ms);
  executor_->remove_node(other_node);

  EXPECT_EQ(fired.load(), 0);
}

TEST_F(Ros2SubscriptionExecutorTest, WatchdogDetectsStuckTask) {
  std::promise<void> release;
  std::shared_future<void> release_f = release.get_future().share();
  sub_exec_->post([release_f] {
    release_f.wait();
  });

  // threshold is 200ms; wait 350ms for watchdog to trip
  std::this_thread::sleep_for(350ms);
  auto s = sub_exec_->stats();
  EXPECT_TRUE(s.degraded);
  EXPECT_GE(s.watchdog_trips, 1u);

  release.set_value();
  std::this_thread::sleep_for(150ms);
  auto s2 = sub_exec_->stats();
  EXPECT_FALSE(s2.degraded);
}

TEST_F(Ros2SubscriptionExecutorTest, DeletedCopyAndMove) {
  EXPECT_FALSE(std::is_copy_constructible_v<Ros2SubscriptionExecutor>);
  EXPECT_FALSE(std::is_move_constructible_v<Ros2SubscriptionExecutor>);
}
