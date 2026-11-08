// Copyright 2026 Wiktor Jelen
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
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/pending_topic_watcher.hpp"

using ros2_medkit_fault_manager::PendingTopicWatcher;
using namespace std::chrono_literals;

namespace {

constexpr auto kPeriod = 20ms;

}  // namespace

class PendingTopicWatcherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_pending_topic_watcher_node");
    // Built here, not as a member initialiser: an executor needs the context
    // rclcpp::init creates.
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override {
    executor_->remove_node(node_);
    executor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  /// Spins until @p done holds or 10 s pass; returns whether it holds.
  bool spin_until(const std::function<bool()> & done) {
    const auto start = std::chrono::steady_clock::now();
    while (!done() && std::chrono::steady_clock::now() - start < 10s) {
      executor_->spin_some();
      std::this_thread::sleep_for(5ms);
    }
    return done();
  }

  /// Spins for @p duration, long enough for many watcher periods to elapse.
  void spin_for(std::chrono::milliseconds duration) {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  /// A watcher whose hand-over takes the node-ops mutex, as SnapshotCapture's
  /// does: a watcher still holding it while handing over would hang here.
  std::unique_ptr<PendingTopicWatcher> make_watcher(std::vector<std::string> topics) {
    return std::make_unique<PendingTopicWatcher>(node_.get(), node_ops_mutex_, std::move(topics), kPeriod,
                                                 [this](const std::string & topic, const std::string & type) {
                                                   std::lock_guard<std::mutex> lock(node_ops_mutex_);
                                                   handed_over_[topic].push_back(type);
                                                 });
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::mutex node_ops_mutex_;
  std::map<std::string, std::vector<std::string>> handed_over_;
};

TEST_F(PendingTopicWatcherTest, HandsOverATopicOnceItsTypeIsKnown) {
  auto watcher = make_watcher({"/watcher/late"});
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/watcher/late", rclcpp::QoS(10));

  ASSERT_TRUE(spin_until([this] {
    return handed_over_.count("/watcher/late") > 0;
  })) << "the watcher never handed over a topic that appeared after it started";

  // Many more periods: a topic is handed over once, not on every tick after
  // its type became known.
  spin_for(15 * kPeriod);
  ASSERT_EQ(handed_over_["/watcher/late"].size(), 1u);
  EXPECT_EQ(handed_over_["/watcher/late"].front(), "std_msgs/msg/Float64");
}

TEST_F(PendingTopicWatcherTest, KeepsWatchingTheTopicsThatAreStillMissing) {
  auto watcher = make_watcher({"/watcher/first", "/watcher/second"});
  auto first = node_->create_publisher<std_msgs::msg::Float64>("/watcher/first", rclcpp::QoS(10));

  ASSERT_TRUE(spin_until([this] {
    return handed_over_.count("/watcher/first") > 0;
  }));
  spin_for(5 * kPeriod);
  EXPECT_EQ(handed_over_.count("/watcher/second"), 0u) << "handed over a topic the graph does not have";

  auto second = node_->create_publisher<std_msgs::msg::Float64>("/watcher/second", rclcpp::QoS(10));
  ASSERT_TRUE(spin_until([this] {
    return handed_over_.count("/watcher/second") > 0;
  })) << "stopped watching the second topic once the first was handed over";
  EXPECT_EQ(handed_over_["/watcher/first"].size(), 1u);
  EXPECT_EQ(handed_over_["/watcher/second"].size(), 1u);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
