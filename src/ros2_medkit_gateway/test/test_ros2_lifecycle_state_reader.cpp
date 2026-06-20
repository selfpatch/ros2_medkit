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

#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/ros2/status/ros2_lifecycle_state_reader.hpp"

using ros2_medkit_gateway::Ros2LifecycleStateReader;

class Ros2LifecycleStateReaderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    service_node_ = std::make_shared<rclcpp::Node>("fake_lifecycle_node");
    service_ = service_node_->create_service<lifecycle_msgs::srv::GetState>(
        "/fake_lifecycle_node/get_state",
        [this](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*request*/,
               const std::shared_ptr<lifecycle_msgs::srv::GetState::Response> & resp) {
          resp->current_state.id = state_id_;
          resp->current_state.label = state_label_;
        });
    exec_.add_node(service_node_);
    spin_ = std::thread([this]() {
      exec_.spin();
    });
    reader_ = std::make_unique<Ros2LifecycleStateReader>(service_node_.get(), std::chrono::seconds(3));
  }

  void TearDown() override {
    exec_.cancel();
    if (spin_.joinable()) {
      spin_.join();
    }
    reader_.reset();
    exec_.remove_node(service_node_);
    service_.reset();
    service_node_.reset();
  }

  uint8_t state_id_{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE};
  std::string state_label_{"active"};
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rclcpp::Node> service_node_;
  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr service_;
  std::thread spin_;
  std::unique_ptr<Ros2LifecycleStateReader> reader_;
};

TEST_F(Ros2LifecycleStateReaderTest, ReadsActiveLabel) {
  auto state = reader_->get_state("/fake_lifecycle_node/get_state");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(*state, "active");
}

TEST_F(Ros2LifecycleStateReaderTest, ReadsInactiveLabel) {
  state_label_ = "inactive";
  state_id_ = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  auto state = reader_->get_state("/fake_lifecycle_node/get_state");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(*state, "inactive");
}

TEST_F(Ros2LifecycleStateReaderTest, UnreachableServiceReturnsNullopt) {
  Ros2LifecycleStateReader r(service_node_.get(), std::chrono::milliseconds(300));
  EXPECT_FALSE(r.get_state("/no_such_node/get_state").has_value());
}
