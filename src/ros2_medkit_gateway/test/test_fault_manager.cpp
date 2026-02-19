// Copyright 2026 mfaferek93
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
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/fault_manager.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/get_snapshots.hpp"

using namespace std::chrono_literals;
using ros2_medkit_gateway::FaultManager;
using ros2_medkit_msgs::srv::GetRosbag;
using ros2_medkit_msgs::srv::GetSnapshots;

class FaultManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    // Isolate from other packages' tests running in parallel (e.g. ros2_medkit_fault_manager
    // launches a real fault_manager_node with /fault_manager/get_snapshots on the default domain).
    // Without this, our mock services collide with real ones on Humble's slower DDS cleanup.
    setenv("ROS_DOMAIN_ID", "99", 1);
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Use unique node names to avoid DDS participant name collisions between tests.
    // On Humble (CycloneDDS), reusing the same node name across sequential tests
    // causes stale discovery state that can corrupt service responses.
    std::string node_name = "test_fault_manager_node_" + std::to_string(test_counter_++);
    node_ = std::make_shared<rclcpp::Node>(node_name, rclcpp::NodeOptions().parameter_overrides({
                                                          {"fault_service_timeout_sec", 3.0},
                                                      }));

    // Create executor for spinning
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override {
    if (spin_thread_.joinable()) {
      executor_->cancel();
      spin_thread_.join();
    }
    executor_.reset();
    node_.reset();
  }

  void start_spinning() {
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  static inline int test_counter_ = 0;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsServiceNotAvailable) {
  FaultManager fault_manager(node_.get());

  // Don't create a service, so it will timeout
  auto result = fault_manager.get_snapshots("TEST_FAULT");

  EXPECT_FALSE(result.success);
  // On Humble, wait_for_service may report ready before DDS confirms absence,
  // so the call proceeds to async_send_request which then times out.
  EXPECT_TRUE(result.error_message == "GetSnapshots service not available" ||
              result.error_message == "GetSnapshots service call timed out");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsSuccessWithValidJson) {
  // Create mock service
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots",
      [](const std::shared_ptr<GetSnapshots::Request> request, std::shared_ptr<GetSnapshots::Response> response) {
        response->success = true;
        nlohmann::json snapshot_data;
        snapshot_data["fault_code"] = request->fault_code;
        snapshot_data["captured_at"] = 1735830000.123;
        snapshot_data["topics"] = nlohmann::json::object();
        snapshot_data["topics"]["/joint_states"] = {{"message_type", "sensor_msgs/msg/JointState"},
                                                    {"data", {{"position", {1.0, 2.0}}}}};
        response->data = snapshot_data.dump();
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  auto result = fault_manager.get_snapshots("MOTOR_OVERHEAT");

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_TRUE(result.data.contains("fault_code"));
  EXPECT_EQ(result.data["fault_code"], "MOTOR_OVERHEAT");
  EXPECT_TRUE(result.data.contains("topics"));
  EXPECT_TRUE(result.data["topics"].contains("/joint_states"));
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsSuccessWithTopicFilter) {
  // Create mock service that verifies topic filter is passed
  std::string received_topic;
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots", [&received_topic](const std::shared_ptr<GetSnapshots::Request> request,
                                                        std::shared_ptr<GetSnapshots::Response> response) {
        received_topic = request->topic;
        response->success = true;
        response->data = "{}";
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  fault_manager.get_snapshots("TEST_FAULT", "/specific_topic");

  EXPECT_EQ(received_topic, "/specific_topic");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsErrorResponse) {
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots",
      [](const std::shared_ptr<GetSnapshots::Request> /*request*/, std::shared_ptr<GetSnapshots::Response> response) {
        response->success = false;
        response->error_message = "Fault not found";
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  auto result = fault_manager.get_snapshots("NONEXISTENT_FAULT");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_message, "Fault not found");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsInvalidJsonFallback) {
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots",
      [](const std::shared_ptr<GetSnapshots::Request> /*request*/, std::shared_ptr<GetSnapshots::Response> response) {
        response->success = true;
        response->data = "not valid json {{{";
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  auto result = fault_manager.get_snapshots("TEST_FAULT");

  EXPECT_TRUE(result.success);
  // When JSON parsing fails, raw_data should be returned
  EXPECT_TRUE(result.data.contains("raw_data"));
  EXPECT_EQ(result.data["raw_data"], "not valid json {{{");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetSnapshotsEmptyResponse) {
  auto service = node_->create_service<GetSnapshots>(
      "/fault_manager/get_snapshots",
      [](const std::shared_ptr<GetSnapshots::Request> /*request*/, std::shared_ptr<GetSnapshots::Response> response) {
        response->success = true;
        response->data = "{}";
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  auto result = fault_manager.get_snapshots("TEST_FAULT");

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.is_object());
  EXPECT_TRUE(result.data.empty());
}

// GetRosbag service client tests

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagServiceNotAvailable) {
  FaultManager fault_manager(node_.get());

  // Don't create a service, so it will timeout
  auto result = fault_manager.get_rosbag("TEST_FAULT");

  EXPECT_FALSE(result.success);
  // On Humble, wait_for_service may report ready before DDS confirms absence,
  // so the call proceeds to async_send_request which then times out.
  EXPECT_TRUE(result.error_message == "GetRosbag service not available" ||
              result.error_message == "GetRosbag service call timed out");
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagSuccess) {
  auto service = node_->create_service<GetRosbag>(
      "/fault_manager/get_rosbag",
      [](const std::shared_ptr<GetRosbag::Request> request, std::shared_ptr<GetRosbag::Response> response) {
        response->success = true;
        response->file_path = "/tmp/test_bag_" + request->fault_code;
        response->format = "sqlite3";
        response->duration_sec = 5.5;
        response->size_bytes = 12345;
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  auto result = fault_manager.get_rosbag("TEST_ROSBAG_FAULT");

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.contains("file_path"));
  EXPECT_EQ(result.data["file_path"], "/tmp/test_bag_TEST_ROSBAG_FAULT");
  EXPECT_EQ(result.data["format"], "sqlite3");
  EXPECT_EQ(result.data["duration_sec"], 5.5);
  EXPECT_EQ(result.data["size_bytes"], 12345);
}

// @verifies REQ_INTEROP_088
TEST_F(FaultManagerTest, GetRosbagNotFound) {
  auto service = node_->create_service<GetRosbag>(
      "/fault_manager/get_rosbag",
      [](const std::shared_ptr<GetRosbag::Request> /*request*/, std::shared_ptr<GetRosbag::Response> response) {
        response->success = false;
        response->error_message = "No rosbag file available for fault";
      });

  start_spinning();
  FaultManager fault_manager(node_.get());

  auto result = fault_manager.get_rosbag("NONEXISTENT_FAULT");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_message, "No rosbag file available for fault");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
