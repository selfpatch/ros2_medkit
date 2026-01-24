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
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/rosbag_capture.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"

using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::RosbagCapture;
using ros2_medkit_fault_manager::RosbagConfig;
using ros2_medkit_fault_manager::SnapshotConfig;

class RosbagCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_rosbag_capture_node");
    storage_ = std::make_unique<InMemoryFaultStorage>();

    // Create temp directory for test bags
    temp_dir_ = std::filesystem::temp_directory_path() / "rosbag_capture_test";
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override {
    node_.reset();
    storage_.reset();
    rclcpp::shutdown();

    // Clean up temp directory
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
  }

  RosbagConfig create_rosbag_config(bool enabled = true) {
    RosbagConfig config;
    config.enabled = enabled;
    config.duration_sec = 2.0;
    config.duration_after_sec = 0.5;
    config.topics = "all";
    config.format = "sqlite3";
    config.storage_path = temp_dir_.string();
    config.max_bag_size_mb = 10;
    config.max_total_storage_mb = 50;
    config.auto_cleanup = true;
    return config;
  }

  SnapshotConfig create_snapshot_config() {
    SnapshotConfig config;
    config.enabled = true;
    config.default_topics = {"/test_topic"};
    return config;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<InMemoryFaultStorage> storage_;
  std::filesystem::path temp_dir_;
};

// Constructor tests

TEST_F(RosbagCaptureTest, ConstructorRequiresValidNode) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  EXPECT_THROW(RosbagCapture(nullptr, storage_.get(), rosbag_config, snapshot_config), std::invalid_argument);
}

TEST_F(RosbagCaptureTest, ConstructorRequiresValidStorage) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  EXPECT_THROW(RosbagCapture(node_.get(), nullptr, rosbag_config, snapshot_config), std::invalid_argument);
}

TEST_F(RosbagCaptureTest, ConstructorSucceedsWithValidParams) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ConstructorWithDisabledRosbag) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ConstructorThrowsOnInvalidFormat) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "invalid_format";
  auto snapshot_config = create_snapshot_config();
  EXPECT_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config), std::runtime_error);
}

TEST_F(RosbagCaptureTest, ConstructorAcceptsSqlite3Format) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "sqlite3";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// State management tests

TEST_F(RosbagCaptureTest, IsEnabledReturnsConfigState) {
  auto rosbag_config = create_rosbag_config(true);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture_enabled(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_TRUE(capture_enabled.is_enabled());

  rosbag_config.enabled = false;
  RosbagCapture capture_disabled(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_FALSE(capture_disabled.is_enabled());
}

TEST_F(RosbagCaptureTest, AutoStartsWhenNotLazy) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = false;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // With lazy_start=false, capture auto-starts on construction
  EXPECT_TRUE(capture.is_running());
}

TEST_F(RosbagCaptureTest, StartMakesRunning) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  capture.start();
  EXPECT_TRUE(capture.is_running());
}

TEST_F(RosbagCaptureTest, StopMakesNotRunning) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  capture.start();
  EXPECT_TRUE(capture.is_running());
  capture.stop();
  EXPECT_FALSE(capture.is_running());
}

TEST_F(RosbagCaptureTest, DoubleStartIsIdempotent) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  capture.start();
  EXPECT_TRUE(capture.is_running());
  capture.start();  // Second start should not throw
  EXPECT_TRUE(capture.is_running());
}

TEST_F(RosbagCaptureTest, StopWithoutStartIsIdempotent) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_NO_THROW(capture.stop());  // Should not throw when already stopped
  EXPECT_FALSE(capture.is_running());
}

// Lazy start tests

TEST_F(RosbagCaptureTest, LazyStartDoesNotRunImmediately) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = true;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // With lazy_start, capture is not running until a fault triggers it
  // The start() would typically be called internally on fault
  EXPECT_FALSE(capture.is_running());
}

TEST_F(RosbagCaptureTest, NonLazyAutoStartsOnConstruction) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = false;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // With lazy_start=false, auto-starts immediately on construction
  EXPECT_TRUE(capture.is_running());
}

// Topic configuration tests

TEST_F(RosbagCaptureTest, AllTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "all";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ConfigTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "config";
  auto snapshot_config = create_snapshot_config();
  snapshot_config.default_topics = {"/topic1", "/topic2"};
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ExplicitTopicsMode) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "explicit";
  rosbag_config.include_topics = {"/topic1", "/topic2"};
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ExcludeTopicsRespected) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.topics = "all";
  rosbag_config.exclude_topics = {"/rosout", "/parameter_events"};
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Fault lifecycle tests

TEST_F(RosbagCaptureTest, OnFaultPrefailedWhileDisabled) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // Should not throw when disabled
  EXPECT_NO_THROW(capture.on_fault_prefailed("TEST_FAULT"));
}

TEST_F(RosbagCaptureTest, OnFaultConfirmedWhileDisabled) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // Should not throw when disabled
  EXPECT_NO_THROW(capture.on_fault_confirmed("TEST_FAULT"));
}

TEST_F(RosbagCaptureTest, OnFaultClearedWhileDisabled) {
  auto rosbag_config = create_rosbag_config(false);
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  // Should not throw when disabled
  EXPECT_NO_THROW(capture.on_fault_cleared("TEST_FAULT"));
}

TEST_F(RosbagCaptureTest, OnFaultPrefailedStartsLazyCapture) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.lazy_start = true;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);
  EXPECT_FALSE(capture.is_running());
  capture.on_fault_prefailed("TEST_FAULT");
  EXPECT_TRUE(capture.is_running());
}

// Storage path tests

TEST_F(RosbagCaptureTest, DefaultStoragePathUsed) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.storage_path = "";  // Empty = use default
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, CustomStoragePathAccepted) {
  auto rosbag_config = create_rosbag_config();
  auto custom_path = temp_dir_ / "custom";
  rosbag_config.storage_path = custom_path.string();
  auto snapshot_config = create_snapshot_config();
  // Storage path creation happens when bag is written, not on construction
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Format tests

TEST_F(RosbagCaptureTest, Sqlite3FormatAccepted) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "sqlite3";
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, McapFormatAccepted) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.format = "mcap";
  auto snapshot_config = create_snapshot_config();
  // Note: mcap might not be available in all ROS installations
  // The constructor should not throw, but actual writing might fail
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Duration configuration tests

TEST_F(RosbagCaptureTest, ZeroDurationHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 0.0;
  rosbag_config.duration_after_sec = 0.0;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, NegativeDurationClamped) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = -1.0;
  rosbag_config.duration_after_sec = -1.0;
  auto snapshot_config = create_snapshot_config();
  // Should not throw, negative values should be handled gracefully
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Size limits tests

TEST_F(RosbagCaptureTest, ZeroMaxBagSizeHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.max_bag_size_mb = 0;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

TEST_F(RosbagCaptureTest, ZeroMaxTotalStorageHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.max_total_storage_mb = 0;
  auto snapshot_config = create_snapshot_config();
  EXPECT_NO_THROW(RosbagCapture(node_.get(), storage_.get(), rosbag_config, snapshot_config));
}

// Integration tests (simplified without actual message publishing)

class RosbagCaptureIntegrationTest : public RosbagCaptureTest {
 protected:
  void spin_for(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

TEST_F(RosbagCaptureIntegrationTest, FullFaultLifecycleWithNoMessages) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 0.5;
  rosbag_config.duration_after_sec = 0.2;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  // Start capture
  capture.start();
  EXPECT_TRUE(capture.is_running());

  // Simulate fault lifecycle
  capture.on_fault_prefailed("TEST_FAULT_001");

  // Let some time pass
  spin_for(std::chrono::milliseconds(100));

  capture.on_fault_confirmed("TEST_FAULT_001");

  // Wait for post-fault timer
  spin_for(std::chrono::milliseconds(300));

  // Clear the fault
  capture.on_fault_cleared("TEST_FAULT_001");

  // Stop capture
  capture.stop();
  EXPECT_FALSE(capture.is_running());
}

TEST_F(RosbagCaptureIntegrationTest, MultipleFaultsHandled) {
  auto rosbag_config = create_rosbag_config();
  rosbag_config.duration_sec = 0.5;
  rosbag_config.duration_after_sec = 0.1;
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();

  // Multiple faults in sequence
  capture.on_fault_prefailed("FAULT_A");
  spin_for(std::chrono::milliseconds(50));

  capture.on_fault_prefailed("FAULT_B");
  spin_for(std::chrono::milliseconds(50));

  capture.on_fault_confirmed("FAULT_A");
  spin_for(std::chrono::milliseconds(50));

  capture.on_fault_confirmed("FAULT_B");
  spin_for(std::chrono::milliseconds(200));

  capture.stop();
}

TEST_F(RosbagCaptureIntegrationTest, FaultClearedBeforeConfirmed) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();

  // Prefail then clear (fault didn't confirm)
  capture.on_fault_prefailed("CLEARED_EARLY");
  spin_for(std::chrono::milliseconds(50));
  capture.on_fault_cleared("CLEARED_EARLY");

  // Should not crash
  spin_for(std::chrono::milliseconds(100));

  capture.stop();
}

TEST_F(RosbagCaptureIntegrationTest, ConfirmedWithoutPrefailed) {
  auto rosbag_config = create_rosbag_config();
  auto snapshot_config = create_snapshot_config();
  RosbagCapture capture(node_.get(), storage_.get(), rosbag_config, snapshot_config);

  capture.start();

  // Direct confirm (edge case)
  capture.on_fault_confirmed("DIRECT_CONFIRM");
  spin_for(std::chrono::milliseconds(200));

  capture.stop();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
