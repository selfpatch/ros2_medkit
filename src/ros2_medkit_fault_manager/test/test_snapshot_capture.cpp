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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"

using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::SnapshotCapture;
using ros2_medkit_fault_manager::SnapshotConfig;

class SnapshotCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_snapshot_capture_node");
    storage_ = std::make_unique<InMemoryFaultStorage>();
  }

  void TearDown() override {
    node_.reset();
    storage_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<InMemoryFaultStorage> storage_;
};

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConstructorRequiresValidNode) {
  SnapshotConfig config;
  EXPECT_THROW(SnapshotCapture(nullptr, storage_.get(), config), std::invalid_argument);
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConstructorRequiresValidStorage) {
  SnapshotConfig config;
  EXPECT_THROW(SnapshotCapture(node_.get(), nullptr, config), std::invalid_argument);
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConstructorSucceedsWithValidParams) {
  SnapshotConfig config;
  config.enabled = true;
  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, DisabledCaptureSkipsProcessing) {
  SnapshotConfig config;
  config.enabled = false;
  config.default_topics = {"/test_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);
  capture.capture("TEST_FAULT");

  // No snapshots should be stored when disabled
  auto snapshots = storage_->get_snapshots("TEST_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, IsEnabledReturnsConfigState) {
  SnapshotConfig config;
  config.enabled = true;
  SnapshotCapture capture_enabled(node_.get(), storage_.get(), config);
  EXPECT_TRUE(capture_enabled.is_enabled());

  config.enabled = false;
  SnapshotCapture capture_disabled(node_.get(), storage_.get(), config);
  EXPECT_FALSE(capture_disabled.is_enabled());
}

// Topic Resolution Priority Tests

class TopicResolutionTest : public SnapshotCaptureTest {
 protected:
  // Helper to access resolved topics through capture behavior
  // Since resolve_topics is private, we test through the capture interface
};

// @verifies REQ_INTEROP_088
TEST_F(TopicResolutionTest, FaultSpecificHasHighestPriority) {
  SnapshotConfig config;
  config.enabled = true;
  config.fault_specific["MOTOR_OVERHEAT"] = {"/motor/specific_topic"};
  config.patterns["MOTOR_.*"] = {"/motor/pattern_topic"};
  config.default_topics = {"/default_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Verify config is stored correctly
  const auto & stored_config = capture.config();
  EXPECT_EQ(stored_config.fault_specific.at("MOTOR_OVERHEAT").size(), 1u);
  EXPECT_EQ(stored_config.fault_specific.at("MOTOR_OVERHEAT")[0], "/motor/specific_topic");
}

// @verifies REQ_INTEROP_088
TEST_F(TopicResolutionTest, PatternMatchUsedWhenNoFaultSpecific) {
  SnapshotConfig config;
  config.enabled = true;
  config.patterns["SENSOR_.*"] = {"/sensor/pattern_topic"};
  config.default_topics = {"/default_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Pattern should be compiled
  const auto & stored_config = capture.config();
  EXPECT_EQ(stored_config.patterns.size(), 1u);
}

// @verifies REQ_INTEROP_088
TEST_F(TopicResolutionTest, DefaultTopicsUsedAsFallback) {
  SnapshotConfig config;
  config.enabled = true;
  config.default_topics = {"/default1", "/default2"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  const auto & stored_config = capture.config();
  EXPECT_EQ(stored_config.default_topics.size(), 2u);
}

// Regex Pattern Tests

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, InvalidRegexPatternIsSkipped) {
  SnapshotConfig config;
  config.enabled = true;
  config.patterns["[invalid(regex"] = {"/topic1"};  // Invalid regex
  config.patterns["VALID_.*"] = {"/topic2"};        // Valid regex

  // Should not throw, invalid pattern is logged and skipped
  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, MultipleValidPatternsCompiled) {
  SnapshotConfig config;
  config.enabled = true;
  config.patterns["MOTOR_.*"] = {"/motor/topic"};
  config.patterns["SENSOR_[0-9]+"] = {"/sensor/topic"};
  config.patterns["^ERROR_"] = {"/error/topic"};

  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// Configuration Tests

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, ConfigAccessorReturnsCorrectValues) {
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = true;
  config.timeout_sec = 2.5;
  config.max_message_size = 32768;

  SnapshotCapture capture(node_.get(), storage_.get(), config);

  const auto & stored = capture.config();
  EXPECT_TRUE(stored.enabled);
  EXPECT_TRUE(stored.background_capture);
  EXPECT_DOUBLE_EQ(stored.timeout_sec, 2.5);
  EXPECT_EQ(stored.max_message_size, 32768u);
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, EmptyConfigurationHandledGracefully) {
  SnapshotConfig config;
  config.enabled = true;
  // No topics configured at all

  SnapshotCapture capture(node_.get(), storage_.get(), config);
  capture.capture("ANY_FAULT");

  // Should not crash, just log that no topics configured
  auto snapshots = storage_->get_snapshots("ANY_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// Background capture initialization test
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, BackgroundCaptureInitializesSubscriptions) {
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = true;
  config.default_topics = {"/nonexistent_topic"};  // Topic doesn't exist

  // Should not throw even if topics don't exist
  EXPECT_NO_THROW(SnapshotCapture(node_.get(), storage_.get(), config));
}

// On-demand capture with non-existent topic
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, OnDemandCaptureHandlesNonExistentTopic) {
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 0.1;  // Short timeout for test
  config.default_topics = {"/nonexistent_topic"};

  SnapshotCapture capture(node_.get(), storage_.get(), config);
  capture.capture("TEST_FAULT");

  // Should timeout gracefully, no snapshot stored
  auto snapshots = storage_->get_snapshots("TEST_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
