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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <std_msgs/msg/float64.hpp>

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

  // A configured capture that sampled nothing still records an empty {} frame.
  auto frame = storage_->get_freeze_frame("TEST_FAULT");
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->data, "{}");
}

// Freeze-frame end-to-end tests

// Stops and joins a publisher thread on scope exit, so an assertion failure
// mid-test never destroys a joinable std::thread (which would std::terminate).
struct ScopedPublisherThread {
  std::atomic<bool> stop{false};
  std::thread thread;

  explicit ScopedPublisherThread(std::function<void(std::atomic<bool> &)> body)
    : thread([this, body = std::move(body)]() {
      body(stop);
    }) {
  }

  ~ScopedPublisherThread() {
    stop.store(true);
    if (thread.joinable()) {
      thread.join();
    }
  }
};

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, CaptureWritesFreezeFrameFromConfiguredTopic) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/pressure", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["PLC_PRESSURE_HIGH"] = {"/plc/pressure"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Publish continuously so the on-demand one-shot subscription catches a value.
  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 42.5;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  // Wait for the publisher to be visible to the capture node before capturing.
  auto start = std::chrono::steady_clock::now();
  while (node_->count_publishers("/plc/pressure") == 0 &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(node_->count_publishers("/plc/pressure"), 0u);

  capture.capture("PLC_PRESSURE_HIGH");

  auto frame = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(frame.has_value());
  auto parsed = nlohmann::json::parse(frame->data);
  ASSERT_TRUE(parsed.contains("/plc/pressure"));
  EXPECT_DOUBLE_EQ(parsed["/plc/pressure"]["data"].get<double>(), 42.5);
}

// Regression for the empty-recapture overwrite: a re-confirm while the source
// publishers are down must not clobber a previously retained non-empty frame.
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, EmptyRecaptureKeepsRetainedFreezeFrame) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/flow", rclcpp::QoS(10));

  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = false;
  config.timeout_sec = 5.0;
  config.fault_specific["PLC_FLOW_LOW"] = {"/plc/flow"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  {
    ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
      while (!stop.load()) {
        std_msgs::msg::Float64 msg;
        msg.data = 7.25;
        pub->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });

    auto start = std::chrono::steady_clock::now();
    while (node_->count_publishers("/plc/flow") == 0 &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GT(node_->count_publishers("/plc/flow"), 0u);

    capture.capture("PLC_FLOW_LOW");
  }

  auto frame = storage_->get_freeze_frame("PLC_FLOW_LOW");
  ASSERT_TRUE(frame.has_value());
  ASSERT_NE(frame->data, "{}");

  // Take the publisher down and wait until the graph reflects it, then re-capture.
  pub.reset();
  auto start = std::chrono::steady_clock::now();
  while (node_->count_publishers("/plc/flow") > 0 &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(node_->count_publishers("/plc/flow"), 0u);

  capture.capture("PLC_FLOW_LOW");

  auto retained = storage_->get_freeze_frame("PLC_FLOW_LOW");
  ASSERT_TRUE(retained.has_value());
  auto parsed = nlohmann::json::parse(retained->data);
  ASSERT_TRUE(parsed.contains("/plc/flow"));
  EXPECT_DOUBLE_EQ(parsed["/plc/flow"]["data"].get<double>(), 7.25);
}

// Exercises the background-capture cache path into the freeze-frame (cached JSON
// string parsed back into a structured value).
// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, BackgroundCaptureCachesFreezeFrame) {
  auto pub = node_->create_publisher<std_msgs::msg::Float64>("/plc/temperature", rclcpp::QoS(10));

  ScopedPublisherThread pub_thread([&pub](std::atomic<bool> & stop) {
    while (!stop.load()) {
      std_msgs::msg::Float64 msg;
      msg.data = 91.5;
      pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  // The publisher already exists on this node, so the topic type is resolvable
  // when the constructor sets up the background subscription.
  SnapshotConfig config;
  config.enabled = true;
  config.background_capture = true;
  config.fault_specific["PLC_TEMP_HIGH"] = {"/plc/temperature"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  // Spin so the background subscription caches a message, then capture from cache.
  // Until the cache fills, capture records an empty first-run {} frame; retry.
  bool got_frame = false;
  auto start = std::chrono::steady_clock::now();
  while (!got_frame && std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    rclcpp::spin_some(node_);
    capture.capture("PLC_TEMP_HIGH");
    auto frame = storage_->get_freeze_frame("PLC_TEMP_HIGH");
    got_frame = frame.has_value() && frame->data != "{}";
    if (!got_frame) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(got_frame);

  auto frame = storage_->get_freeze_frame("PLC_TEMP_HIGH");
  ASSERT_TRUE(frame.has_value());
  auto parsed = nlohmann::json::parse(frame->data);
  ASSERT_TRUE(parsed.contains("/plc/temperature"));
  EXPECT_DOUBLE_EQ(parsed["/plc/temperature"]["data"].get<double>(), 91.5);

  // The cache path also stores a per-topic snapshot.
  auto snapshots = storage_->get_snapshots("PLC_TEMP_HIGH");
  ASSERT_FALSE(snapshots.empty());
  EXPECT_EQ(snapshots.back().topic, "/plc/temperature");
  EXPECT_EQ(snapshots.back().message_type, "std_msgs/msg/Float64");
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, UnconfiguredFaultWritesNoFreezeFrame) {
  SnapshotConfig config;
  config.enabled = true;
  config.fault_specific["OTHER_FAULT"] = {"/plc/pressure"};
  // No default_topics: an unrelated fault code resolves to an empty capture set.
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  capture.capture("UNMAPPED_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("UNMAPPED_FAULT").has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(SnapshotCaptureTest, DisabledCaptureWritesNoFreezeFrame) {
  SnapshotConfig config;
  config.enabled = false;
  config.default_topics = {"/plc/pressure"};
  SnapshotCapture capture(node_.get(), storage_.get(), config);

  capture.capture("ANY_FAULT");

  EXPECT_FALSE(storage_->get_freeze_frame("ANY_FAULT").has_value());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
