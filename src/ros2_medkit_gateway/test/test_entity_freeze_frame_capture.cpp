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
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::DataProvider;
using ros2_medkit_gateway::DataProviderError;
using ros2_medkit_gateway::DataProviderErrorInfo;
using ros2_medkit_gateway::EntityFreezeFrameCapture;
using ros2_medkit_gateway::handlers::FaultHandlers;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;

namespace {

/// Fake plugin DataProvider serving fixed PLC-like values for one entity.
class FakePlcDataProvider : public DataProvider {
 public:
  explicit FakePlcDataProvider(std::string entity_id) : entity_id_(std::move(entity_id)) {
  }

  tl::expected<ros2_medkit_gateway::dto::DataListResult, DataProviderErrorInfo>
  list_data(const std::string & entity_id) override {
    if (entity_id != entity_id_) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::EntityNotFound, "not found", 404});
    }
    json items = json::array();
    items.push_back({{"id", "temperature"}, {"name", "Temperature"}, {"value", 42.5}});
    items.push_back({{"id", "pressure"}, {"name", "Pressure"}, {"value", 3.2}});
    return ros2_medkit_gateway::dto::DataListResult{json{{"items", std::move(items)}}};
  }

  tl::expected<ros2_medkit_gateway::dto::DataValue, DataProviderErrorInfo>
  read_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "unused", 500});
  }

  tl::expected<ros2_medkit_gateway::dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/,
             const json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only", 405});
  }

 private:
  std::string entity_id_;
};

FaultEvent make_confirmed_event(const std::string & fault_code, const std::vector<std::string> & sources) {
  FaultEvent event;
  event.event_type = FaultEvent::EVENT_CONFIRMED;
  event.fault.fault_code = fault_code;
  event.fault.severity = Fault::SEVERITY_ERROR;
  event.fault.status = Fault::STATUS_CONFIRMED;
  event.fault.reporting_sources = sources;
  return event;
}

class EntityFreezeFrameCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("entity_freeze_frame_test_node");
    publisher_ = node_->create_publisher<FaultEvent>("/fault_manager/events", rclcpp::QoS(100).reliable());
    provider_ = std::make_unique<FakePlcDataProvider>("plc_app");
  }

  /// Publish an event and spin until the capture holds frames (or timeout).
  bool publish_and_wait(EntityFreezeFrameCapture & capture, const FaultEvent & event) {
    publisher_->publish(event);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      if (!capture.frames_for(event.fault.fault_code).empty()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<FaultEvent>::SharedPtr publisher_;
  std::unique_ptr<FakePlcDataProvider> provider_;
};

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ConfirmedPluginFaultCapturesEntityValues) {
  EntityFreezeFrameCapture capture(node_.get(), [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_OVERPRESSURE", {"plc_app"})));

  auto frames = capture.frames_for("PLC_OVERPRESSURE");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("temperature", 0.0), 42.5);
  EXPECT_DOUBLE_EQ(frames[0].values.value("pressure", 0.0), 3.2);
  EXPECT_GT(frames[0].captured_at_ns, 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, NonPluginSourceCapturesNothing) {
  EntityFreezeFrameCapture capture(node_.get(), [](const std::string &) -> DataProvider * {
    return nullptr;  // ROS FQN sources are never plugin-owned
  });

  auto event = make_confirmed_event("ROS_FAULT", {"/powertrain/engine/temp_sensor"});
  publisher_->publish(event);
  // Give the callback a chance to run, then assert nothing was captured.
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("ROS_FAULT").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, NonConfirmedEventsAreIgnored) {
  EntityFreezeFrameCapture capture(node_.get(), [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  auto event = make_confirmed_event("PLC_UPDATED_ONLY", {"plc_app"});
  event.event_type = FaultEvent::EVENT_UPDATED;
  publisher_->publish(event);
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_UPDATED_ONLY").empty());
}

TEST(ValuesFromListContent, BuildsCompactDictFromItems) {
  json content = {{"items", json::array({{{"id", "a"}, {"value", 1}},
                                         {{"name", "b"}, {"value", "on"}},  // falls back to "name" key
                                         {{"id", "c"}}})}};                 // no value -> null
  auto values = EntityFreezeFrameCapture::values_from_list_content(content);
  EXPECT_EQ(values["a"], 1);
  EXPECT_EQ(values["b"], "on");
  EXPECT_TRUE(values["c"].is_null());
}

TEST(ValuesFromListContent, NonItemsShapeKeptVerbatim) {
  json content = {{"status", "ok"}, {"raw", 7}};
  EXPECT_EQ(EntityFreezeFrameCapture::values_from_list_content(content), content);
}

TEST(MergeEntityFreezeFrames, AppendsWhenNoConfiguredFreezeFrame) {
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};
  frame.captured_at_ns = 1234;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  const auto & snap = merged["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["name"], "plc_app");
  EXPECT_EQ(json::parse(snap["data"].get<std::string>())["temperature"], 42.5);
  EXPECT_EQ(snap["captured_at_ns"], 1234);
}

TEST(MergeEntityFreezeFrames, ExplicitConfigWins) {
  // A fault_manager-captured freeze-frame (explicit snapshot config) must
  // suppress the zero-config entity frames.
  json env_data = {{"snapshots", json::array({{{"type", "freeze_frame"},
                                               {"snapshot_type", "freeze_frame"},
                                               {"name", "scan"},
                                               {"data", "{}"},
                                               {"topic", "/scan"},
                                               {"message_type", "sensor_msgs/msg/LaserScan"},
                                               {"captured_at_ns", 1}}})}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  EXPECT_EQ(merged["snapshots"][0]["name"], "scan");
}

TEST(MergeEntityFreezeFrames, RosbagSnapshotDoesNotSuppressEntityFrames) {
  // Only a freeze-frame counts as "explicit config captured": a rosbag-only
  // environment still gets the entity frame appended.
  json env_data = {
      {"snapshots",
       json::array({{{"type", "rosbag"}, {"snapshot_type", "rosbag"}, {"name", "rosbag_X"}, {"fault_code", "X"}}})}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 2u);
  EXPECT_EQ(merged["snapshots"][1]["name"], "plc_app");
}

TEST(MergeEntityFreezeFrames, NoFramesNoChange) {
  json env_data = {{"snapshots", json::array()}};
  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {});
  EXPECT_EQ(merged, env_data);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
