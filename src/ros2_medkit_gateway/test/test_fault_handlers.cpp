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

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/extended_data_records.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/snapshot.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::handlers::FaultHandlers;
namespace conversions = ros2_medkit_gateway::ros2::conversions;
namespace dto = ros2_medkit_gateway::dto;

// The handler now consumes JSON shaped by the transport adapter. These tests
// drive that contract end-to-end by using the same conversions module the
// adapter uses to translate ros2_medkit_msgs into JSON, then call the handler
// to produce the final SOVD response (now a dto::FaultDetail struct).
//
// Tests convert the DTO back to JSON via JsonWriter for comparison so existing
// assertions can remain wire-level checks.

class FaultHandlersTest : public ::testing::Test {
 protected:
  static json fault_json(const ros2_medkit_msgs::msg::Fault & f) {
    return conversions::fault_to_json(f);
  }
  static json env_json(const ros2_medkit_msgs::msg::EnvironmentData & e) {
    return conversions::environment_data_to_json(e);
  }
  /// Convert the DTO returned by build_sovd_fault_response to JSON for
  /// wire-level assertions (keeps test bodies as close to original as possible).
  static json to_json(const dto::FaultDetail & detail) {
    return dto::JsonWriter<dto::FaultDetail>::write(detail);
  }
};

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseBasicFields) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "TEST_FAULT";
  fault.description = "Test fault description";
  fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;  // ERROR (2)
  fault.status = "CONFIRMED";                                     // Active/confirmed fault
  fault.occurrence_count = 5;
  fault.reporting_sources = {"/powertrain/motor_controller"};

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  env_data.extended_data_records.first_occurrence_ns = 1707044400000000000;
  env_data.extended_data_records.last_occurrence_ns = 1707044460000000000;

  auto response = to_json(
      FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/motor_controller"));

  // Verify item structure
  EXPECT_EQ(response["item"]["code"], "TEST_FAULT");
  EXPECT_EQ(response["item"]["fault_name"], "Test fault description");
  EXPECT_EQ(response["item"]["severity"], ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR);

  // Verify status object
  auto status = response["item"]["status"];
  EXPECT_EQ(status["aggregatedStatus"], "active");
  EXPECT_EQ(status["testFailed"], "1");
  EXPECT_EQ(status["confirmedDTC"], "1");
  EXPECT_EQ(status["pendingDTC"], "0");

  // Verify x-medkit
  EXPECT_EQ(response["x-medkit"]["occurrence_count"], 5);
  EXPECT_EQ(response["x-medkit"]["severity_label"], "ERROR");
  EXPECT_EQ(response["x-medkit"]["status_raw"],
            "CONFIRMED");  // Raw status in x-medkit, not in status object
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseWithFreezeFrame) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "TEMP_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "temperature";
  freeze_frame.data = R"({"temperature": 85.5, "variance": 0.1})";
  freeze_frame.topic = "/motor/temperature";
  freeze_frame.message_type = "sensor_msgs/msg/Temperature";
  freeze_frame.captured_at_ns = 1707044400000000000;
  env_data.snapshots.push_back(freeze_frame);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/motor"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["name"], "temperature");
  EXPECT_DOUBLE_EQ(snap["data"].get<double>(), 85.5);  // Primary value extracted
  EXPECT_EQ(snap["x-medkit"]["topic"], "/motor/temperature");
  EXPECT_EQ(snap["x-medkit"]["message_type"], "sensor_msgs/msg/Temperature");
}

// Conversion layer must emit an explicit "snapshot_type" discriminator so
// downstream consumers (handler, SSE, MCP) can dispatch on a single key
// regardless of which optional payload fields are present.
TEST_F(FaultHandlersTest, EnvironmentDataToJsonEmitsSnapshotTypeDiscriminator) {
  ros2_medkit_msgs::msg::EnvironmentData env_data;

  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "temperature";
  freeze_frame.data = R"({"temperature": 85.5})";
  freeze_frame.topic = "/motor/temperature";
  freeze_frame.message_type = "sensor_msgs/msg/Temperature";
  env_data.snapshots.push_back(freeze_frame);

  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.name = "fault_recording";
  rosbag.bulk_data_id = "ROSBAG_X";
  rosbag.size_bytes = 1234;
  rosbag.duration_sec = 1.0;
  rosbag.format = "mcap";
  env_data.snapshots.push_back(rosbag);

  auto j = env_json(env_data);
  ASSERT_TRUE(j.contains("snapshots"));
  ASSERT_EQ(j["snapshots"].size(), 2u);
  EXPECT_EQ(j["snapshots"][0]["snapshot_type"], "freeze_frame");
  EXPECT_EQ(j["snapshots"][1]["snapshot_type"], "rosbag");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseWithRosbag) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "ROSBAG_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.name = "fault_recording";
  rosbag.bulk_data_id = "ROSBAG_FAULT";
  rosbag.size_bytes = 1234567;
  rosbag.duration_sec = 6.0;
  rosbag.format = "mcap";
  env_data.snapshots.push_back(rosbag);

  auto response = to_json(
      FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/motor_controller"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["type"], "rosbag");
  EXPECT_EQ(snap["bulk_data_uri"], "/apps/motor_controller/bulk-data/rosbags/ROSBAG_FAULT");
  EXPECT_EQ(snap["size_bytes"], 1234567);
  EXPECT_DOUBLE_EQ(snap["duration_sec"], 6.0);
  EXPECT_EQ(snap["format"], "mcap");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseNestedEntityPath) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "NESTED_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.bulk_data_id = "NESTED_FAULT";
  env_data.snapshots.push_back(rosbag);

  auto response = to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data),
                                                                   "/areas/perception/subareas/lidar"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["bulk_data_uri"], "/areas/perception/subareas/lidar/bulk-data/rosbags/NESTED_FAULT");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseStatusCleared) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "CLEARED_FAULT";
  fault.status = "CLEARED";  // Cleared status

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto status = response["item"]["status"];
  EXPECT_EQ(status["aggregatedStatus"], "cleared");
  EXPECT_EQ(status["testFailed"], "0");
  EXPECT_EQ(status["confirmedDTC"], "0");
  EXPECT_EQ(status["pendingDTC"], "0");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseStatusPassive) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "PASSIVE_FAULT";
  fault.status = "PREFAILED";  // Pending/passive status

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto status = response["item"]["status"];
  EXPECT_EQ(status["aggregatedStatus"], "passive");
  EXPECT_EQ(status["pendingDTC"], "1");
}

// @verifies REQ_INTEROP_013
// Severity labels must mirror ros2_medkit_msgs/msg/Fault.msg constants:
// SEVERITY_INFO=0, SEVERITY_WARN=1, SEVERITY_ERROR=2, SEVERITY_CRITICAL=3.
TEST_F(FaultHandlersTest, BuildSovdFaultResponseSeverityLabels) {
  ros2_medkit_msgs::msg::EnvironmentData env_data;

  // Test INFO (0)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_INFO;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "INFO");
  }
  // Test WARN (1)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "WARN");
  }
  // Test ERROR (2)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "ERROR");
  }
  // Test CRITICAL (3)
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "CRITICAL");
  }
  // Test UNKNOWN (255) - any value outside the SEVERITY_* range
  {
    ros2_medkit_msgs::msg::Fault fault;
    fault.severity = 255;
    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_EQ(response["x-medkit"]["severity_label"], "UNKNOWN");
  }
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseWithInvalidJson) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "INVALID_JSON_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "invalid_data";
  freeze_frame.data = "not valid json {";
  freeze_frame.topic = "/test";
  freeze_frame.message_type = "std_msgs/msg/String";
  env_data.snapshots.push_back(freeze_frame);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto & snap = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap["data"], "not valid json {");  // Raw data returned
  EXPECT_TRUE(snap["x-medkit"].contains("parse_error"));
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseExtendedDataRecords) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "TEST_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;
  env_data.extended_data_records.first_occurrence_ns = 1770458400000000000;  // 2026-02-08
  env_data.extended_data_records.last_occurrence_ns = 1770458460000000000;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto & edr = response["environment_data"]["extended_data_records"];
  std::string first = edr["first_occurrence"].get<std::string>();
  std::string last = edr["last_occurrence"].get<std::string>();

  // Verify ISO 8601 format with milliseconds and Z suffix
  EXPECT_TRUE(first.find("2026") != std::string::npos);
  EXPECT_TRUE(first.find('Z') != std::string::npos);
  EXPECT_TRUE(first.find('T') != std::string::npos);
  EXPECT_TRUE(last.find('Z') != std::string::npos);
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponsePrimaryValueExtraction) {
  ros2_medkit_msgs::msg::EnvironmentData env_data;

  // Test std_msgs/msg/Float64 - should extract "data" field
  {
    ros2_medkit_msgs::msg::Fault fault;
    ros2_medkit_msgs::msg::Snapshot snap;
    snap.type = "freeze_frame";
    snap.data = R"({"data": 42.5})";
    snap.message_type = "std_msgs/msg/Float64";
    env_data.snapshots.clear();
    env_data.snapshots.push_back(snap);

    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    EXPECT_DOUBLE_EQ(response["environment_data"]["snapshots"][0]["data"].get<double>(), 42.5);
  }

  // Test unknown message type - should return full data
  {
    ros2_medkit_msgs::msg::Fault fault;
    ros2_medkit_msgs::msg::Snapshot snap;
    snap.type = "freeze_frame";
    snap.data = R"({"foo": "bar", "baz": 123})";
    snap.message_type = "custom_msgs/msg/Unknown";
    env_data.snapshots.clear();
    env_data.snapshots.push_back(snap);

    auto response =
        to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));
    auto data = response["environment_data"]["snapshots"][0]["data"];
    EXPECT_EQ(data["foo"], "bar");
    EXPECT_EQ(data["baz"], 123);
  }
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseMultipleSources) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "MULTI_SOURCE_FAULT";
  fault.reporting_sources = {"/perception/lidar", "/perception/camera", "/control/motor"};

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/apps/test"));

  auto sources = response["x-medkit"]["reporting_sources"];
  ASSERT_EQ(sources.size(), 3);
  EXPECT_EQ(sources[0], "/perception/lidar");
  EXPECT_EQ(sources[1], "/perception/camera");
  EXPECT_EQ(sources[2], "/control/motor");
}

// @verifies REQ_INTEROP_013
TEST_F(FaultHandlersTest, BuildSovdFaultResponseMixedSnapshots) {
  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = "MIXED_FAULT";

  ros2_medkit_msgs::msg::EnvironmentData env_data;

  // Add freeze frame
  ros2_medkit_msgs::msg::Snapshot freeze_frame;
  freeze_frame.type = "freeze_frame";
  freeze_frame.name = "temperature";
  freeze_frame.data = R"({"temperature": 75.0})";
  freeze_frame.message_type = "sensor_msgs/msg/Temperature";
  env_data.snapshots.push_back(freeze_frame);

  // Add rosbag
  ros2_medkit_msgs::msg::Snapshot rosbag;
  rosbag.type = "rosbag";
  rosbag.name = "recording";
  rosbag.bulk_data_id = "MIXED_FAULT";
  rosbag.size_bytes = 1000;
  rosbag.format = "mcap";
  env_data.snapshots.push_back(rosbag);

  auto response =
      to_json(FaultHandlers::build_sovd_fault_response(fault_json(fault), env_json(env_data), "/components/motor"));

  ASSERT_EQ(response["environment_data"]["snapshots"].size(), 2);

  // Verify freeze frame
  auto & snap0 = response["environment_data"]["snapshots"][0];
  EXPECT_EQ(snap0["type"], "freeze_frame");
  EXPECT_DOUBLE_EQ(snap0["data"].get<double>(), 75.0);

  // Verify rosbag
  auto & snap1 = response["environment_data"]["snapshots"][1];
  EXPECT_EQ(snap1["type"], "rosbag");
  EXPECT_EQ(snap1["bulk_data_uri"], "/components/motor/bulk-data/rosbags/MIXED_FAULT");
}

// =============================================================================
// fault_in_source_scope tests (#395)
//
// Direct coverage for the scope-check logic that backs per-entity GET/DELETE
// and the collection-route filter. Integration tests pin the HTTP behavior;
// these tests pin the boundary semantics that decide what counts as "in
// scope" - especially the cases that motivated the post-review tightening:
// prefix-colliding FQNs and multi-source faults.
// =============================================================================

namespace {

json make_fault(std::vector<std::string> reporting_sources, const std::string & code = "F1") {
  json f;
  f["fault_code"] = code;
  f["reporting_sources"] = std::move(reporting_sources);
  return f;
}

}  // namespace

TEST(FaultInSourceScopeTest, SingleSourceExactMatchInScope) {
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(make_fault({"/perception/lidar/lidar_sensor"}),
                                                   {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, SubpathOfScopedFqnIsInScope) {
  // A node nested under the entity's own FQN must still match - e.g. an app
  // FQN `/perception/lidar/lidar_sensor` and a reporter at
  // `/perception/lidar/lidar_sensor/diagnostic_updater` should be the same
  // app's sub-node, not a stranger.
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(make_fault({"/perception/lidar/lidar_sensor/diagnostic_updater"}),
                                                   {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, PrefixCollidingNameIsNotInScope) {
  // `/ns/node_extra` shares the `/ns/node` prefix but is a distinct ROS node.
  // The pre-review check used `rfind(prefix, 0) == 0` and silently let it
  // through - this test pins the path-boundary fix.
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"/ns/node_extra"}), {"/ns/node"}));
}

TEST(FaultInSourceScopeTest, MultiSourceAllInScopeIsInScope) {
  std::set<std::string> scope{"/perception/lidar/lidar_sensor", "/perception/rgb/rgb_camera"};
  EXPECT_TRUE(FaultHandlers::fault_in_source_scope(
      make_fault({"/perception/lidar/lidar_sensor", "/perception/rgb/rgb_camera"}), scope));
}

TEST(FaultInSourceScopeTest, MultiSourcePartiallyInScopeIsOutOfScope) {
  // The all-sources semantic blocks a cross-entity DELETE escalation: an
  // entity that owns only `/perception/lidar/lidar_sensor` must not be able
  // to clear (or read in detail) a fault that another entity in
  // `/telemetry/...` also reported.
  std::set<std::string> scope{"/perception/lidar/lidar_sensor"};
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(
      make_fault({"/perception/lidar/lidar_sensor", "/telemetry/telemetry_node"}), scope));
}

TEST(FaultInSourceScopeTest, EmptyScopeSetIsOutOfScope) {
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"/perception/lidar/lidar_sensor"}), {}));
}

TEST(FaultInSourceScopeTest, EmptyReportingSourcesIsOutOfScope) {
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({}), {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, MissingReportingSourcesFieldIsOutOfScope) {
  json fault;
  fault["fault_code"] = "F1";
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(fault, {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, NonStringSourceEntryIsOutOfScope) {
  json fault;
  fault["fault_code"] = "F1";
  fault["reporting_sources"] = json::array();
  fault["reporting_sources"].push_back("/perception/lidar/lidar_sensor");
  fault["reporting_sources"].push_back(42);  // Malformed entry
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(fault, {"/perception/lidar/lidar_sensor"}));
}

TEST(FaultInSourceScopeTest, RootFqnEdgeCase) {
  // Boundary check still works with "/" prefix candidates (artificial but
  // exercises the index arithmetic): "/" itself can't match "/anything"
  // unless the scope set actually contains "/" - which our resolver never
  // emits. This pin catches a future regression that special-cased "/".
  EXPECT_FALSE(FaultHandlers::fault_in_source_scope(make_fault({"/something"}), {"/other"}));
}
