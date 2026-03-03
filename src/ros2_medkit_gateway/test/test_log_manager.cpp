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
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/log_manager.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::LogConfig;
using ros2_medkit_gateway::LogEntry;
using ros2_medkit_gateway::LogManager;

// ============================================================
// Static utility tests — no ROS 2 node required
// ============================================================

// @verifies REQ_INTEROP_061
TEST(LogManagerSeverityTest, LevelToSeverityMapping) {
  EXPECT_EQ(LogManager::level_to_severity(10), "debug");
  EXPECT_EQ(LogManager::level_to_severity(20), "info");
  EXPECT_EQ(LogManager::level_to_severity(30), "warning");
  EXPECT_EQ(LogManager::level_to_severity(40), "error");
  EXPECT_EQ(LogManager::level_to_severity(50), "fatal");
  // Unknown level defaults to debug
  EXPECT_EQ(LogManager::level_to_severity(0), "debug");
  EXPECT_EQ(LogManager::level_to_severity(99), "debug");
}

// @verifies REQ_INTEROP_061
TEST(LogManagerSeverityTest, SeverityToLevelMapping) {
  EXPECT_EQ(LogManager::severity_to_level("debug"), 10);
  EXPECT_EQ(LogManager::severity_to_level("info"), 20);
  EXPECT_EQ(LogManager::severity_to_level("warning"), 30);
  EXPECT_EQ(LogManager::severity_to_level("error"), 40);
  EXPECT_EQ(LogManager::severity_to_level("fatal"), 50);
  // Invalid -> 0
  EXPECT_EQ(LogManager::severity_to_level(""), 0);
  EXPECT_EQ(LogManager::severity_to_level("unknown"), 0);
  EXPECT_EQ(LogManager::severity_to_level("WARN"), 0);  // case-sensitive
  EXPECT_EQ(LogManager::severity_to_level("warn"), 0);  // ROS 2 name, not SOVD name
}

// @verifies REQ_INTEROP_061
TEST(LogManagerSeverityTest, IsValidSeverity) {
  EXPECT_TRUE(LogManager::is_valid_severity("debug"));
  EXPECT_TRUE(LogManager::is_valid_severity("info"));
  EXPECT_TRUE(LogManager::is_valid_severity("warning"));
  EXPECT_TRUE(LogManager::is_valid_severity("error"));
  EXPECT_TRUE(LogManager::is_valid_severity("fatal"));
  EXPECT_FALSE(LogManager::is_valid_severity("warn"));  // ROS 2 name
  EXPECT_FALSE(LogManager::is_valid_severity(""));
  EXPECT_FALSE(LogManager::is_valid_severity("WARN"));
  EXPECT_FALSE(LogManager::is_valid_severity("INFORMATION"));
  EXPECT_FALSE(LogManager::is_valid_severity("verbose"));
}

// @verifies REQ_INTEROP_061
TEST(LogManagerFqnTest, NormalizeStripsLeadingSlash) {
  EXPECT_EQ(LogManager::normalize_fqn("/powertrain/engine/temp_sensor"), "powertrain/engine/temp_sensor");
  EXPECT_EQ(LogManager::normalize_fqn("/perception/lidar/lidar_sensor"), "perception/lidar/lidar_sensor");
}

// @verifies REQ_INTEROP_061
TEST(LogManagerFqnTest, NormalizeNoLeadingSlashUnchanged) {
  EXPECT_EQ(LogManager::normalize_fqn("powertrain/engine/temp_sensor"), "powertrain/engine/temp_sensor");
  EXPECT_EQ(LogManager::normalize_fqn("temp_sensor"), "temp_sensor");
}

// @verifies REQ_INTEROP_061
TEST(LogManagerFqnTest, NormalizeEmptyStringUnchanged) {
  EXPECT_EQ(LogManager::normalize_fqn(""), "");
}

// @verifies REQ_INTEROP_061
TEST(LogManagerEntryToJsonTest, BasicFields) {
  LogEntry entry;
  entry.id = 42;
  entry.stamp_sec = 1707044400;
  entry.stamp_nanosec = 123456789;
  entry.level = 30;                              // warning
  entry.name = "powertrain/engine/temp_sensor";  // no leading slash
  entry.msg = "Calibration drift detected";
  entry.function = "read_sensor";
  entry.file = "temp_sensor.cpp";
  entry.line = 99;

  auto j = LogManager::entry_to_json(entry);

  EXPECT_EQ(j["id"], "log_42");
  EXPECT_EQ(j["severity"], "warning");
  EXPECT_EQ(j["message"], "Calibration drift detected");

  // Timestamp must be ISO 8601 with Z suffix
  const auto & ts = j["timestamp"].get<std::string>();
  EXPECT_EQ(ts.back(), 'Z');
  EXPECT_NE(ts.find('T'), std::string::npos);
  // Context
  EXPECT_EQ(j["context"]["node"], "powertrain/engine/temp_sensor");
  EXPECT_EQ(j["context"]["function"], "read_sensor");
  EXPECT_EQ(j["context"]["file"], "temp_sensor.cpp");
  EXPECT_EQ(j["context"]["line"], 99);
}

// @verifies REQ_INTEROP_061
TEST(LogManagerEntryToJsonTest, EmptyOptionalFieldsOmitted) {
  LogEntry entry;
  entry.id = 1;
  entry.stamp_sec = 0;
  entry.stamp_nanosec = 0;
  entry.level = 20;
  entry.name = "my_node";
  entry.msg = "hello";
  entry.function = "";  // empty -> omitted
  entry.file = "";      // empty -> omitted
  entry.line = 0;       // zero -> omitted

  auto j = LogManager::entry_to_json(entry);

  EXPECT_TRUE(j["context"].contains("node"));
  EXPECT_FALSE(j["context"].contains("function"));
  EXPECT_FALSE(j["context"].contains("file"));
  EXPECT_FALSE(j["context"].contains("line"));
}

// @verifies REQ_INTEROP_061
TEST(LogManagerEntryToJsonTest, AllSeverityLevels) {
  for (const auto & [level, expected] : std::vector<std::pair<uint8_t, std::string>>{
           {10, "debug"}, {20, "info"}, {30, "warning"}, {40, "error"}, {50, "fatal"}}) {
    LogEntry e{};
    e.id = 1;
    e.level = level;
    e.name = "n";
    e.msg = "m";
    EXPECT_EQ(LogManager::entry_to_json(e)["severity"], expected) << "level=" << int(level);
  }
}

// ============================================================
// Ring buffer tests — require a ROS 2 node for LogManager construction
// but use inject_entry_for_testing() to bypass /rosout
// ============================================================

class LogManagerBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_log_manager");
    // Small buffer size of 3 for easy eviction testing
    mgr_ = std::make_unique<LogManager>(node_.get(), nullptr, /*max_buffer_size=*/3);
  }

  void TearDown() override {
    mgr_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  LogEntry make_entry(int64_t id, const std::string & name, uint8_t level = 20) {
    LogEntry e{};
    e.id = id;
    e.stamp_sec = id;
    e.stamp_nanosec = 0;
    e.level = level;
    e.name = name;
    e.msg = "msg " + std::to_string(id);
    return e;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<LogManager> mgr_;
};

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, RingBufferEvictsOldestEntryWhenFull) {
  // Buffer size is 3; inject 4 entries for the same node
  mgr_->inject_entry_for_testing(make_entry(1, "my_node"));
  mgr_->inject_entry_for_testing(make_entry(2, "my_node"));
  mgr_->inject_entry_for_testing(make_entry(3, "my_node"));
  mgr_->inject_entry_for_testing(make_entry(4, "my_node"));  // evicts id=1

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_EQ(result.size(), 3u);
  // Oldest (id=1) must be gone; newest 3 remain
  EXPECT_EQ(result[0]["id"], "log_2");
  EXPECT_EQ(result[1]["id"], "log_3");
  EXPECT_EQ(result[2]["id"], "log_4");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, FqnWithLeadingSlashMatchesBuffer) {
  // Buffer stores entries under "my_ns/my_node" (no leading slash)
  // Entity FQN from entity cache is "/my_ns/my_node" (with leading slash)
  // get_logs() must normalize and still find the entries
  mgr_->inject_entry_for_testing(make_entry(10, "my_ns/my_node"));

  auto result = mgr_->get_logs({"/my_ns/my_node"}, false, "", "", "");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0]["id"], "log_10");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, SeverityFilterExcludesLowerLevels) {
  // inject debug(10), info(20), warning(30)
  mgr_->inject_entry_for_testing(make_entry(1, "n", 10));
  mgr_->inject_entry_for_testing(make_entry(2, "n", 20));
  mgr_->inject_entry_for_testing(make_entry(3, "n", 30));

  // min_severity=warning -> only warning (30) should appear
  auto result = mgr_->get_logs({"/n"}, false, "warning", "", "");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0]["severity"], "warning");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, PrefixMatchIncludesChildNamespaces) {
  // Component "engine" prefix-matches "engine/temp_sensor" and "engine/pressure"
  // but NOT "engine_control/sensor" (different namespace)
  mgr_->inject_entry_for_testing(make_entry(1, "engine/temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "engine/pressure"));
  mgr_->inject_entry_for_testing(make_entry(3, "engine_control/sensor"));

  auto result = mgr_->get_logs({"/engine"}, true, "", "", "");
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0]["id"], "log_1");
  EXPECT_EQ(result[1]["id"], "log_2");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, PrefixMatchDoesNotFalsePositiveOnSubstring) {
  // "engine" must NOT match "engine_control" (only full namespace segments)
  mgr_->inject_entry_for_testing(make_entry(1, "engine_control/sensor"));

  auto result = mgr_->get_logs({"/engine"}, true, "", "", "");
  EXPECT_EQ(result.size(), 0u);
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, MaxEntriesCapsMostRecentEntries) {
  // Inject 5 entries, set max_entries=2 via config
  for (int i = 1; i <= 5; ++i) {
    mgr_->inject_entry_for_testing(make_entry(i, "n"));
  }
  mgr_->update_config("my_entity", std::nullopt, 2u);

  auto result = mgr_->get_logs({"/n"}, false, "", "", "my_entity");
  ASSERT_EQ(result.size(), 2u);
  // Most recent 2: ids 4 and 5
  EXPECT_EQ(result[0]["id"], "log_4");
  EXPECT_EQ(result[1]["id"], "log_5");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, ContextFilterMatchesSubstring) {
  mgr_->inject_entry_for_testing(make_entry(1, "powertrain/engine/temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "powertrain/engine/pressure"));
  mgr_->inject_entry_for_testing(make_entry(3, "powertrain/gearbox/speed"));

  // context_filter="temp" -> only temp_sensor
  auto result = mgr_->get_logs({"/powertrain"}, true, "", "temp", "");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0]["context"]["node"], "powertrain/engine/temp_sensor");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, GetLogsMatchesDotNotationLoggerNames) {
  // ROS 2 rosout messages carry logger names using '.' as separator
  // (e.g. "powertrain.engine.temp_sensor") while entity FQNs use '/'.
  // get_logs() must match entries stored under dot-format keys.
  mgr_->inject_entry_for_testing(make_entry(1, "powertrain.engine.temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "powertrain.gearbox.speed_sensor"));

  // Exact match: app FQN "/powertrain/engine/temp_sensor" must resolve dot-format entry
  auto result = mgr_->get_logs({"/powertrain/engine/temp_sensor"}, false, "", "", "temp_sensor");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0]["context"]["node"], "powertrain.engine.temp_sensor");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, GetLogsPrefixMatchesDotNotationLoggerNames) {
  // Component namespace prefix matching must also work against dot-format logger names.
  mgr_->inject_entry_for_testing(make_entry(1, "powertrain.engine.temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "powertrain.engine.rpm_sensor"));
  mgr_->inject_entry_for_testing(make_entry(3, "chassis.brakes.pressure_sensor"));

  // Prefix match: component FQN "/powertrain/engine" should cover both powertrain nodes
  auto result = mgr_->get_logs({"/powertrain/engine"}, true, "", "", "comp");
  ASSERT_EQ(result.size(), 2u);
}

// @verifies REQ_INTEROP_064
TEST_F(LogManagerBufferTest, UpdateConfigRejectsInvalidSeverity) {
  auto err = mgr_->update_config("e", std::string("verbose"), std::nullopt);
  EXPECT_FALSE(err.empty());
}

// @verifies REQ_INTEROP_064
TEST_F(LogManagerBufferTest, UpdateConfigRejectsZeroMaxEntries) {
  auto err = mgr_->update_config("e", std::nullopt, size_t{0});
  EXPECT_FALSE(err.empty());
}

// @verifies REQ_INTEROP_063
TEST_F(LogManagerBufferTest, GetConfigReturnsDefaultsForUnknownEntity) {
  auto cfg = mgr_->get_config("unknown_entity");
  EXPECT_EQ(cfg.severity_filter, "debug");
  EXPECT_EQ(cfg.max_entries, 100u);
}

// @verifies REQ_INTEROP_064
TEST_F(LogManagerBufferTest, PartialConfigUpdatePreservesOtherField) {
  mgr_->update_config("e", std::string("warning"), std::nullopt);
  mgr_->update_config("e", std::nullopt, size_t{500});
  auto cfg = mgr_->get_config("e");
  EXPECT_EQ(cfg.severity_filter, "warning");
  EXPECT_EQ(cfg.max_entries, 500u);
}
