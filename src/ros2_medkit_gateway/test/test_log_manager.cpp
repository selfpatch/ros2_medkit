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
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <mutex>
#include <thread>

#include "ros2_medkit_gateway/log_manager.hpp"
#include "ros2_medkit_gateway/logs/log_provider.hpp"
#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/resource_change_notifier.hpp"

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
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 3u);
  // Oldest (id=1) must be gone; newest 3 remain
  EXPECT_EQ((*result)[0]["id"], "log_2");
  EXPECT_EQ((*result)[1]["id"], "log_3");
  EXPECT_EQ((*result)[2]["id"], "log_4");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, FqnWithLeadingSlashMatchesBuffer) {
  // Buffer stores entries under "my_ns/my_node" (no leading slash)
  // Entity FQN from entity cache is "/my_ns/my_node" (with leading slash)
  // get_logs() must normalize and still find the entries
  mgr_->inject_entry_for_testing(make_entry(10, "my_ns/my_node"));

  auto result = mgr_->get_logs({"/my_ns/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["id"], "log_10");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, SeverityFilterExcludesLowerLevels) {
  // inject debug(10), info(20), warning(30)
  mgr_->inject_entry_for_testing(make_entry(1, "n", 10));
  mgr_->inject_entry_for_testing(make_entry(2, "n", 20));
  mgr_->inject_entry_for_testing(make_entry(3, "n", 30));

  // min_severity=warning -> only warning (30) should appear
  auto result = mgr_->get_logs({"/n"}, false, "warning", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["severity"], "warning");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, PrefixMatchIncludesChildNamespaces) {
  // Component "engine" prefix-matches "engine/temp_sensor" and "engine/pressure"
  // but NOT "engine_control/sensor" (different namespace)
  mgr_->inject_entry_for_testing(make_entry(1, "engine/temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "engine/pressure"));
  mgr_->inject_entry_for_testing(make_entry(3, "engine_control/sensor"));

  auto result = mgr_->get_logs({"/engine"}, true, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);
  EXPECT_EQ((*result)[0]["id"], "log_1");
  EXPECT_EQ((*result)[1]["id"], "log_2");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, PrefixMatchDoesNotFalsePositiveOnSubstring) {
  // "engine" must NOT match "engine_control" (only full namespace segments)
  mgr_->inject_entry_for_testing(make_entry(1, "engine_control/sensor"));

  auto result = mgr_->get_logs({"/engine"}, true, "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 0u);
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, MaxEntriesCapsMostRecentEntries) {
  // Inject 5 entries, set max_entries=2 via config
  for (int i = 1; i <= 5; ++i) {
    mgr_->inject_entry_for_testing(make_entry(i, "n"));
  }
  mgr_->update_config("my_entity", std::nullopt, 2u);

  auto result = mgr_->get_logs({"/n"}, false, "", "", "my_entity");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);
  // Most recent 2: ids 4 and 5
  EXPECT_EQ((*result)[0]["id"], "log_4");
  EXPECT_EQ((*result)[1]["id"], "log_5");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, ContextFilterMatchesSubstring) {
  mgr_->inject_entry_for_testing(make_entry(1, "powertrain/engine/temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "powertrain/engine/pressure"));
  mgr_->inject_entry_for_testing(make_entry(3, "powertrain/gearbox/speed"));

  // context_filter="temp" -> only temp_sensor
  auto result = mgr_->get_logs({"/powertrain"}, true, "", "temp", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["context"]["node"], "powertrain/engine/temp_sensor");
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
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["context"]["node"], "powertrain.engine.temp_sensor");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, GetLogsPrefixMatchesDotNotationLoggerNames) {
  // Component namespace prefix matching must also work against dot-format logger names.
  mgr_->inject_entry_for_testing(make_entry(1, "powertrain.engine.temp_sensor"));
  mgr_->inject_entry_for_testing(make_entry(2, "powertrain.engine.rpm_sensor"));
  mgr_->inject_entry_for_testing(make_entry(3, "chassis.brakes.pressure_sensor"));

  // Prefix match: component FQN "/powertrain/engine" should cover both powertrain nodes
  auto result = mgr_->get_logs({"/powertrain/engine"}, true, "", "", "comp");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);
}

// ============================================================
// add_log_entry() tests (I12)
// ============================================================

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, AddLogEntry_EntryRetrievable) {
  mgr_->add_log_entry("my_node", "info", "trigger fired", json::object());

  // The entry must appear in get_logs() for the matching node
  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["severity"], "info");
  EXPECT_EQ((*result)[0]["message"], "trigger fired");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, AddLogEntry_InvalidSeverityFallsBackToInfo) {
  // "verbose" is not a valid SOVD severity; implementation falls back to INFO (level 20)
  mgr_->add_log_entry("my_node", "verbose", "test message", json::object());

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  // severity_to_level("verbose") == 0 -> falls back to rcl_interfaces::msg::Log::INFO (20) -> "info"
  EXPECT_EQ((*result)[0]["severity"], "info");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, AddLogEntry_MetadataAppendedToMessage) {
  json meta = {{"trigger_id", "trig_1"}, {"entity", "sensor"}};
  mgr_->add_log_entry("my_node", "warning", "threshold exceeded", meta);

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  // Message should contain the metadata JSON suffix
  std::string msg = (*result)[0]["message"].get<std::string>();
  EXPECT_NE(msg.find("threshold exceeded"), std::string::npos);
  EXPECT_NE(msg.find("trig_1"), std::string::npos);
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, AddLogEntry_EmptyMetadataNoSuffix) {
  mgr_->add_log_entry("my_node", "debug", "clean message", json::object());

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  // With empty metadata the message must be exactly the provided string
  EXPECT_EQ((*result)[0]["message"], "clean message");
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
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->severity_filter, "debug");
  EXPECT_EQ(cfg->max_entries, 100u);
}

// @verifies REQ_INTEROP_064
TEST_F(LogManagerBufferTest, PartialConfigUpdatePreservesOtherField) {
  mgr_->update_config("e", std::string("warning"), std::nullopt);
  mgr_->update_config("e", std::nullopt, size_t{500});
  auto cfg = mgr_->get_config("e");
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->severity_filter, "warning");
  EXPECT_EQ(cfg->max_entries, 500u);
}

// ============================================================
// Mock LogProvider plugins for manages_ingestion() tests
// ============================================================

namespace {

using ros2_medkit_gateway::GatewayPlugin;
using ros2_medkit_gateway::LogProvider;
using ros2_medkit_gateway::PluginManager;

/// LogProvider that manages its own ingestion (manages_ingestion() == true).
/// Tracks calls and returns canned responses.
class MockIngestionPlugin : public GatewayPlugin, public LogProvider {
 public:
  std::string name() const override {
    return "mock_ingestion";
  }
  void configure(const json & /*config*/) override {
  }

  bool manages_ingestion() const override {
    return true;
  }

  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    get_logs_called = true;
    return canned_entries;
  }

  LogConfig get_config(const std::string & entity_id) const override {
    get_config_called = true;
    auto it = configs.find(entity_id);
    if (it != configs.end()) {
      return it->second;
    }
    return LogConfig{};
  }

  std::string update_config(const std::string & entity_id, const std::optional<std::string> & severity_filter,
                            const std::optional<size_t> & max_entries) override {
    update_config_called = true;
    auto & cfg = configs[entity_id];
    if (severity_filter.has_value()) {
      cfg.severity_filter = *severity_filter;
    }
    if (max_entries.has_value()) {
      cfg.max_entries = *max_entries;
    }
    return "";
  }

  std::vector<LogEntry> canned_entries = [] {
    LogEntry e{};
    e.id = 1;
    e.stamp_sec = 1000;
    e.stamp_nanosec = 0;
    e.level = 20;
    e.name = "plugin_node";
    e.msg = "from plugin";
    return std::vector<LogEntry>{e};
  }();

  mutable bool get_logs_called = false;
  mutable bool get_config_called = false;
  mutable bool update_config_called = false;
  std::unordered_map<std::string, LogConfig> configs;
};

/// LogProvider with default manages_ingestion() == false (observer/passive mode).
class MockPassivePlugin : public GatewayPlugin, public LogProvider {
 public:
  std::string name() const override {
    return "mock_passive";
  }
  void configure(const json & /*config*/) override {
  }

  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    get_logs_called = true;
    return canned_entries;
  }

  LogConfig get_config(const std::string & entity_id) const override {
    get_config_called = true;
    auto it = configs.find(entity_id);
    if (it != configs.end()) {
      return it->second;
    }
    return LogConfig{};
  }

  std::string update_config(const std::string & entity_id, const std::optional<std::string> & severity_filter,
                            const std::optional<size_t> & max_entries) override {
    update_config_called = true;
    auto & cfg = configs[entity_id];
    if (severity_filter.has_value()) {
      cfg.severity_filter = *severity_filter;
    }
    if (max_entries.has_value()) {
      cfg.max_entries = *max_entries;
    }
    return "";
  }

  std::vector<LogEntry> canned_entries = [] {
    LogEntry e{};
    e.id = 1;
    e.stamp_sec = 1000;
    e.stamp_nanosec = 0;
    e.level = 30;
    e.name = "passive_node";
    e.msg = "from passive";
    return std::vector<LogEntry>{e};
  }();

  mutable bool get_logs_called = false;
  mutable bool get_config_called = false;
  mutable bool update_config_called = false;
  std::unordered_map<std::string, LogConfig> configs;
};

/// Plugin that throws on get_logs() and get_config()
class MockThrowingLogPlugin : public GatewayPlugin, public LogProvider {
 public:
  std::string name() const override {
    return "mock_throwing_log";
  }
  void configure(const json & /*config*/) override {
  }

  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    throw std::runtime_error("plugin get_logs failed");
  }

  LogConfig get_config(const std::string & /*entity_id*/) const override {
    throw std::runtime_error("plugin get_config failed");
  }

  std::string update_config(const std::string & /*entity_id*/, const std::optional<std::string> & /*severity_filter*/,
                            const std::optional<size_t> & /*max_entries*/) override {
    return "";
  }
};

}  // namespace

// ============================================================
// LogManagerIngestionTest fixture
// ============================================================

class LogManagerIngestionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_log_ingestion");
  }

  void TearDown() override {
    mgr_.reset();
    plugin_mgr_.reset();
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
  std::unique_ptr<PluginManager> plugin_mgr_;
  std::unique_ptr<LogManager> mgr_;
};

// @verifies REQ_INTEROP_061
TEST_F(LogManagerIngestionTest, ManagesIngestionDelegatesToPlugin) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  auto plugin = std::make_unique<MockIngestionPlugin>();
  auto * raw = plugin.get();
  plugin_mgr_->add_plugin(std::move(plugin));

  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "entity1");
  EXPECT_TRUE(raw->get_logs_called);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["id"], "log_1");
  EXPECT_EQ((*result)[0]["message"], "from plugin");
}

// @verifies REQ_INTEROP_064
TEST_F(LogManagerIngestionTest, ManagesIngestionUpdateConfigDelegatesToPlugin) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  auto plugin = std::make_unique<MockIngestionPlugin>();
  auto * raw = plugin.get();
  plugin_mgr_->add_plugin(std::move(plugin));

  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  auto err = mgr_->update_config("entity1", std::string("error"), std::nullopt);
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(raw->update_config_called);
  EXPECT_EQ(raw->configs["entity1"].severity_filter, "error");
}

// @verifies REQ_INTEROP_063
TEST_F(LogManagerIngestionTest, ManagesIngestionGetConfigDelegatesToPlugin) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  auto plugin = std::make_unique<MockIngestionPlugin>();
  auto * raw = plugin.get();
  // Pre-populate plugin config
  raw->configs["entity1"] = LogConfig{"warning", 50};
  plugin_mgr_->add_plugin(std::move(plugin));

  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  auto cfg = mgr_->get_config("entity1");
  EXPECT_TRUE(raw->get_config_called);
  ASSERT_TRUE(cfg.has_value());
  EXPECT_EQ(cfg->severity_filter, "warning");
  EXPECT_EQ(cfg->max_entries, 50u);
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerIngestionTest, ManagesIngestionLocalBufferBypassed) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  auto plugin = std::make_unique<MockIngestionPlugin>();
  // Plugin returns empty vector for get_logs
  plugin->canned_entries.clear();
  plugin_mgr_->add_plugin(std::move(plugin));

  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  // Inject entries into local buffer - these should be invisible because plugin owns queries
  mgr_->inject_entry_for_testing(make_entry(1, "my_node"));
  mgr_->inject_entry_for_testing(make_entry(2, "my_node"));

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  // Plugin returns empty - local buffer entries are not visible
  EXPECT_EQ(result->size(), 0u);
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerIngestionTest, DefaultManagesIngestionPreservesCurrentBehavior) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  auto plugin = std::make_unique<MockPassivePlugin>();
  auto * raw = plugin.get();
  plugin_mgr_->add_plugin(std::move(plugin));

  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  auto result = mgr_->get_logs({"/node1"}, false, "", "", "");
  // Passive plugin still receives get_logs() delegation
  EXPECT_TRUE(raw->get_logs_called);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["id"], "log_1");
  EXPECT_EQ((*result)[0]["message"], "from passive");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerIngestionTest, NoPluginPreservesDefaultBehavior) {
  // No plugin manager at all - ring buffer works as before
  mgr_ = std::make_unique<LogManager>(node_.get(), nullptr, 10);

  mgr_->inject_entry_for_testing(make_entry(1, "my_node"));
  mgr_->inject_entry_for_testing(make_entry(2, "my_node"));

  auto result = mgr_->get_logs({"/my_node"}, false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);
  EXPECT_EQ((*result)[0]["id"], "log_1");
  EXPECT_EQ((*result)[1]["id"], "log_2");
}

// @verifies REQ_INTEROP_064
TEST_F(LogManagerIngestionTest, ManagesIngestionStillValidatesBeforeDelegation) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  auto plugin = std::make_unique<MockIngestionPlugin>();
  auto * raw = plugin.get();
  plugin_mgr_->add_plugin(std::move(plugin));

  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  // LogManager validates severity before delegating - "verbose" is invalid
  auto err = mgr_->update_config("e", std::string("verbose"), std::nullopt);
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(raw->update_config_called);

  // Also validate max_entries=0
  auto err2 = mgr_->update_config("e", std::nullopt, size_t{0});
  EXPECT_FALSE(err2.empty());
  EXPECT_FALSE(raw->update_config_called);
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerIngestionTest, PluginGetLogsThrowReturnsError) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  plugin_mgr_->add_plugin(std::make_unique<MockThrowingLogPlugin>());
  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  auto result = mgr_->get_logs({"/node"}, false, "", "", "");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("plugin get_logs failed"), std::string::npos);
}

// @verifies REQ_INTEROP_063
TEST_F(LogManagerIngestionTest, PluginGetConfigThrowReturnsError) {
  plugin_mgr_ = std::make_unique<PluginManager>();
  plugin_mgr_->add_plugin(std::make_unique<MockThrowingLogPlugin>());
  mgr_ = std::make_unique<LogManager>(node_.get(), plugin_mgr_.get(), 10);

  auto result = mgr_->get_config("entity1");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("plugin get_config failed"), std::string::npos);
}

// ============================================================
// Resolver notification tests — verify on_rosout() resolves
// node names to entity IDs and passes them to the notifier
// ============================================================

class LogManagerResolverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_log_resolver");
    notifier_ = std::make_unique<ros2_medkit_gateway::ResourceChangeNotifier>();
    mgr_ = std::make_unique<LogManager>(node_.get(), nullptr, 10);
    mgr_->set_notifier(notifier_.get());

    // Subscribe to notifier to capture entity IDs
    notifier_->subscribe(ros2_medkit_gateway::NotifierFilter{"logs", "", ""},
                         [this](const ros2_medkit_gateway::ResourceChange & change) {
                           std::lock_guard<std::mutex> lock(captured_mutex_);
                           captured_entity_ids_.push_back(change.entity_id);
                         });

    // Create /rosout publisher for sending test log messages
    rosout_pub_ = node_->create_publisher<rcl_interfaces::msg::Log>("/rosout", 10);
  }

  void TearDown() override {
    notifier_->shutdown();
    mgr_.reset();
    notifier_.reset();
    rosout_pub_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  /// Drain any pending /rosout messages (e.g. from LogManager constructor logs)
  void drain_initial_messages() {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(200)) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Allow notifier worker to process
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Clear whatever was captured during startup
    std::lock_guard<std::mutex> lock(captured_mutex_);
    captured_entity_ids_.clear();
  }

  void publish_and_spin(const std::string & logger_name) {
    rcl_interfaces::msg::Log msg;
    msg.level = rcl_interfaces::msg::Log::INFO;
    msg.name = logger_name;
    msg.msg = "test message";
    rosout_pub_->publish(msg);
    // Spin to process the subscription callback
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
      rclcpp::spin_some(node_);
      {
        std::lock_guard<std::mutex> lock(captured_mutex_);
        if (!captured_entity_ids_.empty()) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Allow notifier worker thread to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::vector<std::string> get_captured() {
    std::lock_guard<std::mutex> lock(captured_mutex_);
    return captured_entity_ids_;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<ros2_medkit_gateway::ResourceChangeNotifier> notifier_;
  std::unique_ptr<LogManager> mgr_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr rosout_pub_;

  std::mutex captured_mutex_;
  std::vector<std::string> captured_entity_ids_;
};

// @verifies REQ_INTEROP_061
TEST_F(LogManagerResolverTest, ResolverMatchesWithSlashPrefixReturnsEntityId) {
  // Resolver returns "temp_sensor_app" when called with "/powertrain/temp_sensor"
  mgr_->set_node_to_entity_resolver([](const std::string & fqn) -> std::string {
    if (fqn == "/powertrain/temp_sensor") {
      return "temp_sensor_app";
    }
    return "";
  });

  // Drain startup log messages that arrive on /rosout before our test message
  drain_initial_messages();

  // /rosout msg.name has no leading slash per rcl convention
  publish_and_spin("powertrain/temp_sensor");

  auto captured = get_captured();
  ASSERT_FALSE(captured.empty()) << "Notifier should have received at least one event";
  EXPECT_EQ(captured[0], "temp_sensor_app");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerResolverTest, ResolverMatchesBareNameReturnsEntityId) {
  // Resolver returns "my_app" when called with bare name (no leading slash)
  mgr_->set_node_to_entity_resolver([](const std::string & fqn) -> std::string {
    if (fqn == "simple_node") {
      return "my_app";
    }
    return "";
  });

  drain_initial_messages();

  publish_and_spin("simple_node");

  auto captured = get_captured();
  ASSERT_FALSE(captured.empty()) << "Notifier should have received at least one event";
  EXPECT_EQ(captured[0], "my_app");
}

// @verifies REQ_INTEROP_061
TEST_F(LogManagerResolverTest, ResolverReturnsEmptyFallsBackToLastSegment) {
  // Resolver always returns empty -> on_rosout falls back to last path segment
  mgr_->set_node_to_entity_resolver([](const std::string & /*fqn*/) -> std::string {
    return "";
  });

  drain_initial_messages();

  publish_and_spin("powertrain/engine/temp_sensor");

  auto captured = get_captured();
  ASSERT_FALSE(captured.empty()) << "Notifier should have received at least one event";
  // Fallback: last segment of "powertrain/engine/temp_sensor" is "temp_sensor"
  EXPECT_EQ(captured[0], "temp_sensor");
}

// ============================================================
// Buffer cap test
// ============================================================

// @verifies REQ_INTEROP_061
TEST_F(LogManagerBufferTest, BufferCapDropsNewNodesWhenFull) {
  // LogManager was created with buffer size 3 in the fixture.
  // Buffer cap = size * 10 = 30 distinct nodes.
  // Create a fresh manager with buffer size 5 so cap = 50.
  mgr_.reset();
  mgr_ = std::make_unique<LogManager>(node_.get(), nullptr, /*max_buffer_size=*/5);

  // Fill to the cap: 50 distinct nodes
  for (int i = 0; i < 50; ++i) {
    mgr_->inject_entry_for_testing(make_entry(i, "node_" + std::to_string(i)));
  }

  // Verify existing nodes work
  auto result_first = mgr_->get_logs({"/node_0"}, false, "", "", "");
  ASSERT_TRUE(result_first.has_value());
  EXPECT_EQ(result_first->size(), 1u);

  // Add one more node beyond the cap - should be silently dropped
  mgr_->inject_entry_for_testing(make_entry(999, "new_node_beyond_cap"));

  // The new node's logs should be silently dropped (buffer cap reached)
  auto result_new = mgr_->get_logs({"/new_node_beyond_cap"}, false, "", "", "");
  ASSERT_TRUE(result_new.has_value());
  EXPECT_EQ(result_new->size(), 0u) << "Logs from new nodes beyond cap should be dropped";

  // Existing buffers still work - can add more entries to existing nodes
  mgr_->inject_entry_for_testing(make_entry(1000, "node_49"));
  auto result_existing = mgr_->get_logs({"/node_49"}, false, "", "", "");
  ASSERT_TRUE(result_existing.has_value());
  EXPECT_EQ(result_existing->size(), 2u);
}
