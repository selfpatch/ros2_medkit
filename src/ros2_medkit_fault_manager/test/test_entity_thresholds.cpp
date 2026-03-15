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

#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/entity_threshold_resolver.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace fs = std::filesystem;
using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::EntityDebounceOverride;
using ros2_medkit_fault_manager::EntityThresholdResolver;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::srv::ReportFault;

// ---------------------------------------------------------------------------
// EntityThresholdResolver unit tests
// ---------------------------------------------------------------------------

class ResolverTest : public ::testing::Test {
 protected:
  DebounceConfig global_;

  void SetUp() override {
    global_.confirmation_threshold = -1;
    global_.healing_enabled = false;
    global_.healing_threshold = 3;
  }
};

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, EmptyResolverReturnsGlobal) {
  EntityThresholdResolver resolver;
  auto result = resolver.resolve("/sensors/lidar", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);
  EXPECT_FALSE(result.healing_enabled);
  EXPECT_EQ(result.healing_threshold, 3);
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, ExactMatch) {
  EntityDebounceOverride entry;
  entry.prefix = "/sensors/lidar";
  entry.confirmation_threshold = -1;
  entry.healing_threshold = 1;

  EntityThresholdResolver resolver({entry});
  auto result = resolver.resolve("/sensors/lidar", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);
  EXPECT_EQ(result.healing_threshold, 1);
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, PrefixMatch) {
  EntityDebounceOverride entry;
  entry.prefix = "/sensors/lidar";
  entry.healing_threshold = 1;

  EntityThresholdResolver resolver({entry});
  auto result = resolver.resolve("/sensors/lidar/front", global_);
  EXPECT_EQ(result.healing_threshold, 1);
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, LongestPrefixWins) {
  EntityDebounceOverride broad;
  broad.prefix = "/sensors";
  broad.confirmation_threshold = -5;

  EntityDebounceOverride specific;
  specific.prefix = "/sensors/lidar";
  specific.confirmation_threshold = -1;

  EntityThresholdResolver resolver({broad, specific});
  auto result = resolver.resolve("/sensors/lidar/front", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);  // Specific wins

  auto result2 = resolver.resolve("/sensors/camera", global_);
  EXPECT_EQ(result2.confirmation_threshold, -5);  // Broad matches
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, NoMatchReturnsGlobal) {
  EntityDebounceOverride entry;
  entry.prefix = "/sensors/lidar";
  entry.confirmation_threshold = -5;

  EntityThresholdResolver resolver({entry});
  auto result = resolver.resolve("/powertrain/motor", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);  // Global default
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, PrefixMatchRequiresPathBoundary) {
  EntityDebounceOverride entry;
  entry.prefix = "/sensors/lid";
  entry.confirmation_threshold = -5;

  EntityThresholdResolver resolver({entry});
  // "/sensors/lidar" does NOT start with "/sensors/lid/" - no path boundary
  auto result = resolver.resolve("/sensors/lidar", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);  // Global default, not -5

  // But "/sensors/lid/front" does match at path boundary
  auto result2 = resolver.resolve("/sensors/lid/front", global_);
  EXPECT_EQ(result2.confirmation_threshold, -5);
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, PartialOverrideMergesWithGlobal) {
  EntityDebounceOverride entry;
  entry.prefix = "/safety";
  entry.healing_enabled = false;
  // confirmation_threshold and healing_threshold not set

  EntityThresholdResolver resolver({entry});
  auto result = resolver.resolve("/safety/estop", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);  // From global
  EXPECT_FALSE(result.healing_enabled);          // From entity
  EXPECT_EQ(result.healing_threshold, 3);        // From global
}

// @verifies REQ_INTEROP_095
TEST_F(ResolverTest, SizeReturnsEntryCount) {
  EntityThresholdResolver empty;
  EXPECT_EQ(empty.size(), 0u);

  EntityDebounceOverride a;
  a.prefix = "/a";
  EntityDebounceOverride b;
  b.prefix = "/b";
  EntityThresholdResolver two({a, b});
  EXPECT_EQ(two.size(), 2u);
}

// ---------------------------------------------------------------------------
// YAML loading tests
// ---------------------------------------------------------------------------

class YamlLoadTest : public ::testing::Test {
 protected:
  fs::path tmpdir_;

  void SetUp() override {
    tmpdir_ = fs::temp_directory_path() / "test_entity_thresholds";
    fs::create_directories(tmpdir_);
  }

  void TearDown() override {
    fs::remove_all(tmpdir_);
  }
};

// @verifies REQ_INTEROP_095
TEST_F(YamlLoadTest, LoadValidFile) {
  auto path = tmpdir_ / "thresholds.yaml";
  {
    std::ofstream f(path);
    f << "/sensors/lidar:\n"
      << "  confirmation_threshold: -1\n"
      << "  healing_threshold: 1\n"
      << "/powertrain/motor_left:\n"
      << "  confirmation_threshold: -5\n"
      << "  healing_threshold: 10\n"
      << "/safety:\n"
      << "  healing_enabled: false\n";
  }

  auto entries = EntityThresholdResolver::load_from_yaml(path.string());
  ASSERT_EQ(entries.size(), 3u);

  // Find the motor entry
  bool found_motor = false;
  for (const auto & e : entries) {
    if (e.prefix == "/powertrain/motor_left") {
      EXPECT_EQ(e.confirmation_threshold.value(), -5);
      EXPECT_EQ(e.healing_threshold.value(), 10);
      EXPECT_FALSE(e.healing_enabled.has_value());
      found_motor = true;
    }
  }
  EXPECT_TRUE(found_motor);
}

// @verifies REQ_INTEROP_095
TEST_F(YamlLoadTest, MissingFileReturnsEmpty) {
  auto entries = EntityThresholdResolver::load_from_yaml("/nonexistent/path.yaml");
  EXPECT_TRUE(entries.empty());
}

// @verifies REQ_INTEROP_095
TEST_F(YamlLoadTest, MalformedYamlReturnsEmpty) {
  auto path = tmpdir_ / "bad.yaml";
  {
    std::ofstream f(path);
    f << "{{{{not valid yaml";
  }

  auto entries = EntityThresholdResolver::load_from_yaml(path.string());
  EXPECT_TRUE(entries.empty());
}

// @verifies REQ_INTEROP_095
TEST_F(YamlLoadTest, PositiveConfirmationThresholdAutoNegated) {
  auto path = tmpdir_ / "autonegate.yaml";
  {
    std::ofstream f(path);
    f << "/sensors:\n"
      << "  confirmation_threshold: 5\n";
  }

  auto entries = EntityThresholdResolver::load_from_yaml(path.string());
  ASSERT_EQ(entries.size(), 1u);
  EXPECT_EQ(entries[0].confirmation_threshold.value(), -5);
}

// ---------------------------------------------------------------------------
// Integration: per-entity thresholds with InMemoryFaultStorage
// ---------------------------------------------------------------------------

class PerEntityStorageTest : public ::testing::Test {
 protected:
  InMemoryFaultStorage storage_;
  rclcpp::Clock clock_;
  DebounceConfig global_;
  DebounceConfig lidar_config_;
  DebounceConfig motor_config_;

  void SetUp() override {
    // Global: immediate confirmation, no healing
    global_.confirmation_threshold = -1;
    global_.healing_enabled = false;
    global_.healing_threshold = 3;

    // Lidar: immediate confirmation, healing with threshold 1
    lidar_config_ = global_;
    lidar_config_.healing_enabled = true;
    lidar_config_.healing_threshold = 1;

    // Motor: debounced (-5), healing with threshold 10
    motor_config_ = global_;
    motor_config_.confirmation_threshold = -5;
    motor_config_.healing_enabled = true;
    motor_config_.healing_threshold = 10;

    storage_.set_debounce_config(global_);
  }
};

// @verifies REQ_INTEROP_095
TEST_F(PerEntityStorageTest, LidarFaultConfirmsImmediately) {
  auto ts = clock_.now();
  storage_.report_fault_event("LIDAR_FAIL", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Lidar failure",
                              "/sensors/lidar", ts, lidar_config_);

  auto fault = storage_.get_fault("LIDAR_FAIL");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

// @verifies REQ_INTEROP_095
TEST_F(PerEntityStorageTest, MotorFaultNeedsDebouncing) {
  auto ts = clock_.now();

  // First event: counter = -1, threshold = -5, so PREFAILED
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Motor hot",
                              "/powertrain/motor_left", ts, motor_config_);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);

  // Events 2-4: still PREFAILED (counter = -2, -3, -4)
  for (int i = 0; i < 3; ++i) {
    storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "",
                                "/powertrain/motor_left", clock_.now(), motor_config_);
  }
  fault = storage_.get_fault("MOTOR_OVERHEAT");
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);

  // Event 5: counter = -5, meets threshold -> CONFIRMED
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "",
                              "/powertrain/motor_left", clock_.now(), motor_config_);
  fault = storage_.get_fault("MOTOR_OVERHEAT");
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

// @verifies REQ_INTEROP_095
TEST_F(PerEntityStorageTest, UnknownEntityUsesGlobalConfig) {
  auto ts = clock_.now();
  storage_.report_fault_event("UNKNOWN_FAULT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Unknown",
                              "/some/unknown/entity", ts, global_);

  auto fault = storage_.get_fault("UNKNOWN_FAULT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Global threshold=-1 -> immediate
}

// @verifies REQ_INTEROP_095
TEST_F(PerEntityStorageTest, DifferentEntitiesSameFaultCode) {
  auto ts = clock_.now();

  // Motor reports with motor config (debounced)
  storage_.report_fault_event("OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Motor overheat",
                              "/powertrain/motor_left", ts, motor_config_);
  auto fault = storage_.get_fault("OVERHEAT");
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);  // Motor needs 5 events

  // Lidar reports same fault_code with lidar config (which has threshold=-1)
  // Since counter is now -2 and lidar config threshold is -1, still PREFAILED
  // (counter -2 <= -1 is true, so it confirms!)
  storage_.report_fault_event("OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Lidar overheat",
                              "/sensors/lidar", clock_.now(), lidar_config_);
  fault = storage_.get_fault("OVERHEAT");
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Lidar config confirms at -1, counter is -2
}

// @verifies REQ_INTEROP_095
TEST_F(PerEntityStorageTest, LidarHealingWithThreshold1) {
  auto ts = clock_.now();

  // Confirm fault
  storage_.report_fault_event("LIDAR_FAIL", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Lidar failure",
                              "/sensors/lidar", ts, lidar_config_);
  auto fault = storage_.get_fault("LIDAR_FAIL");
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // One PASSED event should heal (healing_threshold=1 for lidar)
  // Counter goes from -1 to 0 (no status change at 0) then +1 -> healed
  storage_.report_fault_event("LIDAR_FAIL", ReportFault::Request::EVENT_PASSED, 0, "", "/sensors/lidar", clock_.now(),
                              lidar_config_);
  storage_.report_fault_event("LIDAR_FAIL", ReportFault::Request::EVENT_PASSED, 0, "", "/sensors/lidar", clock_.now(),
                              lidar_config_);
  fault = storage_.get_fault("LIDAR_FAIL");
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

// @verifies REQ_INTEROP_095
TEST_F(PerEntityStorageTest, BackwardCompatibleWithoutResolver) {
  // When no per-entity config is used, passing global config gives same behavior as before
  auto ts = clock_.now();
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              ts, global_);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Global threshold=-1
}
