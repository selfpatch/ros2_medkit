// Copyright 2026 selfpatch
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
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/threshold_resolver.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace fs = std::filesystem;
using ros2_medkit_fault_manager::debounce_policy_equal;
using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::EntityDebounceOverride;
using ros2_medkit_fault_manager::EntityThresholdResolver;
using ros2_medkit_fault_manager::FaultCodeDebounceOverride;
using ros2_medkit_fault_manager::FaultCodeThresholdResolver;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::srv::ReportFault;

namespace {

/// The layering the node applies: global, then the entity override matching the
/// source, then the fault code's own override. Mirrors
/// FaultManagerNode::resolve_config so the priority is asserted, not assumed.
DebounceConfig resolve_layered(const EntityThresholdResolver & entities, const FaultCodeThresholdResolver & codes,
                               const std::string & source_id, const std::string & fault_code,
                               const DebounceConfig & global) {
  return codes.resolve(fault_code, entities.resolve(source_id, global));
}

}  // namespace

// ---------------------------------------------------------------------------
// FaultCodeThresholdResolver unit tests
// ---------------------------------------------------------------------------

class FaultCodeResolverTest : public ::testing::Test {
 protected:
  DebounceConfig global_;

  void SetUp() override {
    global_.confirmation_threshold = -1;
    global_.healing_enabled = false;
    global_.healing_threshold = 3;
  }
};

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeResolverTest, EmptyResolverReturnsBase) {
  FaultCodeThresholdResolver resolver;
  auto result = resolver.resolve("MOTOR_OVERHEAT", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);
  EXPECT_FALSE(result.healing_enabled);
  EXPECT_EQ(result.healing_threshold, 3);
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeResolverTest, ExactMatchOverridesBase) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "MOTOR_OVERHEAT";
  entry.confirmation_threshold = -5;
  entry.healing_threshold = 10;

  FaultCodeThresholdResolver resolver({entry});
  auto result = resolver.resolve("MOTOR_OVERHEAT", global_);
  EXPECT_EQ(result.confirmation_threshold, -5);
  EXPECT_EQ(result.healing_threshold, 10);
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeResolverTest, UnknownCodeReturnsBase) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "MOTOR_OVERHEAT";
  entry.confirmation_threshold = -5;

  FaultCodeThresholdResolver resolver({entry});
  auto result = resolver.resolve("LIDAR_FAIL", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);  // Base, untouched
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeResolverTest, MatchIsExactNotPrefix) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "MOTOR";
  entry.confirmation_threshold = -5;

  FaultCodeThresholdResolver resolver({entry});
  // A fault code is an identifier, not a path: "MOTOR" must not capture
  // "MOTOR_OVERHEAT" the way an entity prefix captures a child entity.
  EXPECT_EQ(resolver.resolve("MOTOR_OVERHEAT", global_).confirmation_threshold, -1);
  EXPECT_EQ(resolver.resolve("MOTOR", global_).confirmation_threshold, -5);
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeResolverTest, PartialOverrideKeepsTheRestOfBase) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "ESTOP";
  entry.healing_enabled = true;
  // confirmation_threshold and healing_threshold not set

  FaultCodeThresholdResolver resolver({entry});
  auto result = resolver.resolve("ESTOP", global_);
  EXPECT_EQ(result.confirmation_threshold, -1);  // From base
  EXPECT_TRUE(result.healing_enabled);           // From the code
  EXPECT_EQ(result.healing_threshold, 3);        // From base
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeResolverTest, SizeReturnsEntryCount) {
  FaultCodeThresholdResolver empty;
  EXPECT_EQ(empty.size(), 0u);

  FaultCodeDebounceOverride a;
  a.fault_code = "A";
  FaultCodeDebounceOverride b;
  b.fault_code = "B";
  FaultCodeThresholdResolver two({a, b});
  EXPECT_EQ(two.size(), 2u);
}

// ---------------------------------------------------------------------------
// Layering: fault_code override > entity override > global default
// ---------------------------------------------------------------------------

class LayeredResolutionTest : public ::testing::Test {
 protected:
  DebounceConfig global_;
  EntityThresholdResolver entities_;

  void SetUp() override {
    global_.confirmation_threshold = -2;
    global_.healing_enabled = false;
    global_.healing_threshold = 3;

    EntityDebounceOverride motor;
    motor.prefix = "/powertrain/motor";
    motor.confirmation_threshold = -5;
    motor.healing_threshold = 10;

    EntityDebounceOverride lidar;
    lidar.prefix = "/sensors/lidar";
    lidar.confirmation_threshold = -1;

    entities_ = EntityThresholdResolver({motor, lidar});
  }
};

// @verifies REQ_INTEROP_107
TEST_F(LayeredResolutionTest, FaultCodeOverridesEntity) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "OVERHEAT";
  entry.confirmation_threshold = -3;

  FaultCodeThresholdResolver codes({entry});
  auto result = resolve_layered(entities_, codes, "/powertrain/motor/left", "OVERHEAT", global_);
  EXPECT_EQ(result.confirmation_threshold,
            -3);  // The code wins over the entity's -5
  EXPECT_EQ(result.healing_threshold,
            10);  // The entity still supplies what the code does not
}

// @verifies REQ_INTEROP_107
TEST_F(LayeredResolutionTest, EntityStillAppliesWhenCodeHasNoOverride) {
  FaultCodeThresholdResolver codes;
  auto result = resolve_layered(entities_, codes, "/powertrain/motor/left", "OVERHEAT", global_);
  EXPECT_EQ(result.confirmation_threshold, -5);
  EXPECT_EQ(result.healing_threshold, 10);
}

// @verifies REQ_INTEROP_107
TEST_F(LayeredResolutionTest, FaultCodeOverrideReachesAnUnconfiguredEntity) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "OVERHEAT";
  entry.confirmation_threshold = -3;

  FaultCodeThresholdResolver codes({entry});
  auto result = resolve_layered(entities_, codes, "/some/unconfigured/node", "OVERHEAT", global_);
  EXPECT_EQ(result.confirmation_threshold, -3);
  EXPECT_EQ(result.healing_threshold, 3);  // Global, no entity matched
}

// This is the cross-entity interference issue #275 describes: the debounce
// counter belongs to the fault code while the entity override is picked by the
// reporting source, so two entities reporting one code debounce it two ways. A
// fault-code override collapses that to a single policy.
// @verifies REQ_INTEROP_107
TEST_F(LayeredResolutionTest, OneCodeResolvesTheSameForEverySource) {
  FaultCodeDebounceOverride entry;
  entry.fault_code = "OVERHEAT";
  entry.confirmation_threshold = -4;
  entry.healing_enabled = true;
  entry.healing_threshold = 6;

  FaultCodeThresholdResolver codes({entry});
  auto from_motor = resolve_layered(entities_, codes, "/powertrain/motor/left", "OVERHEAT", global_);
  auto from_lidar = resolve_layered(entities_, codes, "/sensors/lidar/front", "OVERHEAT", global_);
  auto from_unknown = resolve_layered(entities_, codes, "/some/unconfigured/node", "OVERHEAT", global_);

  EXPECT_TRUE(debounce_policy_equal(from_motor, from_lidar));
  EXPECT_TRUE(debounce_policy_equal(from_motor, from_unknown));
  EXPECT_EQ(from_motor.confirmation_threshold, -4);
}

// Without a fault-code override the two sources still disagree - which is
// exactly what the node warns about (issue #276).
// @verifies REQ_INTEROP_108
TEST_F(LayeredResolutionTest, WithoutACodeOverrideTwoSourcesDisagree) {
  FaultCodeThresholdResolver codes;
  auto from_motor = resolve_layered(entities_, codes, "/powertrain/motor/left", "OVERHEAT", global_);
  auto from_lidar = resolve_layered(entities_, codes, "/sensors/lidar/front", "OVERHEAT", global_);
  EXPECT_FALSE(debounce_policy_equal(from_motor, from_lidar));
}

// ---------------------------------------------------------------------------
// debounce_policy_equal
// ---------------------------------------------------------------------------

// @verifies REQ_INTEROP_108
TEST(DebouncePolicyEqualTest, DiffersOnEachOverridableField) {
  DebounceConfig a;
  a.confirmation_threshold = -2;
  a.healing_enabled = true;
  a.healing_threshold = 4;

  EXPECT_TRUE(debounce_policy_equal(a, a));

  DebounceConfig b = a;
  b.confirmation_threshold = -3;
  EXPECT_FALSE(debounce_policy_equal(a, b));

  DebounceConfig c = a;
  c.healing_enabled = false;
  EXPECT_FALSE(debounce_policy_equal(a, c));

  DebounceConfig d = a;
  d.healing_threshold = 5;
  EXPECT_FALSE(debounce_policy_equal(a, d));
}

// @verifies REQ_INTEROP_108
TEST(DebouncePolicyEqualTest, IgnoresGlobalOnlyFields) {
  DebounceConfig a;
  a.confirmation_threshold = -2;
  a.healing_threshold = 4;

  // auto_confirm_after_sec cannot be overridden per entity or per fault code,
  // so a difference in it is never a difference between two sources' policies.
  DebounceConfig b = a;
  b.auto_confirm_after_sec = 30.0;
  EXPECT_TRUE(debounce_policy_equal(a, b));
}

// ---------------------------------------------------------------------------
// YAML loading tests
// ---------------------------------------------------------------------------

class FaultCodeYamlLoadTest : public ::testing::Test {
 protected:
  fs::path tmpdir_;

  void SetUp() override {
    tmpdir_ = fs::temp_directory_path() / "test_fault_code_thresholds";
    fs::create_directories(tmpdir_);
  }

  void TearDown() override {
    fs::remove_all(tmpdir_);
  }
};

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeYamlLoadTest, LoadValidFile) {
  auto path = tmpdir_ / "fault_thresholds.yaml";
  {
    std::ofstream f(path);
    f << "MOTOR_OVERHEAT:\n"
      << "  confirmation_threshold: -5\n"
      << "  healing_threshold: 10\n"
      << "LIDAR_FAIL:\n"
      << "  confirmation_threshold: -1\n"
      << "  healing_threshold: 1\n"
      << "ESTOP:\n"
      << "  healing_enabled: false\n";
  }

  auto entries = FaultCodeThresholdResolver::load_from_yaml(path.string());
  ASSERT_EQ(entries.size(), 3u);

  bool found_motor = false;
  for (const auto & e : entries) {
    if (e.fault_code == "MOTOR_OVERHEAT") {
      EXPECT_EQ(e.confirmation_threshold.value(), -5);
      EXPECT_EQ(e.healing_threshold.value(), 10);
      EXPECT_FALSE(e.healing_enabled.has_value());
      found_motor = true;
    }
  }
  EXPECT_TRUE(found_motor);
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeYamlLoadTest, MissingFileReturnsEmpty) {
  auto entries = FaultCodeThresholdResolver::load_from_yaml("/nonexistent/fault_thresholds.yaml");
  EXPECT_TRUE(entries.empty());
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeYamlLoadTest, MalformedYamlReturnsEmpty) {
  auto path = tmpdir_ / "bad.yaml";
  {
    std::ofstream f(path);
    f << "{{{{not valid yaml";
  }

  auto entries = FaultCodeThresholdResolver::load_from_yaml(path.string());
  EXPECT_TRUE(entries.empty());
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeYamlLoadTest, NonMapRootReturnsEmpty) {
  auto path = tmpdir_ / "sequence.yaml";
  {
    std::ofstream f(path);
    f << "- MOTOR_OVERHEAT\n- LIDAR_FAIL\n";
  }

  auto entries = FaultCodeThresholdResolver::load_from_yaml(path.string());
  EXPECT_TRUE(entries.empty());
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeYamlLoadTest, NonMapEntryIsSkippedAndTheRestLoad) {
  auto path = tmpdir_ / "mixed.yaml";
  {
    std::ofstream f(path);
    f << "MOTOR_OVERHEAT: -5\n"
      << "LIDAR_FAIL:\n"
      << "  confirmation_threshold: -1\n";
  }

  auto entries = FaultCodeThresholdResolver::load_from_yaml(path.string());
  ASSERT_EQ(entries.size(), 1u);
  EXPECT_EQ(entries[0].fault_code, "LIDAR_FAIL");
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeYamlLoadTest, PositiveConfirmationThresholdAutoNegated) {
  auto path = tmpdir_ / "autonegate.yaml";
  {
    std::ofstream f(path);
    f << "MOTOR_OVERHEAT:\n"
      << "  confirmation_threshold: 5\n"
      << "  healing_threshold: -10\n";
  }

  auto entries = FaultCodeThresholdResolver::load_from_yaml(path.string());
  ASSERT_EQ(entries.size(), 1u);
  EXPECT_EQ(entries[0].confirmation_threshold.value(), -5);
  EXPECT_EQ(entries[0].healing_threshold.value(), 10);
}

// ---------------------------------------------------------------------------
// Storage: what a fault-code override buys, against the real debounce counter
// ---------------------------------------------------------------------------

class FaultCodeStorageTest : public ::testing::Test {
 protected:
  InMemoryFaultStorage storage_;
  rclcpp::Clock clock_;
  DebounceConfig global_;
  EntityThresholdResolver entities_;
  FaultCodeThresholdResolver codes_;

  void SetUp() override {
    global_.confirmation_threshold = -1;
    global_.healing_enabled = false;
    global_.healing_threshold = 3;
    storage_.set_debounce_config(global_);

    // Motor debounces hard, lidar confirms on the first event.
    EntityDebounceOverride motor;
    motor.prefix = "/powertrain/motor";
    motor.confirmation_threshold = -5;
    EntityDebounceOverride lidar;
    lidar.prefix = "/sensors/lidar";
    lidar.confirmation_threshold = -1;
    entities_ = EntityThresholdResolver({motor, lidar});

    // The code both of them report is pinned to three events.
    FaultCodeDebounceOverride overheat;
    overheat.fault_code = "OVERHEAT";
    overheat.confirmation_threshold = -3;
    codes_ = FaultCodeThresholdResolver({overheat});
  }

  void report(const std::string & fault_code, const std::string & source_id) {
    storage_.report_fault_event(fault_code, ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "", source_id,
                                clock_.now(), resolve_layered(entities_, codes_, source_id, fault_code, global_));
  }
};

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeStorageTest, LidarNoLongerConfirmsTheMotorsCodeEarly) {
  // Without the fault-code override, lidar's threshold of -1 would confirm this
  // on the second event (the counter is shared). With it, both sources debounce
  // at -3.
  report("OVERHEAT", "/powertrain/motor/left");
  EXPECT_EQ(storage_.get_fault("OVERHEAT")->status, Fault::STATUS_PREFAILED);

  report("OVERHEAT", "/sensors/lidar/front");
  EXPECT_EQ(storage_.get_fault("OVERHEAT")->status, Fault::STATUS_PREFAILED);

  report("OVERHEAT", "/sensors/lidar/front");
  EXPECT_EQ(storage_.get_fault("OVERHEAT")->status, Fault::STATUS_CONFIRMED);
}

// @verifies REQ_INTEROP_107
TEST_F(FaultCodeStorageTest, ACodeWithoutAnOverrideKeepsEntityBehaviour) {
  report("MOTOR_STALL", "/powertrain/motor/left");
  EXPECT_EQ(storage_.get_fault("MOTOR_STALL")->status, Fault::STATUS_PREFAILED);

  report("LIDAR_BLOCKED", "/sensors/lidar/front");
  EXPECT_EQ(storage_.get_fault("LIDAR_BLOCKED")->status, Fault::STATUS_CONFIRMED);
}
