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

#include "ros2_medkit_fault_detection/fault_detection.hpp"

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

namespace fd = ros2_medkit::fault_detection;

namespace {

const fd::FaultSignal * find(const std::vector<fd::FaultSignal> & v, const std::string & code) {
  for (const auto & s : v) {
    if (s.fault_code == code) {
      return &s;
    }
  }
  return nullptr;
}

}  // namespace

// -- Threshold ---------------------------------------------------------------

TEST(ThresholdRule, AboveSetpointRaises) {
  fd::ThresholdRule r;
  r.fault = {"HIGH_TEMP", "ERROR", "Over temperature"};
  r.threshold = 80.0;
  r.above = true;

  auto hot = fd::evaluate(fd::Value{90.0}, r);
  ASSERT_EQ(hot.size(), 1u);
  EXPECT_TRUE(hot[0].active);
  EXPECT_EQ(hot[0].fault_code, "HIGH_TEMP");
  EXPECT_EQ(hot[0].severity, "ERROR");
  EXPECT_EQ(hot[0].message, "Over temperature");

  auto ok = fd::evaluate(fd::Value{70.0}, r);
  ASSERT_EQ(ok.size(), 1u);
  EXPECT_FALSE(ok[0].active);
}

TEST(ThresholdRule, BelowSetpointRaises) {
  fd::ThresholdRule r;
  r.fault = {"LOW_LEVEL", "WARNING", ""};
  r.threshold = 100.0;
  r.above = false;

  EXPECT_TRUE(fd::evaluate(fd::Value{static_cast<std::int64_t>(50)}, r)[0].active);
  EXPECT_FALSE(fd::evaluate(fd::Value{static_cast<std::int64_t>(150)}, r)[0].active);
  // Empty message falls back to the fault code.
  EXPECT_EQ(fd::evaluate(fd::Value{static_cast<std::int64_t>(50)}, r)[0].message, "LOW_LEVEL");
}

TEST(ThresholdRule, BooleanPointIsAlarmOnTrue) {
  fd::ThresholdRule r;
  r.fault = {"TRIGGER", "ERROR", "Trigger set"};
  r.threshold = 0.0;
  r.above = true;

  EXPECT_TRUE(fd::evaluate(fd::Value{true}, r)[0].active);
  EXPECT_FALSE(fd::evaluate(fd::Value{false}, r)[0].active);
}

// -- Status-word bit decode --------------------------------------------------

TEST(StatusWordRule, NamedBitDecode) {
  fd::StatusWordRule r;
  r.bits = {
      {0u, {"E_STOP", "CRITICAL", "Emergency stop"}},
      {3u, {"PUMP_OVERLOAD", "ERROR", "Pump overload"}},
      {7u, {"FILTER_DIRTY", "WARNING", "Filter dirty"}},
  };

  // 0b1001 -> bit 0 and bit 3 set, bit 7 clear.
  auto out = fd::evaluate(fd::Value{static_cast<std::int64_t>(0b1001)}, r);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(find(out, "E_STOP")->active);
  EXPECT_TRUE(find(out, "PUMP_OVERLOAD")->active);
  EXPECT_FALSE(find(out, "FILTER_DIRTY")->active);
  EXPECT_EQ(find(out, "PUMP_OVERLOAD")->message, "Pump overload");
}

TEST(StatusWordRule, AllClearWhenZero) {
  fd::StatusWordRule r;
  r.bits = {{2u, {"FAULT_A", "ERROR", "A"}}, {5u, {"FAULT_B", "ERROR", "B"}}};
  auto out = fd::evaluate(fd::Value{static_cast<std::int64_t>(0)}, r);
  for (const auto & s : out) {
    EXPECT_FALSE(s.active);
  }
}

// -- Fault-code enum ---------------------------------------------------------

TEST(EnumMapRule, CodeMapsToFaultAndText) {
  fd::EnumMapRule r;
  r.ok_value = 0;
  r.codes = {
      {10, {"VFD_OVERVOLTAGE", "ERROR", "DC bus overvoltage"}},
      {11, {"VFD_OVERCURRENT", "ERROR", "Output overcurrent"}},
      {12, {"VFD_OVERTEMP", "WARNING", "Heatsink over temperature"}},
  };

  auto out = fd::evaluate(fd::Value{static_cast<std::int64_t>(11)}, r);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_FALSE(find(out, "VFD_OVERVOLTAGE")->active);
  EXPECT_TRUE(find(out, "VFD_OVERCURRENT")->active);
  EXPECT_EQ(find(out, "VFD_OVERCURRENT")->message, "Output overcurrent");
  EXPECT_FALSE(find(out, "VFD_OVERTEMP")->active);
}

TEST(EnumMapRule, OkValueRaisesNothing) {
  fd::EnumMapRule r;
  r.ok_value = 0;
  r.codes = {{10, {"VFD_OVERVOLTAGE", "ERROR", "x"}}};
  auto out = fd::evaluate(fd::Value{static_cast<std::int64_t>(0)}, r);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].active);
}

// -- Raise / clear transitions ----------------------------------------------

TEST(TransitionTracker, EmitsRaiseThenClearOnce) {
  fd::ThresholdRule r;
  r.fault = {"HIGH_TEMP", "ERROR", "hot"};
  r.threshold = 80.0;
  r.above = true;

  fd::FaultTransitionTracker tracker;

  // Healthy first read: no transition (nothing was ever raised).
  EXPECT_TRUE(tracker.apply(fd::evaluate(fd::Value{70.0}, r)).empty());

  // Crosses up: one raise.
  auto up = tracker.apply(fd::evaluate(fd::Value{90.0}, r));
  ASSERT_EQ(up.size(), 1u);
  EXPECT_TRUE(up[0].active);
  EXPECT_TRUE(tracker.active("HIGH_TEMP"));

  // Still high: no repeat.
  EXPECT_TRUE(tracker.apply(fd::evaluate(fd::Value{95.0}, r)).empty());

  // Crosses down: one clear.
  auto down = tracker.apply(fd::evaluate(fd::Value{60.0}, r));
  ASSERT_EQ(down.size(), 1u);
  EXPECT_FALSE(down[0].active);
  EXPECT_FALSE(tracker.active("HIGH_TEMP"));
}

TEST(TransitionTracker, EnumCodeChangeClearsOldRaisesNew) {
  fd::EnumMapRule r;
  r.ok_value = 0;
  r.codes = {{10, {"OVERVOLT", "ERROR", "ov"}}, {11, {"OVERCURR", "ERROR", "oc"}}};

  fd::FaultTransitionTracker tracker;
  tracker.apply(fd::evaluate(fd::Value{static_cast<std::int64_t>(10)}, r));  // raise OVERVOLT

  // Code switches 10 -> 11: clear OVERVOLT, raise OVERCURR in one step.
  auto changes = tracker.apply(fd::evaluate(fd::Value{static_cast<std::int64_t>(11)}, r));
  ASSERT_EQ(changes.size(), 2u);
  EXPECT_FALSE(find(changes, "OVERVOLT")->active);
  EXPECT_TRUE(find(changes, "OVERCURR")->active);
}
