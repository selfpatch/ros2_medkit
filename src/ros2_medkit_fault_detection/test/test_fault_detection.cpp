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
#include <limits>
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

TEST(ThresholdRule, BooleanHonoursDirection) {
  // above=false expresses the common case where logic-true is healthy and the
  // FAULT is false (normally-closed contact, watchdog-OK / ready bit).
  fd::ThresholdRule r;
  r.fault = {"NOT_READY", "ERROR", "Drive not ready"};
  r.threshold = 0.0;
  r.above = false;

  EXPECT_TRUE(fd::evaluate(fd::Value{false}, r)[0].active);
  EXPECT_FALSE(fd::evaluate(fd::Value{true}, r)[0].active);
}

TEST(ThresholdRule, NonFiniteValueHoldsState) {
  // A disconnected analog sensor reports NaN/inf. The evaluator must emit
  // nothing (not active=false), so the tracker holds a previously-raised fault
  // instead of a bad read clearing a real alarm.
  fd::ThresholdRule r;
  r.fault = {"HIGH_TEMP", "ERROR", "hot"};
  r.threshold = 80.0;
  r.above = true;

  fd::FaultTransitionTracker tracker;
  ASSERT_EQ(tracker.apply(fd::evaluate(fd::Value{90.0}, r)).size(), 1u);
  ASSERT_TRUE(tracker.active("HIGH_TEMP"));

  auto nan_out = fd::evaluate(fd::Value{std::numeric_limits<double>::quiet_NaN()}, r);
  EXPECT_TRUE(nan_out.empty());
  EXPECT_TRUE(tracker.apply(nan_out).empty());
  EXPECT_TRUE(tracker.active("HIGH_TEMP"));

  auto inf_out = fd::evaluate(fd::Value{std::numeric_limits<double>::infinity()}, r);
  EXPECT_TRUE(inf_out.empty());
  EXPECT_TRUE(tracker.apply(inf_out).empty());
  EXPECT_TRUE(tracker.active("HIGH_TEMP"));
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

TEST(StatusWordRule, NonFiniteValueHoldsState) {
  // +inf rounded to int64 is INT64_MAX, which would light up every bit; NaN
  // rounds to 0 (all clear) and would clear a standing fault. Both must instead
  // emit nothing so the tracker preserves prior state.
  fd::StatusWordRule r;
  r.bits = {{0u, {"E_STOP", "CRITICAL", "Emergency stop"}}, {3u, {"PUMP_OVERLOAD", "ERROR", "Pump overload"}}};

  fd::FaultTransitionTracker tracker;
  ASSERT_EQ(tracker.apply(fd::evaluate(fd::Value{static_cast<std::int64_t>(0b1)}, r)).size(), 1u);
  ASSERT_TRUE(tracker.active("E_STOP"));

  auto inf_out = fd::evaluate(fd::Value{std::numeric_limits<double>::infinity()}, r);
  EXPECT_TRUE(inf_out.empty());  // no flood
  EXPECT_TRUE(tracker.apply(inf_out).empty());
  EXPECT_TRUE(tracker.active("E_STOP"));
  EXPECT_FALSE(tracker.active("PUMP_OVERLOAD"));

  auto nan_out = fd::evaluate(fd::Value{std::numeric_limits<double>::quiet_NaN()}, r);
  EXPECT_TRUE(nan_out.empty());
  EXPECT_TRUE(tracker.apply(nan_out).empty());
  EXPECT_TRUE(tracker.active("E_STOP"));
}

TEST(StatusWordRule, WidthMasksSignExtendedHighBits) {
  // A 16-bit signed status word with bit15 set, read as int64, sign-extends and
  // sets bits 16..63. A configured width masks them so only in-range bits fire.
  fd::StatusWordRule r;
  r.width = 16;
  r.bits = {{15u, {"REAL_FAULT", "ERROR", "real"}}, {20u, {"SPURIOUS", "ERROR", "spurious"}}};

  const auto sign_extended = static_cast<std::int64_t>(0xFFFFFFFFFFFF8000ULL);
  auto out = fd::evaluate(fd::Value{sign_extended}, r);
  EXPECT_TRUE(find(out, "REAL_FAULT")->active);
  EXPECT_FALSE(find(out, "SPURIOUS")->active);

  // Without a width the same value would spuriously fire bit 20.
  fd::StatusWordRule wide;
  wide.bits = {{20u, {"SPURIOUS", "ERROR", "spurious"}}};
  EXPECT_TRUE(find(fd::evaluate(fd::Value{sign_extended}, wide), "SPURIOUS")->active);
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

TEST(EnumMapRule, UnmappedNonOkRaisesCatchAll) {
  // A non-ok value with no configured label must stay visible via the catch-all
  // (firmware adds codes the config has not enumerated), not read as healthy.
  fd::EnumMapRule r;
  r.ok_value = 0;
  r.codes = {{10, {"OVERVOLT", "ERROR", "ov"}}, {11, {"OVERCURR", "ERROR", "oc"}}};
  r.unknown_fault = {"VFD_UNMAPPED", "ERROR", ""};

  auto out = fd::evaluate(fd::Value{static_cast<std::int64_t>(99)}, r);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_FALSE(find(out, "OVERVOLT")->active);
  EXPECT_FALSE(find(out, "OVERCURR")->active);
  const auto * unk = find(out, "VFD_UNMAPPED");
  ASSERT_NE(unk, nullptr);
  EXPECT_TRUE(unk->active);
  EXPECT_EQ(unk->message, "unmapped fault code 99");

  // ok and mapped values keep the catch-all inactive so the tracker clears it.
  EXPECT_FALSE(find(fd::evaluate(fd::Value{static_cast<std::int64_t>(0)}, r), "VFD_UNMAPPED")->active);
  auto mapped = fd::evaluate(fd::Value{static_cast<std::int64_t>(10)}, r);
  EXPECT_TRUE(find(mapped, "OVERVOLT")->active);
  EXPECT_FALSE(find(mapped, "VFD_UNMAPPED")->active);
}

TEST(EnumMapRule, NonFiniteValueHoldsState) {
  // NaN decodes to 0 == ok_value (reads healthy) and inf to INT64_MAX; either
  // would clear a standing enum fault. Both must emit nothing instead.
  fd::EnumMapRule r;
  r.ok_value = 0;
  r.codes = {{10, {"OVERVOLT", "ERROR", "ov"}}};
  r.unknown_fault.fault_code = "UNMAPPED";

  fd::FaultTransitionTracker tracker;
  ASSERT_EQ(tracker.apply(fd::evaluate(fd::Value{static_cast<std::int64_t>(10)}, r)).size(), 1u);
  ASSERT_TRUE(tracker.active("OVERVOLT"));

  auto nan_out = fd::evaluate(fd::Value{std::numeric_limits<double>::quiet_NaN()}, r);
  EXPECT_TRUE(nan_out.empty());
  EXPECT_TRUE(tracker.apply(nan_out).empty());
  EXPECT_TRUE(tracker.active("OVERVOLT"));

  EXPECT_TRUE(fd::evaluate(fd::Value{std::numeric_limits<double>::infinity()}, r).empty());
  EXPECT_TRUE(tracker.active("OVERVOLT"));
}

TEST(Evaluator, UncoercibleStringHoldsState) {
  // A string payload (wrong datatype, error string, partial poll) is undecidable
  // for every rule kind: emit nothing so a standing fault is preserved.
  fd::ThresholdRule t;
  t.fault = {"HIGH_TEMP", "ERROR", "hot"};
  t.threshold = 80.0;
  EXPECT_TRUE(fd::evaluate(fd::Value{std::string{"sensor error"}}, t).empty());

  fd::StatusWordRule s;
  s.bits = {{0u, {"E_STOP", "CRITICAL", "e"}}};
  EXPECT_TRUE(fd::evaluate(fd::Value{std::string{"bad"}}, s).empty());

  fd::EnumMapRule e;
  e.codes = {{10, {"OVERVOLT", "ERROR", "ov"}}};
  e.unknown_fault.fault_code = "UNK";
  EXPECT_TRUE(fd::evaluate(fd::Value{std::string{"bad"}}, e).empty());
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
