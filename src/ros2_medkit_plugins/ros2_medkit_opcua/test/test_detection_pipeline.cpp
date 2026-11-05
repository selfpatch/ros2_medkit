// Copyright 2026 ahmedsleem109
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

#include "ros2_medkit_opcua/detection_pipeline.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

namespace {

using std::chrono::milliseconds;
using Pass = DetectionPipeline::Pass;

constexpr const char * kTemp = "ns=1;s=Temp";
constexpr const char * kOther = "ns=1;s=Other";

// Node map with one detection point on kTemp, configured by `extra`, and a
// plain data point on kOther whose changes drive passes under a subscription.
std::string map_yaml(const std::string & extra) {
  return std::string(R"(
area_id: test
component_id: test
nodes:
  - node_id: "ns=1;s=Temp"
    entity_id: tank
    data_name: temp
    data_type: float
)") + extra +
         R"(    alarm:
      fault_code: HIGH_TEMP
      threshold: 80.0
  - node_id: "ns=1;s=Other"
    entity_id: tank
    data_name: other
    data_type: float
)";
}

// One pipeline plus the poller-side state it is driven with.
class Harness {
 public:
  Harness(const std::string & name, const std::string & extra) {
    const auto path = std::filesystem::temp_directory_path() / ("test_detection_pipeline_" + name + ".yaml");
    std::ofstream(path) << map_yaml(extra);
    loaded_ = map_.load(path.string());
    std::remove(path.string().c_str());
    entries_ = map_.detection_entries();
    // The poller sets the spacing to its poll interval; these tests poll at 500 ms.
    pipeline_.set_min_sample_spacing(milliseconds(500));
  }

  bool loaded() const {
    return loaded_ && entries_.size() == 1u;
  }

  // A good read or a data change of `node` delivers `v`.
  void arrive(const std::string & node, double v) {
    values_[node] = OpcuaValue{v};
    pipeline_.record_sample(node);
  }

  // One pass; returns the raise (+1) / clear (-1) edges of HIGH_TEMP, or 0.
  int pass(bool subscribed, Pass p, milliseconds at) {
    auto changes = pipeline_.evaluate(entries_, values_, subscribed, p, t0_ + at, alarms_);
    int edge = 0;
    for (const auto & c : changes) {
      EXPECT_EQ(c.signal.fault_code, "HIGH_TEMP");
      EXPECT_EQ(c.entity_id, "tank");
      edge += c.signal.active ? 1 : -1;
    }
    return edge;
  }

  bool shown_active() const {
    auto it = alarms_.find("HIGH_TEMP");
    return it != alarms_.end() && it->second;
  }

  DetectionPipeline & pipeline() {
    return pipeline_;
  }

 private:
  NodeMap map_;
  bool loaded_{false};
  std::vector<const NodeMapEntry *> entries_;
  DetectionPipeline pipeline_;
  std::unordered_map<std::string, OpcuaValue> values_;
  std::unordered_map<std::string, bool> alarms_;
  DetectionPipeline::Clock::time_point t0_{};
};

constexpr bool kPolled = false;
constexpr bool kSubscribed = true;

}  // namespace

// -- Points without a qualifier ------------------------------------------------

TEST(DetectionPipeline, PointWithoutQualifierRaisesOnTheFirstValue) {
  Harness h("plain", "");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(0)), 1);
  EXPECT_TRUE(h.shown_active());
  EXPECT_FALSE(h.pipeline().has_pending());
}

TEST(DetectionPipeline, PointWithoutQualifierKeepsItsStateOverAFailedRead) {
  // A failed poll leaves the last value in the snapshot. The fault stays as
  // it was, and the next real reading clears it at once.
  Harness h("plain_cached", "");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  ASSERT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(0)), 1);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(500)), 0);  // failed read
  EXPECT_TRUE(h.shown_active());
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1000)), -1);
  EXPECT_FALSE(h.shown_active());
}

// -- Polling --------------------------------------------------------------------

TEST(DetectionPipeline, PolledSamplesRaiseOnTheNthRead) {
  Harness h("poll_samples", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(0)), 0);
  EXPECT_FALSE(h.shown_active());
  EXPECT_TRUE(h.pipeline().has_pending());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(500)), 0);
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1000)), 1);
  EXPECT_TRUE(h.shown_active());
  EXPECT_FALSE(h.pipeline().has_pending());
}

TEST(DetectionPipeline, PolledFailedReadNeitherCountsNorResets) {
  Harness h("poll_failed", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kPolled, Pass::kValues, milliseconds(0));
  h.arrive(kTemp, 90.0);
  h.pass(kPolled, Pass::kValues, milliseconds(500));
  // Two failed reads: the cached 90.0 is not a sample.
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1000)), 0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1500)), 0);
  EXPECT_FALSE(h.shown_active());
  EXPECT_TRUE(h.pipeline().has_pending());
  // The third real reading completes the run that was already going.
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(2000)), 1);
}

TEST(DetectionPipeline, PolledFailedReadsDoNotCompleteADuration) {
  // A dead sensor's last reading must not raise because time passed.
  Harness h("poll_duration", "    raise_after_ms: 1000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kPolled, Pass::kValues, milliseconds(0));
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1500)), 0);  // failed read
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(3000)), 0);  // failed read
  EXPECT_FALSE(h.shown_active());
  // The next real reading still sees the condition, held since 0 ms.
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(3500)), 1);
}

TEST(DetectionPipeline, PolledHealthyReadingRestartsTheWait) {
  Harness h("poll_blip", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kPolled, Pass::kValues, milliseconds(0));
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(500)), 0);
  EXPECT_FALSE(h.pipeline().has_pending());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1000)), 0);
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1500)), 1);
}

TEST(DetectionPipeline, ClearIsImmediate) {
  Harness h("clear", "    raise_after_samples: 2\n    raise_after_ms: 1000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kPolled, Pass::kValues, milliseconds(0));
  h.arrive(kTemp, 90.0);
  ASSERT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1000)), 1);
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1100)), -1);
  EXPECT_FALSE(h.shown_active());
}

// -- Subscriptions --------------------------------------------------------------

TEST(DetectionPipeline, SubscribedOtherNodesChangesAreNotSamples) {
  // Every data change re-evaluates every point. Changes of another node must
  // not count as samples of this one.
  Harness h("sub_other", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  for (int i = 1; i <= 10; ++i) {
    h.arrive(kOther, static_cast<double>(i));
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(10 * i)), 0);
  }
  EXPECT_FALSE(h.shown_active());
  EXPECT_TRUE(h.pipeline().has_pending());
}

TEST(DetectionPipeline, SubscribedHeldValueCountsOnePerTick) {
  // A steady value sends one data change. Without the timer passes a
  // samples-only rule would never raise. Each tick a full poll after the last
  // counted sample is one more.
  Harness h("sub_tick", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(500)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(1000)), 1);
  EXPECT_TRUE(h.shown_active());
}

TEST(DetectionPipeline, SubscribedTickRightAfterAChangeIsNotASecondSample) {
  // Data changes arrive between polls, so a tick can come a few ms after a
  // change. Counting it would let a 2-sample rule raise on a 100 ms blip that
  // polling never sees twice.
  Harness h("sub_blip_samples", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  for (int round = 0; round < 5; ++round) {
    const auto start = milliseconds(1000 * round);
    h.arrive(kTemp, 90.0);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, start), 0);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, start + milliseconds(20)), 0) << "round " << round;
    h.arrive(kTemp, 70.0);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, start + milliseconds(100)), 0);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, start + milliseconds(520)), 0);
  }
  EXPECT_FALSE(h.shown_active());
}

TEST(DetectionPipeline, SubscribedSamplesStayOneTickApart) {
  // A change just before a tick, then the value holds: the second sample is
  // the tick after the next one, a full interval later, as with polling.
  Harness h("sub_spacing", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(980)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(1000)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(2000)), 1);
}

TEST(DetectionPipeline, SubscribedTwoQuickChangesAreOneSample) {
  // 90 then 95 within one poll, then healthy: polling would read that at most
  // once, so a 2-sample rule must not raise on it.
  Harness h("sub_two_changes", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  h.arrive(kTemp, 95.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(100)), 0);
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(200)), 0);
  EXPECT_FALSE(h.shown_active());
}

TEST(DetectionPipeline, SubscribedMovingValueCountsOncePerPoll) {
  // A noisy value above the threshold changes every 50 ms. It must take as
  // long to qualify as under a 500 ms poll: samples at 0, 500 and 1000 ms.
  Harness h("sub_moving", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  for (int i = 0; i < 20; ++i) {
    h.arrive(kTemp, 90.0 + i);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(50 * i)), 0) << "at " << 50 * i << " ms";
  }
  h.arrive(kTemp, 110.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(1000)), 1);
}

TEST(DetectionPipeline, SubscribedRecheckEndsADurationButCountsNoSample) {
  // Between ticks the poll thread rechecks waiting faults, so raise_after_ms
  // ends on time instead of at the next poll. A recheck is not a sample.
  Harness h("sub_recheck_ms", "    raise_after_ms: 300\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(299)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(300)), 1);

  Harness s("sub_recheck_samples", "    raise_after_samples: 2\n");
  ASSERT_TRUE(s.loaded());
  s.arrive(kTemp, 90.0);
  EXPECT_EQ(s.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  for (int i = 1; i <= 50; ++i) {
    EXPECT_EQ(s.pass(kSubscribed, Pass::kRecheck, milliseconds(20 * i)), 0);
  }
  EXPECT_FALSE(s.shown_active());
}

TEST(DetectionPipeline, SubscribedHeldValueCompletesADuration) {
  // Under a subscription the cached value is the current one, so time passes
  // for it on any pass, including one caused by another node.
  Harness h("sub_duration", "    raise_after_ms: 2000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(1999)), 0);
  h.arrive(kOther, 1.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(2000)), 1);
}

TEST(DetectionPipeline, SubscribedChangeBackToHealthyBeforeTheWindowRaisesNothing) {
  Harness h("sub_blip", "    raise_after_ms: 2000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  h.pass(kSubscribed, Pass::kTick, milliseconds(500));
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(1000)), 0);
  EXPECT_FALSE(h.pipeline().has_pending());
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(5000)), 0);
  EXPECT_FALSE(h.shown_active());
}

// -- Outages --------------------------------------------------------------------

TEST(DetectionPipeline, LinkLostRestartsTheWait) {
  Harness h("link", "    raise_after_ms: 1000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  h.pipeline().link_lost();
  EXPECT_FALSE(h.pipeline().has_pending());
  // Back 10 s later with the same value: the wait starts from here.
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(10000)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(10999)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(11000)), 1);
}

TEST(DetectionPipeline, SubscribedRecheckBetweenAChangeAndATickDoesNotArmTheTick) {
  // Only a tick marks the start of an interval. A recheck in between must not,
  // or the tick right after the change would count.
  Harness h("sub_recheck_tick", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(20)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(30)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(1030)), 1);
}

TEST(DetectionPipeline, OnlyADurationRuleAsksForRechecks) {
  // A recheck can end a raise_after_ms wait but never adds a sample, so a
  // samples-only wait must not keep the poll thread rechecking.
  Harness samples("pending_samples", "    raise_after_samples: 3\n");
  ASSERT_TRUE(samples.loaded());
  samples.arrive(kTemp, 90.0);
  samples.pass(kSubscribed, Pass::kValues, milliseconds(0));
  EXPECT_TRUE(samples.pipeline().has_pending());
  EXPECT_FALSE(samples.pipeline().has_pending_duration());

  Harness duration("pending_duration", "    raise_after_ms: 500\n");
  ASSERT_TRUE(duration.loaded());
  duration.arrive(kTemp, 90.0);
  duration.pass(kSubscribed, Pass::kValues, milliseconds(0));
  EXPECT_TRUE(duration.pipeline().has_pending_duration());
  duration.pass(kSubscribed, Pass::kRecheck, milliseconds(500));
  EXPECT_FALSE(duration.pipeline().has_pending_duration()) << "raised, nothing left to wait for";
}

TEST(DetectionPipeline, SubscribedValueFromBeforeAnOutageIsNotASample) {
  // After a reconnect the snapshot still holds the value from before the
  // outage. Until the node reports again, a tick must not count it or start a
  // wait with it, even when another node's change has already arrived.
  Harness h("link_stale", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  h.pipeline().link_lost();
  h.arrive(kOther, 1.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(5000)), 0);
  for (int i = 1; i <= 5; ++i) {
    EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(5000 + 500 * i)), 0) << "tick " << i;
  }
  EXPECT_FALSE(h.pipeline().has_pending());
  // The node reports again: now its value counts, spaced as usual.
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(8000)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(8100)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(8500)), 1);
}

TEST(DetectionPipeline, SubscribedPlainPointIgnoresItsOldValueAfterAnOutage) {
  // A point without a qualifier: after the link is lost, another node's change
  // must not re-evaluate its old value, so nothing it held is raised or
  // cleared until it reports again.
  Harness h("link_plain", "");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  h.pipeline().link_lost();
  h.arrive(kOther, 1.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(1000)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kTick, milliseconds(1500)), 0);
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(2000)), 1);
}

TEST(DetectionPipeline, LinkLostKeepsARaisedFault) {
  Harness h("link_raised", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kPolled, Pass::kValues, milliseconds(0));
  h.arrive(kTemp, 90.0);
  ASSERT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(500)), 1);
  h.pipeline().link_lost();
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(5000)), 0);  // no second raise
  EXPECT_TRUE(h.shown_active());
}

// -- Value translation ----------------------------------------------------------

TEST(DetectionPipeline, TranslatesEveryOpcuaValueType) {
  namespace fd = ros2_medkit::fault_detection;
  EXPECT_EQ(DetectionPipeline::to_fd_value(OpcuaValue{true}), fd::Value{true});
  EXPECT_EQ(DetectionPipeline::to_fd_value(OpcuaValue{int32_t{-7}}), fd::Value{std::int64_t{-7}});
  EXPECT_EQ(DetectionPipeline::to_fd_value(OpcuaValue{int64_t{1} << 40}), fd::Value{std::int64_t{1} << 40});
  EXPECT_EQ(DetectionPipeline::to_fd_value(OpcuaValue{1.5F}), fd::Value{1.5});
  EXPECT_EQ(DetectionPipeline::to_fd_value(OpcuaValue{2.25}), fd::Value{2.25});
  EXPECT_EQ(DetectionPipeline::to_fd_value(OpcuaValue{std::string{"x"}}), fd::Value{std::string{"x"}});
}

}  // namespace ros2_medkit_gateway
