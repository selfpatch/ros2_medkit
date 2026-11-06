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
    // The poller sets its poll interval; these tests poll at 500 ms.
    pipeline_.set_poll_interval(milliseconds(500));
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
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(500)), 0);
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(1000)), 1);
  EXPECT_TRUE(h.shown_active());
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

TEST(DetectionPipeline, SubscribedSamplesWaitNMinusOnePollIntervals) {
  // A held value sends one data change, so there is nothing to count. Three
  // samples at a 500 ms poll span 1000 ms, and a recheck ends the wait then.
  Harness h("sub_samples", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_TRUE(h.pipeline().has_pending());
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(999)), 0);
  EXPECT_FALSE(h.shown_active());
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(1000)), 1);
  EXPECT_TRUE(h.shown_active());
}

TEST(DetectionPipeline, SubscribedSamplesWaitFollowsThePollInterval) {
  Harness h("sub_samples_interval", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.pipeline().set_poll_interval(milliseconds(2000));
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(3999)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(4000)), 1);
}

TEST(DetectionPipeline, SubscribedSamplesAndDurationWaitForTheLonger) {
  Harness longer_ms("sub_both_ms", "    raise_after_samples: 3\n    raise_after_ms: 2500\n");
  ASSERT_TRUE(longer_ms.loaded());
  longer_ms.arrive(kTemp, 90.0);
  EXPECT_EQ(longer_ms.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(longer_ms.pass(kSubscribed, Pass::kRecheck, milliseconds(2499)), 0);
  EXPECT_EQ(longer_ms.pass(kSubscribed, Pass::kRecheck, milliseconds(2500)), 1);

  Harness longer_samples("sub_both_samples", "    raise_after_samples: 3\n    raise_after_ms: 300\n");
  ASSERT_TRUE(longer_samples.loaded());
  longer_samples.arrive(kTemp, 90.0);
  EXPECT_EQ(longer_samples.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(longer_samples.pass(kSubscribed, Pass::kRecheck, milliseconds(999)), 0);
  EXPECT_EQ(longer_samples.pass(kSubscribed, Pass::kRecheck, milliseconds(1000)), 1);
}

TEST(DetectionPipeline, SubscribedBlipShorterThanTheWaitRaisesNothing) {
  // 300 ms above the threshold, rechecked on every step: polling at 500 ms
  // sees that at most once, and a 3-sample rule must not raise on it here
  // either, however often it is rechecked.
  Harness h("sub_blip_samples", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  for (int round = 0; round < 5; ++round) {
    const auto start = milliseconds(2000 * round);
    h.arrive(kTemp, 90.0);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, start), 0);
    for (int i = 1; i <= 2; ++i) {
      EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, start + milliseconds(100 * i)), 0) << "round " << round;
    }
    h.arrive(kTemp, 70.0);
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, start + milliseconds(300)), 0);
    EXPECT_FALSE(h.pipeline().has_pending());
    EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, start + milliseconds(1500)), 0);
  }
  EXPECT_FALSE(h.shown_active());
}

TEST(DetectionPipeline, SubscribedOtherNodesChangesAreNotSamples) {
  // Every data change re-evaluates every point. Changes of another node do not
  // make the wait shorter; they only let it end once its time is up.
  Harness h("sub_other", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  for (int i = 1; i <= 10; ++i) {
    h.arrive(kOther, static_cast<double>(i));
    EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(10 * i)), 0);
  }
  EXPECT_FALSE(h.shown_active());
  h.arrive(kOther, 11.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(1000)), 1);
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

TEST(DetectionPipeline, SubscribedMovingValueTakesAsLongAsUnderPolling) {
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

TEST(DetectionPipeline, SubscribedRecheckEndsADuration) {
  // Between polls the poll thread rechecks waiting faults, so raise_after_ms
  // ends on time instead of at the next poll.
  Harness h("sub_recheck_ms", "    raise_after_ms: 300\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(0)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(299)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(300)), 1);
}

TEST(DetectionPipeline, SubscribedHeldValueCompletesADuration) {
  // Under a subscription the cached value is the current one, so time passes
  // for it on any pass, including one caused by another node.
  Harness h("sub_duration", "    raise_after_ms: 2000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(1999)), 0);
  h.arrive(kOther, 1.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(2000)), 1);
}

TEST(DetectionPipeline, SubscribedChangeBackToHealthyBeforeTheWindowRaisesNothing) {
  Harness h("sub_blip", "    raise_after_ms: 2000\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  h.pass(kSubscribed, Pass::kRecheck, milliseconds(500));
  h.arrive(kTemp, 70.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(1000)), 0);
  EXPECT_FALSE(h.pipeline().has_pending());
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(5000)), 0);
  EXPECT_FALSE(h.shown_active());
}

TEST(DetectionPipeline, AWaitStartedBeforeSubscribingEndsOnARecheck) {
  // While the subscription is set up, values arrive before the poller is in
  // subscription mode, so a samples rule starts its wait as under polling. The
  // rechecks after that still have to end it while the value holds.
  Harness h("pending_switch", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(0)), 0);
  EXPECT_TRUE(h.pipeline().has_pending());
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(999)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(1000)), 1);
  EXPECT_FALSE(h.pipeline().has_pending()) << "raised, nothing left to wait for";
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
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(10999)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(11000)), 1);
}

TEST(DetectionPipeline, SubscribedValueFromBeforeAnOutageStartsNoWait) {
  // After a reconnect the snapshot still holds the value from before the
  // outage. Until the node reports again, no pass may start a wait with it,
  // even when another node's change has already arrived.
  Harness h("link_stale", "    raise_after_samples: 2\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  h.pipeline().link_lost();
  h.arrive(kOther, 1.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(5000)), 0);
  for (int i = 1; i <= 5; ++i) {
    EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(5000 + 500 * i)), 0) << "recheck " << i;
  }
  EXPECT_FALSE(h.pipeline().has_pending());
  // The node reports again: the wait starts from its report.
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(8000)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(8499)), 0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kRecheck, milliseconds(8500)), 1);
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
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kSubscribed, Pass::kValues, milliseconds(2000)), 1);
}

TEST(DetectionPipeline, SwitchingToPollingAfterAnOutageCountsSamplesAgain) {
  // A wait started under a subscription is dropped when the link is lost, so a
  // reconnect that falls back to polling counts the samples rule from zero.
  Harness h("switch", "    raise_after_samples: 3\n");
  ASSERT_TRUE(h.loaded());
  h.arrive(kTemp, 90.0);
  h.pass(kSubscribed, Pass::kValues, milliseconds(0));
  h.pipeline().link_lost();
  for (int i = 0; i < 2; ++i) {
    h.arrive(kTemp, 90.0);
    EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(5000 + 500 * i)), 0) << "read " << i;
  }
  h.arrive(kTemp, 90.0);
  EXPECT_EQ(h.pass(kPolled, Pass::kValues, milliseconds(6000)), 1);
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
