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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ros2_medkit_policy_bridge/core/policy_monitor.hpp"

using ros2_medkit_policy_bridge::core::ChunkSpread;
using ros2_medkit_policy_bridge::core::MonitorConfig;
using ros2_medkit_policy_bridge::core::MonitorResult;
using ros2_medkit_policy_bridge::core::PolicyMonitor;
using ros2_medkit_policy_bridge::core::PolicySample;
using ros2_medkit_policy_bridge::core::RaiseGate;
using ros2_medkit_policy_bridge::core::RawConfig;
using ros2_medkit_policy_bridge::core::validate_config;

namespace {

using Signal = MonitorResult::Signal;

// A chunk of `length` steps x `dim` joints. Step t of the chunk predicted at
// `step` is the absolute step step+t; a confident policy predicts the same
// value for the same absolute step every time. `noise` is added to every value
// of this chunk, so two chunks differ by exactly the difference of their noise.
PolicySample sample(uint32_t episode, uint32_t step, float noise, uint16_t length = 10, uint16_t dim = 2) {
  PolicySample s;
  s.episode = episode;
  s.step = step;
  s.chunk_length = length;
  s.action_dim = dim;
  for (uint16_t t = 0; t < length; ++t) {
    for (uint16_t j = 0; j < dim; ++j) {
      s.chunk.push_back(0.01f * static_cast<float>(step + t) + 0.1f * static_cast<float>(j) + noise);
    }
  }
  return s;
}

MonitorConfig spread_config(double threshold = 0.05, uint32_t raise_after = 3) {
  MonitorConfig c;
  c.spread_threshold = threshold;
  c.raise_after = raise_after;
  return c;
}

// Feed chunks whose consecutive noise alternates +-amp: each new chunk differs
// from the previous one by 2*amp on every value.
std::vector<MonitorResult> feed(PolicyMonitor & m, uint32_t episode, uint32_t first_step, int count, float amp,
                                uint32_t every = 5) {
  std::vector<MonitorResult> out;
  out.reserve(static_cast<size_t>(count));
  for (int k = 0; k < count; ++k) {
    out.push_back(m.update(sample(episode, first_step + every * static_cast<uint32_t>(k), (k % 2 == 0) ? amp : -amp)));
  }
  return out;
}

int raises(const std::vector<MonitorResult> & results, Signal signal) {
  int n = 0;
  for (const auto & r : results) {
    for (const auto & t : r.transitions) {
      n += (t.signal == signal && t.raise) ? 1 : 0;
    }
  }
  return n;
}

}  // namespace

// --- ChunkSpread -------------------------------------------------------------

TEST(ChunkSpread, IdenticalPredictionsHaveZeroSpread) {
  ChunkSpread spread(4, 20, {});
  EXPECT_FALSE(spread.update(0, sample(0, 0, 0.0f)).has_value());  // nothing to compare with yet
  auto v = spread.update(5, sample(0, 5, 0.0f));
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 0.0, 1e-9);
}

TEST(ChunkSpread, ValueIsRmsDifferenceOnSharedSteps) {
  ChunkSpread spread(4, 20, {});
  spread.update(0, sample(0, 0, 0.0f));
  auto v = spread.update(5, sample(0, 5, 0.3f));
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 0.3, 1e-6);
}

TEST(ChunkSpread, MeanOverStoredChunks) {
  ChunkSpread spread(4, 20, {});
  spread.update(0, sample(0, 0, 0.0f));
  spread.update(2, sample(0, 2, 0.2f));
  // Against noise 0.0 -> 0.4, against 0.2 -> 0.2, mean 0.3.
  auto v = spread.update(4, sample(0, 4, 0.4f));
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 0.3, 1e-6);
}

TEST(ChunkSpread, JointScaleDividesEachJoint) {
  ChunkSpread spread(4, 20, {1.0, 4.0});
  spread.update(0, sample(0, 0, 0.0f));
  auto v = spread.update(5, sample(0, 5, 0.4f));
  ASSERT_TRUE(v.has_value());
  // joint 0: 0.4, joint 1: 0.1 -> sqrt((0.16 + 0.01) / 2)
  EXPECT_NEAR(*v, std::sqrt((0.16 + 0.01) / 2.0), 1e-6);
}

TEST(ChunkSpread, ChunkOlderThanItsLengthIsSkipped) {
  ChunkSpread spread(4, 20, {});
  spread.update(0, sample(0, 0, 0.0f, 10));
  // Step 10 is past the last step the first chunk predicted (0..9).
  EXPECT_FALSE(spread.update(10, sample(0, 10, 0.5f, 10)).has_value());
  // Step 9 still shares one step with the chunk at 0.
  ChunkSpread other(4, 20, {});
  other.update(0, sample(0, 0, 0.0f, 10));
  EXPECT_TRUE(other.update(9, sample(0, 9, 0.5f, 10)).has_value());
}

TEST(ChunkSpread, HistoryIsBounded) {
  ChunkSpread spread(2, 20, {});
  for (uint32_t k = 0; k < 10; ++k) {
    spread.update(k, sample(0, k, 0.0f));
  }
  EXPECT_EQ(spread.stored(), 2u);
}

TEST(ChunkSpread, HorizonLimitsComparedSteps) {
  // With horizon 1 only the first shared step counts. Make the chunks agree
  // there and disagree everywhere after it.
  ChunkSpread spread(4, 1, {});
  PolicySample a = sample(0, 0, 0.0f);
  PolicySample b = sample(0, 1, 0.0f);
  for (size_t i = 2; i < b.chunk.size(); ++i) {
    b.chunk[i] += 5.0f;
  }
  spread.update(0, a);
  auto v = spread.update(1, b);
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(*v, 0.0, 1e-6);
}

// --- RaiseGate ---------------------------------------------------------------

TEST(RaiseGate, RaisesOnceAfterNReadingsInARow) {
  RaiseGate gate(1.0, 3, 0);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kRaise);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kNone);
  EXPECT_TRUE(gate.active());
}

TEST(RaiseGate, OneLowReadingResetsTheCount) {
  RaiseGate gate(1.0, 3, 0);
  gate.update(2.0);
  gate.update(2.0);
  gate.update(0.5);
  gate.update(2.0);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kRaise);
}

TEST(RaiseGate, ValueEqualToThresholdIsNotAbove) {
  RaiseGate gate(1.0, 1, 0);
  EXPECT_EQ(gate.update(1.0), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(std::nextafter(1.0, 2.0)), RaiseGate::Edge::kRaise);
}

TEST(RaiseGate, MissingOrNanReadingNeitherCountsNorResets) {
  RaiseGate gate(1.0, 3, 0);
  gate.update(2.0);
  gate.update(2.0);
  EXPECT_EQ(gate.update(std::nullopt), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(std::numeric_limits<double>::quiet_NaN()), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(std::numeric_limits<double>::infinity()), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kRaise);
}

TEST(RaiseGate, ClearAfterZeroNeverClearsOnReadings) {
  RaiseGate gate(1.0, 1, 0);
  gate.update(2.0);
  for (int k = 0; k < 100; ++k) {
    EXPECT_EQ(gate.update(0.0), RaiseGate::Edge::kNone);
  }
  EXPECT_TRUE(gate.active());
}

TEST(RaiseGate, ClearsAfterMReadingsAndRaisesAgain) {
  RaiseGate gate(1.0, 2, 2);
  gate.update(2.0);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kRaise);
  EXPECT_EQ(gate.update(0.0), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(0.0), RaiseGate::Edge::kClear);
  EXPECT_FALSE(gate.active());
  // Qualification starts again from zero.
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kNone);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kRaise);
}

TEST(RaiseGate, ZeroRaiseAfterIsTreatedAsOne) {
  RaiseGate gate(1.0, 0, 0);
  EXPECT_EQ(gate.update(2.0), RaiseGate::Edge::kRaise);
}

// --- PolicyMonitor -----------------------------------------------------------

TEST(PolicyMonitor, DefaultConfigReportsNothing) {
  PolicyMonitor m{MonitorConfig{}};
  auto results = feed(m, 0, 0, 20, 1.0f);
  for (const auto & r : results) {
    EXPECT_TRUE(r.transitions.empty());
    EXPECT_FALSE(r.spread.has_value());
  }
}

TEST(PolicyMonitor, CalmPolicyNeverRaises) {
  PolicyMonitor m(spread_config());
  auto results = feed(m, 0, 0, 50, 0.001f);
  EXPECT_EQ(raises(results, Signal::kChunkDisagreement), 0);
}

TEST(PolicyMonitor, UnsurePolicyRaisesExactlyOnce) {
  PolicyMonitor m(spread_config(0.05, 3));
  auto results = feed(m, 0, 0, 30, 0.2f);
  EXPECT_EQ(raises(results, Signal::kChunkDisagreement), 1);
  EXPECT_TRUE(m.chunk_disagreement_active());
  // The first chunk has nothing to compare with, so the third reading comes
  // with the fourth chunk.
  ASSERT_EQ(results[3].transitions.size(), 1u);
  EXPECT_TRUE(results[3].transitions[0].raise);
  EXPECT_NEAR(results[3].transitions[0].value, *results[3].spread, 1e-12);
}

TEST(PolicyMonitor, NMinusOneReadingsDoNotRaise) {
  PolicyMonitor m(spread_config(0.05, 3));
  auto results = feed(m, 0, 0, 3, 0.2f);  // 2 spread readings
  EXPECT_EQ(raises(results, Signal::kChunkDisagreement), 0);
}

TEST(PolicyMonitor, NewEpisodePassesTheFaultAndDropsHistory) {
  PolicyMonitor m(spread_config(0.05, 1));
  feed(m, 0, 0, 3, 0.2f);
  ASSERT_TRUE(m.chunk_disagreement_active());
  auto r = m.update(sample(1, 0, 5.0f));
  ASSERT_EQ(r.transitions.size(), 1u);
  EXPECT_EQ(r.transitions[0].signal, Signal::kChunkDisagreement);
  EXPECT_FALSE(r.transitions[0].raise);
  // The chunk from episode 0 is not compared with the one from episode 1.
  EXPECT_FALSE(r.spread.has_value());
  EXPECT_FALSE(m.chunk_disagreement_active());
}

TEST(PolicyMonitor, NewEpisodeWithoutAlarmSendsNothing) {
  PolicyMonitor m(spread_config());
  feed(m, 0, 0, 5, 0.0f);
  EXPECT_TRUE(m.update(sample(1, 0, 0.0f)).transitions.empty());
}

TEST(PolicyMonitor, ClearOnNewEpisodeOffKeepsTheAlarm) {
  auto c = spread_config(0.05, 1);
  c.clear_on_new_episode = false;
  PolicyMonitor m(c);
  feed(m, 0, 0, 3, 0.2f);
  ASSERT_TRUE(m.chunk_disagreement_active());
  EXPECT_TRUE(m.update(sample(1, 0, 0.0f)).transitions.empty());
  EXPECT_TRUE(m.chunk_disagreement_active());
}

TEST(PolicyMonitor, ClearAfterReadingsPassesWithinAnEpisode) {
  auto c = spread_config(0.05, 1);
  c.clear_after = 2;
  PolicyMonitor m(c);
  feed(m, 0, 0, 2, 0.2f);  // raises at the second chunk
  ASSERT_TRUE(m.chunk_disagreement_active());
  // Calm chunks from here; the first is still compared with the noisy ones.
  std::vector<MonitorResult> calm;
  for (uint32_t k = 0; k < 8; ++k) {
    calm.push_back(m.update(sample(0, 10 + 5 * k, 0.0f)));
  }
  bool cleared = false;
  for (const auto & r : calm) {
    for (const auto & t : r.transitions) {
      cleared = cleared || (t.signal == Signal::kChunkDisagreement && !t.raise);
    }
  }
  EXPECT_TRUE(cleared);
  EXPECT_FALSE(m.chunk_disagreement_active());
}

TEST(PolicyMonitor, RepeatedStepIsNotANewReading) {
  PolicyMonitor m(spread_config(0.05, 3));
  m.update(sample(0, 0, 0.2f));
  m.update(sample(0, 5, -0.2f));
  // The same message three more times must not reach raise_after.
  for (int k = 0; k < 3; ++k) {
    auto r = m.update(sample(0, 5, -0.2f));
    EXPECT_FALSE(r.rejected.empty());
    EXPECT_TRUE(r.transitions.empty());
  }
  EXPECT_FALSE(m.chunk_disagreement_active());
}

TEST(PolicyMonitor, StepGoingBackRestartsHistoryButKeepsAlarm) {
  PolicyMonitor m(spread_config(0.05, 1));
  feed(m, 0, 100, 3, 0.2f);
  ASSERT_TRUE(m.chunk_disagreement_active());
  auto r = m.update(sample(0, 0, 0.0f));
  EXPECT_FALSE(r.spread.has_value());
  EXPECT_TRUE(r.transitions.empty());
  EXPECT_TRUE(m.chunk_disagreement_active());
}

TEST(PolicyMonitor, BadShapeIsRejectedAndChangesNothing) {
  PolicyMonitor m(spread_config(0.05, 3));
  m.update(sample(0, 0, 0.2f));
  m.update(sample(0, 5, -0.2f));
  m.update(sample(0, 10, 0.2f));
  PolicySample bad = sample(0, 15, -0.2f);
  bad.chunk.pop_back();
  auto r = m.update(bad);
  EXPECT_NE(r.rejected.find("expected chunk_length x action_dim"), std::string::npos);
  EXPECT_FALSE(m.chunk_disagreement_active());
  // The next good reading is the third in a row above threshold.
  EXPECT_EQ(raises({m.update(sample(0, 17, -0.2f))}, Signal::kChunkDisagreement), 1);
}

TEST(PolicyMonitor, ZeroSizesAreRejected) {
  PolicyMonitor m(spread_config());
  PolicySample s = sample(0, 0, 0.0f);
  s.action_dim = 0;
  EXPECT_FALSE(m.update(s).rejected.empty());
}

TEST(PolicyMonitor, NanInChunkIsRejected) {
  PolicyMonitor m(spread_config());
  PolicySample s = sample(0, 0, 0.0f);
  s.chunk[3] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(m.update(s).rejected, "chunk holds NaN or inf");
}

TEST(PolicyMonitor, ActionSizeChangeDropsHistory) {
  PolicyMonitor m(spread_config());
  m.update(sample(0, 0, 0.0f, 10, 2));
  EXPECT_FALSE(m.update(sample(0, 5, 0.0f, 10, 3)).spread.has_value());
}

TEST(PolicyMonitor, SlowInferenceRaisesAndPassesOnNewEpisode) {
  MonitorConfig c;
  c.max_inference_ms = 50.0;
  c.raise_after = 2;
  PolicyMonitor m(c);
  auto s = sample(0, 0, 0.0f);
  s.inference_ms = 80.0f;
  EXPECT_TRUE(m.update(s).transitions.empty());
  s.step = 5;
  auto r = m.update(s);
  ASSERT_EQ(r.transitions.size(), 1u);
  EXPECT_EQ(r.transitions[0].signal, Signal::kSlowInference);
  EXPECT_TRUE(r.transitions[0].raise);
  EXPECT_NEAR(r.transitions[0].value, 80.0, 1e-6);

  auto next = sample(1, 0, 0.0f);
  next.inference_ms = 10.0f;
  r = m.update(next);
  ASSERT_EQ(r.transitions.size(), 1u);
  EXPECT_EQ(r.transitions[0].signal, Signal::kSlowInference);
  EXPECT_FALSE(r.transitions[0].raise);
}

TEST(PolicyMonitor, UnknownInferenceTimeIsIgnored) {
  MonitorConfig c;
  c.max_inference_ms = 50.0;
  c.raise_after = 1;
  PolicyMonitor m(c);
  // 0 is the message default: a publisher that never sets the field.
  for (float unknown : {0.0f, -5.0f, std::nanf(""), std::numeric_limits<float>::infinity()}) {
    auto s = sample(0, 0, 0.0f);
    s.inference_ms = unknown;
    EXPECT_FALSE(ros2_medkit_policy_bridge::core::has_inference_time(s));
  }
  auto s = sample(0, 0, 0.0f);  // inference_ms 0 by default
  EXPECT_TRUE(m.update(s).transitions.empty());
  EXPECT_FALSE(m.slow_inference_active());
}

// An unset inference time between slow ones is not a calm reading: it neither
// resets the count nor passes a raised fault.
TEST(PolicyMonitor, UnsetInferenceTimeNeitherCountsNorClears) {
  MonitorConfig c;
  c.max_inference_ms = 50.0;
  c.raise_after = 3;
  c.clear_after = 1;
  PolicyMonitor m(c);
  auto at = [](uint32_t step, float ms) {
    auto s = sample(0, step, 0.0f);
    s.inference_ms = ms;
    return s;
  };
  EXPECT_TRUE(m.update(at(0, 120.0f)).transitions.empty());
  EXPECT_TRUE(m.update(at(5, 120.0f)).transitions.empty());
  EXPECT_TRUE(m.update(at(10, 0.0f)).transitions.empty());  // unset: no reset
  EXPECT_EQ(raises({m.update(at(15, 120.0f))}, Signal::kSlowInference), 1);
  EXPECT_TRUE(m.update(at(20, 0.0f)).transitions.empty());  // unset: no pass
  EXPECT_TRUE(m.slow_inference_active());
  auto r = m.update(at(25, 10.0f));  // a measured calm reading passes it
  ASSERT_EQ(r.transitions.size(), 1u);
  EXPECT_FALSE(r.transitions[0].raise);
}

// The node does not copy a chunk over the cap; the sample says how big it was.
TEST(PolicyMonitor, ChunkTheNodeDidNotCopyIsRejectedWithItsSize) {
  MonitorConfig c = spread_config(0.05, 1);
  c.max_inference_ms = 50.0;
  c.raise_after = 1;
  PolicyMonitor m(c);
  PolicySample s;
  s.chunk_length = 10;
  s.action_dim = 2;
  s.dropped_chunk_values = 100000;
  s.inference_ms = 90.0f;
  auto r = m.update(s);
  EXPECT_EQ(r.rejection, ros2_medkit_policy_bridge::core::Rejection::kTooLarge);
  EXPECT_NE(r.rejected.find("chunk has 100000 values"), std::string::npos) << r.rejected;
  EXPECT_EQ(raises({r}, Signal::kSlowInference), 1);  // its inference time still counts
}

TEST(PolicyMonitor, EachRejectionHasItsKind) {
  using ros2_medkit_policy_bridge::core::Rejection;
  PolicyMonitor m(spread_config());
  auto shape = sample(0, 0, 0.0f);
  shape.action_dim = 0;
  EXPECT_EQ(m.update(shape).rejection, Rejection::kBadShape);
  auto size = sample(0, 1, 0.0f);
  size.chunk.pop_back();
  EXPECT_EQ(m.update(size).rejection, Rejection::kSizeMismatch);
  auto nan = sample(0, 2, 0.0f);
  nan.chunk[3] = std::nanf("");
  EXPECT_EQ(m.update(nan).rejection, Rejection::kNotFinite);
  EXPECT_EQ(m.update(sample(0, 3, 0.0f)).rejection, Rejection::kNone);
  EXPECT_EQ(m.update(sample(0, 3, 0.0f)).rejection, Rejection::kRepeatedStep);
}

TEST(PolicyMonitor, ChunkOverTheSizeCapIsRejected) {
  PolicyMonitor m(spread_config(0.05, 1));
  PolicySample s;
  s.chunk_length = 1000;
  s.action_dim = 66;  // 66000 values, over kMaxChunkValues
  s.chunk.assign(66000, 0.0f);
  auto r = m.update(s);
  EXPECT_NE(r.rejected.find("more than the 65536"), std::string::npos) << r.rejected;
  // At the cap itself the sample is used.
  PolicySample ok;
  ok.step = 1;
  ok.chunk_length = 1024;
  ok.action_dim = 64;
  ok.chunk.assign(65536, 0.0f);
  EXPECT_TRUE(m.update(ok).rejected.empty());
}

// ACT without temporal ensembling infers every chunk_length steps: no two
// chunks share a step, so the spread can never be computed. Said once.
TEST(PolicyMonitor, ChunksThatNeverOverlapAreNoticedOnce) {
  PolicyMonitor m(spread_config(0.05, 1));
  std::vector<std::string> notices;
  for (uint32_t k = 0; k < 30; ++k) {
    auto r = m.update(sample(0, 10 * k, (k % 2) ? 0.5f : -0.5f));  // length 10, 10 steps apart
    EXPECT_TRUE(r.transitions.empty());
    notices.insert(notices.end(), r.notices.begin(), r.notices.end());
  }
  ASSERT_EQ(notices.size(), 1u);
  EXPECT_NE(notices[0].find("10 steps apart, chunk_length 10"), std::string::npos) << notices[0];
}

TEST(PolicyMonitor, NoOverlapNoticeComesAfterTenAndAgainAfterTenSpreads) {
  using ros2_medkit_policy_bridge::core::kNoOverlapNoticeAfter;
  PolicyMonitor m(spread_config(0.05, 1000));
  uint32_t step = 0;
  auto notices = [&m, &step](uint32_t advance) {
    step += advance;
    return m.update(sample(0, step, 0.0f)).notices.size();
  };
  for (uint32_t k = 0; k + 1 < kNoOverlapNoticeAfter; ++k) {
    EXPECT_EQ(notices(10), 0u) << "sample " << k;
  }
  EXPECT_EQ(notices(10), 1u);  // the tenth without overlap
  EXPECT_EQ(notices(10), 0u);
  for (uint32_t k = 0; k < kNoOverlapNoticeAfter; ++k) {
    EXPECT_EQ(notices(5), 0u);  // overlapping: spreads computed
  }
  for (uint32_t k = 0; k + 1 < kNoOverlapNoticeAfter; ++k) {
    EXPECT_EQ(notices(10), 0u);
  }
  EXPECT_EQ(notices(10), 1u);  // a new run of ten after ten spreads is told again
}

// A gap that jitters around chunk_length: one spread now and then does not
// re-arm the notice.
TEST(PolicyMonitor, NoOverlapNoticeIsNotRepeatedByAJitteringGap) {
  using ros2_medkit_policy_bridge::core::kNoOverlapNoticeAfter;
  PolicyMonitor m(spread_config(0.05, 1000));
  uint32_t step = 0;
  size_t told = 0;
  for (int round = 0; round < 10; ++round) {
    for (uint32_t k = 0; k < kNoOverlapNoticeAfter; ++k) {
      step += 10;
      told += m.update(sample(0, step, 0.0f)).notices.size();
    }
    step += 9;  // one inference just inside the chunk
    told += m.update(sample(0, step, 0.0f)).notices.size();
  }
  EXPECT_EQ(told, 1u);
}

// After a restart within the episode (steps went back) the gap is not a
// wrapped-around unsigned difference.
TEST(PolicyMonitor, NoOverlapNoticeAfterAStepGoingBackNamesNoWrappedGap) {
  using ros2_medkit_policy_bridge::core::kNoOverlapNoticeAfter;
  PolicyMonitor m(spread_config(0.05, 1000));
  uint32_t step = 1000;
  for (uint32_t k = 0; k + 1 < kNoOverlapNoticeAfter; ++k) {
    step += 10;
    EXPECT_TRUE(m.update(sample(0, step, 0.0f)).notices.empty());
  }
  auto r = m.update(sample(0, 3, 0.0f));  // the policy restarted at step 3
  ASSERT_EQ(r.notices.size(), 1u);
  EXPECT_NE(r.notices[0].find("latest 0 steps apart"), std::string::npos) << r.notices[0];
}

TEST(PolicyMonitor, NoOverlapIsNotCheckedWhenTheSpreadCheckIsOff) {
  MonitorConfig c;
  c.max_inference_ms = 50.0;
  PolicyMonitor m(c);
  for (uint32_t k = 0; k < 30; ++k) {
    EXPECT_TRUE(m.update(sample(0, 10 * k, 0.0f)).notices.empty());
  }
}

TEST(PolicyMonitor, JointScaleOfAnotherSizeIsNoticedOncePerActionSize) {
  for (const std::vector<double> & scale : {std::vector<double>{1.0}, std::vector<double>{1.0, 1.0, 1.0}}) {
    MonitorConfig c = spread_config();
    c.joint_scale = scale;
    PolicyMonitor m(c);
    EXPECT_EQ(m.update(sample(0, 0, 0.0f)).notices.size(), 1u);  // dim 2
    EXPECT_TRUE(m.update(sample(0, 5, 0.0f)).notices.empty());
    EXPECT_EQ(m.update(sample(0, 10, 0.0f, 10, 4)).notices.size(), 1u);  // dim 4
  }
  MonitorConfig c = spread_config();
  c.joint_scale = {1.0, 2.0};
  PolicyMonitor m(c);
  EXPECT_TRUE(m.update(sample(0, 0, 0.0f)).notices.empty());
}

TEST(PolicyMonitor, InferenceTimeCountsEvenWhenChunkIsBad) {
  MonitorConfig c;
  c.max_inference_ms = 50.0;
  c.raise_after = 1;
  PolicyMonitor m(c);
  auto s = sample(0, 0, 0.0f);
  s.chunk.clear();
  s.inference_ms = 90.0f;
  auto r = m.update(s);
  EXPECT_FALSE(r.rejected.empty());
  ASSERT_EQ(r.transitions.size(), 1u);
  EXPECT_EQ(r.transitions[0].signal, Signal::kSlowInference);
}

TEST(PolicyMonitor, BothSignalsAreIndependent) {
  MonitorConfig c = spread_config(0.05, 1);
  c.max_inference_ms = 50.0;
  PolicyMonitor m(c);
  auto a = sample(0, 0, 0.2f);
  a.inference_ms = 90.0f;
  auto r = m.update(a);  // latency raises, no spread yet
  EXPECT_EQ(raises({r}, Signal::kSlowInference), 1);
  EXPECT_EQ(raises({r}, Signal::kChunkDisagreement), 0);
  auto b = sample(0, 5, -0.2f);
  b.inference_ms = 10.0f;
  r = m.update(b);  // spread raises, latency below (no clear_after)
  EXPECT_EQ(raises({r}, Signal::kChunkDisagreement), 1);
  EXPECT_TRUE(m.slow_inference_active());
}

// --- validate_config ---------------------------------------------------------

TEST(ValidateConfig, DefaultsPassWithoutWarnings) {
  std::vector<std::string> warnings;
  MonitorConfig c = validate_config(RawConfig{}, &warnings);
  EXPECT_TRUE(warnings.empty());
  EXPECT_EQ(c.spread_threshold, 0.0);
  EXPECT_EQ(c.spread_history, 4u);
  EXPECT_EQ(c.spread_horizon, 20u);
  EXPECT_EQ(c.raise_after, 3u);
  EXPECT_EQ(c.clear_after, 0u);
  EXPECT_TRUE(c.clear_on_new_episode);
  EXPECT_TRUE(c.joint_scale.empty());
}

TEST(ValidateConfig, OutOfRangeIntegersAreClampedWithAWarning) {
  RawConfig raw;
  raw.spread_history = 0;
  raw.spread_horizon = -5;
  raw.raise_after = 4294967298;  // wraps to 2 if narrowed before the check
  raw.clear_after = -1;
  std::vector<std::string> warnings;
  MonitorConfig c = validate_config(raw, &warnings);
  EXPECT_EQ(c.spread_history, 1u);
  EXPECT_EQ(c.spread_horizon, 1u);
  EXPECT_EQ(c.raise_after, 100000u);
  EXPECT_EQ(c.clear_after, 0u);
  ASSERT_EQ(warnings.size(), 4u);
  EXPECT_NE(warnings[2].find("raise_after 4294967298"), std::string::npos);
}

TEST(ValidateConfig, NonFiniteThresholdsTurnTheCheckOff) {
  RawConfig raw;
  raw.spread_threshold = std::numeric_limits<double>::quiet_NaN();
  raw.max_inference_ms = std::numeric_limits<double>::infinity();
  std::vector<std::string> warnings;
  MonitorConfig c = validate_config(raw, &warnings);
  EXPECT_EQ(c.spread_threshold, 0.0);
  EXPECT_EQ(c.max_inference_ms, 0.0);
  EXPECT_EQ(warnings.size(), 2u);
}

TEST(ValidateConfig, NegativeThresholdsTurnTheCheckOffWithAWarning) {
  RawConfig raw;
  raw.spread_threshold = -0.5;
  raw.max_inference_ms = -1.0;
  std::vector<std::string> warnings;
  MonitorConfig c = validate_config(raw, &warnings);
  EXPECT_EQ(c.spread_threshold, 0.0);
  EXPECT_EQ(c.max_inference_ms, 0.0);
  ASSERT_EQ(warnings.size(), 2u);
  EXPECT_NE(warnings[0].find("spread_threshold -0.5 is negative"), std::string::npos) << warnings[0];
}

TEST(ValidateConfig, FaultThatCanNeverPassIsWarned) {
  RawConfig raw;
  raw.max_inference_ms = 50.0;
  raw.clear_after = 0;
  raw.clear_on_new_episode = false;
  std::vector<std::string> warnings;
  validate_config(raw, &warnings);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("never passed"), std::string::npos);
  // With both checks off nothing can be raised, so there is nothing to say.
  raw.max_inference_ms = 0.0;
  warnings.clear();
  validate_config(raw, &warnings);
  EXPECT_TRUE(warnings.empty());
}

TEST(ValidateConfig, BadJointScaleFallsBackToOnes) {
  for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    RawConfig raw;
    raw.joint_scale = {1.0, bad, 2.0};
    std::vector<std::string> warnings;
    MonitorConfig c = validate_config(raw, &warnings);
    EXPECT_TRUE(c.joint_scale.empty());
    EXPECT_EQ(warnings.size(), 1u);
  }
}

TEST(ValidateConfig, NullWarningsPointerIsAllowed) {
  RawConfig raw;
  raw.spread_history = 0;
  EXPECT_EQ(validate_config(raw, nullptr).spread_history, 1u);
}
