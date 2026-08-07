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
#include "ros2_medkit_graph_watchdog/lifecycle_expectation_tracker.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {
using ros2_medkit_graph_watchdog::kDefaultNoMatchWarnTicks;
using ros2_medkit_graph_watchdog::LifecycleExpectationTracker;
using ros2_medkit_graph_watchdog::LifecycleMatch;
using M = std::vector<LifecycleMatch>;

// Violations are keyed by the NODE, so a match carries both the config entry that
// matched and the node's stable fqn.
LifecycleMatch match(const std::string & entry, const std::string & fqn, std::optional<std::string> state) {
  return LifecycleMatch{entry, fqn, std::move(state)};
}

TEST(LifecycleExpectation, RequiredNodeStuckInactivePastGraceRaises) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2);
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty());  // 1 <= grace
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty());  // 2 == grace
  auto report = t.update(M{match("a", "/a", "inactive")});                  // 3 > grace
  ASSERT_EQ(report.affected.size(), 1u);
  EXPECT_TRUE(report.affected.count("/a")) << "the fault must be keyed by the node, not the config entry";
  EXPECT_NE(report.affected.at("/a").find("/a"), std::string::npos);
  EXPECT_NE(report.affected.at("/a").find("required by 'a'"), std::string::npos)
      << "the entry that demanded it is still worth naming, but as context";
}

TEST(LifecycleExpectation, RequiredNodeActiveNeverRaises) {
  LifecycleExpectationTracker t({"a"}, 0);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).affected.empty());
  }
}

TEST(LifecycleExpectation, ReachesActiveWithinGraceNeverRaises) {
  LifecycleExpectationTracker t({"a"}, 2);
  t.update(M{match("a", "/a", "unconfigured")});                          // 1
  t.update(M{match("a", "/a", "inactive")});                              // 2
  EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).affected.empty());  // reset
  EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).affected.empty());
}

TEST(LifecycleExpectation, AbsentNodeIsNotOurJob) {
  LifecycleExpectationTracker t({"a"}, 0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(t.update(M{}).affected.empty());  // absent -> GRAPH_NODE_DISAPPEARED owns it
  }
}

TEST(LifecycleExpectation, UnknownLifecycleStateNeverRaises) {
  LifecycleExpectationTracker t({"a"}, 0);
  // nullopt = not a managed lifecycle node / not yet observed -> no expectation to enforce
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(t.update(M{match("a", "/a", std::nullopt)}).affected.empty());
  }
}

TEST(LifecycleExpectation, EmptyLabelIsBenignNotInactive) {
  // ReliabilityGate::lifecycle_state_of can return optional("") for a tracked node whose
  // GetState seed missed; that is an actually-active node, not inactive. Must never raise.
  LifecycleExpectationTracker t({"a"}, 0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(t.update(M{match("a", "/a", std::string(""))}).affected.empty());
  }
}

// The bare-name form is deliberately fleet-wide, so one entry covers N nodes. Keying the
// violation on the ENTRY meant the fault could not say which namesake broke, and with two
// offenders only one survived into the report at all.
TEST(LifecycleExpectation, BothNamesakesAreReportedSeparately) {
  LifecycleExpectationTracker t({"controller_server"}, /*grace=*/0);
  auto report = t.update(M{match("controller_server", "/left/controller_server", "inactive"),
                           match("controller_server", "/right/controller_server", "unconfigured")});
  ASSERT_EQ(report.affected.size(), 2u) << "one entry, two offenders, two reports";
  EXPECT_TRUE(report.affected.count("/left/controller_server"));
  EXPECT_TRUE(report.affected.count("/right/controller_server"));
}

TEST(LifecycleExpectation, HealthyNamesakeDoesNotMaskABrokenOne) {
  LifecycleExpectationTracker t({"controller_server"}, /*grace=*/0);
  auto report = t.update(M{match("controller_server", "/left/controller_server", "active"),
                           match("controller_server", "/right/controller_server", "inactive")});
  ASSERT_EQ(report.affected.size(), 1u);
  EXPECT_TRUE(report.affected.count("/right/controller_server"));
}

// A node that blinks out of one snapshot used to have its violation streak zeroed outright,
// so a genuinely stuck node on a churning graph could never accumulate grace+1 consecutive
// present-and-inactive ticks.
TEST(LifecycleExpectation, SingleSnapshotBlinkDoesNotResetTheViolationStreak) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/3);
  t.update(M{match("a", "/a", "inactive")});                // 1
  t.update(M{});                                            // blink: absent for one tick
  t.update(M{match("a", "/a", "inactive")});                // 2
  auto report = t.update(M{match("a", "/a", "inactive")});  // 3 > grace
  EXPECT_FALSE(report.affected.empty()) << "one absent snapshot must not restart the count";
}

// The bare-name form is fleet-wide and the full-FQN form pins one robot, so an operator
// combining them - both documented, both recommended - has ONE node named by TWO entries.
// The caller pushes one match per (entry, node) pair, so that node arrives twice in the
// same sweep; counting per match instead of per node advances its streak twice a tick and
// halves the grace the operator configured.
TEST(LifecycleExpectation, TwoEntriesMatchingOneNodeDoNotHalveItsGrace) {
  LifecycleExpectationTracker t({"a", "/a"}, /*grace=*/2);
  const M both{match("a", "/a", "inactive"), match("/a", "/a", "inactive")};
  EXPECT_TRUE(t.update(both).affected.empty()) << "tick 1: one node, one streak - 1 <= grace";
  EXPECT_TRUE(t.update(both).affected.empty()) << "tick 2: 2 == grace, not past it yet";
  auto report = t.update(both);  // tick 3: 3 > grace
  ASSERT_EQ(report.affected.size(), 1u) << "two entries naming one node must yield one report";
  ASSERT_TRUE(report.affected.count("/a"));
  EXPECT_NE(report.affected.at("/a").find("'a'"), std::string::npos) << "every matching entry is context";
  EXPECT_NE(report.affected.at("/a").find("'/a'"), std::string::npos) << "every matching entry is context";
}

// The same node, the same tick, one entry seeing it inactive: a violating read must never
// be tied-out by a benign one from the other entry.
TEST(LifecycleExpectation, ViolatingReadWinsOverABenignOneForTheSameNode) {
  LifecycleExpectationTracker t({"a", "/a"}, /*grace=*/0);
  auto report = t.update(M{match("a", "/a", "active"), match("/a", "/a", "inactive")});
  ASSERT_EQ(report.affected.size(), 1u) << "a measured violation must survive a disagreeing duplicate match";
  EXPECT_TRUE(report.affected.count("/a"));
}

// An empty label is the watcher's "tracked but unread" seed - the same physical event as
// the absence that produced it (a node leaving the snapshot makes the watcher erase and
// re-seed its entry, so the tick or two after a return reads ""). It must age on the
// absence budget, not reset the streak: resetting lets a stuck node on a churning graph
// blink-and-re-seed its way out of ever accumulating grace + 1 stuck ticks.
TEST(LifecycleExpectation, UnreadLabelAfterAStreakCountsAsAbsentNotAsAReset) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/3);
  t.update(M{match("a", "/a", "inactive")});  // 1
  t.update(M{});                              // blink: absent 1, streak kept
  t.update(M{match("a", "/a", "")});          // returned unread: absent 2, streak kept
  t.update(M{match("a", "/a", "inactive")});  // 2
  EXPECT_FALSE(t.update(M{match("a", "/a", "inactive")}).affected.empty())
      << "the re-seed window after a blink restarted the count - the streak must survive it";
}

// The other direction of the same rule: unread is absence-like, so SUSTAINED unread spends
// the absence budget and the streak is discarded exactly as a sustained absence would.
TEST(LifecycleExpectation, SustainedUnreadLabelStillDiscardsTheStreak) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/2);
  t.update(M{match("a", "/a", "inactive")});  // 1
  t.update(M{match("a", "/a", "inactive")});  // 2
  t.update(M{match("a", "/a", "")});          // unread 1
  t.update(M{match("a", "/a", "")});          // unread 2 == absence_grace
  t.update(M{match("a", "/a", "")});          // unread 3 > absence_grace -> streak discarded
  t.update(M{match("a", "/a", "inactive")});  // 1 again
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty())
      << "2 == grace after the discard, not past it";
}

// A node whose label has NEVER been read has no streak to preserve, so the unread tick
// stays entirely benign - no report, and no bookkeeping held on its behalf either.
TEST(LifecycleExpectation, FirstContactUnreadLabelHoldsNoBookkeeping) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0, /*absence_grace=*/3, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2);
  for (int i = 0; i < 5; ++i) {
    auto report = t.update(M{match("a", "/a", "")});
    EXPECT_TRUE(report.affected.empty()) << "iteration " << i;
    EXPECT_TRUE(report.pending.empty()) << "iteration " << i << ": nothing was ever measured not-active";
  }
  EXPECT_EQ(t.tracked_count(), 0u) << "a never-read node must not accumulate absence bookkeeping";
}

// A streak that has started but not yet passed grace is not a fault - and not health
// either. The caller emits a level-triggered clear whenever `affected` is empty, so
// without this the tracker would report "nothing to see" about a node its own last read
// found not-active, just with a young counter.
TEST(LifecycleExpectation, StreakBelowGraceIsExposedAsPending) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2);
  auto first = t.update(M{match("a", "/a", "inactive")});  // 1 <= grace
  EXPECT_TRUE(first.affected.empty());
  EXPECT_EQ(first.pending.count("/a"), 1u) << "a measured-not-active node below grace must be pending";

  auto past = t.update(M{match("a", "/a", "inactive")});
  past = t.update(M{match("a", "/a", "inactive")});  // 3 > grace: reported, not pending
  EXPECT_FALSE(past.affected.empty());
  EXPECT_TRUE(past.pending.empty()) << "a node already in `affected` must not double as pending";

  auto healthy = t.update(M{match("a", "/a", "active")});
  EXPECT_TRUE(healthy.affected.empty());
  EXPECT_TRUE(healthy.pending.empty()) << "an active read ends the streak, so nothing is pending";
}

// Pending stops at the same absence boundary the streak itself does: once the node has
// been gone longer than the absence grace its streak is discarded, and a discarded streak
// cannot hold anything back.
TEST(LifecycleExpectation, PendingStopsWhenAbsenceOutlivesTheAbsenceGrace) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/3, /*absence_grace=*/2);
  EXPECT_EQ(t.update(M{match("a", "/a", "inactive")}).pending.count("/a"), 1u);
  EXPECT_EQ(t.update(M{}).pending.count("/a"), 1u) << "absent 1: within the absence grace, still pending";
  EXPECT_EQ(t.update(M{}).pending.count("/a"), 1u) << "absent 2 == absence_grace: still pending";
  EXPECT_TRUE(t.update(M{}).pending.empty()) << "absent 3 > absence_grace: the streak is gone, so is the hold";
}

TEST(LifecycleExpectation, SustainedAbsenceStillDiscardsTheStreak) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/1);
  t.update(M{match("a", "/a", "inactive")});  // 1
  t.update(M{});                              // absent 1
  t.update(M{});                              // absent 2 -> past absence_grace, streak dropped
  t.update(M{match("a", "/a", "inactive")});  // back: 1 again
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty()) << "2 == grace, not past it yet";
}

// An entry that matches NO node is silent everywhere else: it never reaches the violation
// branch, and the presence class never tracks a node that was never present. A misspelt
// entry, or a required node that crashed during launch, produced no fault and no log line
// at all.
TEST(LifecycleExpectation, EntryMatchingNothingIsReportedOnce) {
  LifecycleExpectationTracker t({"typoed_name"}, /*grace=*/0, /*absence_grace=*/3,
                                /*no_match_warn_ticks=*/2);
  EXPECT_TRUE(t.update(M{}).entries_matching_nothing.empty());  // 1
  EXPECT_TRUE(t.update(M{}).entries_matching_nothing.empty());  // 2 == threshold
  auto report = t.update(M{});                                  // 3 > threshold
  ASSERT_EQ(report.entries_matching_nothing.size(), 1u);
  EXPECT_EQ(report.entries_matching_nothing[0], "typoed_name");
  EXPECT_TRUE(t.update(M{}).entries_matching_nothing.empty()) << "reported once, not every tick";
}

TEST(LifecycleExpectation, EntryThatMatchesIsNeverReportedAsMatchingNothing) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/0, /*absence_grace=*/3, /*no_match_warn_ticks=*/1);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(t.update(M{match("a", "/a", "active")}).entries_matching_nothing.empty());
  }
}

// An entry currently reported (stuck inactive past grace) must never be pruned - it is
// present, not absent, so it can never accumulate absence in the first place.
TEST(LifecycleExpectation, ReportedInactiveEntryIsNeverPruned) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/3, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2);
  t.update(M{match("a", "/a", "inactive")});                // miss 1 <= grace
  auto report = t.update(M{match("a", "/a", "inactive")});  // miss 2 > grace -> reported
  ASSERT_FALSE(report.affected.empty());
  for (int i = 0; i < 10; ++i) {
    report = t.update(M{match("a", "/a", "inactive")});  // present + inactive the whole time
    EXPECT_FALSE(report.affected.empty()) << "iteration " << i;
  }
  EXPECT_EQ(t.tracked_count(), 1u) << "a currently-reported node must never be pruned away";
}

// prune_ticks SHORTER than the absence grace is reachable through the documented config
// (grace: 1 with prune_grace: 2 gives prune_ticks = max(2, 2) = 2, one under the default
// absence grace of 3). The absence loop only ever reached the streak-discarding branch
// while the fqn was still in the absence map, so a horizon that reclaims the absence
// bookkeeping FIRST left the streak behind: the node returned after an eternity and
// resumed a count from a run that ended long ago, reporting it a full tick early.
TEST(LifecycleExpectation, StreakIsDiscardedEvenWhenTheAbsenceBookkeepingIsReclaimedFirst) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/3, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/2);
  t.update(M{match("a", "/a", "inactive")});  // streak 1
  for (int i = 0; i < 6; ++i) {
    t.update(M{});  // absent far past BOTH horizons
  }
  EXPECT_EQ(t.tracked_count(), 0u) << "a node gone this long must hold no bookkeeping at all";
  EXPECT_TRUE(t.update(M{match("a", "/a", "inactive")}).affected.empty())
      << "the streak survived an absence far past the absence grace, so the returning node was "
         "reported on its first present-and-inactive tick instead of after grace + 1";
}

// The boundedness promise under identity churn, with the same inverted horizons: a bare
// entry matched by an ever-new fqn (namespaced respawns) must not accumulate one
// permanent bookkeeping entry per name it has ever seen.
TEST(LifecycleExpectation, ChurningIdentitiesStayBoundedWhenPruneOutrunsTheAbsenceGrace) {
  constexpr int kPruneTicks = 2;
  LifecycleExpectationTracker t({"a"}, /*grace=*/1, /*absence_grace=*/3, kDefaultNoMatchWarnTicks, kPruneTicks);
  for (int i = 0; i < 50; ++i) {
    t.update(M{match("a", "/ns" + std::to_string(i) + "/a", "inactive")});
    // Each identity is seen once and never again, so nothing may be held past its horizon.
    EXPECT_LE(t.tracked_count(), static_cast<std::size_t>(kPruneTicks) + 1)
        << "iteration " << i << ": bookkeeping grows with every fqn ever seen";
  }
  for (int i = 0; i < 5; ++i) {
    t.update(M{});
  }
  EXPECT_EQ(t.tracked_count(), 0u) << "every churned identity must be reclaimed once it is gone";
}

// A node absent for more than prune_ticks consecutive ticks has its bookkeeping reclaimed,
// so the map stays bounded under identity churn.
TEST(LifecycleExpectation, NodeAbsentPastPruneTicksIsPrunedAndMapShrinks) {
  LifecycleExpectationTracker t({"a"}, /*grace=*/2, /*absence_grace=*/1, kDefaultNoMatchWarnTicks,
                                /*prune_ticks=*/3);
  t.update(M{match("a", "/a", "inactive")});
  EXPECT_EQ(t.tracked_count(), 1u);
  t.update(M{});  // absent 1
  t.update(M{});  // absent 2
  t.update(M{});  // absent 3 (== prune_ticks, not past it yet)
  EXPECT_EQ(t.tracked_count(), 1u);
  t.update(M{});  // absent 4 (> prune_ticks) -> reclaimed
  EXPECT_EQ(t.tracked_count(), 0u) << "a node absent past prune_ticks must be reclaimed";
}
}  // namespace
