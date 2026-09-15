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

#include <nlohmann/json.hpp>

#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/fault_storage.hpp"

using ros2_medkit_fault_manager::has_reported_evidence;
using ros2_medkit_fault_manager::kMaxEvidenceEntries;
using ros2_medkit_fault_manager::kMaxEvidenceValueChars;
using ros2_medkit_fault_manager::kReportedEvidenceKey;
using ros2_medkit_fault_manager::merge_reported_evidence;
using ros2_medkit_fault_manager::preserve_reported_evidence;

namespace {

using Pairs = std::vector<std::pair<std::string, std::string>>;

nlohmann::json reported_of(const std::string & frame_json) {
  return nlohmann::json::parse(frame_json).at(kReportedEvidenceKey);
}

}  // namespace

// ---------------------------------------------------------------------------
// merge_reported_evidence
// ---------------------------------------------------------------------------

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, WritesEntriesUnderTheReservedKey) {
  size_t dropped = 0;
  auto out = merge_reported_evidence("", Pairs{{"rejected_fixes", "37"}, {"nis", "0.03"}}, dropped);

  EXPECT_EQ(dropped, 0u);
  auto reported = reported_of(out);
  EXPECT_EQ(reported.at("rejected_fixes"), "37");
  EXPECT_EQ(reported.at("nis"), "0.03");
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, LeavesCapturedTopicValuesAlone) {
  // The frame's other keys are topics sampled by the fault manager. Evidence is a
  // separate provenance and must not silently overwrite a sampled value.
  const std::string frame = R"({"/odom":{"x":1}})";
  size_t dropped = 0;
  auto out = merge_reported_evidence(frame, Pairs{{"nis", "0.03"}}, dropped);

  auto parsed = nlohmann::json::parse(out);
  EXPECT_EQ(parsed.at("/odom").at("x"), 1);
  EXPECT_EQ(parsed.at(kReportedEvidenceKey).at("nis"), "0.03");
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, LaterReportsUpdateKeysAndKeepTheRest) {
  size_t dropped = 0;
  auto first = merge_reported_evidence("", Pairs{{"nis", "0.03"}, {"gate", "9.2"}}, dropped);
  auto second = merge_reported_evidence(first, Pairs{{"nis", "0.31"}}, dropped);

  auto reported = reported_of(second);
  EXPECT_EQ(reported.at("nis"), "0.31");  // updated
  EXPECT_EQ(reported.at("gate"), "9.2");  // and the untouched key survived
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, DropsAnEntryWithNoKey) {
  size_t dropped = 0;
  auto out = merge_reported_evidence("", Pairs{{"", "37"}, {"nis", "0.03"}}, dropped);

  EXPECT_EQ(dropped, 1u);
  auto reported = reported_of(out);
  EXPECT_EQ(reported.size(), 1u);
  EXPECT_TRUE(reported.contains("nis"));
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, DropsAnOversizedValueWholeRatherThanTruncating) {
  const std::string too_long(kMaxEvidenceValueChars + 1, 'x');
  size_t dropped = 0;
  auto out = merge_reported_evidence("", Pairs{{"blob", too_long}, {"nis", "0.03"}}, dropped);

  EXPECT_EQ(dropped, 1u);
  auto reported = reported_of(out);
  // Half a number read back later is worse than a logged absence, so nothing is stored
  // for the dropped key at all.
  EXPECT_FALSE(reported.contains("blob"));
  EXPECT_EQ(reported.at("nis"), "0.03");
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, KeepsAValueExactlyAtTheBound) {
  const std::string at_bound(kMaxEvidenceValueChars, 'x');
  size_t dropped = 0;
  auto out = merge_reported_evidence("", Pairs{{"blob", at_bound}}, dropped);

  EXPECT_EQ(dropped, 0u);
  EXPECT_EQ(reported_of(out).at("blob"), at_bound);
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, BoundsTheNumberOfEntriesPerFaultCode) {
  Pairs many;
  for (size_t i = 0; i < kMaxEvidenceEntries + 5; ++i) {
    many.emplace_back("k" + std::to_string(i), "v");
  }

  size_t dropped = 0;
  auto out = merge_reported_evidence("", many, dropped);

  EXPECT_EQ(dropped, 5u);
  EXPECT_EQ(reported_of(out).size(), kMaxEvidenceEntries);
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, AFullFrameStillAcceptsUpdatesToKeysItAlreadyHas) {
  Pairs many;
  for (size_t i = 0; i < kMaxEvidenceEntries; ++i) {
    many.emplace_back("k" + std::to_string(i), "v");
  }
  size_t dropped = 0;
  auto full = merge_reported_evidence("", many, dropped);
  ASSERT_EQ(dropped, 0u);

  // At the bound, a steady reporter must still be able to refresh its own numbers -
  // otherwise the stored evidence freezes at whatever the first report happened to say.
  auto out = merge_reported_evidence(full, Pairs{{"k0", "updated"}}, dropped);
  EXPECT_EQ(dropped, 0u);
  EXPECT_EQ(reported_of(out).at("k0"), "updated");

  // A genuinely new key is still refused.
  auto refused = merge_reported_evidence(out, Pairs{{"brand_new", "v"}}, dropped);
  EXPECT_EQ(dropped, 1u);
  EXPECT_FALSE(reported_of(refused).contains("brand_new"));
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, TreatsAnUnreadableFrameAsEmptyRatherThanFailing) {
  // A frame written by a build this one cannot read must not cost the fault its evidence.
  size_t dropped = 0;
  auto out = merge_reported_evidence("not json at all", Pairs{{"nis", "0.03"}}, dropped);
  EXPECT_EQ(reported_of(out).at("nis"), "0.03");
}

// @verifies REQ_INTEROP_110
TEST(MergeReportedEvidence, WritesNoKeyWhenEverythingWasDropped) {
  size_t dropped = 0;
  auto out = merge_reported_evidence(R"({"/odom":1})", Pairs{{"", "x"}}, dropped);

  EXPECT_EQ(dropped, 1u);
  // An empty evidence object would claim the reporter said something when it did not.
  EXPECT_FALSE(nlohmann::json::parse(out).contains(kReportedEvidenceKey));
}

// ---------------------------------------------------------------------------
// preserve_reported_evidence - the capture path must not erase what it cannot see
// ---------------------------------------------------------------------------

// @verifies REQ_INTEROP_110
TEST(PreserveReportedEvidence, CarriesEvidenceIntoAFreshlyCapturedFrame) {
  size_t dropped = 0;
  const auto previous = merge_reported_evidence("", Pairs{{"nis", "0.03"}}, dropped);
  // What a confirmation capture builds: topic values only, no evidence key.
  const std::string captured = R"({"/odom":{"x":1}})";

  auto out = preserve_reported_evidence(captured, previous);
  auto parsed = nlohmann::json::parse(out);
  EXPECT_EQ(parsed.at("/odom").at("x"), 1);
  EXPECT_EQ(parsed.at(kReportedEvidenceKey).at("nis"), "0.03");
}

// @verifies REQ_INTEROP_110
TEST(PreserveReportedEvidence, LeavesAFrameAloneWhenThereIsNothingToCarry) {
  const std::string captured = R"({"/odom":{"x":1}})";
  EXPECT_EQ(preserve_reported_evidence(captured, R"({"/odom":{"x":0}})"), captured);
  EXPECT_EQ(preserve_reported_evidence(captured, ""), captured);
}

// @verifies REQ_INTEROP_110
TEST(PreserveReportedEvidence, DoesNotOverwriteNewerEvidenceOnTheIncomingFrame) {
  size_t dropped = 0;
  const auto older = merge_reported_evidence("", Pairs{{"nis", "0.03"}}, dropped);
  const auto newer = merge_reported_evidence("", Pairs{{"nis", "0.99"}}, dropped);

  EXPECT_EQ(reported_of(preserve_reported_evidence(newer, older)).at("nis"), "0.99");
}

// ---------------------------------------------------------------------------
// has_reported_evidence - what the read path gates on
// ---------------------------------------------------------------------------

// @verifies REQ_INTEROP_110
TEST(HasReportedEvidence, TrueOnlyForAFrameCarryingEntries) {
  size_t dropped = 0;
  EXPECT_TRUE(has_reported_evidence(merge_reported_evidence("", Pairs{{"nis", "0.03"}}, dropped)));

  EXPECT_FALSE(has_reported_evidence(""));
  EXPECT_FALSE(has_reported_evidence("{}"));
  EXPECT_FALSE(has_reported_evidence(R"({"/odom":1})"));
  EXPECT_FALSE(has_reported_evidence(R"({"x-reported":{}})"));
  EXPECT_FALSE(has_reported_evidence(R"({"x-reported":"not an object"})"));
}
