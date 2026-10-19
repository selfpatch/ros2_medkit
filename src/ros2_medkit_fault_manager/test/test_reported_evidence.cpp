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

#include <filesystem>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"

using ros2_medkit_fault_manager::build_served_freeze_frame;
using ros2_medkit_fault_manager::FaultStorage;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::kMaxEvidenceEntries;
using ros2_medkit_fault_manager::kMaxEvidenceKeyBytes;
using ros2_medkit_fault_manager::kMaxEvidenceValueBytes;
using ros2_medkit_fault_manager::oldest_kept_evidence_occurrence;
using ros2_medkit_fault_manager::plan_evidence_update;
using ros2_medkit_fault_manager::ReportedEvidence;
using ros2_medkit_fault_manager::SqliteFaultStorage;

namespace {

using Pairs = std::vector<std::pair<std::string, std::string>>;
using Keys = std::set<std::pair<std::string, std::string>>;

constexpr const char * kCode = "FUSION_DIVERGED";
constexpr const char * kSourceA = "/sensor_fusion";
constexpr const char * kSourceB = "/gnss";

ReportedEvidence entry(const std::string & source_id, const std::string & key, const std::string & value,
                       int64_t reported_at_ns) {
  return ReportedEvidence{kCode, 1, source_id, key, value, reported_at_ns};
}

}  // namespace

// ---------------------------------------------------------------------------
// plan_evidence_update - the bounds every backend applies
// ---------------------------------------------------------------------------

TEST(PlanEvidenceUpdate, StoresEveryEntryWithinTheBounds) {
  auto update = plan_evidence_update({}, kSourceA, Pairs{{"rejected_fixes", "37"}, {"nis", "0.03"}});

  EXPECT_EQ(update.dropped, 0u);
  EXPECT_TRUE(update.deletes.empty());
  EXPECT_EQ(update.upserts, (Pairs{{"rejected_fixes", "37"}, {"nis", "0.03"}}));
}

TEST(PlanEvidenceUpdate, DropsAnEntryWithNoKey) {
  auto update = plan_evidence_update({}, kSourceA, Pairs{{"", "37"}, {"nis", "0.03"}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_EQ(update.upserts, (Pairs{{"nis", "0.03"}}));
}

TEST(PlanEvidenceUpdate, KeepsAValueExactlyAtTheBound) {
  const std::string at_bound(kMaxEvidenceValueBytes, 'x');
  auto update = plan_evidence_update({}, kSourceA, Pairs{{"blob", at_bound}});

  EXPECT_EQ(update.dropped, 0u);
  EXPECT_EQ(update.upserts, (Pairs{{"blob", at_bound}}));
}

TEST(PlanEvidenceUpdate, BoundsValueLengthInBytesNotCharacters) {
  // 256 two-byte characters = 512 bytes, right at the bound. One more goes over.
  std::string at_bound;
  for (size_t i = 0; i < kMaxEvidenceValueBytes / 2; ++i) {
    at_bound += "\xc2\xb0";
  }
  EXPECT_EQ(plan_evidence_update({}, kSourceA, Pairs{{"deg", at_bound}}).dropped, 0u);
  EXPECT_EQ(plan_evidence_update({}, kSourceA, Pairs{{"deg", at_bound + "\xc2\xb0"}}).dropped, 1u);
}

TEST(PlanEvidenceUpdate, AnOversizedValueDeletesTheStoredKey) {
  // The stored value is older than this report. Keeping it would show an old number as current.
  const std::string too_long(kMaxEvidenceValueBytes + 1, 'x');
  auto update = plan_evidence_update(Keys{{kSourceA, "blob"}}, kSourceA, Pairs{{"blob", too_long}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_TRUE(update.upserts.empty());
  EXPECT_EQ(update.deletes, (std::vector<std::string>{"blob"}));
}

TEST(PlanEvidenceUpdate, AnOversizedValueForANewKeyDeletesNothing) {
  const std::string too_long(kMaxEvidenceValueBytes + 1, 'x');
  auto update = plan_evidence_update(Keys{{kSourceB, "blob"}}, kSourceA, Pairs{{"blob", too_long}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_TRUE(update.deletes.empty());
}

TEST(PlanEvidenceUpdate, DropsAnOversizedKey) {
  const std::string long_key(kMaxEvidenceKeyBytes + 1, 'k');
  const std::string key_at_bound(kMaxEvidenceKeyBytes, 'k');
  auto update = plan_evidence_update({}, kSourceA, Pairs{{long_key, "1"}, {key_at_bound, "2"}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_EQ(update.upserts, (Pairs{{key_at_bound, "2"}}));
}

TEST(PlanEvidenceUpdate, InvalidUtf8IsReplacedBeforeTheBoundsAreChecked) {
  // A Latin-1 degree sign is not valid UTF-8.
  const std::string replacement = "\xef\xbf\xbd";  // U+FFFD
  auto update = plan_evidence_update({}, kSourceA,
                                     Pairs{{"temp",
                                            "80\xb0"
                                            "C"},
                                           {"\xb0key", "1"}});

  EXPECT_EQ(update.dropped, 0u);
  EXPECT_EQ(update.upserts, (Pairs{{"temp", "80" + replacement + "C"}, {replacement + "key", "1"}}));
}

TEST(PlanEvidenceUpdate, AnInvalidUtf8KeyMatchesItsStoredForm) {
  // The bridge sends the same key every cycle. It must update one entry, not add a new one.
  Keys stored{{kSourceA, "\xef\xbf\xbdkey"}};
  for (size_t i = 1; i < kMaxEvidenceEntries; ++i) {
    stored.emplace(kSourceA, "k" + std::to_string(i));
  }
  auto update = plan_evidence_update(stored, kSourceA, Pairs{{"\xb0key", "2"}});

  EXPECT_EQ(update.dropped, 0u);
  EXPECT_EQ(update.upserts, (Pairs{{"\xef\xbf\xbdkey", "2"}}));
}

TEST(PlanEvidenceUpdate, BoundsTheKeysOfAnOccurrenceOverAllSources) {
  // The bound is per occurrence, not per source, so a code with many sources stays bounded too.
  Keys stored;
  for (size_t i = 0; i < kMaxEvidenceEntries - 1; ++i) {
    stored.emplace(kSourceB, "k" + std::to_string(i));
  }
  auto update = plan_evidence_update(stored, kSourceA, Pairs{{"first", "1"}, {"second", "2"}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_EQ(update.upserts, (Pairs{{"first", "1"}}));
}

TEST(PlanEvidenceUpdate, AFullOccurrenceStillAcceptsUpdatesToItsKeys) {
  Keys stored;
  for (size_t i = 0; i < kMaxEvidenceEntries; ++i) {
    stored.emplace(kSourceA, "k" + std::to_string(i));
  }
  auto update = plan_evidence_update(stored, kSourceA, Pairs{{"k0", "updated"}, {"brand_new", "v"}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_EQ(update.upserts, (Pairs{{"k0", "updated"}}));
}

TEST(PlanEvidenceUpdate, TheSameKeyFromAnotherSourceIsANewEntry) {
  Keys stored;
  for (size_t i = 0; i < kMaxEvidenceEntries; ++i) {
    stored.emplace(kSourceB, "k" + std::to_string(i));
  }
  auto update = plan_evidence_update(stored, kSourceA, Pairs{{"k0", "1"}});

  EXPECT_EQ(update.dropped, 1u);
  EXPECT_TRUE(update.upserts.empty());
}

// ---------------------------------------------------------------------------
// oldest_kept_evidence_occurrence
// ---------------------------------------------------------------------------

TEST(OldestKeptEvidenceOccurrence, WithoutRetentionOnlyTheCurrentOccurrenceIsKept) {
  EXPECT_EQ(oldest_kept_evidence_occurrence(5, false, 0), 5u);
  EXPECT_EQ(oldest_kept_evidence_occurrence(5, false, 3), 5u);
}

TEST(OldestKeptEvidenceOccurrence, WithRetentionTheLastNOccurrencesAreKept) {
  EXPECT_EQ(oldest_kept_evidence_occurrence(5, true, 3), 3u);
  EXPECT_EQ(oldest_kept_evidence_occurrence(5, true, 1), 5u);
  EXPECT_EQ(oldest_kept_evidence_occurrence(2, true, 3), 0u);
  EXPECT_EQ(oldest_kept_evidence_occurrence(5, true, 0), 0u);  // unlimited
}

// ---------------------------------------------------------------------------
// build_served_freeze_frame - the shape GetFault serves
// ---------------------------------------------------------------------------

TEST(BuildServedFreezeFrame, NestsEvidenceBySourceUnderXMedkit) {
  auto out = nlohmann::json::parse(
      build_served_freeze_frame(R"({"/odom":{"x":1}})", {entry(kSourceA, "nis", "0.03", 1'700'000'000'123'000'000),
                                                         entry(kSourceB, "nis", "0.5", 1'700'000'000'000'000'000)}));

  EXPECT_EQ(out.at("/odom").at("x"), 1);
  const auto & reported = out.at("x-medkit").at("reported");
  EXPECT_EQ(reported.at(kSourceA).at("nis").at("value"), "0.03");
  EXPECT_EQ(reported.at(kSourceA).at("nis").at("reported_at"), "2023-11-14T22:13:20.123Z");
  EXPECT_EQ(reported.at(kSourceB).at("nis").at("value"), "0.5");
}

TEST(BuildServedFreezeFrame, AnEvidenceKeyCannotClashWithTheTimestamp) {
  auto out = nlohmann::json::parse(build_served_freeze_frame("", {entry(kSourceA, "reported_at", "7", 0)}));

  EXPECT_EQ(out.at("x-medkit").at("reported").at(kSourceA).at("reported_at").at("value"), "7");
}

TEST(BuildServedFreezeFrame, WithoutEvidenceTheTopicValuesAreServedAsTheyAre) {
  auto out = nlohmann::json::parse(build_served_freeze_frame(R"({"/odom":{"x":1}})", {}));

  EXPECT_EQ(out, nlohmann::json::parse(R"({"/odom":{"x":1}})"));
}

TEST(BuildServedFreezeFrame, AnUnreadableFrameStillServesTheEvidence) {
  auto out = nlohmann::json::parse(build_served_freeze_frame("not json", {entry(kSourceA, "nis", "0.03", 0)}));

  EXPECT_EQ(out.at("x-medkit").at("reported").at(kSourceA).at("nis").at("value"), "0.03");
}

// ---------------------------------------------------------------------------
// store_reported_evidence / get_reported_evidence - same behaviour in every backend
// ---------------------------------------------------------------------------

template <typename T>
class ReportedEvidenceStorageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if constexpr (std::is_same_v<T, SqliteFaultStorage>) {
      std::random_device rd;
      std::mt19937_64 gen(rd());
      std::uniform_int_distribution<uint64_t> dist;
      db_path_ = std::filesystem::temp_directory_path() / ("test_evidence_" + std::to_string(dist(gen)) + ".db");
      storage_ = std::make_unique<SqliteFaultStorage>(db_path_.string());
    } else {
      storage_ = std::make_unique<InMemoryFaultStorage>();
    }
  }

  void TearDown() override {
    storage_.reset();
    if (!db_path_.empty()) {
      std::error_code ec;
      std::filesystem::remove(db_path_, ec);
    }
  }

  /// Keys of one occurrence, as "source:key=value".
  std::vector<std::string> read(uint32_t occurrence) const {
    std::vector<std::string> out;
    for (const auto & e : storage_->get_reported_evidence(kCode, occurrence)) {
      out.push_back(e.source_id + ":" + e.key + "=" + e.value);
    }
    return out;
  }

  std::filesystem::path db_path_;
  std::unique_ptr<FaultStorage> storage_;
};

using Backends = ::testing::Types<InMemoryFaultStorage, SqliteFaultStorage>;
TYPED_TEST_SUITE(ReportedEvidenceStorageTest, Backends);

TYPED_TEST(ReportedEvidenceStorageTest, ALaterReportUpdatesItsKeysAndKeepsTheRest) {
  this->storage_->store_reported_evidence(kCode, 1, kSourceA, Pairs{{"nis", "0.03"}, {"gate", "9.2"}}, 100);
  this->storage_->store_reported_evidence(kCode, 1, kSourceA, Pairs{{"nis", "0.31"}}, 200);

  auto stored = this->storage_->get_reported_evidence(kCode, 1);
  ASSERT_EQ(stored.size(), 2u);
  EXPECT_EQ(stored[0].key, "gate");
  EXPECT_EQ(stored[0].reported_at_ns, 100);  // per key: an old key keeps its own time
  EXPECT_EQ(stored[1].key, "nis");
  EXPECT_EQ(stored[1].value, "0.31");
  EXPECT_EQ(stored[1].reported_at_ns, 200);
}

TYPED_TEST(ReportedEvidenceStorageTest, TwoSourcesKeepTheirOwnValues) {
  this->storage_->store_reported_evidence(kCode, 1, "/a", Pairs{{"temp", "80"}, {"gate", "1"}}, 100);
  this->storage_->store_reported_evidence(kCode, 1, "/b", Pairs{{"temp", "20"}}, 200);

  EXPECT_EQ(this->read(1), (std::vector<std::string>{"/a:gate=1", "/a:temp=80", "/b:temp=20"}));
}

TYPED_TEST(ReportedEvidenceStorageTest, AnOversizedUpdateRemovesTheStoredKey) {
  this->storage_->store_reported_evidence(kCode, 1, kSourceA, Pairs{{"blob", "short"}, {"nis", "0.03"}}, 100);
  const size_t dropped = this->storage_->store_reported_evidence(
      kCode, 1, kSourceA, Pairs{{"blob", std::string(kMaxEvidenceValueBytes + 1, 'x')}}, 200);

  EXPECT_EQ(dropped, 1u);
  EXPECT_EQ(this->read(1), (std::vector<std::string>{std::string(kSourceA) + ":nis=0.03"}));
}

TYPED_TEST(ReportedEvidenceStorageTest, ANewOccurrenceStartsFromNothing) {
  this->storage_->store_reported_evidence(kCode, 1, kSourceA, Pairs{{"old", "1"}}, 100);
  this->storage_->store_reported_evidence(kCode, 2, kSourceA, Pairs{{"new", "2"}}, 200);

  EXPECT_TRUE(this->read(1).empty());
  EXPECT_EQ(this->read(2), (std::vector<std::string>{std::string(kSourceA) + ":new=2"}));
}

TYPED_TEST(ReportedEvidenceStorageTest, ANewOccurrenceDoesNotInheritTheKeyBound) {
  Pairs many;
  for (size_t i = 0; i < kMaxEvidenceEntries; ++i) {
    many.emplace_back("k" + std::to_string(i), "v");
  }
  ASSERT_EQ(this->storage_->store_reported_evidence(kCode, 1, kSourceA, many, 100), 0u);

  EXPECT_EQ(this->storage_->store_reported_evidence(kCode, 2, kSourceA, Pairs{{"fresh", "1"}}, 200), 0u);
}

TYPED_TEST(ReportedEvidenceStorageTest, WithRetentionOlderOccurrencesAreKeptUpToTheCap) {
  this->storage_->set_retain_snapshots_on_clear(true);
  this->storage_->set_max_evidence_occurrences_per_fault(2);
  for (uint32_t occurrence = 1; occurrence <= 3; ++occurrence) {
    this->storage_->store_reported_evidence(kCode, occurrence, kSourceA, Pairs{{"n", std::to_string(occurrence)}},
                                            occurrence * 100);
  }

  EXPECT_TRUE(this->read(1).empty());  // the oldest went first
  EXPECT_EQ(this->read(2), (std::vector<std::string>{std::string(kSourceA) + ":n=2"}));
  EXPECT_EQ(this->read(3), (std::vector<std::string>{std::string(kSourceA) + ":n=3"}));
}

TYPED_TEST(ReportedEvidenceStorageTest, EvidenceOfOneCodeLeavesOtherCodesAlone) {
  this->storage_->store_reported_evidence("OTHER_CODE", 1, kSourceA, Pairs{{"n", "1"}}, 100);
  this->storage_->store_reported_evidence(kCode, 2, kSourceA, Pairs{{"n", "2"}}, 200);

  EXPECT_EQ(this->storage_->get_reported_evidence("OTHER_CODE", 1).size(), 1u);
}

TYPED_TEST(ReportedEvidenceStorageTest, NothingIsWrittenWhenEverythingIsDropped) {
  EXPECT_EQ(this->storage_->store_reported_evidence(kCode, 1, kSourceA, Pairs{{"", "x"}}, 100), 1u);

  EXPECT_TRUE(this->read(1).empty());
}

TEST(SqliteReportedEvidence, SurvivesARestart) {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  const auto db_path = std::filesystem::temp_directory_path() / ("test_evidence_" + std::to_string(dist(gen)) + ".db");
  {
    SqliteFaultStorage storage(db_path.string());
    storage.store_reported_evidence(kCode, 1, kSourceA, Pairs{{"nis", "0.03"}}, 100);
  }
  {
    SqliteFaultStorage storage(db_path.string());
    auto stored = storage.get_reported_evidence(kCode, 1);
    ASSERT_EQ(stored.size(), 1u);
    EXPECT_EQ(stored[0].value, "0.03");
  }
  std::error_code ec;
  std::filesystem::remove(db_path, ec);
}
