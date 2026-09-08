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

#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/core/sqlite_entity_freeze_frame_store.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::EntityFreezeFrameStore;
using ros2_medkit_gateway::InMemoryEntityFreezeFrameStore;
using ros2_medkit_gateway::SqliteEntityFreezeFrameStore;
using ros2_medkit_gateway::StoredEntityFreezeFrame;

namespace {

StoredEntityFreezeFrame make_row(const std::string & fault_code, const std::string & entity_id, int64_t captured_at_ns,
                                 json frame = json{{"values", {{"temperature", 42.5}, {"pressure", 3.2}}},
                                                   {"connected", false},
                                                   {"source_timestamp", "2026-09-08T17:51:40.387Z"}}) {
  StoredEntityFreezeFrame row;
  row.fault_code = fault_code;
  row.entity_id = entity_id;
  row.frame = std::move(frame);
  row.captured_at_ns = captured_at_ns;
  row.source = "plugin_x_plc_data_route";
  row.capture_origin = "";
  return row;
}

/// Both backends must behave identically: the in-memory one is what tests and
/// a path-less gateway get, and a difference between them would only show up
/// on the box.
enum class Backend { InMemory, Sqlite };

class EntityFreezeFrameStoreTest : public ::testing::TestWithParam<Backend> {
 protected:
  void SetUp() override {
    db_path_ = std::filesystem::temp_directory_path() /
               ("test_entity_freeze_frame_store_" + std::to_string(::getpid()) + ".db");
    std::filesystem::remove(db_path_);
    store_ = open();
  }

  void TearDown() override {
    store_.reset();
    std::filesystem::remove(db_path_);
  }

  std::unique_ptr<EntityFreezeFrameStore> open() {
    if (GetParam() == Backend::InMemory) {
      return std::make_unique<InMemoryEntityFreezeFrameStore>();
    }
    return std::make_unique<SqliteEntityFreezeFrameStore>(db_path_.string());
  }

  std::filesystem::path db_path_;
  std::unique_ptr<EntityFreezeFrameStore> store_;
};

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_P(EntityFreezeFrameStoreTest, RoundTripsEveryFieldOfARow) {
  auto row = make_row("JAM_INFEED", "plc_app", 1757353900387000000);
  row.capture_origin = "startup";
  ASSERT_TRUE(store_->replace_frames("JAM_INFEED", {row}).has_value());

  auto loaded = store_->load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  const auto & got = (*loaded)[0];
  EXPECT_EQ(got.fault_code, "JAM_INFEED");
  EXPECT_EQ(got.entity_id, "plc_app");
  EXPECT_EQ(got.frame, row.frame);
  EXPECT_EQ(got.captured_at_ns, 1757353900387000000);  // nanoseconds need the full 64 bits
  EXPECT_EQ(got.source, "plugin_x_plc_data_route");
  EXPECT_EQ(got.capture_origin, "startup");
}

/// @verifies REQ_INTEROP_088
TEST_P(EntityFreezeFrameStoreTest, ReplaceDropsEntitiesTheNewCaptureNoLongerReports) {
  ASSERT_TRUE(store_
                  ->replace_frames("JAM_INFEED",
                                   {make_row("JAM_INFEED", "plc_app", 10), make_row("JAM_INFEED", "second_app", 20)})
                  .has_value());
  // A re-confirm that only frames one entity must not leave the other's row
  // behind: the served frames are replaced as a unit, so the rows are too.
  ASSERT_TRUE(store_->replace_frames("JAM_INFEED", {make_row("JAM_INFEED", "plc_app", 30)}).has_value());

  auto loaded = store_->load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ((*loaded)[0].entity_id, "plc_app");
  EXPECT_EQ((*loaded)[0].captured_at_ns, 30);
}

/// @verifies REQ_INTEROP_088
TEST_P(EntityFreezeFrameStoreTest, OneCodePerFaultRowsAreKeptApart) {
  ASSERT_TRUE(store_->replace_frames("JAM_INFEED", {make_row("JAM_INFEED", "plc_app", 10)}).has_value());
  ASSERT_TRUE(store_->replace_frames("SAFETY_CURTAIN", {make_row("SAFETY_CURTAIN", "plc_app", 20)}).has_value());

  ASSERT_TRUE(store_->erase_frames("JAM_INFEED").has_value());

  auto loaded = store_->load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ((*loaded)[0].fault_code, "SAFETY_CURTAIN");
}

/// @verifies REQ_INTEROP_088
TEST_P(EntityFreezeFrameStoreTest, LoadAllServesOldestCaptureFirst) {
  // The caller keeps the newest codes when the store holds more than its
  // retained-frame bound, so the order is part of the contract, not a detail.
  ASSERT_TRUE(store_->replace_frames("NEWEST", {make_row("NEWEST", "plc_app", 300)}).has_value());
  ASSERT_TRUE(store_->replace_frames("OLDEST", {make_row("OLDEST", "plc_app", 100)}).has_value());
  ASSERT_TRUE(store_->replace_frames("MIDDLE", {make_row("MIDDLE", "plc_app", 200)}).has_value());

  auto loaded = store_->load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 3u);
  EXPECT_EQ((*loaded)[0].fault_code, "OLDEST");
  EXPECT_EQ((*loaded)[1].fault_code, "MIDDLE");
  EXPECT_EQ((*loaded)[2].fault_code, "NEWEST");
}

/// @verifies REQ_INTEROP_088
TEST_P(EntityFreezeFrameStoreTest, ErasingACodeWithNoRowsIsNotAnError) {
  // The caller evicts by fault code and does not track what reached the file.
  EXPECT_TRUE(store_->erase_frames("NEVER_STORED").has_value());
}

/// @verifies REQ_INTEROP_088
TEST_P(EntityFreezeFrameStoreTest, AnEmptyReplaceLeavesNoRows) {
  ASSERT_TRUE(store_->replace_frames("JAM_INFEED", {make_row("JAM_INFEED", "plc_app", 10)}).has_value());
  ASSERT_TRUE(store_->replace_frames("JAM_INFEED", {}).has_value());

  auto loaded = store_->load_all();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_TRUE(loaded->empty());
}

INSTANTIATE_TEST_SUITE_P(Backends, EntityFreezeFrameStoreTest, ::testing::Values(Backend::InMemory, Backend::Sqlite),
                         [](const ::testing::TestParamInfo<Backend> & param_info) {
                           return param_info.param == Backend::InMemory ? "InMemory" : "Sqlite";
                         });

// ===========================================================================
// SQLite only: the point of the file is that it outlives the process.
// ===========================================================================

/// @verifies REQ_INTEROP_088
TEST(SqliteEntityFreezeFrameStoreFile, RowsSurviveReopen) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("test_entity_freeze_frame_reopen_" + std::to_string(::getpid()) + ".db");
  std::filesystem::remove(path);

  {
    SqliteEntityFreezeFrameStore store(path.string());
    ASSERT_TRUE(
        store.replace_frames("JAM_INFEED", {make_row("JAM_INFEED", "plc_app", 1757353900387000000)}).has_value());
  }

  SqliteEntityFreezeFrameStore reopened(path.string());
  auto loaded = reopened.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ((*loaded)[0].fault_code, "JAM_INFEED");
  EXPECT_EQ((*loaded)[0].captured_at_ns, 1757353900387000000);
  EXPECT_EQ((*loaded)[0].frame["values"]["temperature"], 42.5);
  EXPECT_EQ((*loaded)[0].frame["connected"], false);

  std::filesystem::remove(path);
}

/// @verifies REQ_INTEROP_088
TEST(SqliteEntityFreezeFrameStoreFile, AnUnreadableRowCostsOnlyItself) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("test_entity_freeze_frame_corrupt_" + std::to_string(::getpid()) + ".db");
  std::filesystem::remove(path);

  {
    SqliteEntityFreezeFrameStore store(path.string());
    ASSERT_TRUE(store.replace_frames("GOOD", {make_row("GOOD", "plc_app", 20)}).has_value());
  }
  // Hand-edit one blob into something that is not JSON, as a half-written file
  // or a fat-fingered sqlite3 session would.
  {
    sqlite3 * db = nullptr;
    ASSERT_EQ(sqlite3_open(path.c_str(), &db), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db,
                           "INSERT INTO entity_freeze_frames "
                           "(fault_code, entity_id, frame, captured_at_ns, source, capture_origin) "
                           "VALUES ('BROKEN','plc_app','{not json',10,'','')",
                           nullptr, nullptr, nullptr),
              SQLITE_OK);
    sqlite3_close(db);
  }

  SqliteEntityFreezeFrameStore reopened(path.string());
  auto loaded = reopened.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ((*loaded)[0].fault_code, "GOOD");

  std::filesystem::remove(path);
}
