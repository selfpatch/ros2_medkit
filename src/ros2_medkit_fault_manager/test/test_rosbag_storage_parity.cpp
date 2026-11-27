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

/// Evidence retention parity between the two storage backends.
///
/// A fault holding several recordings was enforced by the SQLite schema before, and
/// separately by a std::map in the in-memory backend. Two enforcement points means two
/// chances to diverge, and `storage_type: memory` silently keeping the old behaviour
/// would be invisible to a suite that only exercises SQLite. Every assertion here
/// therefore runs against both backends from one body.
///
/// Snapshots have the same shape and were left out of the first version of this file,
/// which is how two backends ended up disagreeing about whether a capture larger than
/// the cap survives at all, and about which capture `get_snapshots` returns first.
/// Both suites live here so a new evidence table cannot be added to one backend only.

#include <gtest/gtest.h>

#include <sqlite3.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef POSTGRES_SUPPORT
#include <pqxx/pqxx>
#endif

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/core/rosbag_retention.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#ifdef POSTGRES_SUPPORT
#include "ros2_medkit_fault_manager/postgres_fault_storage.hpp"
#endif
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"
#include "test_log_sink.hpp"
#include "test_rosbag_rows.hpp"

namespace {

using ros2_medkit_fault_manager::FaultStorage;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::rosbag_recording_id;
using ros2_medkit_fault_manager::RosbagApplyResult;
using ros2_medkit_fault_manager::RosbagChangeSet;
using ros2_medkit_fault_manager::RosbagFileInfo;
using ros2_medkit_fault_manager::RosbagLink;
using ros2_medkit_fault_manager::SnapshotData;
using ros2_medkit_fault_manager::SqliteFaultStorage;
using ros2_medkit_fault_manager::core::RosbagRetention;
using ros2_medkit_fault_manager::core::RosbagRetentionLimits;
using ros2_medkit_fault_manager::core::StoreResult;
#ifdef POSTGRES_SUPPORT
using ros2_medkit_fault_manager::PgFaultStorage;
#endif

/// Backends differ only in how they are constructed, so the fixture reaches them
/// through this rather than through #ifdef-style branching inside the test bodies.
template <typename Backend>
struct BackendFactory;

template <>
struct BackendFactory<InMemoryFaultStorage> {
  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & /*dir*/) {
    return std::make_unique<InMemoryFaultStorage>();
  }
  static void cleanup(const std::filesystem::path & /*dir*/) {
  }
};

template <>
struct BackendFactory<SqliteFaultStorage> {
  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & dir) {
    return std::make_unique<SqliteFaultStorage>((dir / "parity.db").string(), test_log_sink());
  }
  static void cleanup(const std::filesystem::path & /*dir*/) {
  }

  /// A trigger that raises on every INSERT or DELETE of a row of `fault_code`.
  static void add_failing_trigger(const std::filesystem::path & dir, const std::string & event,
                                  const std::string & fault_code) {
    const std::string row = event == "INSERT" ? "NEW" : "OLD";
    const std::string when = row + ".fault_code = '" + fault_code + "'";
    exec(dir, "CREATE TRIGGER fail_" + event + " BEFORE " + event + " ON rosbag_files WHEN " + when +
                  " BEGIN SELECT RAISE(ABORT, 'forced failure'); END");
  }

  static void exec(const std::filesystem::path & dir, const std::string & sql) {
    sqlite3 * db = nullptr;
    ASSERT_EQ(sqlite3_open((dir / "parity.db").c_str(), &db), SQLITE_OK);
    char * err = nullptr;
    const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &err);
    const std::string message = err ? err : "";
    sqlite3_free(err);
    sqlite3_close(db);
    ASSERT_EQ(rc, SQLITE_OK) << message;
  }
};

#ifdef POSTGRES_SUPPORT
/// One schema per test on the server named by ROS2_MEDKIT_TEST_PG_CONN, as test_postgres_storage does.
template <>
struct BackendFactory<PgFaultStorage> {
  static std::string base_conn_info() {
    if (const char * env = std::getenv("ROS2_MEDKIT_TEST_PG_CONN")) {
      return env;
    }
    return "postgresql://user:password@localhost:5432/ros2_medkit_faults_database";
  }

  static std::string schema_of(const std::filesystem::path & dir) {
    return "parity_" + dir.filename().string();
  }

  static std::string conn_info(const std::filesystem::path & dir) {
    return base_conn_info() + "?options=-csearch_path%3D" + schema_of(dir);
  }

  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & dir) {
    try {
      pqxx::connection conn(base_conn_info());
      pqxx::work tx(conn);
      tx.exec("CREATE SCHEMA " + tx.quote_name(schema_of(dir)));
      tx.commit();
    } catch (const std::exception & e) {
      ADD_FAILURE() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
      return nullptr;
    }
    return std::make_unique<PgFaultStorage>(conn_info(dir), test_log_sink());
  }

  static void cleanup(const std::filesystem::path & dir) {
    try {
      pqxx::connection conn(base_conn_info());
      pqxx::work tx(conn);
      tx.exec("DROP SCHEMA IF EXISTS " + tx.quote_name(schema_of(dir)) + " CASCADE");
      tx.commit();
    } catch (const std::exception &) {
      // The server is gone; nothing is left to drop.
    }
  }

  static void add_failing_trigger(const std::filesystem::path & dir, const std::string & event,
                                  const std::string & fault_code) {
    const std::string row = event == "INSERT" ? "NEW" : "OLD";
    const std::string name = "fail_" + event;
    const std::string when = row + ".fault_code = '" + fault_code + "'";
    exec(dir, "CREATE FUNCTION " + name +
                  "_fn() RETURNS trigger AS $$ BEGIN RAISE EXCEPTION 'forced failure'; END $$ LANGUAGE plpgsql");
    exec(dir, "CREATE TRIGGER " + name + " BEFORE " + event + " ON rosbag_files FOR EACH ROW WHEN (" + when +
                  ") EXECUTE FUNCTION " + name + "_fn()");
  }

  static void exec(const std::filesystem::path & dir, const std::string & sql) {
    pqxx::connection conn(conn_info(dir));
    pqxx::work tx(conn);
    tx.exec(sql);
    tx.commit();
  }
};
#endif

template <typename Backend>
class RosbagRetentionParityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ =
        std::filesystem::temp_directory_path() / ("medkit_parity_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    storage_ = BackendFactory<Backend>::make(dir_);
    ASSERT_NE(storage_, nullptr);
  }

  void TearDown() override {
    retention_.reset();
    storage_.reset();
    BackendFactory<Backend>::cleanup(dir_);
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  static RosbagRetentionLimits cap(size_t max_bags_per_fault) {
    RosbagRetentionLimits limits;
    limits.max_bags_per_fault = max_bags_per_fault;
    return limits;
  }

  /// The retention every store() below goes through, over this backend.
  void use_retention(RosbagRetentionLimits limits) {
    retention_ = std::make_unique<RosbagRetention>(*storage_, limits, test_log_sink(), "rosbag_retention_parity");
  }

  /// Store one recording through the retention and keep the paths it hands over.
  void store(std::vector<RosbagFileInfo> rows) {
    const auto outcome = retention_->store(std::move(rows));
    EXPECT_EQ(outcome.result, StoreResult::kStored) << outcome.error;
    hand_over(outcome.unlink_paths);
  }

  /// Keep the paths of one retention call. Each is listed once across the whole test.
  void hand_over(const std::vector<std::string> & paths) {
    expect_unlinkable(paths);
    for (const auto & path : paths) {
      EXPECT_FALSE(unlinked(path)) << path << " was handed over twice";
      unlink_paths_.push_back(path);
    }
  }

  /// Whether a retention call of this test handed @p path over for removal.
  bool unlinked(const std::string & path) const {
    return listed(unlink_paths_, path);
  }

  /// Create a real bag directory so unlink behaviour is observable, not just row state.
  /// Bags are directories on disk, which is what both backends remove.
  std::string make_bag(const std::string & name) {
    const auto path = dir_ / name;
    std::filesystem::create_directories(path);
    std::ofstream(path / "metadata.yaml") << "version: 9\n";
    return path.string();
  }

  RosbagFileInfo row(const std::string & code, const std::string & bag_name, int64_t created_ns) {
    RosbagFileInfo info;
    info.fault_code = code;
    info.file_path = make_bag(bag_name);
    info.recording_id = rosbag_recording_id(info.file_path);
    info.format = "mcap";
    info.duration_sec = 5.0;
    info.size_bytes = 1024;
    info.created_at_ns = created_ns;
    return info;
  }

  std::filesystem::path dir_;
  std::unique_ptr<FaultStorage> storage_;
  std::unique_ptr<RosbagRetention> retention_;
  std::vector<std::string> unlink_paths_;
};

#ifdef POSTGRES_SUPPORT
using Backends = ::testing::Types<InMemoryFaultStorage, SqliteFaultStorage, PgFaultStorage>;
using SqlBackends = ::testing::Types<SqliteFaultStorage, PgFaultStorage>;
#else
using Backends = ::testing::Types<InMemoryFaultStorage, SqliteFaultStorage>;
using SqlBackends = ::testing::Types<SqliteFaultStorage>;
#endif
TYPED_TEST_SUITE(RosbagRetentionParityTest, Backends);

TYPED_TEST(RosbagRetentionParityTest, RecordingIdIsTheBasenameOfEveryStoredRow) {
  // The gateway addresses a recording by this string and the fault_manager resolves it
  // back to rows, so a backend that stored anything else would break the URL silently.
  seed_rosbag_rows(*this->storage_, {this->row("BASENAME", "fault_BASENAME_1", 1000)});
  seed_rosbag_rows(*this->storage_, {this->row("BASENAME", "fault_BASENAME_2", 2000)});

  const auto rows = this->storage_->get_rosbag_files("BASENAME");
  ASSERT_EQ(rows.size(), 2u);
  for (const auto & r : rows) {
    EXPECT_EQ(r.recording_id, std::filesystem::path(r.file_path).filename().string());
  }
}

TYPED_TEST(RosbagRetentionParityTest, RecordingIdIsDerivedWhenTheCallerLeftItEmpty) {
  // rosbag_capture fills it, but the interface is public and embedders re-store rows.
  // A row with no recording_id is unaddressable, so neither backend may store one.
  auto info = this->row("DERIVED", "fault_DERIVED_1", 1000);
  info.recording_id.clear();
  seed_rosbag_rows(*this->storage_, {info});

  const auto rows = this->storage_->get_rosbag_files("DERIVED");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].recording_id, "fault_DERIVED_1");
}

TYPED_TEST(RosbagRetentionParityTest, SeveralRecordingsForOneFaultSurviveUnderACap) {
  // The feature itself: this is exactly what the schema made impossible before.
  this->use_retention(this->cap(3));
  this->store({this->row("FLAP", "fault_FLAP_1", 1000)});
  this->store({this->row("FLAP", "fault_FLAP_2", 2000)});
  this->store({this->row("FLAP", "fault_FLAP_3", 3000)});

  const auto rows = this->storage_->get_rosbag_files("FLAP");
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_EQ(rows[0].recording_id, "fault_FLAP_3") << "newest first";
  EXPECT_EQ(rows[2].recording_id, "fault_FLAP_1");
  for (const auto & r : rows) {
    EXPECT_FALSE(this->unlinked(r.file_path));
  }
}

TYPED_TEST(RosbagRetentionParityTest, CapKeepsTheNewestAndUnlinksTheEvicted) {
  this->use_retention(this->cap(2));
  const auto oldest = this->row("CAP", "fault_CAP_1", 1000);
  this->store({oldest});
  this->store({this->row("CAP", "fault_CAP_2", 2000)});
  this->store({this->row("CAP", "fault_CAP_3", 3000)});

  const auto rows = this->storage_->get_rosbag_files("CAP");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].recording_id, "fault_CAP_3");
  EXPECT_EQ(rows[1].recording_id, "fault_CAP_2");
  EXPECT_TRUE(this->unlinked(oldest.file_path)) << "the evicted bag must not outlive its last row";
}

TYPED_TEST(RosbagRetentionParityTest, ACapOfOneReproducesThePreviousBehaviourExactly) {
  // The shipped default. Landing at parity is what makes any regression report about
  // the plumbing rather than about the retention policy.
  this->use_retention(this->cap(1));
  const auto first = this->row("ONE", "fault_ONE_1", 1000);
  this->store({first});
  this->store({this->row("ONE", "fault_ONE_2", 2000)});

  const auto rows = this->storage_->get_rosbag_files("ONE");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].recording_id, "fault_ONE_2");
  EXPECT_TRUE(this->unlinked(first.file_path));

  const auto newest = this->storage_->get_rosbag_file("ONE");
  ASSERT_TRUE(newest.has_value());
  EXPECT_EQ(newest->recording_id, "fault_ONE_2");
}

TYPED_TEST(RosbagRetentionParityTest, ACapOfOneIsTheDefaultWithoutConfiguringAnything) {
  // A retention built without configuring anything - tests, embedders - must not silently
  // start accumulating recordings just because the cap became configurable.
  EXPECT_EQ(RosbagRetentionLimits{}.max_bags_per_fault, 1u);
  this->use_retention(RosbagRetentionLimits{});
  const auto first = this->row("DEFAULT", "fault_DEFAULT_1", 1000);
  this->store({first});
  this->store({this->row("DEFAULT", "fault_DEFAULT_2", 2000)});

  EXPECT_EQ(this->storage_->get_rosbag_files("DEFAULT").size(), 1u);
  EXPECT_TRUE(this->unlinked(first.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, ZeroMeansUnlimited) {
  this->use_retention(this->cap(0));
  for (int i = 1; i <= 6; ++i) {
    this->store({this->row("UNBOUNDED", "fault_UNBOUNDED_" + std::to_string(i), i * 1000)});
  }
  EXPECT_EQ(this->storage_->get_rosbag_files("UNBOUNDED").size(), 6u);
}

TYPED_TEST(RosbagRetentionParityTest, ReStoringTheSamePathIsAnUpsertNotASecondRecording) {
  // Uniqueness is on (fault_code, file_path). Re-storing the same bag - a restart
  // replaying its rows, say - must update in place, not consume a cap slot.
  this->use_retention(this->cap(3));
  auto info = this->row("UPSERT", "fault_UPSERT_1", 1000);
  this->store({info});
  info.size_bytes = 4096;
  info.duration_sec = 9.0;
  this->store({info});

  const auto rows = this->storage_->get_rosbag_files("UPSERT");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].size_bytes, 4096u);
  EXPECT_DOUBLE_EQ(rows[0].duration_sec, 9.0);
  EXPECT_FALSE(this->unlinked(rows[0].file_path)) << "an upsert must not unlink the bag it just updated";
}

TYPED_TEST(RosbagRetentionParityTest, RefreshingALinkDoesNotMoveItWithinATieGroup) {
  // The tiebreak is positional - SQLite's id, the in-memory sequence number - so a
  // refresh must not renumber the row. SQLite's INSERT OR REPLACE would: it deletes
  // and re-inserts, handing the row a fresh id and silently promoting it past rows
  // it was stored before.
  auto first = this->row("REFRESH", "fault_REFRESH_1", 5000);
  seed_rosbag_rows(*this->storage_, {first});
  seed_rosbag_rows(*this->storage_, {this->row("REFRESH", "fault_REFRESH_2", 5000)});

  first.size_bytes = 8192;
  seed_rosbag_rows(*this->storage_, {first});

  const auto rows = this->storage_->get_rosbag_files("REFRESH");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].recording_id, "fault_REFRESH_2") << "the refresh must not reorder the tie group";
  EXPECT_EQ(rows[1].recording_id, "fault_REFRESH_1");
  EXPECT_EQ(rows[1].size_bytes, 8192u) << "but it must still update the row";
}

TYPED_TEST(RosbagRetentionParityTest, NewestFirstIsStableWhenAWholeBurstSharesATimestamp) {
  // Every row of one burst carries the same created_at_ns, so ties are guaranteed
  // rather than exotic. SQLite breaks them by id; the in-memory backend needs its own
  // sequence counter to agree. Without it the two orders diverge only sometimes,
  // which is a flaky test rather than an honest failure.
  for (int i = 1; i <= 4; ++i) {
    seed_rosbag_rows(*this->storage_, {this->row("TIE", "fault_TIE_" + std::to_string(i), 7000)});
  }

  const auto rows = this->storage_->get_rosbag_files("TIE");
  ASSERT_EQ(rows.size(), 4u);
  EXPECT_EQ(rows[0].recording_id, "fault_TIE_4") << "insertion order breaks the tie, newest first";
  EXPECT_EQ(rows[1].recording_id, "fault_TIE_3");
  EXPECT_EQ(rows[2].recording_id, "fault_TIE_2");
  EXPECT_EQ(rows[3].recording_id, "fault_TIE_1");
}

TYPED_TEST(RosbagRetentionParityTest, ACapTieIsBrokenByInsertionOrderNotArbitrarily) {
  // The same tie, now deciding which bag gets deleted. Whichever row wins, both
  // backends must agree, and the survivor's bag must be the one still on disk.
  this->use_retention(this->cap(1));
  const auto first = this->row("TIECAP", "fault_TIECAP_1", 7000);
  this->store({first});
  const auto second = this->row("TIECAP", "fault_TIECAP_2", 7000);
  this->store({second});

  const auto rows = this->storage_->get_rosbag_files("TIECAP");
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].recording_id, "fault_TIECAP_2");
  EXPECT_FALSE(this->unlinked(second.file_path));
  EXPECT_TRUE(this->unlinked(first.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, ABurstRecordingSurvivesWhileASiblingRowReferencesIt) {
  // One bag, several faults. The cap evicts A's link but B still holds the bytes.
  this->use_retention(this->cap(1));
  auto shared = this->row("BURST_A", "fault_BURST_1", 1000);
  this->store({shared});
  shared.fault_code = "BURST_B";
  this->store({shared});

  const auto shared_path = shared.file_path;
  this->store({this->row("BURST_A", "fault_BURST_2", 2000)});

  EXPECT_FALSE(this->unlinked(shared_path)) << "the sibling fault still owns the shared bag";
  const auto sibling = this->storage_->get_rosbag_files("BURST_B");
  ASSERT_EQ(sibling.size(), 1u);
  EXPECT_EQ(sibling[0].file_path, shared_path);
}

TYPED_TEST(RosbagRetentionParityTest, RecordingLookupReturnsEveryFaultOfTheBurst) {
  // This is what the gateway authorizes against: the union of faults attached to a
  // recording. Missing one would 404 a download the entity is entitled to.
  auto shared = this->row("LOOKUP_A", "fault_LOOKUP_1", 1000);
  seed_rosbag_rows(*this->storage_, {shared});
  shared.fault_code = "LOOKUP_B";
  seed_rosbag_rows(*this->storage_, {shared});
  shared.fault_code = "LOOKUP_C";
  seed_rosbag_rows(*this->storage_, {shared});

  const auto rows = this->storage_->get_rosbag_files_by_recording("fault_LOOKUP_1");
  ASSERT_EQ(rows.size(), 3u);
  std::vector<std::string> codes;
  for (const auto & r : rows) {
    codes.push_back(r.fault_code);
    EXPECT_EQ(r.file_path, shared.file_path);
  }
  std::sort(codes.begin(), codes.end());
  EXPECT_EQ(codes, (std::vector<std::string>{"LOOKUP_A", "LOOKUP_B", "LOOKUP_C"}));
}

TYPED_TEST(RosbagRetentionParityTest, AnUnknownRecordingLooksUpEmptyRatherThanThrowing) {
  // The gateway's compatibility path depends on this: empty means "not a recording,
  // try it as a fault code", so it must be a normal answer.
  EXPECT_TRUE(this->storage_->get_rosbag_files_by_recording("fault_NOPE_1").empty());
  EXPECT_TRUE(this->storage_->get_rosbag_files("NOPE").empty());
  EXPECT_FALSE(this->storage_->get_rosbag_file("NOPE").has_value());
}

TYPED_TEST(RosbagRetentionParityTest, DeletingARecordingRemovesEveryLinkAndTheBag) {
  // Removal by recording, as the stale-row cleanup does. Deleting by fault code here would
  // wipe the whole history of every fault the bag touched.
  auto shared = this->row("DELREC_A", "fault_DELREC_1", 1000);
  seed_rosbag_rows(*this->storage_, {shared});
  shared.fault_code = "DELREC_B";
  seed_rosbag_rows(*this->storage_, {shared});
  const auto kept = this->row("DELREC_A", "fault_DELREC_2", 2000);
  seed_rosbag_rows(*this->storage_, {kept});
  this->use_retention(RosbagRetentionLimits{});

  const auto removal = this->retention_->forget_recording("fault_DELREC_1");
  this->hand_over(removal.unlink_paths);
  EXPECT_EQ(removal.removed_rows, 2u);
  EXPECT_TRUE(this->unlinked(shared.file_path));
  EXPECT_TRUE(this->storage_->get_rosbag_files("DELREC_B").empty());

  const auto survivors = this->storage_->get_rosbag_files("DELREC_A");
  ASSERT_EQ(survivors.size(), 1u) << "the other fault's other recording is untouched";
  EXPECT_EQ(survivors[0].recording_id, "fault_DELREC_2");
  EXPECT_FALSE(this->unlinked(kept.file_path));

  const auto again = this->retention_->forget_recording("fault_DELREC_1");
  this->hand_over(again.unlink_paths);
  EXPECT_EQ(again.removed_rows, 0u);
}

TYPED_TEST(RosbagRetentionParityTest, DeletingAFaultDropsAllItsRecordings) {
  const auto a = this->row("DELALL", "fault_DELALL_1", 1000);
  const auto b = this->row("DELALL", "fault_DELALL_2", 2000);
  seed_rosbag_rows(*this->storage_, {a});
  seed_rosbag_rows(*this->storage_, {b});
  this->use_retention(RosbagRetentionLimits{});

  const auto removal = this->retention_->on_fault_cleared("DELALL");
  this->hand_over(removal.unlink_paths);
  EXPECT_EQ(removal.removed_rows, 2u);
  EXPECT_TRUE(this->storage_->get_rosbag_files("DELALL").empty());
  EXPECT_TRUE(this->unlinked(a.file_path));
  EXPECT_TRUE(this->unlinked(b.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, ASharedBagCountsOnceTowardsStorageAcrossSeveralRecordings) {
  // The quota sums bytes per path, not per row. N recordings per fault multiplies the
  // rows, so a per-row sum would over-report and evict healthy bags under no pressure.
  auto shared = this->row("QUOTA_A", "fault_QUOTA_1", 1000);
  seed_rosbag_rows(*this->storage_, {shared});
  shared.fault_code = "QUOTA_B";
  seed_rosbag_rows(*this->storage_, {shared});
  shared.fault_code = "QUOTA_C";
  seed_rosbag_rows(*this->storage_, {shared});
  seed_rosbag_rows(*this->storage_, {this->row("QUOTA_A", "fault_QUOTA_2", 2000)});

  EXPECT_EQ(this->storage_->get_total_rosbag_storage_bytes(), 2048u) << "two distinct bags of 1024, counted once each";
}

TYPED_TEST(RosbagRetentionParityTest, GetAllRosbagFilesListsEveryRecordingOldestFirst) {
  // The quota sweep walks this list and evicts from the front, so the extra rows this
  // change introduces have to appear here in a defined order.
  seed_rosbag_rows(*this->storage_, {this->row("ALL_A", "fault_ALL_1", 3000)});
  seed_rosbag_rows(*this->storage_, {this->row("ALL_A", "fault_ALL_2", 1000)});
  seed_rosbag_rows(*this->storage_, {this->row("ALL_B", "fault_ALL_3", 2000)});

  const auto rows = this->storage_->get_all_rosbag_files();
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_EQ(rows[0].recording_id, "fault_ALL_2");
  EXPECT_EQ(rows[1].recording_id, "fault_ALL_3");
  EXPECT_EQ(rows[2].recording_id, "fault_ALL_1");
}

TYPED_TEST(RosbagRetentionParityTest, OneFaultKeepsSeveralRecordingsWhenTheCapAllows) {
  this->use_retention(this->cap(3));
  this->store({this->row("FLAP", "fault_FLAP_100", 100)});
  this->store({this->row("FLAP", "fault_FLAP_200", 200)});

  const auto rows = this->storage_->get_rosbag_files("FLAP");
  ASSERT_EQ(rows.size(), 2u) << "the second recording must not replace the first";
  EXPECT_EQ(rows[0].recording_id, "fault_FLAP_200") << "newest first";
  EXPECT_EQ(rows[1].recording_id, "fault_FLAP_100");

  // get_rosbag_file is "the newest", deterministically.
  const auto newest = this->storage_->get_rosbag_file("FLAP");
  ASSERT_TRUE(newest.has_value());
  EXPECT_EQ(newest->recording_id, "fault_FLAP_200");
}

TYPED_TEST(RosbagRetentionParityTest, CapKeepsTheNewestAndUnlinksTheEvictedBag) {
  this->use_retention(this->cap(1));
  const auto a = this->row("CAP", "fault_CAP_100", 100);
  const auto b = this->row("CAP", "fault_CAP_200", 200);
  this->store({a});
  this->store({b});

  const auto rows = this->storage_->get_rosbag_files("CAP");
  ASSERT_EQ(rows.size(), 1u) << "cap 1 is the historical behaviour: one recording per fault";
  EXPECT_EQ(rows[0].file_path, b.file_path);
  EXPECT_TRUE(this->unlinked(a.file_path)) << "the evicted bag must be unlinked";
  EXPECT_FALSE(this->unlinked(b.file_path));
}

TYPED_TEST(RosbagRetentionParityTest, EvictingOneFaultsLinkKeepsABagASiblingStillReferences) {
  // A burst shares one recording. Evicting it for one fault must not take the bytes
  // the other fault still points at.
  this->use_retention(this->cap(1));
  auto shared_b = this->row("B", "fault_SHARED_100", 100);
  auto shared_a = shared_b;
  shared_a.fault_code = "A";
  this->store({shared_a, shared_b});
  // A re-confirms with its own new bag, so A's link to the shared one is evicted.
  this->store({this->row("A", "fault_A_200", 200)});

  EXPECT_FALSE(this->unlinked(shared_b.file_path)) << "B still references it";
  const auto b_rows = this->storage_->get_rosbag_files("B");
  ASSERT_EQ(b_rows.size(), 1u);
  EXPECT_EQ(b_rows[0].file_path, shared_b.file_path);
}

// Re-store guard: a fault re-confirming stores a row with a new path, and the
// old bag must be unlinked only when no sibling fault still references it.

TYPED_TEST(RosbagRetentionParityTest, RestoreWithNewPathUnlinksTheOldExclusiveBag) {
  this->use_retention(RosbagRetentionLimits{});
  const auto old_bag = this->row("X", "exclusive_bag", 1000);
  this->store({old_bag});

  auto info = old_bag;
  info.file_path = this->make_bag("exclusive_bag_new");
  info.recording_id = rosbag_recording_id(info.file_path);
  this->store({info});

  EXPECT_TRUE(this->unlinked(old_bag.file_path)) << "nobody references the old bag, it must be unlinked";
  auto row = this->storage_->get_rosbag_file("X");
  ASSERT_TRUE(row.has_value());
  EXPECT_EQ(row->file_path, info.file_path);
}

TYPED_TEST(RosbagRetentionParityTest, RestoreWithNewPathKeepsTheBagASiblingStillReferences) {
  this->use_retention(RosbagRetentionLimits{});
  auto info = this->row("X", "shared_bag", 1000);
  const auto shared_path = info.file_path;
  this->store({info});
  info.fault_code = "Y";
  this->store({info});

  info.fault_code = "X";
  info.file_path = this->make_bag("shared_bag_new");
  info.recording_id = rosbag_recording_id(info.file_path);
  this->store({info});

  EXPECT_FALSE(this->unlinked(shared_path)) << "the sibling fault still owns the shared bag";
  auto sibling = this->storage_->get_rosbag_file("Y");
  ASSERT_TRUE(sibling.has_value());
  EXPECT_EQ(sibling->file_path, shared_path);
}

TYPED_TEST(RosbagRetentionParityTest, ABatchStoreUnlinksASharedReplacedBagExactlyOnce) {
  // Two faults of one burst can share the bag being replaced, and whether it is still
  // referenced has to be judged on the finished batch. Judged row by row beforehand,
  // each row sees its sibling still pointing at the old bag and neither unlinks it.
  this->use_retention(RosbagRetentionLimits{});
  auto shared = this->row("SHARED_A", "shared_old_bag", 1000);
  const auto old_bag = shared.file_path;
  this->store({shared});
  shared.fault_code = "SHARED_B";
  this->store({shared});
  ASSERT_FALSE(this->unlinked(old_bag));

  auto moved = shared;
  moved.file_path = this->make_bag("shared_new_bag");
  moved.recording_id = rosbag_recording_id(moved.file_path);
  moved.created_at_ns = 2000;
  std::vector<RosbagFileInfo> rows;
  rows.reserve(2);
  moved.fault_code = "SHARED_A";
  rows.push_back(moved);
  moved.fault_code = "SHARED_B";
  rows.push_back(moved);
  this->store(rows);

  EXPECT_TRUE(this->unlinked(old_bag)) << "the replaced bag leaked: nothing references it any more";
  EXPECT_FALSE(this->unlinked(moved.file_path)) << "the bag the batch points at was removed";
  auto row_a = this->storage_->get_rosbag_file("SHARED_A");
  auto row_b = this->storage_->get_rosbag_file("SHARED_B");
  ASSERT_TRUE(row_a.has_value());
  ASSERT_TRUE(row_b.has_value());
  EXPECT_EQ(row_a->file_path, moved.file_path);
  EXPECT_EQ(row_b->file_path, moved.file_path);
}

// apply_rosbag_changes: the one write primitive. Same body over every backend.

TYPED_TEST(RosbagRetentionParityTest, ApplyTwiceIsIdempotent) {
  const auto a = this->row("TWICE_A", "fault_TWICE_1", 1000);
  const auto b = this->row("TWICE_B", "fault_TWICE_2", 2000);
  seed_rosbag_rows(*this->storage_, {a});
  seed_rosbag_rows(*this->storage_, {b});

  RosbagChangeSet set;
  set.insert.push_back(this->row("TWICE_C", "fault_TWICE_3", 3000));
  set.remove.push_back({"TWICE_A", a.file_path});

  const auto first = this->storage_->apply_rosbag_changes(set);
  EXPECT_EQ(first.orphaned_paths, std::vector<std::string>{a.file_path});
  const auto after_first = this->storage_->get_all_rosbag_files();

  const auto second = this->storage_->apply_rosbag_changes(set);
  EXPECT_TRUE(second.orphaned_paths.empty());
  const auto after_second = this->storage_->get_all_rosbag_files();

  ASSERT_EQ(after_first.size(), 2u);
  ASSERT_EQ(after_second.size(), after_first.size());
  for (size_t i = 0; i < after_first.size(); ++i) {
    EXPECT_EQ(after_second[i].fault_code, after_first[i].fault_code);
    EXPECT_EQ(after_second[i].file_path, after_first[i].file_path);
    EXPECT_EQ(after_second[i].size_bytes, after_first[i].size_bytes);
    EXPECT_EQ(after_second[i].created_at_ns, after_first[i].created_at_ns);
  }
}

TYPED_TEST(RosbagRetentionParityTest, OrphanedPathOnlyWhenLastLinkRemoved) {
  auto shared = this->row("ORPHAN_A", "fault_ORPHAN_1", 1000);
  seed_rosbag_rows(*this->storage_, {shared});
  shared.fault_code = "ORPHAN_B";
  seed_rosbag_rows(*this->storage_, {shared});

  RosbagChangeSet first;
  first.remove.push_back({"ORPHAN_A", shared.file_path});
  EXPECT_TRUE(this->storage_->apply_rosbag_changes(first).orphaned_paths.empty())
      << "ORPHAN_B still references the bag";

  RosbagChangeSet unknown;
  unknown.remove.push_back({"ORPHAN_NONE", shared.file_path});
  unknown.remove.push_back({"ORPHAN_B", "/no/such/bag"});
  EXPECT_TRUE(this->storage_->apply_rosbag_changes(unknown).orphaned_paths.empty())
      << "a link that was not there frees nothing";

  RosbagChangeSet last;
  last.remove.push_back({"ORPHAN_B", shared.file_path});
  last.remove.push_back({"ORPHAN_B", shared.file_path});
  EXPECT_EQ(this->storage_->apply_rosbag_changes(last).orphaned_paths, std::vector<std::string>{shared.file_path})
      << "listed once, however often the set names the link";
}

TYPED_TEST(RosbagRetentionParityTest, ASetThatMovesTheLastLinkToAnotherFaultOrphansNothing) {
  const auto old_link = this->row("MOVE_A", "fault_MOVE_1", 1000);
  seed_rosbag_rows(*this->storage_, {old_link});

  RosbagChangeSet set;
  set.remove.push_back({"MOVE_A", old_link.file_path});
  auto moved = old_link;
  moved.fault_code = "MOVE_B";
  set.insert.push_back(moved);

  EXPECT_TRUE(this->storage_->apply_rosbag_changes(set).orphaned_paths.empty());
  EXPECT_EQ(this->storage_->get_rosbag_files("MOVE_B").size(), 1u);
  EXPECT_TRUE(this->storage_->get_rosbag_files("MOVE_A").empty());
}

TYPED_TEST(RosbagRetentionParityTest, UpsertOnLinkKeepsOneRow) {
  auto info = this->row("APPLY_UPSERT", "fault_APPLY_UPSERT_1", 1000);
  const auto sibling = this->row("APPLY_UPSERT", "fault_APPLY_UPSERT_2", 1000);

  RosbagChangeSet first;
  first.insert.push_back(info);
  first.insert.push_back(sibling);
  this->storage_->apply_rosbag_changes(first);

  info.size_bytes = 4096;
  RosbagChangeSet refresh;
  refresh.insert.push_back(info);
  this->storage_->apply_rosbag_changes(refresh);

  // A refresh replaces every field but the link itself.
  auto full = this->row("APPLY_UPSERT_FULL", "fault_APPLY_UPSERT_3", 3000);
  RosbagChangeSet seed;
  seed.insert.push_back(full);
  this->storage_->apply_rosbag_changes(seed);
  full.recording_id = "renamed_recording";
  full.format = "sqlite3";
  full.duration_sec = 9.5;
  full.size_bytes = 8192;
  full.created_at_ns = 7000;
  RosbagChangeSet replace;
  replace.insert.push_back(full);
  this->storage_->apply_rosbag_changes(replace);
  const auto replaced = this->storage_->get_rosbag_files("APPLY_UPSERT_FULL");
  ASSERT_EQ(replaced.size(), 1u);
  EXPECT_EQ(replaced[0].file_path, full.file_path);
  EXPECT_EQ(replaced[0].recording_id, "renamed_recording");
  EXPECT_EQ(replaced[0].format, "sqlite3");
  EXPECT_DOUBLE_EQ(replaced[0].duration_sec, 9.5);
  EXPECT_EQ(replaced[0].size_bytes, 8192u);
  EXPECT_EQ(replaced[0].created_at_ns, 7000);

  const auto rows = this->storage_->get_rosbag_files("APPLY_UPSERT");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].recording_id, "fault_APPLY_UPSERT_2") << "a refresh keeps the row's place in a tie group";
  EXPECT_EQ(rows[1].recording_id, "fault_APPLY_UPSERT_1");
  EXPECT_EQ(rows[1].size_bytes, 4096u);
}

TYPED_TEST(RosbagRetentionParityTest, RemoveRecordingsRemovesEveryLink) {
  auto burst = this->row("RECS_A", "fault_RECS_1", 1000);
  seed_rosbag_rows(*this->storage_, {burst});
  burst.fault_code = "RECS_B";
  seed_rosbag_rows(*this->storage_, {burst});
  const auto other = this->row("RECS_A", "fault_RECS_2", 2000);
  seed_rosbag_rows(*this->storage_, {other});

  RosbagChangeSet set;
  set.remove_recordings = {"fault_RECS_1", "fault_RECS_1", "fault_RECS_UNKNOWN", ""};
  const auto result = this->storage_->apply_rosbag_changes(set);

  EXPECT_EQ(result.orphaned_paths, std::vector<std::string>{burst.file_path});
  EXPECT_TRUE(this->storage_->get_rosbag_files_by_recording("fault_RECS_1").empty());
  const auto survivors = this->storage_->get_rosbag_files("RECS_A");
  ASSERT_EQ(survivors.size(), 1u);
  EXPECT_EQ(survivors[0].recording_id, "fault_RECS_2");
}

TYPED_TEST(RosbagRetentionParityTest, ApplyNeverUnlinks) {
  const auto a = this->row("KEEP_A", "fault_KEEP_1", 1000);
  const auto b = this->row("KEEP_B", "fault_KEEP_2", 2000);
  seed_rosbag_rows(*this->storage_, {a});
  seed_rosbag_rows(*this->storage_, {b});

  RosbagChangeSet set;
  set.remove.push_back({"KEEP_A", a.file_path});
  set.remove_recordings.push_back("fault_KEEP_2");
  const auto result = this->storage_->apply_rosbag_changes(set);

  EXPECT_EQ(result.orphaned_paths.size(), 2u);
  EXPECT_TRUE(std::filesystem::exists(a.file_path)) << "unlinking is the caller's decision";
  EXPECT_TRUE(std::filesystem::exists(b.file_path));
  EXPECT_TRUE(this->storage_->get_all_rosbag_files().empty());
}

TYPED_TEST(RosbagRetentionParityTest, ApplyAppliesNoCap) {
  RosbagChangeSet set;
  for (int i = 1; i <= 3; ++i) {
    set.insert.push_back(this->row("NOCAP", "fault_NOCAP_" + std::to_string(i), i * 1000));
  }
  const auto result = this->storage_->apply_rosbag_changes(set);

  EXPECT_TRUE(result.orphaned_paths.empty());
  EXPECT_EQ(this->storage_->get_rosbag_files("NOCAP").size(), 3u);
}

TYPED_TEST(RosbagRetentionParityTest, RejectsInsertAndRemoveOfTheSameLink) {
  const auto kept = this->row("REJECT_A", "fault_REJECT_1", 1000);
  seed_rosbag_rows(*this->storage_, {kept});
  const auto before = this->storage_->get_all_rosbag_files();

  auto fresh = this->row("REJECT_B", "fault_REJECT_2", 2000);

  RosbagChangeSet direct;
  direct.remove.push_back({"REJECT_A", kept.file_path});  // would apply on its own
  direct.insert.push_back(fresh);
  direct.remove.push_back({"REJECT_B", fresh.file_path});
  EXPECT_THROW(this->storage_->apply_rosbag_changes(direct), std::invalid_argument);

  RosbagChangeSet by_recording;
  by_recording.remove.push_back({"REJECT_A", kept.file_path});
  by_recording.insert.push_back(fresh);
  by_recording.remove_recordings.push_back("fault_REJECT_2");
  EXPECT_THROW(this->storage_->apply_rosbag_changes(by_recording), std::invalid_argument);

  fresh.recording_id.clear();  // derived from the path, as the store does
  RosbagChangeSet derived;
  derived.insert.push_back(fresh);
  derived.remove_recordings.push_back("fault_REJECT_2");
  EXPECT_THROW(this->storage_->apply_rosbag_changes(derived), std::invalid_argument);

  const auto after = this->storage_->get_all_rosbag_files();
  ASSERT_EQ(after.size(), before.size());
  EXPECT_EQ(after[0].file_path, kept.file_path);
}

/// Atomicity needs a failure in the middle of one transaction, which only the SQL backends can be made to produce.
template <typename Backend>
class RosbagApplyAtomicityTest : public RosbagRetentionParityTest<Backend> {};
TYPED_TEST_SUITE(RosbagApplyAtomicityTest, SqlBackends);

TYPED_TEST(RosbagApplyAtomicityTest, ApplyIsAtomicOnInsertFailure) {
  const auto a = this->row("ATOM_A", "fault_ATOM_1", 1000);
  seed_rosbag_rows(*this->storage_, {a});
  BackendFactory<TypeParam>::add_failing_trigger(this->dir_, "INSERT", "ATOM_FAIL");

  RosbagChangeSet set;
  set.remove.push_back({"ATOM_A", a.file_path});
  set.insert.push_back(this->row("ATOM_OK", "fault_ATOM_2", 2000));
  set.insert.push_back(this->row("ATOM_FAIL", "fault_ATOM_3", 3000));
  EXPECT_THROW(this->storage_->apply_rosbag_changes(set), std::exception);

  const auto rows = this->storage_->get_all_rosbag_files();
  ASSERT_EQ(rows.size(), 1u) << "the removal ran before the failing insert and must have been undone";
  EXPECT_EQ(rows[0].fault_code, "ATOM_A");
  EXPECT_TRUE(this->storage_->get_rosbag_files("ATOM_OK").empty());
}

TYPED_TEST(RosbagApplyAtomicityTest, ApplyIsAtomicOnDeleteFailure) {
  const auto a = this->row("ATOM_A", "fault_ATOM_1", 1000);
  const auto failing = this->row("ATOM_FAIL", "fault_ATOM_2", 2000);
  seed_rosbag_rows(*this->storage_, {a});
  seed_rosbag_rows(*this->storage_, {failing});
  BackendFactory<TypeParam>::add_failing_trigger(this->dir_, "DELETE", "ATOM_FAIL");

  RosbagChangeSet set;
  set.remove.push_back({"ATOM_A", a.file_path});
  set.remove.push_back({"ATOM_FAIL", failing.file_path});
  set.insert.push_back(this->row("ATOM_OK", "fault_ATOM_3", 3000));
  EXPECT_THROW(this->storage_->apply_rosbag_changes(set), std::exception);

  const auto rows = this->storage_->get_all_rosbag_files();
  ASSERT_EQ(rows.size(), 2u) << "the first removal ran before the failing one and must have been undone";
  EXPECT_EQ(rows[0].fault_code, "ATOM_A");
  EXPECT_EQ(rows[1].fault_code, "ATOM_FAIL");
  EXPECT_TRUE(this->storage_->get_rosbag_files("ATOM_OK").empty());
}

/// Snapshot retention parity. Same two backends, same reason.
template <typename Backend>
class SnapshotRetentionParityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("medkit_snap_parity_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    storage_ = BackendFactory<Backend>::make(dir_);
  }

  void TearDown() override {
    storage_.reset();
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  /// One capture: `topics` rows sharing a capture_id, as SnapshotCapture writes them.
  std::vector<SnapshotData> capture(const std::string & code, int64_t capture_id, size_t topics) {
    std::vector<SnapshotData> rows;
    for (size_t i = 0; i < topics; ++i) {
      SnapshotData row;
      row.fault_code = code;
      row.topic = "/topic_" + std::to_string(i);
      row.message_type = "std_msgs/msg/Float64";
      row.data = R"({"data": )" + std::to_string(capture_id) + "}";
      row.captured_at_ns = capture_id * 1000 + static_cast<int64_t>(i);
      row.capture_id = capture_id;
      rows.push_back(row);
    }
    return rows;
  }

  void confirm(const std::string & code) {
    rclcpp::Clock clock;
    storage_->report_fault_event(code, fd::ReportKind::kFailed, ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR, "parity",
                                 "/node", clock.now().nanoseconds(), ros2_medkit_fault_manager::DebounceConfig{});
  }

  std::filesystem::path dir_;
  std::unique_ptr<FaultStorage> storage_;
};

using SnapshotBackends = ::testing::Types<InMemoryFaultStorage, SqliteFaultStorage>;
TYPED_TEST_SUITE(SnapshotRetentionParityTest, SnapshotBackends);

TYPED_TEST(SnapshotRetentionParityTest, ACaptureLargerThanTheCapIsKeptWholeRatherThanTorn) {
  // The in-memory trim used to evict the newest set too, leaving the fault with no
  // readings at all, while SQLite exempted it. `storage_type: memory` with the
  // default cap of 10 and eleven configured topics lost every freeze frame.
  this->storage_->set_max_snapshots_per_fault(3);
  this->confirm("BIG");

  this->storage_->store_snapshots(this->capture("BIG", 1, 5));

  const auto rows = this->storage_->get_snapshots("BIG");
  EXPECT_EQ(rows.size(), 5u) << "a capture over the cap on its own is kept entire";
  for (const auto & row : rows) {
    EXPECT_EQ(row.capture_id, 1);
  }
}

TYPED_TEST(SnapshotRetentionParityTest, TheOldestWholeCaptureGoesFirstPastTheCap) {
  this->storage_->set_max_snapshots_per_fault(4);
  this->confirm("EVICT");

  this->storage_->store_snapshots(this->capture("EVICT", 1, 2));
  this->storage_->store_snapshots(this->capture("EVICT", 2, 2));
  this->storage_->store_snapshots(this->capture("EVICT", 3, 2));

  const auto rows = this->storage_->get_snapshots("EVICT");
  ASSERT_EQ(rows.size(), 4u);
  for (const auto & row : rows) {
    EXPECT_NE(row.capture_id, 1) << "the oldest set goes whole, not row by row";
  }
}

TYPED_TEST(SnapshotRetentionParityTest, GetSnapshotsReturnsTheNewestCaptureFirst) {
  // The reader folds rows into a topic map and keeps the leading capture, so the
  // order decides which values a client sees. In-memory returned insertion order,
  // which is the oldest set first - the exact opposite of SQLite.
  this->storage_->set_max_snapshots_per_fault(0);
  this->confirm("ORDER");

  this->storage_->store_snapshots(this->capture("ORDER", 1, 2));
  this->storage_->store_snapshots(this->capture("ORDER", 2, 2));

  const auto rows = this->storage_->get_snapshots("ORDER");
  ASSERT_EQ(rows.size(), 4u);
  EXPECT_EQ(rows.front().capture_id, 2) << "newest capture leads";
  EXPECT_EQ(rows.back().capture_id, 1);
}

TYPED_TEST(SnapshotRetentionParityTest, MaxCaptureIdIsWhatARestartMustContinueFrom) {
  // SnapshotCapture seeds its counter from this. A backend answering 0 with rows
  // present would hand the next capture an id below the stored ones, and eviction
  // protects the highest id - so the capture just written would be the one dropped.
  this->storage_->set_max_snapshots_per_fault(0);
  EXPECT_EQ(this->storage_->get_max_capture_id(), 0) << "nothing stored yet";

  this->confirm("SEED_A");
  this->confirm("SEED_B");
  this->storage_->store_snapshots(this->capture("SEED_A", 4, 1));
  this->storage_->store_snapshots(this->capture("SEED_B", 7, 1));

  EXPECT_EQ(this->storage_->get_max_capture_id(), 7) << "global, not per fault";
}

TYPED_TEST(SnapshotRetentionParityTest, ClearFaultKeepsSnapshotsWhenEvidenceIsRetained) {
  this->storage_->set_max_snapshots_per_fault(0);
  this->confirm("KEEP");
  this->storage_->set_retain_snapshots_on_clear(true);
  this->storage_->store_snapshots(this->capture("KEEP", 1, 2));

  ASSERT_TRUE(this->storage_->clear_fault("KEEP"));

  EXPECT_EQ(this->storage_->get_snapshots("KEEP").size(), 2u);
  EXPECT_TRUE(this->storage_->retains_snapshots_on_clear());
}

TYPED_TEST(SnapshotRetentionParityTest, ClearFaultDropsSnapshotsByDefault) {
  this->storage_->set_max_snapshots_per_fault(0);
  this->confirm("DROP");
  this->storage_->store_snapshots(this->capture("DROP", 1, 2));

  ASSERT_TRUE(this->storage_->clear_fault("DROP"));

  EXPECT_TRUE(this->storage_->get_snapshots("DROP").empty());
  EXPECT_FALSE(this->storage_->retains_snapshots_on_clear());
}

}  // namespace
