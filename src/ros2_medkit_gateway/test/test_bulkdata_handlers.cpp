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

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/bulk_data_store.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

using namespace ros2_medkit_gateway;
// No `using json = nlohmann::json` here: the namespace pulled in above already
// declares that alias, and redeclaring it shadows it.
using ros2_medkit_gateway::handlers::BulkDataHandlers;

class BulkDataHandlersTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }
  void TearDown() override {
  }
};

// === MIME type tests ===

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeMcap) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("mcap"), "application/x-mcap");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeSqlite3) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("sqlite3"), "application/x-sqlite3");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeDb3) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("db3"), "application/x-sqlite3");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeUnknown) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("unknown"), "application/octet-stream");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeEmpty) {
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype(""), "application/octet-stream");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, GetRosbagMimetypeCasesSensitive) {
  // MCAP should not match mcap (case sensitive)
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("MCAP"), "application/octet-stream");
  EXPECT_EQ(BulkDataHandlers::get_rosbag_mimetype("Mcap"), "application/octet-stream");
}

// === Shared-recording identifier tests ===
// The bag directory basename is the recording's public name: it addresses the
// bag under /bulk-data/rosbags/{id} and groups the link rows serving the same
// bytes.

TEST_F(BulkDataHandlersTest, RecordingIdIsTheBagDirectoryBasename) {
  EXPECT_EQ(handlers::detail::rosbag_recording_id("/var/bags/fault_MOTOR_OVERHEAT_1738664999000"),
            "fault_MOTOR_OVERHEAT_1738664999000");
}

TEST_F(BulkDataHandlersTest, RecordingIdIsTheSameForEveryFaultOfTheBurst) {
  // Rows of one burst carry different fault codes but the same path: their
  // descriptors must group under one id, distinct from any other bag's.
  const std::string burst_bag = "/tmp/fault_ROOT_CAUSE_1700000000000";
  const std::string other_bag = "/tmp/fault_UNRELATED_1700000000042";
  EXPECT_EQ(handlers::detail::rosbag_recording_id(burst_bag), "fault_ROOT_CAUSE_1700000000000");
  EXPECT_NE(handlers::detail::rosbag_recording_id(burst_bag), handlers::detail::rosbag_recording_id(other_bag));
}

TEST_F(BulkDataHandlersTest, RecordingIdToleratesTrailingSlashAndEmptyPath) {
  EXPECT_EQ(handlers::detail::rosbag_recording_id("/var/bags/fault_X_123/"), "fault_X_123");
  EXPECT_EQ(handlers::detail::rosbag_recording_id(""), "");
}

// === Descriptor folding tests ===
// The fault manager returns one row per (fault, recording) link. A burst of
// correlated faults is several rows naming one bag, and one fault holding a
// history is several rows with distinct bags. Both shapes have to come out as
// one descriptor per recording.

namespace {

json rosbag_row(const std::string & fault_code, const std::string & recording_id, uint64_t size_bytes = 1024) {
  return json{{"fault_code", fault_code}, {"recording_id", recording_id}, {"file_path", "/var/bags/" + recording_id},
              {"format", "mcap"},         {"duration_sec", 5.0},          {"size_bytes", size_bytes}};
}

json fault_at(double first_occurred) {
  return json{{"first_occurred", first_occurred}};
}

/// A row carrying the recording's own timestamp, which is what the fault manager
/// sends now.
json rosbag_row_made_at(const std::string & fault_code, const std::string & recording_id, int64_t created_at_ns) {
  json row = rosbag_row(fault_code, recording_id);
  row["created_at_ns"] = created_at_ns;
  return row;
}

}  // namespace

TEST_F(BulkDataHandlersTest, OneFaultWithSeveralRecordingsYieldsOneDescriptorEach) {
  // The feature: a flapping fault keeps a history, and every recording in it
  // has to be separately addressable.
  const std::vector<json> rows{rosbag_row("FLAP", "fault_FLAP_3"), rosbag_row("FLAP", "fault_FLAP_2"),
                               rosbag_row("FLAP", "fault_FLAP_1")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 3u);
  EXPECT_EQ(descriptors[0].id, "fault_FLAP_3") << "order follows the fault manager's listing";
  EXPECT_EQ(descriptors[1].id, "fault_FLAP_2");
  EXPECT_EQ(descriptors[2].id, "fault_FLAP_1");
}

TEST_F(BulkDataHandlersTest, ABurstCollapsesToOneDescriptorCarryingEveryFault) {
  // Three rows, one bag. Emitting three items would repeat the id and report
  // the bag's size three times, which reads as three bags worth of storage.
  const std::vector<json> rows{rosbag_row("ROOT_CAUSE", "fault_ROOT_CAUSE_17"),
                               rosbag_row("DOWNSTREAM_B", "fault_ROOT_CAUSE_17"),
                               rosbag_row("DOWNSTREAM_A", "fault_ROOT_CAUSE_17")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].id, "fault_ROOT_CAUSE_17");
  EXPECT_EQ(descriptors[0].size, 1024u) << "the bag is counted once, not once per attached fault";

  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  const auto & x = *descriptors[0].x_medkit;
  ASSERT_TRUE(x.contains("fault_codes"));
  EXPECT_EQ(x["fault_codes"], (json{"DOWNSTREAM_A", "DOWNSTREAM_B", "ROOT_CAUSE"})) << "sorted, so output is stable";
  EXPECT_EQ(x["recording_id"], "fault_ROOT_CAUSE_17");
  EXPECT_EQ(x["format"], "mcap");
  EXPECT_DOUBLE_EQ(x["duration_sec"].get<double>(), 5.0);
}

TEST_F(BulkDataHandlersTest, TheSameFaultTwiceOnOneRecordingIsNotListedTwice) {
  // Two source filters can both resolve to the same app, so the same row can
  // arrive twice. A repeated code in fault_codes would be visible in the API.
  const std::vector<json> rows{rosbag_row("DUP", "fault_DUP_1"), rosbag_row("DUP", "fault_DUP_1")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  EXPECT_EQ((*descriptors[0].x_medkit)["fault_codes"], (json{"DUP"}));
}

TEST_F(BulkDataHandlersTest, ARecordingIsDatedByTheEarliestFaultOfItsBurst) {
  // Downstream faults confirm after the root cause, and the recording covers
  // the whole burst, so the earliest is the honest creation date.
  const std::vector<json> rows{rosbag_row("DOWNSTREAM", "fault_ROOT_9"), rosbag_row("ROOT", "fault_ROOT_9")};
  const std::unordered_map<std::string, json> faults{{"DOWNSTREAM", fault_at(1700000900.0)},
                                                     {"ROOT", fault_at(1700000000.0)}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, faults);
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].creation_date, format_timestamp_ns(int64_t{1700000000} * 1'000'000'000));
}

TEST_F(BulkDataHandlersTest, EachRecordingOfOneFaultIsDatedByItsOwnCapture) {
  // Dating a recording by its fault gave every recording of that fault the same
  // date - and holding more than one is the whole point of the change, so the date
  // is exactly what tells the occurrences apart.
  const std::vector<json> rows{rosbag_row_made_at("FLAP", "fault_FLAP_2", int64_t{1700000900} * 1'000'000'000),
                               rosbag_row_made_at("FLAP", "fault_FLAP_1", int64_t{1700000000} * 1'000'000'000)};
  const std::unordered_map<std::string, json> faults{{"FLAP", fault_at(1700000000.0)}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, faults);
  ASSERT_EQ(descriptors.size(), 2u);
  EXPECT_EQ(descriptors[0].creation_date, format_timestamp_ns(int64_t{1700000900} * 1'000'000'000));
  EXPECT_EQ(descriptors[1].creation_date, format_timestamp_ns(int64_t{1700000000} * 1'000'000'000));
  EXPECT_NE(descriptors[0].creation_date, descriptors[1].creation_date);
}

TEST_F(BulkDataHandlersTest, AnAcknowledgedFaultsRecordingsKeepTheirRealDate) {
  // list_faults excludes cleared faults by default, so an acknowledged fault is
  // absent from the map while its rows are still listed. Reading the date off the
  // fault dated those recordings 1970 - and this change is what makes an
  // acknowledged fault keep them in the first place.
  const std::vector<json> rows{rosbag_row_made_at("ACKED", "fault_ACKED_1", int64_t{1700000500} * 1'000'000'000)};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].creation_date, format_timestamp_ns(int64_t{1700000500} * 1'000'000'000));
  EXPECT_EQ(descriptors[0].creation_date.rfind("1970", 0), std::string::npos) << "not the epoch";
}

TEST_F(BulkDataHandlersTest, DescriptorIdFallsBackToTheBasenameWhenTheRowHasNoRecordingId) {
  // A peer or a replay predating the stored field still has to be addressable.
  const std::vector<json> rows{json{{"fault_code", "OLD"}, {"file_path", "/var/bags/fault_OLD_5"}, {"format", "mcap"}}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].id, "fault_OLD_5");
}

TEST_F(BulkDataHandlersTest, ARowWithNeitherIdNorPathIsDroppedRatherThanAdvertised) {
  // An empty id would render as /bulk-data/rosbags/ - a 404 the client cannot
  // act on. Better absent than advertised and broken.
  const std::vector<json> rows{json{{"fault_code", "GHOST"}, {"format", "mcap"}}, rosbag_row("REAL", "fault_REAL_1")};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].id, "fault_REAL_1");
}

TEST_F(BulkDataHandlersTest, DistinctRecordingsEachReportTheirOwnSize) {
  // The paths in these rows do not exist on this host, so each descriptor keeps
  // the row's own figure - the fallback the sizing test below covers explicitly.
  const std::vector<json> rows{rosbag_row("A", "fault_A_1", 2048), rosbag_row("B", "fault_B_1", 4096)};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 2u);
  EXPECT_EQ(descriptors[0].size, 2048u);
  EXPECT_EQ(descriptors[1].size, 4096u);
}

TEST_F(BulkDataHandlersTest, NoRowsYieldsNoDescriptors) {
  EXPECT_TRUE(handlers::detail::fold_rosbag_rows_into_descriptors({}, {}).empty());
}

// === Descriptor size vs served bytes ===
// A rosbag2 bag is a directory: one storage file plus metadata.yaml. The
// download resolves the storage file and streams that alone, so the descriptor
// has to be sized on the same file. The fault manager's stored figure is the
// directory total, which is the recording's disk footprint and larger than the
// transfer. Reporting it made every listing overstate the download.

class RosbagBagDirectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    bag_dir_ = std::filesystem::temp_directory_path() /
               ("bulkdata_bag_test_" + std::to_string(getpid()) + "_" + std::to_string(counter_++));
    std::filesystem::create_directories(bag_dir_);
    write_file(bag_dir_ / "recording_0.db3", std::string(4096, 'x'));
    write_metadata(bag_dir_, {"recording_0.db3"});
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(bag_dir_, ec);
  }

  static void write_file(const std::filesystem::path & path, const std::string & content) {
    std::ofstream out(path, std::ios::binary);
    out << content;
  }

  /// A ``metadata.yaml`` in the shape rosbag2 writes, naming @p storage_files in
  /// ``relative_file_paths``. Only the fields this code reads are filled in, but
  /// the nesting is the real one: the helper looks up
  /// ``rosbag2_bagfile_information.relative_file_paths``, so a flat document
  /// would pass a test that production data fails.
  ///
  /// An empty @p storage_files is written as the flow-style ``[]``, which is an
  /// empty sequence. A block sequence with no items under it is a YAML *null*
  /// instead, and a null is refused one step earlier than an empty list is, by
  /// the shape check rather than by the count, so a test built on one passes
  /// without ever reaching the rule it names.
  static void write_metadata(const std::filesystem::path & dir, const std::vector<std::string> & storage_files) {
    std::string yaml =
        "rosbag2_bagfile_information:\n"
        "  version: 9\n"
        "  storage_identifier: sqlite3\n"
        "  message_count: 0\n";
    if (storage_files.empty()) {
      yaml += "  relative_file_paths: []\n";
    } else {
      yaml += "  relative_file_paths:\n";
      for (const auto & file : storage_files) {
        yaml += "    - " + file + "\n";
      }
    }
    yaml += "  ros_distro: jazzy\n";
    write_file(dir / "metadata.yaml", yaml);
  }

  /// The storage file the directory-order fallback inside
  /// ``resolve_rosbag_file_path`` reaches first: the first regular ``.db3`` or
  /// ``.mcap`` the directory yields. Empty when the directory holds none.
  ///
  /// The split tests use this to choose a metadata order that cannot coincide
  /// with the directory's. Which file a directory yields first is the
  /// filesystem's own business - it is neither creation order nor lexical order
  /// on the overlay these tests run on - so a test that writes down an expected
  /// answer and hopes it differs from directory order proves nothing on the run
  /// where the two agree. Asking at run time makes "the bag's own order decides"
  /// falsifiable everywhere.
  static std::filesystem::path first_in_directory_order(const std::filesystem::path & dir) {
    for (const auto & entry : std::filesystem::directory_iterator(dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto ext = entry.path().extension().string();
      if (ext == ".db3" || ext == ".mcap") {
        return entry.path();
      }
    }
    return {};
  }

  // What the fault manager stores: every regular file under the bag directory.
  uint64_t directory_total() const {
    uint64_t total = 0;
    for (const auto & entry : std::filesystem::recursive_directory_iterator(bag_dir_)) {
      if (entry.is_regular_file()) {
        total += entry.file_size();
      }
    }
    return total;
  }

  std::filesystem::path bag_dir_;
  static int counter_;
};

int RosbagBagDirectoryTest::counter_ = 0;

TEST_F(RosbagBagDirectoryTest, DescriptorSizeIsTheBytesTheDownloadServesNotTheBagDirectoryTotal) {
  // The two operations download() performs to fill Content-Length: resolve the
  // bag directory to its storage file, then take that file's size.
  const std::string served_path = BulkDataHandlers::resolve_rosbag_file_path(bag_dir_.string());
  ASSERT_EQ(served_path, (bag_dir_ / "recording_0.db3").string());
  const uint64_t served_bytes = std::filesystem::file_size(served_path);

  // Not vacuous: the directory holds metadata.yaml as well, so the stored figure
  // and the served figure are genuinely different numbers.
  ASSERT_GT(directory_total(), served_bytes);

  // The row carries the directory total, which is what the fault manager stores.
  const json row{{"fault_code", "MOTOR_OVERHEAT"},
                 {"recording_id", bag_dir_.filename().string()},
                 {"file_path", bag_dir_.string()},
                 {"format", "sqlite3"},
                 {"duration_sec", 6.0},
                 {"size_bytes", directory_total()}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].size, served_bytes) << "the listing must promise the bytes the download sends";
  EXPECT_NE(descriptors[0].size, directory_total()) << "metadata.yaml is not served, so it must not be counted";
}

TEST_F(RosbagBagDirectoryTest, ServedBytesIsUnknownRatherThanZeroWhenTheBagIsNotVisible) {
  // Positive control for the absence below: the same helper does answer for a
  // bag it can see, so a nullopt is the missing bag and not a broken helper.
  ASSERT_TRUE(handlers::detail::rosbag_served_bytes(bag_dir_.string()).has_value());

  EXPECT_FALSE(handlers::detail::rosbag_served_bytes("").has_value());
  EXPECT_FALSE(handlers::detail::rosbag_served_bytes((bag_dir_ / "no_such_bag").string()).has_value());

  // A directory with no metadata.yaml and no storage file in it. The metadata
  // gate is what declines here, before the resolver is reached: the bag does not
  // say how many storage files it holds, so this side will not guess one. The
  // resolver would also find nothing, but that is no longer what the test turns
  // on.
  const auto empty_bag = bag_dir_ / "empty_bag";
  std::filesystem::create_directories(empty_bag);
  EXPECT_FALSE(handlers::detail::rosbag_served_bytes(empty_bag.string()).has_value());
}

TEST_F(RosbagBagDirectoryTest, ASplitRecordingIsListedAtTheRowsFigureNotAtOneSegment) {
  // Past the configured maximum bag size rosbag2 splits a recording across
  // several storage files. The download route hands over whichever one the
  // resolver reaches first, so no single file is the transfer, and sizing the
  // descriptor by that file advertised a split recording at the size of one part
  // of it. The fault manager reports the recording's total for a split, and the
  // two API surfaces have to agree on one recording.
  const auto split_dir = bag_dir_ / "split";
  std::filesystem::create_directories(split_dir);
  write_file(split_dir / "split_0.db3", std::string(16384, 'a'));
  write_file(split_dir / "split_1.db3", std::string(53248, 'b'));
  write_metadata(split_dir, {"split_0.db3", "split_1.db3"});

  uint64_t split_total = 0;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(split_dir)) {
    if (entry.is_regular_file()) {
      split_total += entry.file_size();
    }
  }
  const uint64_t first_segment = std::filesystem::file_size(split_dir / "split_0.db3");
  const uint64_t second_segment = std::filesystem::file_size(split_dir / "split_1.db3");

  // The row carries what the fault manager reports for a split: the total.
  const json row{{"fault_code", "SPLIT_FAULT"},
                 {"recording_id", "fault_SPLIT_FAULT_1738664999000"},
                 {"file_path", split_dir.string()},
                 {"format", "sqlite3"},
                 {"duration_sec", 6.0},
                 {"size_bytes", split_total}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].size, split_total) << "a split recording keeps the figure the fault manager reported";
  EXPECT_NE(descriptors[0].size, first_segment) << "one segment is not the recording";
  EXPECT_NE(descriptors[0].size, second_segment) << "and neither is the other";

  // The helper declines rather than guessing, which is what makes the fallback fire.
  EXPECT_FALSE(handlers::detail::rosbag_served_bytes(split_dir.string()).has_value());
}

TEST_F(RosbagBagDirectoryTest, ABareStorageFileIsItsOwnRecordingAndIsSizedAsSuch) {
  // A row's file_path can be the storage file itself rather than a bag
  // directory. The resolver has always accepted that and the download serves it,
  // so the listing has to size it too. A bare file has no metadata.yaml beside it
  // under that name, so consulting the metadata first made the helper decline
  // every such row - harmless while the row carries a figure to fall back on, and
  // a recording listed at zero the moment one does not.
  const auto bare_file = bag_dir_ / "standalone_recording.db3";
  write_file(bare_file, std::string(7168, 'z'));
  const uint64_t bare_size = std::filesystem::file_size(bare_file);

  EXPECT_EQ(handlers::detail::rosbag_served_bytes(bare_file.string()), bare_size);

  const json row{{"fault_code", "BARE_FILE_FAULT"},
                 {"recording_id", "standalone_recording.db3"},
                 {"file_path", bare_file.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 1}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].size, bare_size) << "a bare storage file is measured, not declined";
  EXPECT_NE(descriptors[0].size, 1u) << "and the row's figure is not what was reported";
}

TEST_F(RosbagBagDirectoryTest, TheMetadataNamesTheStorageFileRatherThanDirectoryOrder) {
  // A stray .db3 beside the recording - a leftover segment, a copy - used to be
  // servable and sizeable in place of the real one, because the resolver took
  // whichever file the directory iterator yielded first. The fault manager sizes
  // relative_file_paths.front() (rosbag_capture.cpp, rosbag_served_bytes), so the
  // two sides reported different numbers for the same directory. The bag's own
  // metadata is the tie-break on both sides now.
  const auto strays = bag_dir_ / "with_stray";
  std::filesystem::create_directories(strays);
  write_file(strays / "recording_0.db3", std::string(4096, 'a'));
  write_file(strays / "recording_1.db3", std::string(65536, 'b'));
  write_metadata(strays, {"recording_0.db3"});

  const uint64_t named_size = std::filesystem::file_size(strays / "recording_0.db3");
  const uint64_t stray_size = std::filesystem::file_size(strays / "recording_1.db3");
  ASSERT_NE(named_size, stray_size) << "the two files are the same size, so nothing is being told apart";

  // The download resolves the named file, so the bytes on the wire are its bytes.
  EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(strays.string()), (strays / "recording_0.db3").string());
  EXPECT_EQ(handlers::detail::rosbag_served_bytes(strays.string()), named_size);

  const json row{{"fault_code", "STRAY_FAULT"},
                 {"recording_id", "with_stray"},
                 {"file_path", strays.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 999999}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  // 4096 is also what the fault manager's own helper answers for this directory
  // shape, which is the point of reading the same field on both sides. Its
  // behaviour is pinned by ReportsTheStorageFileNotTheDirectoryTotal in
  // test_rosbag_capture.cpp.
  EXPECT_EQ(descriptors[0].size, named_size) << "the listing must report the file the bag names";
  EXPECT_EQ(descriptors[0].size, 4096u) << "and that is the number the fault manager reports too";
  EXPECT_NE(descriptors[0].size, stray_size) << "directory order must not decide which file a recording is";
}

// === A split recording: which segment is downloaded, and how many there are ===
// Past the configured maximum bag size rosbag2 splits a recording across several
// storage files. The download hands over one of them, and that used to be
// whichever the directory iterator yielded first - a segment from the middle of
// the recording as readily as its start, decided by nothing a client could see
// or predict. The recording's own metadata.yaml lists its segments in capture
// order, so the first name in it is where the recording starts, and that is the
// file to hand over. The count goes into the descriptor because a client holding
// one segment has no other way to learn that more of the recording exists.

TEST_F(RosbagBagDirectoryTest, ASplitRecordingResolvesToTheFirstSegmentTheMetadataNames) {
  // Three segments, and the one the metadata names first is chosen so that
  // neither of the rules this one replaces can reach it: it is not the file the
  // directory yields first, and it is not the lexically smallest. So a resolver
  // that walks the directory, and a resolver that sorts, both have to fail here.
  const auto split_dir = bag_dir_ / "split_ordered";
  std::filesystem::create_directories(split_dir);
  const std::vector<std::filesystem::path> segments{split_dir / "recording_0.db3", split_dir / "recording_1.db3",
                                                    split_dir / "recording_2.db3"};
  write_file(segments[0], std::string(16384, 'a'));
  write_file(segments[1], std::string(53248, 'b'));
  write_file(segments[2], std::string(32768, 'c'));
  // Written before the order is read, and rewritten in place afterwards, so
  // adding metadata.yaml cannot move the entries the answer was read from.
  write_metadata(split_dir, {"recording_0.db3"});

  const auto directory_first = first_in_directory_order(split_dir);
  ASSERT_FALSE(directory_first.empty()) << "no storage file in the directory, so nothing is being told apart";
  const auto & lexically_first = segments.front();

  std::filesystem::path metadata_first;
  for (const auto & segment : segments) {
    if (segment != directory_first && segment != lexically_first) {
      metadata_first = segment;
      break;
    }
  }
  ASSERT_FALSE(metadata_first.empty()) << "three segments always leave one that is neither";

  std::vector<std::string> names{metadata_first.filename().string()};
  for (const auto & segment : segments) {
    if (segment != metadata_first) {
      names.push_back(segment.filename().string());
    }
  }
  write_metadata(split_dir, names);

  EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(split_dir.string()), metadata_first.string())
      << "the download must hand over the segment the recording names first, not the one the directory offers";

  uint64_t split_total = 0;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(split_dir)) {
    if (entry.is_regular_file()) {
      split_total += entry.file_size();
    }
  }
  const uint64_t first_named = std::filesystem::file_size(metadata_first);

  // The row carries what the fault manager reports for a split: the total.
  const json row{{"fault_code", "SPLIT_FAULT"},
                 {"recording_id", "fault_SPLIT_FAULT_1738664999000"},
                 {"file_path", split_dir.string()},
                 {"format", "sqlite3"},
                 {"duration_sec", 6.0},
                 {"size_bytes", split_total}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  ASSERT_TRUE(descriptors[0].x_medkit->contains("storage_files")) << "a split recording must say how many files it has";
  EXPECT_EQ((*descriptors[0].x_medkit)["storage_files"], 3) << "the count is what the bag's own metadata names";
  EXPECT_EQ(descriptors[0].size, split_total) << "a split recording is still listed at the whole recording's size";
  EXPECT_NE(descriptors[0].size, first_named) << "the segment on the wire is not the recording";
}

TEST_F(RosbagBagDirectoryTest, ASplitRecordingSkipsAFirstSegmentThatIsNoLongerOnDisk) {
  // Quota eviction and a half-copied bag both leave metadata naming a file that
  // is gone. Resolving to a path that does not exist answers 500 for a recording
  // whose remaining segments are readable, so the first name that IS on disk is
  // served instead. The survivor named first is again the one the directory does
  // not yield first, so directory order cannot produce this answer either.
  const auto split_dir = bag_dir_ / "split_first_gone";
  std::filesystem::create_directories(split_dir);
  const std::filesystem::path evicted = split_dir / "recording_0.db3";
  write_file(split_dir / "recording_1.db3", std::string(24576, 'c'));
  write_file(split_dir / "recording_2.db3", std::string(40960, 'd'));
  write_metadata(split_dir, {"recording_1.db3"});
  ASSERT_FALSE(std::filesystem::exists(evicted)) << "the first segment has to be missing";

  const auto directory_first = first_in_directory_order(split_dir);
  ASSERT_FALSE(directory_first.empty());
  const std::filesystem::path survivor =
      directory_first == split_dir / "recording_1.db3" ? split_dir / "recording_2.db3" : split_dir / "recording_1.db3";

  write_metadata(split_dir,
                 {evicted.filename().string(), survivor.filename().string(), directory_first.filename().string()});

  EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(split_dir.string()), survivor.string())
      << "a named segment that is not on disk cannot be the one served, and the directory does not get the vote";

  const json row{{"fault_code", "SPLIT_FAULT"},
                 {"recording_id", "fault_SPLIT_FAULT_1738664999001"},
                 {"file_path", split_dir.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 99999}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  ASSERT_TRUE(descriptors[0].x_medkit->contains("storage_files"));
  EXPECT_EQ((*descriptors[0].x_medkit)["storage_files"], 3)
      << "the count is what the recording holds, not what survived on disk";
}

TEST_F(RosbagBagDirectoryTest, ASplitRecordingWithNoSegmentLeftResolvesToNothing) {
  // Nothing to hand over, and the download route turns an empty resolution into
  // its scoped error. The count still answers, because the metadata is readable
  // and it is the recording's own record of what it held.
  const auto split_dir = bag_dir_ / "split_all_gone";
  std::filesystem::create_directories(split_dir);
  write_metadata(split_dir, {"recording_0.db3", "recording_1.db3"});

  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(split_dir.string()), ""); });

  const json row{{"fault_code", "SPLIT_FAULT"},
                 {"recording_id", "fault_SPLIT_FAULT_1738664999002"},
                 {"file_path", split_dir.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 4242}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  ASSERT_TRUE(descriptors[0].x_medkit->contains("storage_files"));
  EXPECT_EQ((*descriptors[0].x_medkit)["storage_files"], 2);
  EXPECT_EQ(descriptors[0].size, 4242u) << "an unmeasurable recording keeps the figure its row carried";
}

// === A readable metadata is the whole answer, the directory is not consulted ===
// The directory walk exists for a bag that will not say what it holds. Reaching
// it after the bag HAS said, because none of the names it gave is on disk, put a
// file the recording never named on the wire under that recording's id and under
// a storage_files the served file is not one of. What a client received was then
// neither the recording nor an error, and nothing in the response said so.

TEST_F(RosbagBagDirectoryTest, ASplitRecordingWithNoSegmentLeftDoesNotServeAStrayBesideIt) {
  // Several names, none on disk, a stray storage file beside them. The stray is
  // a .db3, so the directory walk would reach it, and it is not one of the two
  // files the metadata names.
  const auto split_dir = bag_dir_ / "split_gone_with_stray";
  std::filesystem::create_directories(split_dir);
  write_file(split_dir / "stray.db3", std::string(8192, 's'));
  write_metadata(split_dir, {"recording_0.db3", "recording_1.db3"});
  ASSERT_FALSE(std::filesystem::exists(split_dir / "recording_0.db3"));
  ASSERT_FALSE(std::filesystem::exists(split_dir / "recording_1.db3"));
  ASSERT_EQ(first_in_directory_order(split_dir), split_dir / "stray.db3")
      << "the directory walk cannot reach the stray, so nothing is being told apart";

  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(split_dir.string()), ""); })
      << "a file the recording never named must not be served in its place";

  const json row{{"fault_code", "SPLIT_FAULT"},
                 {"recording_id", "fault_SPLIT_FAULT_1738664999004"},
                 {"file_path", split_dir.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 4242}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  ASSERT_TRUE(descriptors[0].x_medkit->contains("storage_files"));
  EXPECT_EQ((*descriptors[0].x_medkit)["storage_files"], 2) << "the count is still what the metadata names";
  EXPECT_EQ(descriptors[0].size, 4242u) << "and the row's figure is kept, not the stray's size";
}

TEST_F(RosbagBagDirectoryTest, AWholeRecordingWhoseOnlyFileIsGoneDoesNotServeAStrayBesideIt) {
  // One name, not on disk, a stray storage file beside it. The same rule as the
  // several-name case above, on the shape that reaches a user first, because a
  // recording held in one file is the normal case.
  const auto gone_dir = bag_dir_ / "single_gone_with_stray";
  std::filesystem::create_directories(gone_dir);
  write_file(gone_dir / "stray.db3", std::string(8192, 's'));
  write_metadata(gone_dir, {"recording_0.db3"});
  ASSERT_FALSE(std::filesystem::exists(gone_dir / "recording_0.db3"));
  ASSERT_EQ(first_in_directory_order(gone_dir), gone_dir / "stray.db3");

  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(gone_dir.string()), ""); })
      << "a file the recording never named must not be served in its place";

  const json row{{"fault_code", "GONE_FAULT"},
                 {"recording_id", "single_gone_with_stray"},
                 {"file_path", gone_dir.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 777}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  ASSERT_TRUE(descriptors[0].x_medkit->contains("storage_files"));
  EXPECT_EQ((*descriptors[0].x_medkit)["storage_files"], 1);
  EXPECT_EQ(descriptors[0].size, 777u) << "the listing keeps the row's figure rather than measuring the stray";
}

TEST_F(RosbagBagDirectoryTest, AnEmptyStorageFileListIsTreatedAsABagThatWillNotSay) {
  // A metadata that names nothing (`relative_file_paths: []`). That is a bag
  // which did not answer, not a bag holding zero files: there is a storage file
  // in the directory. Counting it at zero would describe the recording as empty,
  // which is the value the count exists to avoid, so the field is omitted and the
  // directory walk answers, exactly as for metadata that cannot be read at all.
  const auto empty_list_dir = bag_dir_ / "empty_list";
  std::filesystem::create_directories(empty_list_dir);
  write_file(empty_list_dir / "recording_0.db3", std::string(3072, 'e'));
  write_metadata(empty_list_dir, {});

  EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(empty_list_dir.string()),
            (empty_list_dir / "recording_0.db3").string())
      << "an empty list is not an answer, so the directory is still consulted";
  EXPECT_FALSE(handlers::detail::rosbag_storage_file_count(empty_list_dir.string()).has_value())
      << "a bag that named nothing must not be counted at zero";

  const json row{{"fault_code", "EMPTY_LIST"},
                 {"recording_id", "empty_list"},
                 {"file_path", empty_list_dir.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 555}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  EXPECT_FALSE(descriptors[0].x_medkit->contains("storage_files")) << "omitted, not zero";
}

TEST_F(RosbagBagDirectoryTest, ANameThatLeavesTheBagDirectoryIsSkipped) {
  // A name that climbs out of the bag directory, and a name that is absolute.
  // The names come out of a file on disk and are joined onto the bag path, so a
  // name that climbs out of the directory, or replaces it outright by being
  // absolute, would resolve to a file outside the recording and the download
  // would stream it. Both are skipped, and with no other named file present that
  // leaves nothing to serve.
  const auto escape_dir = bag_dir_ / "escape";
  std::filesystem::create_directories(escape_dir);
  const auto outside = bag_dir_ / "outside_target.db3";
  write_file(outside, std::string(6144, 'o'));
  ASSERT_TRUE(std::filesystem::exists(outside)) << "the escape target has to exist, or nothing is being told apart";

  write_metadata(escape_dir, {"../outside_target.db3"});
  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(escape_dir.string()), ""); })
      << "a relative name that climbs out of the bag must not be served";

  write_metadata(escape_dir, {outside.string()});
  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(escape_dir.string()), ""); })
      << "an absolute name replaces the bag path outright and must not be served";

  // Positive control on the same harness: the same file, named the way rosbag2
  // names one, is served. So the two refusals above are the escape being
  // refused and not the resolver failing to find anything at all.
  write_file(escape_dir / "recording_0.db3", std::string(1024, 'r'));
  write_metadata(escape_dir, {"recording_0.db3"});
  EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(escape_dir.string()), (escape_dir / "recording_0.db3").string());
}

TEST_F(RosbagBagDirectoryTest, StorageFileCountIsOneForAWholeRecordingAndAbsentWhenTheBagWillNotSay) {
  // One has to be stated rather than left out, because an absent field already
  // means something else here: that this side could not read the recording's
  // metadata at all. A client cannot tell "one file" from "unknown" if both are
  // silence.
  const json whole_row{{"fault_code", "MOTOR_OVERHEAT"},
                       {"recording_id", bag_dir_.filename().string()},
                       {"file_path", bag_dir_.string()},
                       {"format", "sqlite3"},
                       {"size_bytes", directory_total()}};

  const auto whole = handlers::detail::fold_rosbag_rows_into_descriptors({whole_row}, {});
  ASSERT_EQ(whole.size(), 1u);
  ASSERT_TRUE(whole[0].x_medkit.has_value());
  ASSERT_TRUE(whole[0].x_medkit->contains("storage_files")) << "a whole recording states its one file";
  EXPECT_EQ((*whole[0].x_medkit)["storage_files"], 1);

  // A bare storage file is one storage file by definition, and has no
  // metadata.yaml beside it under that name to consult.
  const auto bare_file = bag_dir_ / "standalone.db3";
  write_file(bare_file, std::string(2048, 'z'));
  const json bare_row{{"fault_code", "BARE"},
                      {"recording_id", "standalone.db3"},
                      {"file_path", bare_file.string()},
                      {"format", "sqlite3"},
                      {"size_bytes", 1}};
  const auto bare = handlers::detail::fold_rosbag_rows_into_descriptors({bare_row}, {});
  ASSERT_EQ(bare.size(), 1u);
  ASSERT_TRUE(bare[0].x_medkit.has_value());
  ASSERT_TRUE(bare[0].x_medkit->contains("storage_files"));
  EXPECT_EQ((*bare[0].x_medkit)["storage_files"], 1);

  // No metadata to read: the field is omitted rather than guessed at one. The
  // assertions above are the positive control for this absence - the same helper
  // on the same harness does emit the field when the bag answers.
  const auto silent_dir = bag_dir_ / "no_metadata";
  std::filesystem::create_directories(silent_dir);
  write_file(silent_dir / "recording_0.db3", std::string(1024, 'q'));
  const json silent_row{{"fault_code", "SILENT"},
                        {"recording_id", "no_metadata"},
                        {"file_path", silent_dir.string()},
                        {"format", "sqlite3"},
                        {"size_bytes", 1024}};
  const auto silent = handlers::detail::fold_rosbag_rows_into_descriptors({silent_row}, {});
  ASSERT_EQ(silent.size(), 1u);
  ASSERT_TRUE(silent[0].x_medkit.has_value());
  EXPECT_FALSE(silent[0].x_medkit->contains("storage_files")) << "a bag that will not say must not be counted at one";
}

TEST_F(RosbagBagDirectoryTest, TheDownloadAndTheListingAgreeOnWhichSegmentIsServed) {
  // download() resolves the row's file_path through
  // BulkDataHandlers::resolve_rosbag_file_path and reports that file's length as
  // Content-Length. The call below is that one, with that argument, so the two
  // sides cannot answer differently for one recording. What is pinned here is
  // the pair: the transfer is the recording's first segment while the descriptor
  // keeps the whole recording's size, and the gap between them is what a client
  // reads as "this is a part".
  const auto split_dir = bag_dir_ / "split_agreement";
  std::filesystem::create_directories(split_dir);
  write_file(split_dir / "recording_0.db3", std::string(16384, 'a'));
  write_file(split_dir / "recording_1.db3", std::string(53248, 'b'));
  write_metadata(split_dir, {"recording_0.db3"});

  const auto directory_first = first_in_directory_order(split_dir);
  ASSERT_FALSE(directory_first.empty());
  const std::filesystem::path metadata_first =
      directory_first == split_dir / "recording_0.db3" ? split_dir / "recording_1.db3" : split_dir / "recording_0.db3";
  write_metadata(split_dir, {metadata_first.filename().string(), directory_first.filename().string()});

  uint64_t split_total = 0;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(split_dir)) {
    if (entry.is_regular_file()) {
      split_total += entry.file_size();
    }
  }

  const std::string served_path = BulkDataHandlers::resolve_rosbag_file_path(split_dir.string());
  ASSERT_EQ(served_path, metadata_first.string());
  std::error_code ec;
  const uint64_t content_length = std::filesystem::file_size(served_path, ec);
  ASSERT_FALSE(static_cast<bool>(ec));

  const json row{{"fault_code", "SPLIT_FAULT"},
                 {"recording_id", "fault_SPLIT_FAULT_1738664999003"},
                 {"file_path", split_dir.string()},
                 {"format", "sqlite3"},
                 {"size_bytes", split_total}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(content_length, std::filesystem::file_size(metadata_first)) << "the transfer is that segment, whole";
  EXPECT_GT(descriptors[0].size, content_length) << "size exceeding Content-Length is how a client sees a split";
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  ASSERT_TRUE(descriptors[0].x_medkit->contains("storage_files"));
  EXPECT_EQ((*descriptors[0].x_medkit)["storage_files"], 2) << "and the count tells it how many there were";
}

// A bag directory this process cannot walk must cost its own row and nothing
// else. The resolver used the throwing filesystem overloads, so one EACCES or
// ENOENT threw out of the listing handler, which has no catch in its chain, and
// the whole request answered 500 - every recording of the entity gone because of
// one unreadable directory.
//
// The trigger is a symlink loop rather than a 0000 directory because it has to
// be refused for any uid, and these tests do not run under one uid. In CI they
// run as root: the workflow's jobs declare a plain `container:` with no `user:`
// key, and that runs as uid 0. Locally they run as uid 1000. Root ignores mode
// bits, so a 0000 directory is readable in CI and a test built on one would pass
// there without the failure it claims to reproduce. ELOOP is refused for every
// uid alike, so this case means the same thing in both places.
class UnreadableBagTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() /
            ("bulkdata_unreadable_" + std::to_string(getpid()) + "_" + std::to_string(counter_++));
    std::filesystem::create_directories(root_);

    // Readable control bag: one storage file, real metadata.
    readable_ = root_ / "readable_bag";
    std::filesystem::create_directories(readable_);
    {
      std::ofstream out(readable_ / "recording_0.db3", std::ios::binary);
      out << std::string(2048, 'x');
    }
    {
      std::ofstream out(readable_ / "metadata.yaml", std::ios::binary);
      out << "rosbag2_bagfile_information:\n  version: 9\n  relative_file_paths:\n    - recording_0.db3\n";
    }

    // Unreadable bag: a symlink pointing at itself. Every filesystem query on it
    // fails with ELOOP, for root as much as for anyone else.
    loop_ = root_ / "loop_bag";
    std::error_code ec;
    std::filesystem::create_symlink(loop_, loop_, ec);
    symlink_created_ = !ec;
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  std::filesystem::path root_;
  std::filesystem::path readable_;
  std::filesystem::path loop_;
  bool symlink_created_{false};
  static int counter_;
};

int UnreadableBagTest::counter_ = 0;

TEST_F(UnreadableBagTest, AnUnreadableBagCostsItsOwnRowAndNotTheListing) {
  ASSERT_TRUE(symlink_created_) << "could not create the symlink loop, so nothing is being tested";
  // The loop really is refused by the filesystem, whatever uid this runs as.
  std::error_code probe_ec;
  const bool loop_is_a_directory = std::filesystem::is_directory(loop_, probe_ec);
  ASSERT_TRUE(static_cast<bool>(probe_ec)) << "the symlink loop resolved, so it is not an unreadable bag";
  ASSERT_FALSE(loop_is_a_directory) << "a path that errored cannot also be a readable directory";

  const uint64_t readable_served = std::filesystem::file_size(readable_ / "recording_0.db3");

  const std::vector<json> rows{
      json{{"fault_code", "READABLE"},
           {"recording_id", "fault_READABLE_1"},
           {"file_path", readable_.string()},
           {"format", "sqlite3"},
           {"size_bytes", 999999}},
      json{{"fault_code", "UNREADABLE"},
           {"recording_id", "fault_UNREADABLE_1"},
           {"file_path", loop_.string()},
           {"format", "sqlite3"},
           {"size_bytes", 4242}},
  };

  // The listing answers, and answers with BOTH recordings.
  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 2u) << "an unreadable bag removed another recording from the listing";
  EXPECT_EQ(descriptors[0].id, "fault_READABLE_1");
  EXPECT_EQ(descriptors[0].size, readable_served) << "the readable bag is still measured locally";
  EXPECT_EQ(descriptors[1].id, "fault_UNREADABLE_1");
  EXPECT_EQ(descriptors[1].size, 4242u) << "the unreadable bag keeps the figure its row carried";

  // These two are the assertions that actually pin the throwing overloads, and the
  // listing assertion above is defence in depth rather than the guard. Order is
  // why: rosbag_served_bytes reads the bag's metadata before it resolves any file,
  // and an unreadable bag has no readable metadata either, so it returns nullopt
  // before the resolver is ever reached. download() has no such gate in front of
  // it - it calls the resolver directly - so the resolver's own refusal to throw
  // is what that route depends on, and it is asserted here directly.
  EXPECT_NO_THROW({ EXPECT_FALSE(handlers::detail::rosbag_served_bytes(loop_.string()).has_value()); });
  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(loop_.string()), ""); });
  // The count reaches the filesystem on its own, before any metadata is read, to
  // decide whether the path is a bare storage file. It runs on the same row of
  // the same listing, so it has to decline an unreadable bag the same way.
  EXPECT_NO_THROW({ EXPECT_FALSE(handlers::detail::rosbag_storage_file_count(loop_.string()).has_value()); });
}

// The second shape: a directory whose mode denies everyone. This one does test
// something under uid 1000, where it runs today, and cannot under uid 0, where CI
// runs it. It probes first and skips with the uid rather than passing on a
// permission that was never actually denied.
TEST_F(UnreadableBagTest, AModeZeroDirectoryIsAlsoDeclinedRatherThanThrown) {
  const auto locked = root_ / "locked_bag";
  std::filesystem::create_directories(locked);
  {
    std::ofstream out(locked / "recording_0.db3", std::ios::binary);
    out << std::string(1024, 'x');
  }
  std::error_code ec;
  std::filesystem::permissions(locked, std::filesystem::perms::none, ec);
  ASSERT_FALSE(static_cast<bool>(ec)) << "could not drop the directory's permissions";

  std::error_code probe_ec;
  std::filesystem::directory_iterator probe(locked, probe_ec);
  if (!probe_ec) {
    std::filesystem::permissions(locked, std::filesystem::perms::owner_all, ec);
    GTEST_SKIP() << "running as uid " << ::getuid() << ", which ignores mode bits, so a 0000 directory is readable. "
                 << "The symlink-loop case above is the one that covers every uid";
  }

  EXPECT_NO_THROW({ EXPECT_FALSE(handlers::detail::rosbag_served_bytes(locked.string()).has_value()); });
  EXPECT_NO_THROW({ EXPECT_EQ(BulkDataHandlers::resolve_rosbag_file_path(locked.string()), ""); });
  EXPECT_NO_THROW({ EXPECT_FALSE(handlers::detail::rosbag_storage_file_count(locked.string()).has_value()); });

  std::filesystem::permissions(locked, std::filesystem::perms::owner_all, ec);
}

TEST_F(RosbagBagDirectoryTest, AnUnreachableBagKeepsTheStoredFigureRatherThanReportingZero) {
  const json row{{"fault_code", "MOTOR_OVERHEAT"},
                 {"recording_id", "fault_MOTOR_OVERHEAT_1738664999000"},
                 {"file_path", (bag_dir_ / "gone").string()},
                 {"format", "sqlite3"},
                 {"size_bytes", 35943}};

  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors({row}, {});
  ASSERT_EQ(descriptors.size(), 1u);
  EXPECT_EQ(descriptors[0].size, 35943u);
}

// === Shared timestamp utility tests ===

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsValidTimestamp) {
  // 2026-02-08T00:00:00.000Z
  int64_t ns = 1770458400000000000;
  auto result = ros2_medkit_gateway::format_timestamp_ns(ns);
  EXPECT_TRUE(result.find("2026") != std::string::npos);
  EXPECT_TRUE(result.find("T") != std::string::npos);
  EXPECT_TRUE(result.find("Z") != std::string::npos);
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsEpoch) {
  auto result = ros2_medkit_gateway::format_timestamp_ns(0);
  EXPECT_EQ(result, "1970-01-01T00:00:00.000Z");
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsWithMilliseconds) {
  // 1 second + 123 ms
  int64_t ns = 1'000'000'000 + 123'000'000;
  auto result = ros2_medkit_gateway::format_timestamp_ns(ns);
  EXPECT_TRUE(result.find(".123Z") != std::string::npos);
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataHandlersTest, FormatTimestampNsNegativeFallback) {
  // Negative timestamps should return fallback
  auto result = ros2_medkit_gateway::format_timestamp_ns(-1);
  EXPECT_FALSE(result.empty());
  EXPECT_TRUE(result.find("Z") != std::string::npos);
}

// === Descriptor to JSON conversion tests ===

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, DescriptorToJsonConversion) {
  ros2_medkit_gateway::BulkDataStore::ItemDescriptor desc;
  desc.id = "calibration_123_abcd1234";
  desc.name = "test.bin";
  desc.mime_type = "application/octet-stream";
  desc.size = 1024;
  desc.created = "2026-01-01T00:00:00.000Z";
  desc.description = "Test upload";
  desc.metadata = nlohmann::json::object();

  nlohmann::json j = {{"id", desc.id},
                      {"name", desc.name},
                      {"mimetype", desc.mime_type},
                      {"size", desc.size},
                      {"creation_date", desc.created},
                      {"description", desc.description}};

  EXPECT_EQ(j["id"], "calibration_123_abcd1234");
  EXPECT_EQ(j["name"], "test.bin");
  EXPECT_EQ(j["mimetype"], "application/octet-stream");
  EXPECT_EQ(j["size"], 1024);
  EXPECT_EQ(j["creation_date"], "2026-01-01T00:00:00.000Z");
  EXPECT_EQ(j["description"], "Test upload");
  EXPECT_FALSE(j.contains("x-medkit"));
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, DescriptorToJsonWithMetadata) {
  ros2_medkit_gateway::BulkDataStore::ItemDescriptor desc;
  desc.id = "calibration_123_abcd1234";
  desc.name = "cal.bin";
  desc.mime_type = "application/octet-stream";
  desc.size = 512;
  desc.created = "2026-01-01T00:00:00.000Z";
  desc.description = "";
  desc.metadata = {{"sensor", "lidar"}, {"version", 2}};

  nlohmann::json j = {{"id", desc.id},
                      {"name", desc.name},
                      {"mimetype", desc.mime_type},
                      {"size", desc.size},
                      {"creation_date", desc.created},
                      {"description", desc.description}};
  if (!desc.metadata.empty()) {
    j["x-medkit"] = desc.metadata;
  }

  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["sensor"], "lidar");
  EXPECT_EQ(j["x-medkit"]["version"], 2);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, DescriptorToJsonWithoutDescription) {
  ros2_medkit_gateway::BulkDataStore::ItemDescriptor desc;
  desc.id = "firmware_456_ef012345";
  desc.name = "fw.img";
  desc.mime_type = "application/octet-stream";
  desc.size = 2048;
  desc.created = "2026-06-15T12:00:00.000Z";
  desc.description = "";
  desc.metadata = nlohmann::json::object();

  nlohmann::json j = {{"id", desc.id},
                      {"name", desc.name},
                      {"mimetype", desc.mime_type},
                      {"size", desc.size},
                      {"creation_date", desc.created}};
  // Only add description if non-empty (matching handler pattern)
  if (!desc.description.empty()) {
    j["description"] = desc.description;
  }
  if (!desc.metadata.empty()) {
    j["x-medkit"] = desc.metadata;
  }

  EXPECT_FALSE(j.contains("description"));
  EXPECT_FALSE(j.contains("x-medkit"));
}

// === Error code tests ===

// @verifies REQ_INTEROP_074
TEST_F(BulkDataHandlersTest, PayloadTooLargeErrorCodeDefined) {
  EXPECT_NE(ros2_medkit_gateway::ERR_PAYLOAD_TOO_LARGE, nullptr);
  EXPECT_STREQ(ros2_medkit_gateway::ERR_PAYLOAD_TOO_LARGE, "payload-too-large");
}

// =============================================================================
// compute_bulkdata_source_filters tests
//
// Pin the entity-type branching that drives rosbag descriptor lookups + the
// download ownership check. Crucial because synthetic / runtime-discovered
// components have empty fqn AND empty namespace_path: without aggregation
// from hosted apps the handler used to silently return zero source filters.
//
// Tested as a pure free function in detail:: against a directly-constructed
// ThreadSafeEntityCache so no GatewayNode / DDS context is needed.
// =============================================================================

namespace {

App make_test_app(const std::string & id, const std::string & node_name, const std::string & ns,
                  const std::string & component_id) {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = component_id;
  App::RosBinding rb;
  rb.node_name = node_name;
  rb.namespace_pattern = ns;
  a.ros_binding = rb;
  return a;
}

EntityInfo make_entity_info(EntityType type, const std::string & id, const std::string & namespace_path,
                            const std::string & fqn) {
  EntityInfo info;
  info.type = type;
  info.id = id;
  info.namespace_path = namespace_path;
  info.fqn = fqn;
  return info;
}

}  // namespace

class BulkDataSourceFiltersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Synthetic component: empty fqn AND empty namespace_path.
    Component synthetic;
    synthetic.id = "runtime_engine";
    synthetic.name = "Runtime Engine";
    synthetic.namespace_path = "";
    synthetic.fqn = "";

    // Manifest-only component: declares namespace but groups topics, not nodes.
    Component manifest_only;
    manifest_only.id = "topics_group";
    manifest_only.name = "Topics Group";
    manifest_only.namespace_path = "/topics/group";
    manifest_only.fqn = "/topics/group";

    auto app1 = make_test_app("temp_sensor", "temp_sensor", "/powertrain/engine", "runtime_engine");
    auto app2 = make_test_app("rpm_sensor", "rpm_sensor", "/powertrain/engine", "runtime_engine");

    Function func;
    func.id = "powertrain_diag";
    func.name = "Powertrain Diagnostics";
    func.hosts = {"temp_sensor", "rpm_sensor"};

    Function empty_func;
    empty_func.id = "empty_func";
    empty_func.name = "Empty Function";

    cache_.update_all({}, {synthetic, manifest_only}, {app1, app2}, {func, empty_func});
  }

  ThreadSafeEntityCache cache_;
};

// COMPONENT with hosted apps - returns app effective FQNs (synthetic component
// has empty fqn / namespace_path; without aggregation this would be {}).
TEST_F(BulkDataSourceFiltersTest, ComponentWithHostedAppsReturnsAppFqns) {
  auto entity = make_entity_info(EntityType::COMPONENT, "runtime_engine", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 2u);
  std::set<std::string> as_set(filters.begin(), filters.end());
  EXPECT_TRUE(as_set.count("/powertrain/engine/temp_sensor"));
  EXPECT_TRUE(as_set.count("/powertrain/engine/rpm_sensor"));
}

// COMPONENT hosting plugin-provided apps - those apps have no ROS binding and
// report faults under their bare entity id, so that id must become the filter.
// Resolving by effective_fqn() alone yielded zero filters and the download
// ownership check answered "Bulk-data not found for this entity" for a bag that
// existed on disk.
TEST_F(BulkDataSourceFiltersTest, ComponentWithExternalAppsReturnsBareEntityIds) {
  App plc_app;
  plc_app.id = "plc_line1";
  plc_app.name = "PLC Line 1";
  plc_app.component_id = "plc_hw";
  plc_app.external = true;
  cache_.update_apps({plc_app});

  auto entity = make_entity_info(EntityType::COMPONENT, "plc_hw", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc_line1");
}

// APP provided by a protocol plugin - no ROS binding, so fqn and namespace_path
// are empty and the generic path returned no filter at all: the rosbag the fault
// detail advertises under /apps/<id>/bulk-data/rosbags/<code> belonged to nobody
// and every download 404'd. Its bare id is its reporting source.
TEST_F(BulkDataSourceFiltersTest, ExternalAppResolvesToItsBareEntityId) {
  App device;
  device.id = "twincat_runtime_device";
  device.name = "TwinCAT 3 Runtime (device)";
  device.component_id = "twincat_runtime";
  device.external = true;
  cache_.update_apps({device});

  auto entity = make_entity_info(EntityType::APP, "twincat_runtime_device", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "twincat_runtime_device");
}

// An external COMPONENT reports faults under its own id as well (a bridge raises
// PLC_COMMS_LOST there), so it owns that source alongside its hosted apps.
TEST_F(BulkDataSourceFiltersTest, ExternalComponentAlsoOwnsItsOwnId) {
  Component plc;
  plc.id = "plc_hw2";
  plc.name = "PLC";
  plc.external = true;
  cache_.update_components({plc});
  App child;
  child.id = "plc_hw2_device";
  child.name = "PLC (device)";
  child.component_id = "plc_hw2";
  child.external = true;
  cache_.update_apps({child});

  auto entity = make_entity_info(EntityType::COMPONENT, "plc_hw2", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  std::set<std::string> as_set(filters.begin(), filters.end());
  EXPECT_TRUE(as_set.count("plc_hw2_device"));
  EXPECT_TRUE(as_set.count("plc_hw2"));
}

// COMPONENT with no hosted apps but non-empty fqn falls through to fqn path
// (manifest deployment grouping topics rather than nodes).
TEST_F(BulkDataSourceFiltersTest, ComponentManifestOnlyFallsThroughToFqn) {
  auto entity = make_entity_info(EntityType::COMPONENT, "topics_group", "/topics/group", "/topics/group");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/topics/group");
}

// COMPONENT with no hosted apps AND no fqn / namespace_path returns empty -
// nothing to query.
TEST_F(BulkDataSourceFiltersTest, ComponentSyntheticWithoutAppsReturnsEmpty) {
  auto entity = make_entity_info(EntityType::COMPONENT, "nonexistent_comp", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  EXPECT_TRUE(filters.empty());
}

// FUNCTION with hosted apps - returns app effective FQNs. Crucially, FUNCTION
// must NOT fall through to namespace_path/fqn even when the host list is
// non-empty - functions are pure aggregated views.
TEST_F(BulkDataSourceFiltersTest, FunctionWithHostsReturnsAppFqns) {
  auto entity = make_entity_info(EntityType::FUNCTION, "powertrain_diag", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 2u);
  std::set<std::string> as_set(filters.begin(), filters.end());
  EXPECT_TRUE(as_set.count("/powertrain/engine/temp_sensor"));
  EXPECT_TRUE(as_set.count("/powertrain/engine/rpm_sensor"));
}

// FUNCTION without hosted apps returns empty - no fall-through to fqn even if
// the entity carried one (regression guard for the original FUNCTION semantics
// after the COMPONENT/FUNCTION split).
TEST_F(BulkDataSourceFiltersTest, FunctionWithoutHostsReturnsEmptyEvenIfFqnSet) {
  auto entity = make_entity_info(EntityType::FUNCTION, "empty_func", "/some/ns", "/some/fqn");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  EXPECT_TRUE(filters.empty());
}

// APP entity returns its own fqn as the single filter - no aggregation.
TEST_F(BulkDataSourceFiltersTest, AppReturnsSingleFqnFilter) {
  auto entity =
      make_entity_info(EntityType::APP, "temp_sensor", "/powertrain/engine", "/powertrain/engine/temp_sensor");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/powertrain/engine/temp_sensor");
}

// APP without fqn falls through to namespace_path filter.
TEST_F(BulkDataSourceFiltersTest, AppWithEmptyFqnFallsThroughToNamespacePath) {
  auto entity = make_entity_info(EntityType::APP, "some_app", "/the/namespace", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/the/namespace");
}

// APP with neither fqn nor namespace_path returns empty.
TEST_F(BulkDataSourceFiltersTest, AppWithEmptyFqnAndNamespaceReturnsEmpty) {
  auto entity = make_entity_info(EntityType::APP, "some_app", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  EXPECT_TRUE(filters.empty());
}

// AREA not present in the cache (no hosted apps resolvable) falls back to fqn.
TEST_F(BulkDataSourceFiltersTest, AreaReturnsFqnAsFilter) {
  auto entity = make_entity_info(EntityType::AREA, "powertrain", "/powertrain", "/powertrain");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache_, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "/powertrain");
}

// AREA with hosted apps resolves them like the fault scope does, recursing
// subareas and keeping external bare ids. The namespace fallback alone never
// matched a bag: list_rosbags_for_entity compares reporting sources exactly,
// so /areas/<id>/faults advertised a bulk_data_uri that 404'd.
TEST_F(BulkDataSourceFiltersTest, AreaResolvesHostedAppsIncludingSubareas) {
  Area cell;
  cell.id = "plc-cell";
  cell.name = "PLC Cell";
  cell.namespace_path = "/plc_cell";
  Area sub;
  sub.id = "cabinet";
  sub.name = "Cabinet";
  sub.namespace_path = "/plc_cell/cabinet";
  sub.parent_area_id = "plc-cell";

  Component plc;
  plc.id = "s7-plc";
  plc.name = "PLC";
  plc.area = "cabinet";
  App proc;
  proc.id = "plc-process";
  proc.name = "PLC Process";
  proc.component_id = "s7-plc";
  proc.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({cell, sub}, {plc}, {proc}, {});

  auto entity = make_entity_info(EntityType::AREA, "plc-cell", "/plc_cell", "/plc_cell");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc-process");
}

// FUNCTION whose host is a Component (not an app) resolves the component's
// apps. The app-index lookup dropped component hosts: the function listed the
// fault but its bag download 404'd.
TEST_F(BulkDataSourceFiltersTest, FunctionWithComponentHostResolvesComponentApps) {
  Component plc;
  plc.id = "plc_hw";
  plc.name = "PLC";
  App proc;
  proc.id = "plc-process";
  proc.name = "PLC Process";
  proc.component_id = "plc_hw";
  proc.external = true;
  Function func;
  func.id = "level-control";
  func.name = "Level Control";
  func.hosts = {"plc_hw"};

  ThreadSafeEntityCache cache;
  cache.update_all({}, {plc}, {proc}, {func});

  auto entity = make_entity_info(EntityType::FUNCTION, "level-control", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc-process");
}

// Download ownership uses fault_in_source_scope over the computed filters:
// exact match or '/'-boundary prefix only. A raw prefix match (the transport's
// get_fault(code, source) semantics) would let app id "plc" claim the bag of
// "plc_line1".
// === Download authorization tests ===
// A recording is shared by a whole burst, so ownership is the union over its
// attached faults. The scope matcher itself is unchanged and pinned below; what
// is new is which codes get fed to it.

TEST_F(BulkDataSourceFiltersTest, AttachedFaultCodesComeFromTheRecordingNotTheUrl) {
  const nlohmann::json rosbag = {{"file_path", "/var/bags/fault_ROOT_1"},
                                 {"recording_id", "fault_ROOT_1"},
                                 {"fault_codes", {"ROOT", "DOWNSTREAM_A", "DOWNSTREAM_B"}}};

  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(rosbag, "fault_ROOT_1"),
            (std::vector<std::string>{"ROOT", "DOWNSTREAM_A", "DOWNSTREAM_B"}));
}

TEST_F(BulkDataSourceFiltersTest, AttachedFaultCodesFallBackToTheRequestedIdOnAnOlderPeer) {
  // The compatibility path: the id addressed was the fault code, and a peer
  // that predates the field sends no list. Authorizing against the requested id
  // is exactly the check that shipped before.
  const nlohmann::json rosbag = {{"file_path", "/var/bags/fault_MOTOR_1"}};
  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(rosbag, "MOTOR_OVERHEAT"),
            (std::vector<std::string>{"MOTOR_OVERHEAT"}));
}

TEST_F(BulkDataSourceFiltersTest, AttachedFaultCodesFallBackOnAnEmptyOrMalformedList) {
  // Never return empty: an empty list makes any_of vacuously false, which would
  // 404 a download the entity owns.
  const nlohmann::json empty_list = {{"fault_codes", nlohmann::json::array()}};
  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(empty_list, "X"), (std::vector<std::string>{"X"}));

  const nlohmann::json not_an_array = {{"fault_codes", "X"}};
  EXPECT_EQ(handlers::detail::rosbag_attached_fault_codes(not_an_array, "X"), (std::vector<std::string>{"X"}));
}

TEST_F(BulkDataSourceFiltersTest, AFaultCodeUrlIsRecognisedAsTheCompatibilityPath) {
  // The segment named a fault; the answer is that fault's newest recording, whose
  // id is something else. Authorizing on the union alone would 200 a code the
  // entity does not own, so this path also demands the requested code in scope.
  const nlohmann::json resolved_by_code = {{"recording_id", "fault_MOTOR_OVERHEAT_1738664999000"},
                                           {"fault_codes", {"MOTOR_OVERHEAT", "MOTOR_STALL"}}};
  EXPECT_TRUE(handlers::detail::rosbag_resolved_by_fault_code(resolved_by_code, "MOTOR_OVERHEAT"));
}

TEST_F(BulkDataSourceFiltersTest, ARecordingIdUrlIsNotTheCompatibilityPath) {
  const nlohmann::json resolved_by_id = {{"recording_id", "fault_MOTOR_OVERHEAT_1738664999000"},
                                         {"fault_codes", {"MOTOR_OVERHEAT", "MOTOR_STALL"}}};
  EXPECT_FALSE(handlers::detail::rosbag_resolved_by_fault_code(resolved_by_id, "fault_MOTOR_OVERHEAT_1738664999000"));
}

TEST_F(BulkDataSourceFiltersTest, APeerWithoutRecordingIdsIsNotTreatedAsCompatibilityPath) {
  // An older peer answers by fault code only and sends no recording id. The
  // attached-codes fallback already reduces to the pre-#620 check there, so
  // demanding a second one would 404 downloads that used to work.
  const nlohmann::json older_peer = {{"file_path", "/var/bags/fault_MOTOR_1"}};
  EXPECT_FALSE(handlers::detail::rosbag_resolved_by_fault_code(older_peer, "MOTOR_OVERHEAT"));
}

TEST_F(BulkDataSourceFiltersTest, ABurstRecordingIsOwnedByAnyEntityOwningOneOfItsFaults) {
  // One bag, three faults, two apps. Each app reaches the bag through its own
  // fault - which is what it could already do when the bag was addressed by
  // fault code.
  App plc;
  plc.id = "plc";
  plc.external = true;
  App vision;
  vision.id = "vision";
  vision.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {plc, vision}, {});

  auto plc_entity = make_entity_info(EntityType::APP, "plc", "", "");
  auto plc_filters = handlers::detail::compute_bulkdata_source_filters(cache, plc_entity);
  std::set<std::string> plc_scope(plc_filters.begin(), plc_filters.end());

  const std::vector<nlohmann::json> burst_faults = {nlohmann::json{{"reporting_sources", {"vision"}}},
                                                    nlohmann::json{{"reporting_sources", {"plc"}}}};

  const bool plc_authorized = std::any_of(burst_faults.begin(), burst_faults.end(), [&](const nlohmann::json & f) {
    return faults::fault_in_source_scope(f, plc_scope);
  });
  EXPECT_TRUE(plc_authorized) << "the PLC owns one of the burst's faults";
}

TEST_F(BulkDataSourceFiltersTest, ABurstRecordingIsRejectedWhenNoAttachedFaultIsInScope) {
  App plc;
  plc.id = "plc";
  plc.external = true;
  App plc_line1;
  plc_line1.id = "plc_line1";
  plc_line1.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {plc, plc_line1}, {});

  auto entity = make_entity_info(EntityType::APP, "plc", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  std::set<std::string> scope(filters.begin(), filters.end());

  // The prefix-sibling rule has to survive the union: "plc" must not reach a
  // burst owned entirely by "plc_line1", no matter how many faults it holds.
  const std::vector<nlohmann::json> foreign_burst = {nlohmann::json{{"reporting_sources", {"plc_line1"}}},
                                                     nlohmann::json{{"reporting_sources", {"plc_line1/axis2"}}}};

  const bool authorized = std::any_of(foreign_burst.begin(), foreign_burst.end(), [&](const nlohmann::json & f) {
    return faults::fault_in_source_scope(f, scope);
  });
  EXPECT_FALSE(authorized);

  // ...while a descendant of the entity's own scope still reaches it.
  const std::vector<nlohmann::json> own_burst = {nlohmann::json{{"reporting_sources", {"plc_line1"}}},
                                                 nlohmann::json{{"reporting_sources", {"plc/axis1"}}}};
  EXPECT_TRUE(std::any_of(own_burst.begin(), own_burst.end(), [&](const nlohmann::json & f) {
    return faults::fault_in_source_scope(f, scope);
  }));
}

TEST_F(BulkDataSourceFiltersTest, DownloadOwnershipScopeRejectsPrefixSiblingApp) {
  App plc;
  plc.id = "plc";
  plc.name = "PLC";
  plc.external = true;
  App plc_line1;
  plc_line1.id = "plc_line1";
  plc_line1.name = "PLC Line 1";
  plc_line1.external = true;

  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {plc, plc_line1}, {});

  auto entity = make_entity_info(EntityType::APP, "plc", "", "");
  auto filters = handlers::detail::compute_bulkdata_source_filters(cache, entity);
  ASSERT_EQ(filters.size(), 1u);
  EXPECT_EQ(filters[0], "plc");
  std::set<std::string> scope(filters.begin(), filters.end());

  nlohmann::json sibling_fault = {{"reporting_sources", {"plc_line1"}}};
  EXPECT_FALSE(faults::fault_in_source_scope(sibling_fault, scope));

  nlohmann::json own_fault = {{"reporting_sources", {"plc"}}};
  EXPECT_TRUE(faults::fault_in_source_scope(own_fault, scope));

  nlohmann::json child_fault = {{"reporting_sources", {"plc/axis1"}}};
  EXPECT_TRUE(faults::fault_in_source_scope(child_fault, scope));
}
