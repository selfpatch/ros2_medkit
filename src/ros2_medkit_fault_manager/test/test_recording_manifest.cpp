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

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_fault_manager/core/recording_manifest.hpp"
#include "ros2_medkit_fault_manager/fault_audit_log.hpp"

namespace core = ros2_medkit_fault_manager::core;
using ros2_medkit_fault_manager::FaultAuditLog;

namespace {

const char * const kSha256OfAbc = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
const char * const kSha256OfNothing = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";

class RecordingManifestTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() / ("recording_manifest_test_" + std::to_string(getpid()));
    std::filesystem::remove_all(root_);
    bag_ = root_ / "fault_MOTOR_OVERHEAT_1785441426087";
    std::filesystem::create_directories(bag_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  void write_file(const std::string & name, const std::string & content) const {
    std::ofstream(bag_ / name, std::ios::binary) << content;
  }

  void write_metadata(const std::string & storage_id, const std::vector<std::string> & files) const {
    std::ostringstream yaml;
    yaml << "rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: " << storage_id
         << "\n  relative_file_paths:\n";
    for (const auto & file : files) {
      yaml << "    - " << file << "\n";
    }
    yaml << "  message_count: 0\n";
    write_file("metadata.yaml", yaml.str());
  }

  std::string read_manifest() const {
    std::ifstream file(bag_ / core::kRecordingManifestFile, std::ios::binary);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  std::filesystem::path root_;
  std::filesystem::path bag_;
};

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, Sha256MatchesKnownVectors) {
  write_file("abc", "abc");
  write_file("empty", "");
  EXPECT_EQ(core::sha256_file_hex((bag_ / "abc").string()), kSha256OfAbc);
  EXPECT_EQ(core::sha256_file_hex((bag_ / "empty").string()), kSha256OfNothing);
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, Sha256CoversAFileLargerThanOneReadBuffer) {
  std::string content(3 * 1024 * 1024 + 17, '\0');
  for (size_t i = 0; i < content.size(); ++i) {
    content[i] = static_cast<char>(i * 31 + (i >> 11));
  }
  write_file("big", content);
  EXPECT_EQ(core::sha256_file_hex((bag_ / "big").string()), FaultAuditLog::sha256_hex(content));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, Sha256OfAMissingFileIsNoValue) {
  EXPECT_FALSE(core::sha256_file_hex((bag_ / "absent").string()).has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, ListsFilesInMetadataOrderWithSizeAndHash) {
  // `_10` sorts before `_2` in a directory listing, so order must come from metadata.yaml.
  std::vector<std::string> names;
  for (int i = 0; i < 12; ++i) {
    names.push_back("fault_MOTOR_OVERHEAT_1785441426087_" + std::to_string(i) + ".mcap");
    write_file(names.back(), std::string(static_cast<size_t>(100 + i), static_cast<char>('a' + i)));
  }
  write_metadata("mcap", names);

  ASSERT_FALSE(core::write_recording_manifest(bag_.string(), 1785441426087000000).has_value());

  const auto manifest = nlohmann::json::parse(read_manifest());
  EXPECT_EQ(manifest["manifest_version"], 1);
  EXPECT_EQ(manifest["recording_id"], "fault_MOTOR_OVERHEAT_1785441426087");
  EXPECT_EQ(manifest["format"], "mcap");
  EXPECT_EQ(manifest["created_at_ns"], 1785441426087000000);
  ASSERT_EQ(manifest["files"].size(), names.size());
  uint64_t total = 0;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto & entry = manifest["files"][i];
    EXPECT_EQ(entry["name"], names[i]);
    EXPECT_EQ(entry["size"], 100 + i);
    EXPECT_EQ(entry["sha256"], FaultAuditLog::sha256_hex(std::string(100 + i, static_cast<char>('a' + i))));
    total += 100 + i;
  }
  // metadata.yaml is neither listed nor counted.
  EXPECT_EQ(manifest["total_bytes"], total);
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, TheManifestBytesAreTheDocumentedLayout) {
  write_file("bag \"0\".mcap", "abc");
  write_file("bag_1.mcap", "");
  write_metadata("mcap", {"'bag \"0\".mcap'", "bag_1.mcap"});

  ASSERT_FALSE(core::write_recording_manifest(bag_.string(), 42).has_value());

  EXPECT_EQ(read_manifest(),
            "{\n"
            "  \"manifest_version\": 1,\n"
            "  \"recording_id\": \"fault_MOTOR_OVERHEAT_1785441426087\",\n"
            "  \"format\": \"mcap\",\n"
            "  \"created_at_ns\": 42,\n"
            "  \"total_bytes\": 3,\n"
            "  \"files\": [\n"
            "    {\n"
            "      \"name\": \"bag \\\"0\\\".mcap\",\n"
            "      \"size\": 3,\n"
            "      \"sha256\": \"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad\"\n"
            "    },\n"
            "    {\n"
            "      \"name\": \"bag_1.mcap\",\n"
            "      \"size\": 0,\n"
            "      \"sha256\": \"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855\"\n"
            "    }\n"
            "  ]\n"
            "}\n");
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, AStorageFileThatIsALinkIsNotListed) {
  write_file("bag_0.mcap", "data");
  std::ofstream(root_ / "outside.mcap") << "not part of the recording";
  std::filesystem::create_symlink(root_ / "outside.mcap", bag_ / "bag_1.mcap");
  write_metadata("mcap", {"bag_0.mcap", "bag_1.mcap"});
  const auto error = core::write_recording_manifest(bag_.string(), 1);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("bag_1.mcap"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(bag_ / core::kRecordingManifestFile));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, ReportsTheStorageFormatOfTheBag) {
  write_file("bag_0.db3", "SQLite format 3");
  write_metadata("sqlite3", {"bag_0.db3"});
  ASSERT_FALSE(core::write_recording_manifest(bag_.string(), 1).has_value());
  EXPECT_EQ(nlohmann::json::parse(read_manifest())["format"], "sqlite3");
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, AManifestIsNeverRewritten) {
  write_file("bag_0.mcap", "first");
  write_metadata("mcap", {"bag_0.mcap"});
  ASSERT_FALSE(core::write_recording_manifest(bag_.string(), 1).has_value());
  const std::string first = read_manifest();

  write_file("bag_0.mcap", "changed afterwards");
  EXPECT_TRUE(core::write_recording_manifest(bag_.string(), 2).has_value());
  EXPECT_EQ(read_manifest(), first);
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, LeavesNoTemporaryFileBehind) {
  write_file("bag_0.mcap", "data");
  write_metadata("mcap", {"bag_0.mcap"});
  ASSERT_FALSE(core::write_recording_manifest(bag_.string(), 1).has_value());

  std::vector<std::string> names;
  for (const auto & entry : std::filesystem::directory_iterator(bag_)) {
    names.push_back(entry.path().filename().string());
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(names, (std::vector<std::string>{"bag_0.mcap", "manifest.json", "metadata.yaml"}));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, NoManifestWhenAStorageFileIsMissing) {
  write_file("bag_0.mcap", "data");
  write_metadata("mcap", {"bag_0.mcap", "bag_1.mcap"});
  const auto error = core::write_recording_manifest(bag_.string(), 1);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("bag_1.mcap"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(bag_ / core::kRecordingManifestFile));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, NoManifestWithoutMetadata) {
  write_file("bag_0.mcap", "data");
  EXPECT_TRUE(core::write_recording_manifest(bag_.string(), 1).has_value());
  EXPECT_FALSE(std::filesystem::exists(bag_ / core::kRecordingManifestFile));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, NoManifestWhenMetadataNamesNoFile) {
  write_metadata("mcap", {});
  EXPECT_TRUE(core::write_recording_manifest(bag_.string(), 1).has_value());
  EXPECT_FALSE(std::filesystem::exists(bag_ / core::kRecordingManifestFile));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, NoManifestForAnUnknownStorageFormat) {
  write_file("bag_0.bin", "data");
  write_metadata("custom", {"bag_0.bin"});
  EXPECT_TRUE(core::write_recording_manifest(bag_.string(), 1).has_value());
  EXPECT_FALSE(std::filesystem::exists(bag_ / core::kRecordingManifestFile));
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, AFileNamedTwiceIsRefused) {
  write_file("bag_0.mcap", "data");
  write_metadata("mcap", {"bag_0.mcap", "bag_0.mcap"});
  EXPECT_TRUE(core::write_recording_manifest(bag_.string(), 1).has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(RecordingManifestTest, APathInMetadataIsReducedToItsFileName) {
  // Older rosbag2 releases prefix the entries with the bag directory name.
  write_file("bag_0.mcap", "data");
  write_metadata("mcap", {"fault_MOTOR_OVERHEAT_1785441426087/bag_0.mcap"});
  ASSERT_FALSE(core::write_recording_manifest(bag_.string(), 1).has_value());
  EXPECT_EQ(nlohmann::json::parse(read_manifest())["files"][0]["name"], "bag_0.mcap");
}

}  // namespace

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
