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

#include "ros2_medkit_gateway/bulk_data_store.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>

namespace ros2_medkit_gateway {
namespace {

class BulkDataStoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create unique temp directory for each test
    test_dir_ = std::filesystem::temp_directory_path() / ("bulk_data_test_" + std::to_string(getpid()) + "_" +
                                                          std::to_string(test_counter_++));
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::filesystem::path test_dir_;
  static int test_counter_;
};

int BulkDataStoreTest::test_counter_ = 0;

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreAndRetrieve) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("my_app", "calibration", "test.bin", "application/octet-stream", "hello world");
  ASSERT_TRUE(result.has_value()) << result.error();

  auto desc = result.value();
  EXPECT_FALSE(desc.id.empty());
  EXPECT_EQ(desc.category, "calibration");
  EXPECT_EQ(desc.name, "test.bin");
  EXPECT_EQ(desc.mime_type, "application/octet-stream");
  EXPECT_EQ(desc.size, 11u);
  EXPECT_FALSE(desc.created.empty());

  // Retrieve
  auto item = store.get_item("my_app", "calibration", desc.id);
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->id, desc.id);
  EXPECT_EQ(item->name, "test.bin");
  EXPECT_EQ(item->size, 11u);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreCreatesDirectoryStructure) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("entity_1", "calibration", "data.bin", "application/octet-stream", "payload");
  ASSERT_TRUE(result.has_value());

  auto dir = test_dir_ / "entity_1" / "calibration" / result->id;
  EXPECT_TRUE(std::filesystem::is_directory(dir));
  EXPECT_TRUE(std::filesystem::is_regular_file(dir / "descriptor.json"));
  EXPECT_TRUE(std::filesystem::is_regular_file(dir / "data.bin"));
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreWithMetadata) {
  BulkDataStore store(test_dir_.string(), 0, {"firmware"});

  nlohmann::json meta = {{"version", "2.1.0"}, {"board", "rev3"}};
  auto result = store.store("ecu", "firmware", "fw.bin", "application/octet-stream", "firmware_data",
                            "Firmware update v2.1", meta);
  ASSERT_TRUE(result.has_value());

  EXPECT_EQ(result->description, "Firmware update v2.1");
  EXPECT_EQ(result->metadata["version"], "2.1.0");
  EXPECT_EQ(result->metadata["board"], "rev3");

  // Verify it persists
  auto item = store.get_item("ecu", "firmware", result->id);
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->description, "Firmware update v2.1");
  EXPECT_EQ(item->metadata["version"], "2.1.0");
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, ListItemsInCategory) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  ASSERT_TRUE(store.store("app1", "calibration", "a.bin", "application/octet-stream", "aaa").has_value());
  ASSERT_TRUE(store.store("app1", "calibration", "b.bin", "application/octet-stream", "bbb").has_value());
  ASSERT_TRUE(store.store("app1", "calibration", "c.bin", "application/octet-stream", "ccc").has_value());

  auto items = store.list_items("app1", "calibration");
  EXPECT_EQ(items.size(), 3u);

  // Verify all items have distinct IDs
  std::set<std::string> ids;
  for (const auto & item : items) {
    ids.insert(item.id);
  }
  EXPECT_EQ(ids.size(), 3u);
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataStoreTest, ListCategories) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration", "firmware", "comlogs"});

  auto cats = store.list_categories();
  EXPECT_EQ(cats.size(), 3u);

  std::set<std::string> cat_set(cats.begin(), cats.end());
  EXPECT_TRUE(cat_set.count("calibration") > 0);
  EXPECT_TRUE(cat_set.count("firmware") > 0);
  EXPECT_TRUE(cat_set.count("comlogs") > 0);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, RemoveItem) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "calibration", "data.bin", "application/octet-stream", "payload");
  ASSERT_TRUE(result.has_value());
  auto id = result->id;

  // Verify exists
  EXPECT_TRUE(store.get_item("app1", "calibration", id).has_value());

  // Remove
  auto remove_result = store.remove("app1", "calibration", id);
  ASSERT_TRUE(remove_result.has_value());

  // Verify gone
  EXPECT_FALSE(store.get_item("app1", "calibration", id).has_value());

  // Directory should be removed
  auto dir = test_dir_ / "app1" / "calibration" / id;
  EXPECT_FALSE(std::filesystem::exists(dir));
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, RemoveNonexistent) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.remove("app1", "calibration", "nonexistent_id");
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("not found") != std::string::npos);
}

// @verifies REQ_INTEROP_073
TEST_F(BulkDataStoreTest, GetFilePath) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "calibration", "test.bin", "application/octet-stream", "hello world");
  ASSERT_TRUE(result.has_value());

  auto path = store.get_file_path("app1", "calibration", result->id);
  ASSERT_TRUE(path.has_value());
  EXPECT_TRUE(std::filesystem::is_regular_file(*path));

  // Verify content
  std::ifstream ifs(*path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  EXPECT_EQ(content, "hello world");
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, GenerateUniqueIds) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  std::set<std::string> ids;
  for (int i = 0; i < 100; ++i) {
    auto result = store.store("app1", "calibration", "file.bin", "application/octet-stream", "x");
    ASSERT_TRUE(result.has_value()) << "Failed at iteration " << i << ": " << result.error();
    ids.insert(result->id);
  }
  EXPECT_EQ(ids.size(), 100u);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, MaxUploadSizeEnforced) {
  BulkDataStore store(test_dir_.string(), 100, {"calibration"});  // 100 bytes max

  // Should succeed (50 bytes)
  auto small = store.store("app1", "calibration", "small.bin", "application/octet-stream", std::string(50, 'x'));
  EXPECT_TRUE(small.has_value());

  // Should fail (200 bytes)
  auto large = store.store("app1", "calibration", "large.bin", "application/octet-stream", std::string(200, 'x'));
  EXPECT_FALSE(large.has_value());
  EXPECT_TRUE(large.error().find("exceeds maximum") != std::string::npos);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, UnknownCategory) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  EXPECT_TRUE(store.is_known_category("calibration"));
  EXPECT_FALSE(store.is_known_category("bogus"));
  EXPECT_FALSE(store.is_known_category("rosbags"));  // rosbags not managed by BulkDataStore
}

// @verifies REQ_INTEROP_071
TEST_F(BulkDataStoreTest, EmptyCategories) {
  BulkDataStore store(test_dir_.string(), 0);

  auto cats = store.list_categories();
  EXPECT_TRUE(cats.empty());
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, PathTraversalRejected) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  // entity_id with path traversal
  auto result1 = store.store("../etc", "calibration", "evil.bin", "application/octet-stream", "data");
  EXPECT_FALSE(result1.has_value());
  EXPECT_TRUE(result1.error().find("'..'") != std::string::npos);

  // entity_id with slash
  auto result2 = store.store("a/b", "calibration", "evil.bin", "application/octet-stream", "data");
  EXPECT_FALSE(result2.has_value());
  EXPECT_TRUE(result2.error().find("path separator") != std::string::npos);

  // category with traversal
  auto result3 = store.store("app1", "../secret", "evil.bin", "application/octet-stream", "data");
  EXPECT_FALSE(result3.has_value());
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreAtomicWriteDescriptorExists) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "calibration", "data.bin", "application/octet-stream", "payload");
  ASSERT_TRUE(result.has_value());

  auto dir = test_dir_ / "app1" / "calibration" / result->id;
  EXPECT_TRUE(std::filesystem::is_regular_file(dir / "descriptor.json"));

  // No orphan temp files
  EXPECT_FALSE(std::filesystem::exists(dir / ".data.tmp"));
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreUnknownCategoryRejected) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "bogus", "data.bin", "application/octet-stream", "payload");
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(result.error().find("Unknown category") != std::string::npos);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, ListItemsEmptyCategory) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto items = store.list_items("app1", "calibration");
  EXPECT_TRUE(items.empty());
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, ListItemsWrongEntity) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  ASSERT_TRUE(store.store("app1", "calibration", "data.bin", "application/octet-stream", "payload").has_value());

  auto items = store.list_items("app2", "calibration");
  EXPECT_TRUE(items.empty());
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreEmptyFile) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "calibration", "empty.bin", "application/octet-stream", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size, 0u);

  auto item = store.get_item("app1", "calibration", result->id);
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(item->size, 0u);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, StoreEmptyFilename) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "calibration", "", "application/octet-stream", "data");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->name, "upload");  // Default filename
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, RemoveThenListDoesNotInclude) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto r1 = store.store("app1", "calibration", "a.bin", "application/octet-stream", "aaa");
  auto r2 = store.store("app1", "calibration", "b.bin", "application/octet-stream", "bbb");
  ASSERT_TRUE(r1.has_value());
  ASSERT_TRUE(r2.has_value());

  ASSERT_TRUE(store.remove("app1", "calibration", r1->id).has_value());

  auto items = store.list_items("app1", "calibration");
  EXPECT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0].id, r2->id);
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, IdFormatIsReadable) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto result = store.store("app1", "calibration", "data.bin", "application/octet-stream", "x");
  ASSERT_TRUE(result.has_value());

  // ID should start with category name
  EXPECT_TRUE(result->id.rfind("calibration_", 0) == 0) << "ID: " << result->id;
  // ID should contain underscore-separated parts
  EXPECT_GT(result->id.size(), std::string("calibration_").size());
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, GetFilePathNonexistent) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});

  auto path = store.get_file_path("app1", "calibration", "nonexistent");
  EXPECT_FALSE(path.has_value());
}

// @verifies REQ_INTEROP_074
TEST_F(BulkDataStoreTest, MaxUploadZeroMeansUnlimited) {
  BulkDataStore store(test_dir_.string(), 0, {"calibration"});  // 0 = unlimited

  auto result = store.store("app1", "calibration", "big.bin", "application/octet-stream", std::string(10000, 'x'));
  EXPECT_TRUE(result.has_value());
}

}  // namespace
}  // namespace ros2_medkit_gateway
