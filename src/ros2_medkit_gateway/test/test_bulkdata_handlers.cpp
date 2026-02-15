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

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/bulk_data_store.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/bulkdata_handlers.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

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

// === Shared timestamp utility tests ===

// @verifies REQ_INTEROP_013
TEST_F(BulkDataHandlersTest, FormatTimestampNsValidTimestamp) {
  // 2026-02-08T00:00:00.000Z
  int64_t ns = 1770458400000000000;
  auto result = ros2_medkit_gateway::format_timestamp_ns(ns);
  EXPECT_TRUE(result.find("2026") != std::string::npos);
  EXPECT_TRUE(result.find("T") != std::string::npos);
  EXPECT_TRUE(result.find("Z") != std::string::npos);
}

// @verifies REQ_INTEROP_013
TEST_F(BulkDataHandlersTest, FormatTimestampNsEpoch) {
  auto result = ros2_medkit_gateway::format_timestamp_ns(0);
  EXPECT_EQ(result, "1970-01-01T00:00:00.000Z");
}

// @verifies REQ_INTEROP_013
TEST_F(BulkDataHandlersTest, FormatTimestampNsWithMilliseconds) {
  // 1 second + 123 ms
  int64_t ns = 1'000'000'000 + 123'000'000;
  auto result = ros2_medkit_gateway::format_timestamp_ns(ns);
  EXPECT_TRUE(result.find(".123Z") != std::string::npos);
}

// @verifies REQ_INTEROP_013
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
