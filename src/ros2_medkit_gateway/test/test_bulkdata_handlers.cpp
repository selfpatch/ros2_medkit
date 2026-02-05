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
