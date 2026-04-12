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
#include <httplib.h>

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/http/fan_out_helpers.hpp"

using namespace ros2_medkit_gateway;  // NOLINT(google-build-using-namespace)
using nlohmann::json;

// =============================================================================
// url_encode_param
// =============================================================================

TEST(FanOutHelpers, url_encode_param_empty_string) {
  EXPECT_EQ(url_encode_param(""), "");
}

TEST(FanOutHelpers, url_encode_param_unreserved_chars_pass_through) {
  EXPECT_EQ(url_encode_param("abcXYZ019"), "abcXYZ019");
  EXPECT_EQ(url_encode_param("-_.~"), "-_.~");
}

TEST(FanOutHelpers, url_encode_param_encodes_space) {
  EXPECT_EQ(url_encode_param("hello world"), "hello%20world");
}

TEST(FanOutHelpers, url_encode_param_encodes_special_chars) {
  EXPECT_EQ(url_encode_param("a=b&c"), "a%3Db%26c");
  EXPECT_EQ(url_encode_param("100%"), "100%25");
  EXPECT_EQ(url_encode_param("foo+bar"), "foo%2Bbar");
  EXPECT_EQ(url_encode_param("/path/to"), "%2Fpath%2Fto");
}

TEST(FanOutHelpers, url_encode_param_encodes_unicode_bytes) {
  // UTF-8 for U+00E9 (e-acute) is 0xC3 0xA9
  std::string input = "\xC3\xA9";
  EXPECT_EQ(url_encode_param(input), "%C3%A9");
}

// =============================================================================
// build_fan_out_path
// =============================================================================

TEST(FanOutHelpers, build_fan_out_path_no_params) {
  httplib::Request req;
  req.path = "/api/v1/apps/temp_sensor/logs";
  EXPECT_EQ(build_fan_out_path(req), "/api/v1/apps/temp_sensor/logs");
}

TEST(FanOutHelpers, build_fan_out_path_single_param) {
  httplib::Request req;
  req.path = "/api/v1/apps/sensor/logs";
  req.params.emplace("severity", "error");
  EXPECT_EQ(build_fan_out_path(req), "/api/v1/apps/sensor/logs?severity=error");
}

TEST(FanOutHelpers, build_fan_out_path_multiple_params) {
  httplib::Request req;
  req.path = "/api/v1/apps/sensor/logs";
  req.params.emplace("context", "my.logger");
  req.params.emplace("severity", "warning");
  auto result = build_fan_out_path(req);
  // multimap iteration order is sorted by key
  EXPECT_TRUE(result.find("context=my.logger") != std::string::npos);
  EXPECT_TRUE(result.find("severity=warning") != std::string::npos);
  EXPECT_EQ(result[req.path.size()], '?');
}

TEST(FanOutHelpers, build_fan_out_path_encodes_special_values) {
  httplib::Request req;
  req.path = "/api/v1/apps/sensor/logs";
  req.params.emplace("context", "my logger/test");
  auto result = build_fan_out_path(req);
  EXPECT_TRUE(result.find("context=my%20logger%2Ftest") != std::string::npos);
}

TEST(FanOutHelpers, build_fan_out_path_encodes_keys) {
  httplib::Request req;
  req.path = "/api/v1/test";
  req.params.emplace("a=b", "val");
  auto result = build_fan_out_path(req);
  EXPECT_TRUE(result.find("a%3Db=val") != std::string::npos);
}

// =============================================================================
// merge_peer_items
// =============================================================================

TEST(FanOutHelpers, merge_peer_items_null_aggregation_manager_is_noop) {
  httplib::Request req;
  req.path = "/api/v1/test";
  json result;
  result["items"] = json::array({{"id", "local1"}});
  XMedkit ext;

  merge_peer_items(nullptr, req, result, ext);

  EXPECT_EQ(result["items"].size(), 1u);
  EXPECT_TRUE(ext.empty());
}

TEST(FanOutHelpers, merge_peer_items_skips_when_no_fan_out_header_set) {
  httplib::Request req;
  req.path = "/api/v1/test";
  req.headers.emplace("X-Medkit-No-Fan-Out", "1");
  json result;
  result["items"] = json::array();
  XMedkit ext;

  // Even with a non-null pointer, should skip due to header.
  // We pass a bogus pointer since it should never be dereferenced.
  auto * fake_agg = reinterpret_cast<AggregationManager *>(0x1);  // NOLINT
  merge_peer_items(fake_agg, req, result, ext);

  EXPECT_EQ(result["items"].size(), 0u);
  EXPECT_TRUE(ext.empty());
}

TEST(FanOutHelpers, merge_peer_items_null_agg_does_not_touch_result) {
  httplib::Request req;
  req.path = "/api/v1/test";
  json result;
  // No "items" key at all
  XMedkit ext;

  merge_peer_items(nullptr, req, result, ext);

  // null agg = no-op, result should be untouched
  EXPECT_FALSE(result.contains("items"));
}
