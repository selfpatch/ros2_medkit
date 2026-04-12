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

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

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

// =============================================================================
// merge_peer_items with real AggregationManager + mock server
// =============================================================================

namespace {

class MockServer {
 public:
  ~MockServer() {
    if (server_) {
      server_->stop();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  MockServer(const MockServer &) = delete;
  MockServer & operator=(const MockServer &) = delete;
  MockServer(MockServer &&) = delete;
  MockServer & operator=(MockServer &&) = delete;
  MockServer() = default;

  httplib::Server & server() {
    if (!server_) {
      server_ = std::make_unique<httplib::Server>();
    }
    return *server_;
  }

  int start() {
    int port = server().bind_to_any_port("127.0.0.1");
    thread_ = std::thread([this]() {
      server_->listen_after_bind();
    });
    return port;
  }

 private:
  std::unique_ptr<httplib::Server> server_;
  std::thread thread_;
};

}  // namespace

TEST(FanOutHelpers, merge_peer_items_appends_peer_items_to_result) {
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/functions/f1/logs", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "peer_log_1"}}, {{"id", "peer_log_2"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "test_peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/functions/f1/logs";
  json result;
  result["items"] = json::array({{{"id", "local_log_1"}}});
  XMedkit ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_EQ(result["items"].size(), 3u);
  EXPECT_EQ(result["items"][0]["id"], "local_log_1");
  EXPECT_EQ(result["items"][1]["id"], "peer_log_1");
  EXPECT_EQ(result["items"][2]["id"], "peer_log_2");
  EXPECT_TRUE(ext.empty());  // no partial failure
}

TEST(FanOutHelpers, merge_peer_items_sets_partial_on_peer_failure) {
  // Use a dead port so the peer is unreachable
  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 200;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:59999";
  peer.name = "dead_peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  // Don't check health - peer starts as unhealthy, fan_out_get skips it.
  // Force healthy flag via a mock health endpoint that 404s:
  // Actually, fan_out_get only queries healthy peers. With no health check,
  // peers default to unhealthy and fan_out returns empty. Instead, use a
  // mock that returns health OK but fails on the actual request.
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  // No handler for /api/v1/functions/f1/logs -> peer returns 404
  int port = mock.start();

  AggregationConfig config2;
  config2.enabled = true;
  config2.timeout_ms = 2000;
  AggregationConfig::PeerConfig peer2;
  peer2.url = "http://127.0.0.1:" + std::to_string(port);
  peer2.name = "failing_peer";
  config2.peers.push_back(peer2);

  AggregationManager agg2(config2);
  agg2.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/functions/f1/logs";
  json result;
  result["items"] = json::array();
  XMedkit ext;

  merge_peer_items(&agg2, req, result, ext);

  EXPECT_EQ(result["items"].size(), 0u);
  EXPECT_FALSE(ext.empty());
  auto built = ext.build();
  EXPECT_TRUE(built.value("partial", false));
  ASSERT_TRUE(built.contains("failed_peers"));
  EXPECT_EQ(built["failed_peers"].size(), 1u);
  EXPECT_EQ(built["failed_peers"][0], "failing_peer");
}

TEST(FanOutHelpers, merge_peer_items_creates_items_when_missing_and_peer_has_data) {
  MockServer mock;
  mock.server().Get("/api/v1/health", [](const httplib::Request &, httplib::Response & res) {
    res.set_content(R"({"status":"healthy"})", "application/json");
  });
  mock.server().Get("/api/v1/apps/a/data", [](const httplib::Request &, httplib::Response & res) {
    json body = {{"items", {{{"id", "peer_item"}}}}};
    res.set_content(body.dump(), "application/json");
  });
  int port = mock.start();

  AggregationConfig config;
  config.enabled = true;
  config.timeout_ms = 5000;
  AggregationConfig::PeerConfig peer;
  peer.url = "http://127.0.0.1:" + std::to_string(port);
  peer.name = "peer";
  config.peers.push_back(peer);

  AggregationManager agg(config);
  agg.check_all_health();

  httplib::Request req;
  req.path = "/api/v1/apps/a/data";
  json result;
  // No "items" key - merge_peer_items should create it
  XMedkit ext;

  merge_peer_items(&agg, req, result, ext);

  ASSERT_TRUE(result.contains("items"));
  ASSERT_EQ(result["items"].size(), 1u);
  EXPECT_EQ(result["items"][0]["id"], "peer_item");
}
