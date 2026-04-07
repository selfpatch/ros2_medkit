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

#include <regex>
#include <string>

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_http_types.hpp"

using namespace ros2_medkit_gateway;

TEST(PluginRequestTest, PathParam) {
  httplib::Request req;
  req.path = "/api/v1/apps/my_app/x-medkit-test";
  std::string path = "/api/v1/apps/my_app/x-medkit-test";
  std::regex pattern(R"(/api/v1/apps/([^/]+)/x-medkit-test)");
  std::regex_search(path, req.matches, pattern);

  PluginRequest preq(&req);
  EXPECT_EQ(preq.path_param(1), "my_app");
}

TEST(PluginRequestTest, PathParamOutOfRange) {
  httplib::Request req;
  PluginRequest preq(&req);
  EXPECT_EQ(preq.path_param(5), "");
}

TEST(PluginRequestTest, Header) {
  httplib::Request req;
  req.set_header("X-Client-Id", "test-client");

  PluginRequest preq(&req);
  EXPECT_EQ(preq.header("X-Client-Id"), "test-client");
  EXPECT_EQ(preq.header("Missing-Header"), "");
}

TEST(PluginRequestTest, Path) {
  httplib::Request req;
  req.path = "/api/v1/apps/my_app/data";

  PluginRequest preq(&req);
  EXPECT_EQ(preq.path(), "/api/v1/apps/my_app/data");
}

TEST(PluginRequestTest, Body) {
  httplib::Request req;
  req.body = R"({"key": "value"})";

  PluginRequest preq(&req);
  EXPECT_EQ(preq.body(), R"({"key": "value"})");
}

TEST(PluginRequestTest, QueryParam) {
  httplib::Request req;
  req.params.emplace("filter", "active");
  req.params.emplace("limit", "10");

  PluginRequest preq(&req);
  EXPECT_EQ(preq.query_param("filter"), "active");
  EXPECT_EQ(preq.query_param("limit"), "10");
  EXPECT_EQ(preq.query_param("nonexistent"), "");
}

TEST(PluginResponseTest, SendJson) {
  httplib::Response res;

  PluginResponse pres(&res);
  pres.send_json({{"status", "ok"}});

  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["status"], "ok");
}

TEST(PluginResponseTest, SendError) {
  httplib::Response res;

  PluginResponse pres(&res);
  pres.send_error(404, "not-found", "Entity not found");

  EXPECT_EQ(res.status, 404);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["error_code"], "not-found");
  EXPECT_EQ(body["message"], "Entity not found");
}

TEST(PluginResponseTest, SendErrorClampsLowStatus) {
  httplib::Response res;
  PluginResponse pres(&res);
  pres.send_error(200, "bad-status", "Should clamp to 400");
  EXPECT_EQ(res.status, 400);
}

TEST(PluginResponseTest, SendErrorClampsHighStatus) {
  httplib::Response res;
  PluginResponse pres(&res);
  pres.send_error(999, "bad-status", "Should clamp to 599");
  EXPECT_EQ(res.status, 599);
}

TEST(PluginResponseTest, SendErrorClampsNegativeStatus) {
  httplib::Response res;
  PluginResponse pres(&res);
  pres.send_error(-1, "bad-status", "Should clamp to 400");
  EXPECT_EQ(res.status, 400);
}

// =============================================================================
// send_plugin_error tests (HandlerContext static method)
// =============================================================================

TEST(SendPluginErrorTest, ClampsLowStatus) {
  httplib::Response res;
  handlers::HandlerContext::send_plugin_error(res, 200, "test error");
  EXPECT_EQ(res.status, 400);
}

TEST(SendPluginErrorTest, ClampsHighStatus) {
  httplib::Response res;
  handlers::HandlerContext::send_plugin_error(res, 999, "test error");
  EXPECT_EQ(res.status, 599);
}

TEST(SendPluginErrorTest, PassesThroughValidStatus) {
  httplib::Response res;
  handlers::HandlerContext::send_plugin_error(res, 503, "service unavailable");
  EXPECT_EQ(res.status, 503);
  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["message"], "service unavailable");
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-plugin-error");
}

TEST(SendPluginErrorTest, TruncatesLongMessage) {
  httplib::Response res;
  std::string long_msg(600, 'x');
  handlers::HandlerContext::send_plugin_error(res, 500, long_msg);
  auto body = nlohmann::json::parse(res.body);
  std::string msg = body["message"];
  EXPECT_LE(msg.size(), 516u);  // 512 + "..."
  EXPECT_TRUE(msg.find("...") != std::string::npos);
}

TEST(SendPluginErrorTest, IncludesExtraParams) {
  httplib::Response res;
  handlers::HandlerContext::send_plugin_error(res, 500, "fail", {{"entity_id", "ecu1"}});
  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["parameters"]["entity_id"], "ecu1");
}
