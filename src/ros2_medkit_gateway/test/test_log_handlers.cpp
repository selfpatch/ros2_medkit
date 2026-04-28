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

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/log_handlers.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::LogHandlers;

// LogHandlers uses a null GatewayNode and null AuthManager.
// This is safe because all three handler methods check req.matches.size() < 2
// before accessing ctx_.node(), so default-constructed requests (size 0) return 400 first.

class LogHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  LogHandlers handlers_{ctx_};
};

// ============================================================================
// handle_get_logs — returns 400 when route matches are missing
// ============================================================================

// @verifies REQ_INTEROP_061
TEST_F(LogHandlersTest, GetLogsReturnsBadRequestWhenMatchesMissing) {
  // Default-constructed req has empty matches (size 0 < 2)
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs(req, res);
  EXPECT_EQ(res.status, 400);
}

// @verifies REQ_INTEROP_061
TEST_F(LogHandlersTest, GetLogsBadRequestBodyIsValidJson) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs(req, res);
  EXPECT_NO_THROW(json::parse(res.body));
}

// @verifies REQ_INTEROP_061
TEST_F(LogHandlersTest, GetLogsBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// ============================================================================
// handle_get_logs_configuration — returns 400 when route matches are missing
// ============================================================================

// @verifies REQ_INTEROP_063
TEST_F(LogHandlersTest, GetLogsConfigurationReturnsBadRequestWhenMatchesMissing) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs_configuration(req, res);
  EXPECT_EQ(res.status, 400);
}

// @verifies REQ_INTEROP_063
TEST_F(LogHandlersTest, GetLogsConfigurationBadRequestBodyIsValidJson) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs_configuration(req, res);
  EXPECT_NO_THROW(json::parse(res.body));
}

// @verifies REQ_INTEROP_063
TEST_F(LogHandlersTest, GetLogsConfigurationBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_logs_configuration(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// ============================================================================
// handle_put_logs_configuration — returns 400 when route matches are missing
// ============================================================================

// @verifies REQ_INTEROP_064
TEST_F(LogHandlersTest, PutLogsConfigurationReturnsBadRequestWhenMatchesMissing) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_put_logs_configuration(req, res);
  EXPECT_EQ(res.status, 400);
}

// @verifies REQ_INTEROP_064
TEST_F(LogHandlersTest, PutLogsConfigurationBadRequestBodyIsValidJson) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_put_logs_configuration(req, res);
  EXPECT_NO_THROW(json::parse(res.body));
}

// @verifies REQ_INTEROP_064
TEST_F(LogHandlersTest, PutLogsConfigurationBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_put_logs_configuration(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}
