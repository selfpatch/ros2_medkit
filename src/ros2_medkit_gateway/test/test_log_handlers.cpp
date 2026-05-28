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

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/log_handlers.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::LogHandlers;
namespace dto = ros2_medkit_gateway::dto;
namespace http = ros2_medkit_gateway::http;

// LogHandlers uses a null GatewayNode and null AuthManager.
// PR-403 commit 23: all three handler methods now return `Result<TResponse>`
// and read the entity-id capture via `req.path_param("1")`. A default-
// constructed TypedRequest has no captures, so the helpers short-circuit with
// a 400 ERR_INVALID_REQUEST ErrorInfo before ever touching ctx_.node().
class LogHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  LogHandlers handlers_{ctx_};

  // Build a TypedRequest with no path captures. The typed `path_param("1")`
  // lookup returns ERR_INVALID_PARAMETER for the empty matches array, which
  // the handler maps back to ERR_INVALID_REQUEST. Held by reference inside
  // TypedRequest, so the underlying request must outlive the wrapper.
  static httplib::Request empty_request() {
    return httplib::Request{};
  }
};

// ============================================================================
// get_logs - returns 400 when entity-id capture is missing
// ============================================================================

// @verifies REQ_INTEROP_061
TEST_F(LogHandlersTest, GetLogsReturnsBadRequestWhenCaptureMissing) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.get_logs(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_061
TEST_F(LogHandlersTest, GetLogsErrorCarriesInvalidRequestCode) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.get_logs(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// ============================================================================
// get_logs_configuration - returns 400 when entity-id capture is missing
// ============================================================================

// @verifies REQ_INTEROP_063
TEST_F(LogHandlersTest, GetLogsConfigurationReturnsBadRequestWhenCaptureMissing) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.get_logs_configuration(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_063
TEST_F(LogHandlersTest, GetLogsConfigurationErrorCarriesInvalidRequestCode) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.get_logs_configuration(typed);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// ============================================================================
// put_logs_configuration - returns 400 when entity-id capture is missing
// ============================================================================

// @verifies REQ_INTEROP_064
TEST_F(LogHandlersTest, PutLogsConfigurationReturnsBadRequestWhenCaptureMissing) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.put_logs_configuration(typed, dto::LogConfiguration{});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_064
TEST_F(LogHandlersTest, PutLogsConfigurationErrorCarriesInvalidRequestCode) {
  auto req = empty_request();
  http::TypedRequest typed(req);
  auto result = handlers_.put_logs_configuration(typed, dto::LogConfiguration{});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}
