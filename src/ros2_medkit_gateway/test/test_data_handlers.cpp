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
#include <string>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/data_handlers.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::DataHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
namespace http = ros2_medkit_gateway::http;

// PR-403 commit 28: validation-only tests (no GatewayNode). These cover the
// path_param("1") / path_param("2") short-circuit at the top of each typed
// handler. Default-constructed TypedRequest carries no captures, so the
// handler returns ERR_INVALID_REQUEST (400) before touching the cache.
// data_categories / data_groups always render 501 because they ignore the
// request entirely.

class DataHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  DataHandlers handlers_{ctx_};
};

// ============================================================================
// data_categories - always returns 501 Not Implemented (ISO 17978-3 §7.9)
// ============================================================================

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesReturns501) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_categories(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesErrorCodeIsNotImplemented) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_categories(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesErrorBodyContainsMessage) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_categories(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().message.empty());
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesErrorBodyContainsFeatureParameter) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_categories(req);
  ASSERT_FALSE(result.has_value());
  ASSERT_TRUE(result.error().params.contains("feature"));
  EXPECT_EQ(result.error().params["feature"], "data-categories");
}

// ============================================================================
// data_groups - always returns 501 Not Implemented (ISO 17978-3 §7.9)
// ============================================================================

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsReturns501) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_groups(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsErrorCodeIsNotImplemented) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_groups(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsErrorBodyContainsMessage) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_groups(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().message.empty());
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsErrorBodyContainsFeatureParameter) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.data_groups(req);
  ASSERT_FALSE(result.has_value());
  ASSERT_TRUE(result.error().params.contains("feature"));
  EXPECT_EQ(result.error().params["feature"], "data-groups");
}

// ============================================================================
// list_data - returns 400 when route captures are missing
// ============================================================================

// @verifies REQ_INTEROP_018
TEST_F(DataHandlersTest, ListDataReturnsBadRequestWhenMatchesMissing) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.list_data(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_018
TEST_F(DataHandlersTest, ListDataBadRequestBodyContainsErrorCode) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.list_data(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().code.empty());
}

// ============================================================================
// get_data_item - returns 400 when route captures are missing
// ============================================================================

// @verifies REQ_INTEROP_019
TEST_F(DataHandlersTest, GetDataItemReturnsBadRequestWhenMatchesMissing) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.get_data_item(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_019
TEST_F(DataHandlersTest, GetDataItemBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.get_data_item(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// ============================================================================
// put_data_item - returns 400 when route captures are missing
// ============================================================================

// @verifies REQ_INTEROP_020
TEST_F(DataHandlersTest, PutDataItemReturnsBadRequestWhenMatchesMissing) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.put_data_item(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 400);
}

// @verifies REQ_INTEROP_020
TEST_F(DataHandlersTest, PutDataItemBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request raw_req;
  http::TypedRequest req(raw_req);
  auto result = handlers_.put_data_item(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_REQUEST);
}
