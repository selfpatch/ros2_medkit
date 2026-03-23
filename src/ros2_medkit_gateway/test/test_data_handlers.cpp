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

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/data_handlers.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::DataHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;

// DataHandlers uses a null GatewayNode and null AuthManager.
// This is safe because:
// - handle_data_categories/handle_data_groups only call HandlerContext::send_error() (static)
// - handle_list_data/handle_get_data_item/handle_put_data_item check req.matches.size()
//   before accessing ctx_.node(), so default-constructed requests (size 0) return 400 first

class DataHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  DataHandlers handlers_{ctx_};
};

// ============================================================================
// handle_data_categories — always returns 501 Not Implemented (ISO 17978-3 §7.9)
// ============================================================================

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesReturns501) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_categories(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesResponseBodyIsValidJson) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_categories(req, res);
  EXPECT_NO_THROW(json::parse(res.body));
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesErrorCodeIsNotImplemented) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_categories(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesErrorBodyContainsMessage) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_categories(req, res);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("message"));
}

// @verifies REQ_INTEROP_016
TEST_F(DataHandlersTest, DataCategoriesErrorBodyContainsFeatureParameter) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_categories(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("parameters"));
  EXPECT_EQ(body["parameters"]["feature"], "data-categories");
}

// ============================================================================
// handle_data_groups — always returns 501 Not Implemented (ISO 17978-3 §7.9)
// ============================================================================

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsReturns501) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_groups(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsResponseBodyIsValidJson) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_groups(req, res);
  EXPECT_NO_THROW(json::parse(res.body));
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsErrorCodeIsNotImplemented) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_groups(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsErrorBodyContainsMessage) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_groups(req, res);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("message"));
}

// @verifies REQ_INTEROP_017
TEST_F(DataHandlersTest, DataGroupsErrorBodyContainsFeatureParameter) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_data_groups(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("parameters"));
  EXPECT_EQ(body["parameters"]["feature"], "data-groups");
}

// ============================================================================
// handle_list_data — returns 400 when route matches are missing
// ============================================================================

// @verifies REQ_INTEROP_018
TEST_F(DataHandlersTest, ListDataReturnsBadRequestWhenMatchesMissing) {
  // Default-constructed req has empty matches (size 0 < 2)
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_list_data(req, res);
  EXPECT_EQ(res.status, 400);
}

// @verifies REQ_INTEROP_018
TEST_F(DataHandlersTest, ListDataBadRequestBodyContainsErrorCode) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_list_data(req, res);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("error_code"));
}

// ============================================================================
// handle_get_data_item — returns 400 when route matches are missing
// ============================================================================

// @verifies REQ_INTEROP_019
TEST_F(DataHandlersTest, GetDataItemReturnsBadRequestWhenMatchesMissing) {
  // Default-constructed req has empty matches (size 0 < 3)
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_data_item(req, res);
  EXPECT_EQ(res.status, 400);
}

// @verifies REQ_INTEROP_019
TEST_F(DataHandlersTest, GetDataItemBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_data_item(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// ============================================================================
// handle_put_data_item — returns 400 when route matches are missing
// ============================================================================

// @verifies REQ_INTEROP_020
TEST_F(DataHandlersTest, PutDataItemReturnsBadRequestWhenMatchesMissing) {
  // Default-constructed req has empty matches (size 0 < 3)
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_put_data_item(req, res);
  EXPECT_EQ(res.status, 400);
}

// @verifies REQ_INTEROP_020
TEST_F(DataHandlersTest, PutDataItemBadRequestBodyContainsInvalidRequestErrorCode) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_put_data_item(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("error_code"));
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}
