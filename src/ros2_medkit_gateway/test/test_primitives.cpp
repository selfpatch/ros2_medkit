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
#include <type_traits>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"

namespace ros2_medkit_gateway {
namespace http {
namespace detail {

/// Test-only bridge that constructs FrameworkOrPluginAccess tokens for
/// direct primitive invocation. Forward-declared as a friend in
/// primitives.hpp so it can reach the otherwise-private default ctor.
struct PrimitivesAccessForTesting {
  static FrameworkOrPluginAccess token() {
    return FrameworkOrPluginAccess{};
  }
};

}  // namespace detail
}  // namespace http
}  // namespace ros2_medkit_gateway

namespace {

using ros2_medkit_gateway::ErrorInfo;
using ros2_medkit_gateway::http::detail::FrameworkOrPluginAccess;
using ros2_medkit_gateway::http::detail::PrimitivesAccessForTesting;
using ros2_medkit_gateway::http::detail::write_generic_error;
using ros2_medkit_gateway::http::detail::write_json_body;
using ros2_medkit_gateway::http::detail::write_oauth2_error;

/// Convenience accessor for the friend-gated token. Keeps test bodies short.
FrameworkOrPluginAccess access_token() {
  return PrimitivesAccessForTesting::token();
}

}  // namespace

// -----------------------------------------------------------------------------
// FrameworkOrPluginAccess friend-gating contract.
// -----------------------------------------------------------------------------

// At namespace scope (outside the friend list), the default ctor is private
// and inaccessible, so std::is_default_constructible_v evaluates to false.
// This static_assert breaks the build if a future refactor accidentally
// makes the constructor public.
static_assert(!std::is_default_constructible_v<FrameworkOrPluginAccess>,
              "FrameworkOrPluginAccess must not be default-constructible from outside its friend list");

TEST(PrimitivesAccess, TokenIsNotDefaultConstructibleFromOutside) {
  // Runtime mirror of the static_assert above - the friend list is the
  // only mechanism by which production code can synthesize a token.
  EXPECT_FALSE(std::is_default_constructible_v<FrameworkOrPluginAccess>);
}

// -----------------------------------------------------------------------------
// write_json_body
// -----------------------------------------------------------------------------

TEST(WriteJsonBody, SetsContentTypeBodyAndDefaultStatus) {
  httplib::Response res;
  nlohmann::json payload{{"hello", "world"}, {"count", 42}};

  write_json_body(access_token(), res, payload);

  EXPECT_EQ(res.status, 200);
  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");
  EXPECT_EQ(res.body, payload.dump(2));
}

TEST(WriteJsonBody, IndentsWithTwoSpaces) {
  httplib::Response res;
  nlohmann::json payload{{"nested", nlohmann::json{{"key", "value"}}}};

  write_json_body(access_token(), res, payload);

  // The two-space indent convention is observable: the closing `}` for the
  // nested object appears on its own line with leading spaces.
  EXPECT_NE(res.body.find("  \"nested\""), std::string::npos) << "Body must be indented with two spaces; got:\n"
                                                              << res.body;
}

TEST(WriteJsonBody, AcceptsCustomStatus) {
  httplib::Response res;
  nlohmann::json payload{{"created", true}};

  write_json_body(access_token(), res, payload, 201);

  EXPECT_EQ(res.status, 201);
  EXPECT_EQ(res.body, payload.dump(2));
}

TEST(WriteJsonBody, StatusZeroSentinelLeavesResStatusUntouched) {
  // Raw-route caller contract (PluginResponse / DocsHandlers /
  // SSEFaultHandler): when status=0, the writer must not overwrite
  // res.status. This lets callers pre-set 201/204 before calling the
  // primitive. cpp-httplib uses -1 as the "not yet set" marker and 201 as
  // a typical pre-set value; we exercise both.
  {
    httplib::Response res;
    res.status = 201;
    write_json_body(access_token(), res, nlohmann::json{{"x", 1}}, /*status=*/0);
    EXPECT_EQ(res.status, 201);
  }
  {
    httplib::Response res;  // Default-constructed: status == -1
    write_json_body(access_token(), res, nlohmann::json{{"x", 1}}, /*status=*/0);
    EXPECT_EQ(res.status, -1);
  }
}

// -----------------------------------------------------------------------------
// write_generic_error - SOVD GenericError schema
// -----------------------------------------------------------------------------

TEST(WriteGenericError, EmitsErrorCodeAndMessage) {
  httplib::Response res;
  ErrorInfo err;
  err.code = ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND;
  err.message = "Entity not found";
  err.http_status = 404;
  err.params = nlohmann::json::object();

  write_generic_error(access_token(), res, err);

  EXPECT_EQ(res.status, 404);
  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");
  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["error_code"], err.code);
  EXPECT_EQ(body["message"], err.message);
  EXPECT_FALSE(body.contains("parameters")) << "Empty params must not produce a 'parameters' key";
}

TEST(WriteGenericError, IncludesParametersWhenNonEmpty) {
  httplib::Response res;
  ErrorInfo err;
  err.code = ros2_medkit_gateway::ERR_INVALID_PARAMETER;
  err.message = "Invalid input";
  err.http_status = 400;
  err.params = nlohmann::json{{"field", "entity_id"}, {"value", "*"}};

  write_generic_error(access_token(), res, err);

  auto body = nlohmann::json::parse(res.body);
  ASSERT_TRUE(body.contains("parameters"));
  EXPECT_EQ(body["parameters"]["field"], "entity_id");
  EXPECT_EQ(body["parameters"]["value"], "*");
}

TEST(WriteGenericError, OmitsParametersForNullParams) {
  httplib::Response res;
  ErrorInfo err;
  err.code = ros2_medkit_gateway::ERR_INTERNAL_ERROR;
  err.message = "boom";
  err.http_status = 500;
  err.params = nullptr;  // SOVD GenericError omits 'parameters' for null

  write_generic_error(access_token(), res, err);

  auto body = nlohmann::json::parse(res.body);
  EXPECT_FALSE(body.contains("parameters"));
}

TEST(WriteGenericError, VendorCodeIsRemappedToVendorError) {
  httplib::Response res;
  ErrorInfo err;
  err.code = "x-medkit-ros2-service-unavailable";
  err.message = "ROS 2 service timed out";
  err.http_status = 503;

  write_generic_error(access_token(), res, err);

  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_VENDOR_ERROR);
  EXPECT_EQ(body["vendor_code"], err.code);
  EXPECT_EQ(body["message"], err.message);
}

TEST(WriteGenericError, ClampsStatusBelow400) {
  httplib::Response res;
  ErrorInfo err{"some-code", "msg", 200, nlohmann::json::object()};

  write_generic_error(access_token(), res, err);

  EXPECT_EQ(res.status, 400) << "Sub-400 status must be clamped to 400";
}

TEST(WriteGenericError, ClampsStatusAbove599) {
  httplib::Response res;
  ErrorInfo err{"some-code", "msg", 999, nlohmann::json::object()};

  write_generic_error(access_token(), res, err);

  EXPECT_EQ(res.status, 599) << "Above-599 status must be clamped to 599";
}

TEST(WriteGenericError, PreservesBoundaryStatus400And599) {
  {
    httplib::Response res;
    ErrorInfo err{"some-code", "msg", 400, nlohmann::json::object()};
    write_generic_error(access_token(), res, err);
    EXPECT_EQ(res.status, 400);
  }
  {
    httplib::Response res;
    ErrorInfo err{"some-code", "msg", 599, nlohmann::json::object()};
    write_generic_error(access_token(), res, err);
    EXPECT_EQ(res.status, 599);
  }
}

TEST(WriteGenericError, PreservesProvidedCode) {
  // Non-vendor codes pass through unchanged.
  httplib::Response res;
  ErrorInfo err{"custom-app-error", "details", 422, nlohmann::json::object()};

  write_generic_error(access_token(), res, err);

  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["error_code"], "custom-app-error");
  EXPECT_FALSE(body.contains("vendor_code"));
}

// -----------------------------------------------------------------------------
// write_oauth2_error - RFC 6749 §5.2
// -----------------------------------------------------------------------------

TEST(WriteOauth2Error, UsesErrorAndErrorDescription) {
  httplib::Response res;
  ErrorInfo err{"invalid_grant", "credentials rejected", 400, nlohmann::json::object()};

  write_oauth2_error(access_token(), res, err);

  EXPECT_EQ(res.status, 400);
  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");
  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["error"], "invalid_grant");
  EXPECT_EQ(body["error_description"], "credentials rejected");
}

TEST(WriteOauth2Error, OmitsSovdSpecificKeys) {
  httplib::Response res;
  ErrorInfo err{"invalid_client", "auth failed", 401, nlohmann::json{{"hint", "ignored"}}};

  write_oauth2_error(access_token(), res, err);

  auto body = nlohmann::json::parse(res.body);
  // RFC 6749 wire shape does not include any SOVD keys.
  EXPECT_FALSE(body.contains("error_code"));
  EXPECT_FALSE(body.contains("message"));
  EXPECT_FALSE(body.contains("parameters"));
  EXPECT_FALSE(body.contains("vendor_code"));
}

TEST(WriteOauth2Error, DoesNotRemapVendorCodes) {
  // OAuth2 endpoints would never legitimately receive an x-medkit-* code,
  // but if one were passed, the primitive must NOT apply the SOVD vendor
  // remap - it just writes the code verbatim into `error`.
  httplib::Response res;
  ErrorInfo err{"x-medkit-ros2-service-unavailable", "passthrough", 503, nlohmann::json::object()};

  write_oauth2_error(access_token(), res, err);

  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["error"], "x-medkit-ros2-service-unavailable");
  EXPECT_FALSE(body.contains("vendor_code"));
}

TEST(WriteOauth2Error, ClampsStatusBelow400) {
  httplib::Response res;
  ErrorInfo err{"invalid_request", "msg", 200, nlohmann::json::object()};

  write_oauth2_error(access_token(), res, err);

  EXPECT_EQ(res.status, 400);
}

TEST(WriteOauth2Error, ClampsStatusAbove599) {
  httplib::Response res;
  ErrorInfo err{"invalid_request", "msg", 1000, nlohmann::json::object()};

  write_oauth2_error(access_token(), res, err);

  EXPECT_EQ(res.status, 599);
}
