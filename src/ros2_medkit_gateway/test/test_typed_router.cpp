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

#include <regex>
#include <string>
#include <variant>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace {

using ros2_medkit_gateway::ErrorInfo;
using ros2_medkit_gateway::dto::FaultListQuery;
using ros2_medkit_gateway::http::Forwarded;
using ros2_medkit_gateway::http::NoContent;
using ros2_medkit_gateway::http::ResponseAttachments;
using ros2_medkit_gateway::http::Result;
using ros2_medkit_gateway::http::TypedRequest;
using ros2_medkit_gateway::http::ValidatorResult;

// Local stand-in for EntityInfo - the real type lives behind the gateway_ros2
// layer; here we only need a value type to parameterize ValidatorResult.
struct EntityInfoStub {
  std::string id;
};

// -----------------------------------------------------------------------------
// ResponseAttachments
// -----------------------------------------------------------------------------

TEST(TypedRouter_ResponseAttachments, BuilderChainSetsStatusAndAppendsHeaders) {
  ResponseAttachments attachments;
  attachments.with_status(201).with_header("Location", "/api/v1/components/c1");

  ASSERT_TRUE(attachments.status_override.has_value());
  EXPECT_EQ(*attachments.status_override, 201);

  ASSERT_EQ(attachments.headers.size(), 1U);
  EXPECT_EQ(attachments.headers[0].first, "Location");
  EXPECT_EQ(attachments.headers[0].second, "/api/v1/components/c1");
}

TEST(TypedRouter_ResponseAttachments, MultipleHeadersAreAppendedInOrder) {
  ResponseAttachments attachments;
  attachments.with_header("X-Medkit-Local-Only", "1").with_header("X-Medkit-Trace-Id", "abc123");

  ASSERT_EQ(attachments.headers.size(), 2U);
  EXPECT_EQ(attachments.headers[0].first, "X-Medkit-Local-Only");
  EXPECT_EQ(attachments.headers[0].second, "1");
  EXPECT_EQ(attachments.headers[1].first, "X-Medkit-Trace-Id");
  EXPECT_EQ(attachments.headers[1].second, "abc123");
}

TEST(TypedRouter_ResponseAttachments, DefaultIsEmpty) {
  ResponseAttachments attachments;
  EXPECT_FALSE(attachments.status_override.has_value());
  EXPECT_TRUE(attachments.headers.empty());
}

// -----------------------------------------------------------------------------
// Result<T>
// -----------------------------------------------------------------------------

TEST(TypedRouter_Result, OkRoundTrip) {
  Result<int> r{42};
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r.value(), 42);
}

TEST(TypedRouter_Result, ErrorRoundTrip) {
  ErrorInfo info;
  info.code = "x-medkit-test";
  info.message = "boom";
  info.http_status = 418;

  Result<int> r = tl::make_unexpected(info);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().code, "x-medkit-test");
  EXPECT_EQ(r.error().message, "boom");
  EXPECT_EQ(r.error().http_status, 418);
}

TEST(TypedRouter_Result, NoContentCanBeReturned) {
  Result<NoContent> r{NoContent{}};
  ASSERT_TRUE(r.has_value());
}

// -----------------------------------------------------------------------------
// ValidatorResult<T> with Forwarded
// -----------------------------------------------------------------------------

TEST(TypedRouter_ValidatorResult, HoldsForwardedOnProxy) {
  ValidatorResult<EntityInfoStub> r = tl::make_unexpected(std::variant<ErrorInfo, Forwarded>{Forwarded{}});

  ASSERT_FALSE(r.has_value());
  EXPECT_TRUE(std::holds_alternative<Forwarded>(r.error()));
  EXPECT_FALSE(std::holds_alternative<ErrorInfo>(r.error()));
}

TEST(TypedRouter_ValidatorResult, HoldsErrorInfoOnFailure) {
  ErrorInfo info;
  info.code = "x-medkit-entity-not-found";
  info.message = "not found";
  info.http_status = 404;

  ValidatorResult<EntityInfoStub> r = tl::make_unexpected(std::variant<ErrorInfo, Forwarded>{info});

  ASSERT_FALSE(r.has_value());
  ASSERT_TRUE(std::holds_alternative<ErrorInfo>(r.error()));
  EXPECT_EQ(std::get<ErrorInfo>(r.error()).code, "x-medkit-entity-not-found");
  EXPECT_EQ(std::get<ErrorInfo>(r.error()).http_status, 404);
}

TEST(TypedRouter_ValidatorResult, OkCarriesEntity) {
  ValidatorResult<EntityInfoStub> r{EntityInfoStub{"sensor_a"}};
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r.value().id, "sensor_a");
}

// -----------------------------------------------------------------------------
// TypedRequest
// -----------------------------------------------------------------------------

TEST(TypedRouter_TypedRequest, QueryParamReturnsNulloptWhenAbsent) {
  httplib::Request req;
  req.path = "/api/v1/components/c1/data";
  TypedRequest wrapper(req);

  EXPECT_FALSE(wrapper.query_param("severity").has_value());
}

TEST(TypedRouter_TypedRequest, QueryParamReturnsValueWhenPresent) {
  httplib::Request req;
  req.path = "/api/v1/components/c1/logs";
  req.params.emplace("severity", "error");
  req.params.emplace("context", "my.logger");
  TypedRequest wrapper(req);

  ASSERT_TRUE(wrapper.query_param("severity").has_value());
  EXPECT_EQ(*wrapper.query_param("severity"), "error");
  ASSERT_TRUE(wrapper.query_param("context").has_value());
  EXPECT_EQ(*wrapper.query_param("context"), "my.logger");
  EXPECT_FALSE(wrapper.query_param("limit").has_value());
}

TEST(TypedRouter_TypedRequest, TypedQueryParsesPresentParamsIntoDto) {
  httplib::Request req;
  req.path = "/api/v1/faults";
  req.params.emplace("status", "confirmed");
  req.params.emplace("include_muted", "true");
  TypedRequest wrapper(req);

  const auto q = wrapper.query<FaultListQuery>();
  ASSERT_TRUE(q.status.has_value());
  EXPECT_EQ(*q.status, "confirmed");
  EXPECT_TRUE(q.include_muted);
  EXPECT_FALSE(q.include_clusters);  // absent boolean -> default false
}

TEST(TypedRouter_TypedRequest, TypedQueryLeavesAbsentParamsAtDefault) {
  httplib::Request req;
  req.path = "/api/v1/faults";
  TypedRequest wrapper(req);

  const auto q = wrapper.query<FaultListQuery>();
  EXPECT_FALSE(q.status.has_value());
  EXPECT_FALSE(q.include_muted);
  EXPECT_FALSE(q.include_clusters);
}

TEST(TypedRouter_TypedRequest, FanOutDisabledTrueOnlyWhenHeaderPresent) {
  {
    httplib::Request req;
    TypedRequest wrapper(req);
    EXPECT_FALSE(wrapper.fan_out_disabled());
  }
  {
    httplib::Request req;
    req.headers.emplace("X-Medkit-No-Fan-Out", "1");
    TypedRequest wrapper(req);
    EXPECT_TRUE(wrapper.fan_out_disabled());
  }
  {
    // Different header that just happens to start with the same prefix must
    // not enable the flag.
    httplib::Request req;
    req.headers.emplace("X-Medkit-No-Fan-Out-Other", "1");
    TypedRequest wrapper(req);
    EXPECT_FALSE(wrapper.fan_out_disabled());
  }
}

TEST(TypedRouter_TypedRequest, HeaderReturnsValueWhenPresent) {
  httplib::Request req;
  req.headers.emplace("Authorization", "Bearer abc");
  TypedRequest wrapper(req);

  ASSERT_TRUE(wrapper.header("Authorization").has_value());
  EXPECT_EQ(*wrapper.header("Authorization"), "Bearer abc");
  EXPECT_FALSE(wrapper.header("X-Missing").has_value());
}

TEST(TypedRouter_TypedRequest, RawForFrameworkReturnsUnderlyingReference) {
  httplib::Request req;
  req.path = "/api/v1/health";
  TypedRequest wrapper(req);

  // raw_for_framework() is intentionally `[[deprecated]]` so any non-framework
  // caller gets a warning; suppress it here because the framework boundary is
  // exactly what this test exercises.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  EXPECT_EQ(&wrapper.raw_for_framework(), &req);
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
}

// -----------------------------------------------------------------------------
// TypedRequest::path_param
// -----------------------------------------------------------------------------

TEST(TypedRequestPathParam, EmptyNameReturnsInvalidParameter) {
  httplib::Request req;
  req.path = "/api/v1/components/c1";
  TypedRequest wrapper(req);

  auto result = wrapper.path_param("");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
  EXPECT_EQ(result.error().http_status, 400);
}

TEST(TypedRequestPathParam, NonNumericNameReturnsInvalidParameter) {
  httplib::Request req;
  req.path = "/api/v1/components/c1";
  TypedRequest wrapper(req);

  auto result = wrapper.path_param("foo");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
  EXPECT_EQ(result.error().http_status, 400);
}

TEST(TypedRequestPathParam, OutOfRangeIndexReturnsInvalidParameter) {
  httplib::Request req;
  // Populate matches with exactly 1 entry so index 5 is out of range.
  const std::string subject = "x";
  std::regex pattern("(x)");
  std::regex_match(subject, req.matches, pattern);
  ASSERT_GE(req.matches.size(), 1U);

  TypedRequest wrapper(req);
  auto result = wrapper.path_param("5");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_INVALID_PARAMETER);
  EXPECT_EQ(result.error().http_status, 400);
}

TEST(TypedRequestPathParam, ValidIndexReturnsValue) {
  httplib::Request req;
  // Build a real std::smatch by matching a fixed path against a regex with one
  // capture group, mirroring what cpp-httplib does internally for path-param
  // routes.
  const std::string subject = "/api/v1/components/c1";
  std::regex pattern("/api/v1/components/([^/]+)");
  ASSERT_TRUE(std::regex_match(subject, req.matches, pattern));
  ASSERT_GE(req.matches.size(), 2U);

  TypedRequest wrapper(req);
  // matches[0] is the full match; matches[1] is the first capture group.
  auto full = wrapper.path_param("0");
  ASSERT_TRUE(full.has_value());
  EXPECT_EQ(*full, "/api/v1/components/c1");

  auto entity_id = wrapper.path_param("1");
  ASSERT_TRUE(entity_id.has_value());
  EXPECT_EQ(*entity_id, "c1");
}

}  // namespace
