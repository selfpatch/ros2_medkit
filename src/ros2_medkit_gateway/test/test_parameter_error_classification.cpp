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

#include <string>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/parameter_error_classification.hpp"

using ros2_medkit_gateway::ERR_INTERNAL_ERROR;
using ros2_medkit_gateway::ERR_INVALID_PARAMETER;
using ros2_medkit_gateway::ERR_INVALID_REQUEST;
using ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND;
using ros2_medkit_gateway::ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
using ros2_medkit_gateway::ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
using ros2_medkit_gateway::ParameterErrorCode;
using ros2_medkit_gateway::ParameterResult;
using ros2_medkit_gateway::handlers::classify_parameter_error;

namespace {

ParameterResult failure(ParameterErrorCode code, const std::string & message) {
  ParameterResult result;
  result.success = false;
  result.error_message = message;
  result.error_code = code;
  return result;
}

}  // namespace

// =============================================================================
// classify_parameter_error maps the structured ParameterErrorCode set by the
// transport to an HTTP status + SOVD error code. These pin the contract so the
// mapping cannot silently drift.
// =============================================================================

TEST(ParameterErrorClassification, TimeoutIs503) {
  auto c =
      classify_parameter_error(failure(ParameterErrorCode::TIMEOUT, "Parameter service did not respond for node: /x"));
  EXPECT_EQ(c.status_code, 503);
  EXPECT_EQ(c.error_code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
}

TEST(ParameterErrorClassification, ServiceUnavailableIs503) {
  auto c = classify_parameter_error(
      failure(ParameterErrorCode::SERVICE_UNAVAILABLE, "Parameter service not available for node: /x"));
  EXPECT_EQ(c.status_code, 503);
  EXPECT_EQ(c.error_code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
}

TEST(ParameterErrorClassification, NotFoundIs404) {
  auto c = classify_parameter_error(failure(ParameterErrorCode::NOT_FOUND, "Parameter not currently set: foo"));
  EXPECT_EQ(c.status_code, 404);
  EXPECT_EQ(c.error_code, ERR_RESOURCE_NOT_FOUND);
}

TEST(ParameterErrorClassification, ReadOnlyIs403) {
  auto c = classify_parameter_error(failure(ParameterErrorCode::READ_ONLY, "read-only"));
  EXPECT_EQ(c.status_code, 403);
  EXPECT_EQ(c.error_code, ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY);
}

TEST(ParameterErrorClassification, InvalidValueIs400) {
  auto c = classify_parameter_error(failure(ParameterErrorCode::INVALID_VALUE, "bad value"));
  EXPECT_EQ(c.status_code, 400);
  EXPECT_EQ(c.error_code, ERR_INVALID_PARAMETER);
}

TEST(ParameterErrorClassification, InternalErrorIs500) {
  auto c = classify_parameter_error(failure(ParameterErrorCode::INTERNAL_ERROR, "boom"));
  EXPECT_EQ(c.status_code, 500);
  EXPECT_EQ(c.error_code, ERR_INTERNAL_ERROR);
}

// Regression pin for issue #542: the removed legacy string-matching fallback
// only ran on a failure carrying ParameterErrorCode::NONE, which the shipped
// transport never emits, so it was dead code. Had it run it would have mapped
// by message substrings - agreeing with the structured mapping for some (e.g.
// "not available" -> 503) and misrouting the rest to 400 ERR_INVALID_REQUEST.
// A NONE failure is a gateway-side defect, so it must surface as 500
// internal-error regardless of message, never as a 400 that blames the client.
TEST(ParameterErrorClassification, NoneFailureIsInternalErrorNot400) {
  // A message the old fallback matched and would have misrouted to 503 (its
  // "not available" substring), still NONE-coded.
  auto not_available =
      classify_parameter_error(failure(ParameterErrorCode::NONE, "Parameter service not available for node: /x"));
  EXPECT_EQ(not_available.status_code, 500);
  EXPECT_EQ(not_available.error_code, ERR_INTERNAL_ERROR);
  EXPECT_NE(not_available.status_code, 400);
  EXPECT_NE(not_available.error_code, ERR_INVALID_REQUEST);

  // A message the old fallback did not match, so it fell through to 400
  // invalid-request.
  auto not_set = classify_parameter_error(failure(ParameterErrorCode::NONE, "Parameter not currently set: foo"));
  EXPECT_EQ(not_set.status_code, 500);
  EXPECT_EQ(not_set.error_code, ERR_INTERNAL_ERROR);
  EXPECT_NE(not_set.status_code, 400);
  EXPECT_NE(not_set.error_code, ERR_INVALID_REQUEST);
}
