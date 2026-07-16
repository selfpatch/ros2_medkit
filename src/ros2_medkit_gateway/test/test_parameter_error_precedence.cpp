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
#include "ros2_medkit_gateway/core/http/handlers/config_handlers.hpp"

namespace ros2_medkit_gateway {
namespace {

using handlers::ParameterErrorAccumulator;

ParameterResult make_failure(ParameterErrorCode code, const std::string & message) {
  ParameterResult r;
  r.success = false;
  r.error_message = message;
  r.error_code = code;
  return r;
}

// Regression for the multi-node GET probe loop: a NOT_FOUND from one node
// must not mask a timeout/unavailable from another, in either iteration
// order.

TEST(ParameterErrorPrecedence, TimeoutThenNotFoundSurfacesUnavailable) {
  ParameterErrorAccumulator acc;
  acc.add(make_failure(ParameterErrorCode::TIMEOUT, "timed out"));
  acc.add(make_failure(ParameterErrorCode::NOT_FOUND, "Parameter not found"));

  EXPECT_FALSE(acc.all_not_found());
  EXPECT_EQ(acc.classification().status_code, 503);
  EXPECT_EQ(acc.classification().error_code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
  EXPECT_EQ(acc.worst().error_code, ParameterErrorCode::TIMEOUT);
}

TEST(ParameterErrorPrecedence, NotFoundThenTimeoutSurfacesUnavailable) {
  ParameterErrorAccumulator acc;
  acc.add(make_failure(ParameterErrorCode::NOT_FOUND, "Parameter not found"));
  acc.add(make_failure(ParameterErrorCode::TIMEOUT, "timed out"));

  EXPECT_FALSE(acc.all_not_found());
  EXPECT_EQ(acc.classification().status_code, 503);
  EXPECT_EQ(acc.classification().error_code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
  EXPECT_EQ(acc.worst().error_code, ParameterErrorCode::TIMEOUT);
}

TEST(ParameterErrorPrecedence, NotFoundNeverReplacesUnavailable) {
  ParameterErrorAccumulator acc;
  acc.add(make_failure(ParameterErrorCode::SERVICE_UNAVAILABLE, "service not available"));
  acc.add(make_failure(ParameterErrorCode::NOT_FOUND, "Parameter not found"));
  acc.add(make_failure(ParameterErrorCode::NOT_FOUND, "Parameter not found"));

  EXPECT_FALSE(acc.all_not_found());
  EXPECT_EQ(acc.classification().status_code, 503);
  EXPECT_EQ(acc.worst().error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);
}

TEST(ParameterErrorPrecedence, AllNotFoundStays404) {
  ParameterErrorAccumulator acc;
  acc.add(make_failure(ParameterErrorCode::NOT_FOUND, "Parameter not found"));
  acc.add(make_failure(ParameterErrorCode::NOT_FOUND, "Parameter not found"));

  EXPECT_TRUE(acc.all_not_found());
  EXPECT_EQ(acc.classification().status_code, 404);
  EXPECT_EQ(acc.classification().error_code, ERR_RESOURCE_NOT_FOUND);
}

}  // namespace
}  // namespace ros2_medkit_gateway
