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

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/models/error_info.hpp"

using ros2_medkit_gateway::ErrorInfo;

TEST(ErrorInfoTest, DefaultConstructs) {
  ErrorInfo err;
  EXPECT_TRUE(err.code.empty());
  EXPECT_TRUE(err.message.empty());
  EXPECT_EQ(err.http_status, 500);
  EXPECT_TRUE(err.params.is_object());
  EXPECT_TRUE(err.params.empty());
}

TEST(ErrorInfoTest, BracedInitSetsFields) {
  ErrorInfo err{ros2_medkit_gateway::ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE, "topic /foo not found", 404,
                nlohmann::json{{"topic", "/foo"}}};
  EXPECT_EQ(err.code, ros2_medkit_gateway::ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE);
  EXPECT_EQ(err.message, "topic /foo not found");
  EXPECT_EQ(err.http_status, 404);
  EXPECT_EQ(err.params["topic"], "/foo");
}

TEST(ErrorInfoTest, CopyableAndMovable) {
  ErrorInfo a{ros2_medkit_gateway::ERR_INTERNAL_ERROR, "boom", 500, nlohmann::json::object()};
  ErrorInfo b = a;
  EXPECT_EQ(b.code, ros2_medkit_gateway::ERR_INTERNAL_ERROR);
  ErrorInfo c = std::move(a);
  EXPECT_EQ(c.message, "boom");
}

TEST(ErrorInfoTest, HttpStatusIsInSovdRange) {
  // Not a runtime check - documents the contract. SOVD spec requires 4xx/5xx.
  ErrorInfo err{ros2_medkit_gateway::ERR_INVALID_REQUEST, "malformed", 400, nlohmann::json::object()};
  EXPECT_GE(err.http_status, 400);
  EXPECT_LT(err.http_status, 600);
}
