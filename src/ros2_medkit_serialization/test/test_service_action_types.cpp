// Copyright 2026 Selfpatch
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

#include "ros2_medkit_serialization/service_action_types.hpp"

namespace ros2_medkit_serialization {

class ServiceActionTypesTest : public ::testing::Test {};

TEST_F(ServiceActionTypesTest, GetServiceRequestType) {
  EXPECT_EQ(ServiceActionTypes::get_service_request_type("std_srvs/srv/SetBool"), "std_srvs/srv/SetBool_Request");
}

TEST_F(ServiceActionTypesTest, GetServiceResponseType) {
  EXPECT_EQ(ServiceActionTypes::get_service_response_type("std_srvs/srv/SetBool"), "std_srvs/srv/SetBool_Response");
}

TEST_F(ServiceActionTypesTest, GetActionGoalType) {
  EXPECT_EQ(ServiceActionTypes::get_action_goal_type("example_interfaces/action/Fibonacci"),
            "example_interfaces/action/Fibonacci_Goal");
}

TEST_F(ServiceActionTypesTest, GetActionResultType) {
  EXPECT_EQ(ServiceActionTypes::get_action_result_type("example_interfaces/action/Fibonacci"),
            "example_interfaces/action/Fibonacci_Result");
}

TEST_F(ServiceActionTypesTest, GetActionFeedbackType) {
  EXPECT_EQ(ServiceActionTypes::get_action_feedback_type("example_interfaces/action/Fibonacci"),
            "example_interfaces/action/Fibonacci_Feedback");
}

TEST_F(ServiceActionTypesTest, ParseInterfaceTypeMsg) {
  auto result = ServiceActionTypes::parse_interface_type("std_msgs/msg/String");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<0>(*result), "std_msgs");
  EXPECT_EQ(std::get<1>(*result), "msg");
  EXPECT_EQ(std::get<2>(*result), "String");
}

TEST_F(ServiceActionTypesTest, ParseInterfaceTypeSrv) {
  auto result = ServiceActionTypes::parse_interface_type("std_srvs/srv/SetBool");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<0>(*result), "std_srvs");
  EXPECT_EQ(std::get<1>(*result), "srv");
  EXPECT_EQ(std::get<2>(*result), "SetBool");
}

TEST_F(ServiceActionTypesTest, ParseInterfaceTypeAction) {
  auto result = ServiceActionTypes::parse_interface_type("example_interfaces/action/Fibonacci");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<0>(*result), "example_interfaces");
  EXPECT_EQ(std::get<1>(*result), "action");
  EXPECT_EQ(std::get<2>(*result), "Fibonacci");
}

TEST_F(ServiceActionTypesTest, ParseInterfaceTypeInvalid) {
  EXPECT_FALSE(ServiceActionTypes::parse_interface_type("invalid").has_value());
  EXPECT_FALSE(ServiceActionTypes::parse_interface_type("").has_value());
  EXPECT_FALSE(ServiceActionTypes::parse_interface_type("pkg/type").has_value());
}

TEST_F(ServiceActionTypesTest, IsServiceType) {
  EXPECT_TRUE(ServiceActionTypes::is_service_type("std_srvs/srv/SetBool"));
  EXPECT_FALSE(ServiceActionTypes::is_service_type("std_msgs/msg/String"));
  EXPECT_FALSE(ServiceActionTypes::is_service_type("example_interfaces/action/Fibonacci"));
}

TEST_F(ServiceActionTypesTest, IsActionType) {
  EXPECT_TRUE(ServiceActionTypes::is_action_type("example_interfaces/action/Fibonacci"));
  EXPECT_FALSE(ServiceActionTypes::is_action_type("std_msgs/msg/String"));
  EXPECT_FALSE(ServiceActionTypes::is_action_type("std_srvs/srv/SetBool"));
}

TEST_F(ServiceActionTypesTest, IsMessageType) {
  EXPECT_TRUE(ServiceActionTypes::is_message_type("std_msgs/msg/String"));
  EXPECT_FALSE(ServiceActionTypes::is_message_type("std_srvs/srv/SetBool"));
  EXPECT_FALSE(ServiceActionTypes::is_message_type("example_interfaces/action/Fibonacci"));
}

}  // namespace ros2_medkit_serialization

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
