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
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
using namespace ros2_medkit_gateway;

TEST(EntityEquality, AppFieldSensitivity) {
  App a;
  a.id = "n1";
  a.name = "N";
  a.component_id = "c";
  a.services = {{"s", "/s", "t", std::nullopt}};
  App b = a;
  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  b.name = "M";
  EXPECT_NE(a, b);
  b = a;
  b.services[0].type_info = nlohmann::json{{"k", 1}};
  EXPECT_NE(a, b);
  b = a;
  b.is_online = !a.is_online;
  EXPECT_NE(a, b);
  b = a;
  b.ros_binding = App::RosBinding{"node", "/ns", ""};
  EXPECT_NE(a, b);
}
TEST(EntityEquality, ComponentHostMetadata) {
  Component a;
  a.id = "c";
  a.area = "ar";
  Component b = a;
  EXPECT_EQ(a, b);
  b.host_metadata = nlohmann::json{{"os", "linux"}};
  EXPECT_NE(a, b);
}
TEST(EntityEquality, AreaAndFunction) {
  Area a;
  a.id = "ar";
  a.parent_area_id = "root";
  Area b = a;
  EXPECT_EQ(a, b);
  b.source = "x";
  EXPECT_NE(a, b);
  Function f;
  f.id = "fn";
  f.hosts = {"a1"};
  Function g = f;
  EXPECT_EQ(f, g);
  g.hosts = {"a2"};
  EXPECT_NE(f, g);
}
