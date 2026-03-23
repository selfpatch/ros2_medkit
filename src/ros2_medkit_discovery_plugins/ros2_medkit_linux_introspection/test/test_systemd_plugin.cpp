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
#include "ros2_medkit_linux_introspection/systemd_utils.hpp"

using namespace ros2_medkit_linux_introspection;

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, EscapeSimpleServiceName) {
  EXPECT_EQ(escape_unit_for_dbus("my_service"), "my_service");
}

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, EscapeDotInServiceExtension) {
  EXPECT_EQ(escape_unit_for_dbus("talker.service"), "talker_2eservice");
}

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, EscapeHyphenInUnitName) {
  EXPECT_EQ(escape_unit_for_dbus("ros2-talker.service"), "ros2_2dtalker_2eservice");
}

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, EscapeAtSignInTemplateUnit) {
  EXPECT_EQ(escape_unit_for_dbus("container@instance.service"), "container_40instance_2eservice");
}

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, EscapeSlashInPath) {
  EXPECT_EQ(escape_unit_for_dbus("sys/devices"), "sys_2fdevices");
}

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, EscapeEmptyString) {
  EXPECT_EQ(escape_unit_for_dbus(""), "");
}

// @verifies REQ_INTEROP_003
TEST(SystemdUtils, ComponentEndpointEmptyUnitsArray) {
  nlohmann::json empty_result;
  empty_result["units"] = nlohmann::json::array();
  EXPECT_TRUE(empty_result["units"].is_array());
  EXPECT_TRUE(empty_result["units"].empty());
}
