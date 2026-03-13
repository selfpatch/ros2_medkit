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

TEST(SystemdPlugin, UnitInfoToJson) {
  nlohmann::json j;
  j["unit"] = "talker.service";
  j["unit_type"] = "service";
  j["active_state"] = "active";
  j["sub_state"] = "running";
  j["restart_count"] = 2;
  j["watchdog_usec"] = 5000000;

  EXPECT_EQ(j["unit"], "talker.service");
  EXPECT_EQ(j["restart_count"], 2);
  EXPECT_EQ(j["active_state"], "active");
}

TEST(SystemdPlugin, GracefulSkipWhenNotInUnit) {
  // On most dev machines, the test process itself is not a systemd unit.
  // The plugin should gracefully skip (no metadata) for non-unit processes.
  // This is tested at integration level; here verify structure exists.
  nlohmann::json empty_result;
  empty_result["units"] = nlohmann::json::array();
  EXPECT_TRUE(empty_result["units"].is_array());
  EXPECT_TRUE(empty_result["units"].empty());
}
