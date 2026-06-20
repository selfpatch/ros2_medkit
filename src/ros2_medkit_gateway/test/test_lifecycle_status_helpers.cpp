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

#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"

using ros2_medkit_gateway::find_lifecycle_get_state_path;
using ros2_medkit_gateway::lifecycle_status_from_state;
using ros2_medkit_gateway::ServiceInfo;

TEST(LifecycleStatusHelpers, ActiveIsReady) {
  EXPECT_EQ(lifecycle_status_from_state(std::optional<std::string>("active")), "ready");
}

TEST(LifecycleStatusHelpers, NonActiveIsNotReady) {
  EXPECT_EQ(lifecycle_status_from_state(std::optional<std::string>("inactive")), "notReady");
  EXPECT_EQ(lifecycle_status_from_state(std::optional<std::string>("unconfigured")), "notReady");
  EXPECT_EQ(lifecycle_status_from_state(std::optional<std::string>("finalized")), "notReady");
}

TEST(LifecycleStatusHelpers, NulloptIsNotReady) {
  EXPECT_EQ(lifecycle_status_from_state(std::nullopt), "notReady");
}

TEST(LifecycleStatusHelpers, FindPathRequiresBothServices) {
  std::vector<ServiceInfo> only_get;
  ServiceInfo gs;
  gs.full_path = "/lc_node/get_state";
  gs.type = "lifecycle_msgs/srv/GetState";
  only_get.push_back(gs);
  EXPECT_FALSE(find_lifecycle_get_state_path(only_get).has_value());

  std::vector<ServiceInfo> both = only_get;
  ServiceInfo cs;
  cs.full_path = "/lc_node/change_state";
  cs.type = "lifecycle_msgs/srv/ChangeState";
  both.push_back(cs);
  auto path = find_lifecycle_get_state_path(both);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, "/lc_node/get_state");
}

TEST(LifecycleStatusHelpers, FindPathPlainNodeReturnsNullopt) {
  std::vector<ServiceInfo> plain;
  ServiceInfo trig;
  trig.full_path = "/plain/trigger";
  trig.type = "std_srvs/srv/Trigger";
  plain.push_back(trig);
  EXPECT_FALSE(find_lifecycle_get_state_path(plain).has_value());
}
