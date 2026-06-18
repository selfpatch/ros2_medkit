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

#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/lifecycle.hpp"

namespace ros2_medkit_gateway::dto {

TEST(LifecycleDto, WritesStatusOnlyWhenNoTransitions) {
  LifecycleStatusResponse r;
  r.status = "ready";
  const nlohmann::json j = JsonWriter<LifecycleStatusResponse>::write(r);
  EXPECT_EQ(j.at("status"), "ready");
  EXPECT_FALSE(j.contains("start"));
  EXPECT_FALSE(j.contains("force-restart"));
}

TEST(LifecycleDto, EmitsHyphenatedTransitionKeys) {
  LifecycleStatusResponse r;
  r.status = "ready";
  r.force_restart = "/api/v1/apps/x/status/force-restart";
  const nlohmann::json j = JsonWriter<LifecycleStatusResponse>::write(r);
  EXPECT_EQ(j.at("force-restart"), "/api/v1/apps/x/status/force-restart");
  EXPECT_FALSE(j.contains("force_restart"));
}

}  // namespace ros2_medkit_gateway::dto
