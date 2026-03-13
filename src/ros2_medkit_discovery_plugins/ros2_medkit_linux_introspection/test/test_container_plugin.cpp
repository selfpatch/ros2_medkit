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
#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"

using namespace ros2_medkit_linux_introspection;

TEST(ContainerPlugin, CgroupInfoToJson) {
  CgroupInfo info;
  info.container_id = "a1b2c3d4e5f6";
  info.container_runtime = "docker";
  info.memory_limit_bytes = 1073741824;
  info.cpu_quota_us = 100000;
  info.cpu_period_us = 100000;

  nlohmann::json j;
  j["container_id"] = info.container_id;
  j["runtime"] = info.container_runtime;
  if (info.memory_limit_bytes) {
    j["memory_limit_bytes"] = *info.memory_limit_bytes;
  }
  if (info.cpu_quota_us) {
    j["cpu_quota_us"] = *info.cpu_quota_us;
  }
  if (info.cpu_period_us) {
    j["cpu_period_us"] = *info.cpu_period_us;
  }

  EXPECT_EQ(j["container_id"], "a1b2c3d4e5f6");
  EXPECT_EQ(j["runtime"], "docker");
  EXPECT_EQ(j["memory_limit_bytes"], 1073741824u);
}

TEST(ContainerPlugin, NotContainerizedSkipped) {
  // When is_containerized() returns false, plugin should not return metadata
  EXPECT_FALSE(is_containerized(1, "/nonexistent_root"));
}
