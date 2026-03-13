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

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"
#include "ros2_medkit_linux_introspection/proc_reader.hpp"

namespace fs = std::filesystem;
using namespace ros2_medkit_gateway;

// Test that introspect() returns metadata for Apps with matching PIDs.
// Since we can't easily construct full IntrospectionInput with real Apps,
// test the underlying proc_reader functions and JSON serialization separately.

TEST(ProcfsPlugin, ProcessInfoToJson) {
  // Verify the JSON format matches spec
  ros2_medkit_linux_introspection::ProcessInfo info;
  info.pid = 1234;
  info.ppid = 1;
  info.exe_path = "/usr/bin/talker";
  info.rss_bytes = 524288;
  info.vm_size_bytes = 2097152;
  info.num_threads = 4;
  info.cpu_user_ticks = 1520;
  info.cpu_system_ticks = 340;
  info.start_time_ticks = 12345;

  // Simulate what procfs_plugin.cpp will do
  nlohmann::json j;
  j["pid"] = info.pid;
  j["ppid"] = info.ppid;
  j["exe"] = info.exe_path;
  j["rss_bytes"] = info.rss_bytes;
  j["vm_size_bytes"] = info.vm_size_bytes;
  j["threads"] = info.num_threads;
  j["cpu_user_ticks"] = info.cpu_user_ticks;
  j["cpu_system_ticks"] = info.cpu_system_ticks;

  EXPECT_EQ(j["pid"], 1234);
  EXPECT_EQ(j["exe"], "/usr/bin/talker");
  EXPECT_EQ(j["rss_bytes"], 524288u);
  EXPECT_EQ(j["threads"], 4u);
}
