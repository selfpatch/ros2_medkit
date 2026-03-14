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
#include "ros2_medkit_linux_introspection/procfs_utils.hpp"

using namespace ros2_medkit_linux_introspection;

// @verifies REQ_INTEROP_003
TEST(ProcfsPlugin, ProcessInfoToJsonAllFields) {
  ProcessInfo info;
  info.pid = 1234;
  info.ppid = 1;
  info.state = "S";
  info.exe_path = "/usr/bin/talker";
  info.cmdline = "/usr/bin/talker --ros-args __node:=talker";
  info.rss_bytes = 524288;
  info.vm_size_bytes = 2097152;
  info.num_threads = 4;
  info.cpu_user_ticks = 1520;
  info.cpu_system_ticks = 340;
  info.start_time_ticks = 12345;

  double system_uptime = 50000.0;
  auto j = process_info_to_json(info, system_uptime);

  EXPECT_EQ(j["pid"], 1234);
  EXPECT_EQ(j["ppid"], 1);
  EXPECT_EQ(j["state"], "S");
  EXPECT_EQ(j["exe"], "/usr/bin/talker");
  EXPECT_EQ(j["cmdline"], "/usr/bin/talker --ros-args __node:=talker");
  EXPECT_EQ(j["rss_bytes"], 524288u);
  EXPECT_EQ(j["vm_size_bytes"], 2097152u);
  EXPECT_EQ(j["threads"], 4u);
  EXPECT_EQ(j["cpu_user_ticks"], 1520u);
  EXPECT_EQ(j["cpu_system_ticks"], 340u);

  // cpu_*_seconds should be ticks / sysconf(_SC_CLK_TCK)
  EXPECT_GT(j["cpu_user_seconds"].get<double>(), 0.0);
  EXPECT_GT(j["cpu_system_seconds"].get<double>(), 0.0);

  // uptime_seconds should be > 0 given valid start_time and system_uptime
  EXPECT_GT(j["uptime_seconds"].get<double>(), 0.0);
}

// @verifies REQ_INTEROP_003
TEST(ProcfsPlugin, ProcessInfoToJsonZeroUptime) {
  ProcessInfo info;
  info.pid = 1;
  info.start_time_ticks = 0;  // No start time

  auto j = process_info_to_json(info, 50000.0);
  EXPECT_EQ(j["uptime_seconds"].get<double>(), 0.0);
}

// @verifies REQ_INTEROP_003
TEST(ProcfsPlugin, ProcessInfoToJsonNegativeUptimeClamped) {
  ProcessInfo info;
  info.pid = 1;
  // start_time_ticks larger than system_uptime * ticks_per_sec
  info.start_time_ticks = 999999999;

  auto j = process_info_to_json(info, 1.0);
  // Should be clamped to 0, not negative
  EXPECT_GE(j["uptime_seconds"].get<double>(), 0.0);
}
