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
#include <unistd.h>
#include "ros2_medkit_linux_introspection/proc_reader.hpp"

namespace fs = std::filesystem;
using namespace ros2_medkit_linux_introspection;

TEST(ProcReader, ReadSelfProcess) {
  auto result = read_process_info(getpid());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto & info = result.value();
  EXPECT_EQ(info.pid, getpid());
  EXPECT_GT(info.ppid, 0);
  EXPECT_GT(info.rss_bytes, 0u);
  EXPECT_GT(info.vm_size_bytes, 0u);
  EXPECT_GT(info.num_threads, 0u);
  EXPECT_FALSE(info.exe_path.empty());
}

TEST(ProcReader, ReadNonexistentPidFails) {
  auto result = read_process_info(999999999);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}

TEST(ProcReader, ReadSyntheticProc) {
  auto tmpdir = fs::temp_directory_path() / "test_proc_reader";
  fs::create_directories(tmpdir / "proc" / "42");

  // Write synthetic /proc/42/stat
  {
    std::ofstream f(tmpdir / "proc" / "42" / "stat");
    f << "42 (test_node) S 1 42 42 0 -1 4194304 1000 0 0 0 150 30 0 0 20 0 3 0 12345 "
         "1048576 128 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0 0 0 0 0 0 0 0 0";
  }

  // Write synthetic /proc/42/status
  {
    std::ofstream f(tmpdir / "proc" / "42" / "status");
    f << "Name:\ttest_node\nVmSize:\t1024 kB\nVmRSS:\t512 kB\n";
  }

  // Write synthetic /proc/42/cmdline (null-separated)
  {
    std::ofstream f(tmpdir / "proc" / "42" / "cmdline");
    std::string cmdline = std::string("/usr/bin/test_node") + '\0' + std::string("--ros-args") + '\0' +
                          std::string("-r") + '\0' + std::string("__node:=my_node") + '\0' +
                          std::string("__ns:=/test_ns") + '\0';
    f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
  }

  // Symlink for /proc/42/exe
  fs::create_symlink("/usr/bin/test_node", tmpdir / "proc" / "42" / "exe");

  auto result = read_process_info(42, tmpdir.string());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto & info = result.value();
  EXPECT_EQ(info.pid, 42);
  EXPECT_EQ(info.ppid, 1);
  EXPECT_EQ(info.rss_bytes, 512u * 1024u);
  EXPECT_EQ(info.vm_size_bytes, 1024u * 1024u);
  EXPECT_EQ(info.cpu_user_ticks, 150u);
  EXPECT_EQ(info.cpu_system_ticks, 30u);
  EXPECT_EQ(info.num_threads, 3u);
  EXPECT_EQ(info.start_time_ticks, 12345u);

  fs::remove_all(tmpdir);
}

TEST(ProcReader, FindPidForNodeInSyntheticProc) {
  auto tmpdir = fs::temp_directory_path() / "test_find_pid";
  fs::create_directories(tmpdir / "proc" / "100");
  fs::create_directories(tmpdir / "proc" / "200");

  // PID 100: ROS 2 node "talker" in namespace "/demo"
  {
    std::ofstream f(tmpdir / "proc" / "100" / "cmdline");
    std::string cmdline = std::string("/usr/bin/talker") + '\0' + std::string("--ros-args") + '\0' +
                          std::string("__node:=talker") + '\0' + std::string("__ns:=/demo") + '\0';
    f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
  }

  // PID 200: non-ROS process
  {
    std::ofstream f(tmpdir / "proc" / "200" / "cmdline");
    f << "/usr/bin/bash";
  }

  auto result = find_pid_for_node("talker", "/demo", tmpdir.string());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result.value(), 100);

  // Node not found
  auto missing = find_pid_for_node("listener", "/demo", tmpdir.string());
  ASSERT_FALSE(missing.has_value());

  fs::remove_all(tmpdir);
}
