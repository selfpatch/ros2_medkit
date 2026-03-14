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

// ---------------------------------------------------------------------------
// Fixture for tests that create a synthetic /proc tree
// ---------------------------------------------------------------------------

class SyntheticProcTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmpdir_ = fs::temp_directory_path() / "test_proc_reader_synth";
    fs::remove_all(tmpdir_);
    fs::create_directories(tmpdir_ / "proc");
  }

  void TearDown() override {
    fs::remove_all(tmpdir_);
  }

  fs::path tmpdir_;
};

// ---------------------------------------------------------------------------
// Standalone tests (no synthetic dirs needed)
// ---------------------------------------------------------------------------

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

TEST(ProcReader, StateFieldPopulated) {
  // Read our own process - should have state "R" (running) or "S" (sleeping)
  auto result = read_process_info(getpid());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result.value().state.empty());
}

TEST(ProcReader, ReadSystemUptime) {
  auto result = read_system_uptime("/");
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_GT(*result, 0.0);
}

TEST(ProcReader, ReadSystemUptimeMissingFile) {
  auto result = read_system_uptime("/nonexistent_root");
  ASSERT_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// Fixture-based tests (use SyntheticProcTest)
// ---------------------------------------------------------------------------

TEST_F(SyntheticProcTest, ReadSyntheticProc) {
  fs::create_directories(tmpdir_ / "proc" / "42");

  // Write synthetic /proc/42/stat
  {
    std::ofstream f(tmpdir_ / "proc" / "42" / "stat");
    f << "42 (test_node) S 1 42 42 0 -1 4194304 1000 0 0 0 150 30 0 0 20 0 3 0 12345 "
         "1048576 128 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0 0 0 0 0 0 0 0 0";
  }

  // Write synthetic /proc/42/status
  {
    std::ofstream f(tmpdir_ / "proc" / "42" / "status");
    f << "Name:\ttest_node\nVmSize:\t1024 kB\nVmRSS:\t512 kB\n";
  }

  // Write synthetic /proc/42/cmdline (null-separated)
  {
    std::ofstream f(tmpdir_ / "proc" / "42" / "cmdline");
    std::string cmdline = std::string("/usr/bin/test_node") + '\0' + std::string("--ros-args") + '\0' +
                          std::string("-r") + '\0' + std::string("__node:=my_node") + '\0' +
                          std::string("__ns:=/test_ns") + '\0';
    f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
  }

  // Symlink for /proc/42/exe
  fs::create_symlink("/usr/bin/test_node", tmpdir_ / "proc" / "42" / "exe");

  auto result = read_process_info(42, tmpdir_.string());
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
}

TEST_F(SyntheticProcTest, FindPidForNodeInSyntheticProc) {
  fs::create_directories(tmpdir_ / "proc" / "100");
  fs::create_directories(tmpdir_ / "proc" / "200");

  // PID 100: ROS 2 node "talker" in namespace "/demo"
  {
    std::ofstream f(tmpdir_ / "proc" / "100" / "cmdline");
    std::string cmdline = std::string("/usr/bin/talker") + '\0' + std::string("--ros-args") + '\0' +
                          std::string("__node:=talker") + '\0' + std::string("__ns:=/demo") + '\0';
    f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
  }

  // PID 200: non-ROS process
  {
    std::ofstream f(tmpdir_ / "proc" / "200" / "cmdline");
    f << "/usr/bin/bash";
  }

  auto result = find_pid_for_node("talker", "/demo", tmpdir_.string());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result.value(), 100);

  // Node not found
  auto missing = find_pid_for_node("listener", "/demo", tmpdir_.string());
  ASSERT_FALSE(missing.has_value());
}

TEST_F(SyntheticProcTest, ReadSystemUptimeSynthetic) {
  {
    std::ofstream f(tmpdir_ / "proc" / "uptime");
    f << "12345.67 98765.43\n";
  }
  auto result = read_system_uptime(tmpdir_.string());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NEAR(*result, 12345.67, 0.01);
}

TEST_F(SyntheticProcTest, MalformedStatMissingComm) {
  fs::create_directories(tmpdir_ / "proc" / "42");
  {
    std::ofstream f(tmpdir_ / "proc" / "42" / "stat");
    f << "42 S 1 42 42";  // Missing (comm) parens
  }
  auto result = read_process_info(42, tmpdir_.string());
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("Malformed"), std::string::npos);
}

TEST_F(SyntheticProcTest, EmptyStatFile) {
  fs::create_directories(tmpdir_ / "proc" / "42");
  {
    std::ofstream f(tmpdir_ / "proc" / "42" / "stat");
    // Intentionally empty
  }
  auto result = read_process_info(42, tmpdir_.string());
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("Cannot read"), std::string::npos);
}
