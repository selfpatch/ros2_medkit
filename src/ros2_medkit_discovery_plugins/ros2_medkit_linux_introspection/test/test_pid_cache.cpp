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
#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace ros2_medkit_linux_introspection;

class PidCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmpdir_ = fs::temp_directory_path() / "test_pid_cache";
    fs::create_directories(tmpdir_ / "proc" / "100");
    fs::create_directories(tmpdir_ / "proc" / "200");

    // PID 100: /demo/talker
    {
      std::ofstream f(tmpdir_ / "proc" / "100" / "cmdline");
      std::string cmdline = std::string("/usr/bin/talker") + '\0' + std::string("--ros-args") + '\0' +
                            std::string("__node:=talker") + '\0' + std::string("__ns:=/demo") + '\0';
      f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
    }

    // PID 200: /demo/listener
    {
      std::ofstream f(tmpdir_ / "proc" / "200" / "cmdline");
      std::string cmdline = std::string("/usr/bin/listener") + '\0' + std::string("--ros-args") + '\0' +
                            std::string("__node:=listener") + '\0' + std::string("__ns:=/demo") + '\0';
      f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
    }
  }

  void TearDown() override {
    fs::remove_all(tmpdir_);
  }

  fs::path tmpdir_;
};

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, LookupAfterRefresh) {
  PidCache cache(std::chrono::seconds{60});
  cache.refresh(tmpdir_.string());

  auto pid = cache.lookup("/demo/talker", tmpdir_.string());
  ASSERT_TRUE(pid.has_value());
  EXPECT_EQ(pid.value(), 100);

  pid = cache.lookup("/demo/listener", tmpdir_.string());
  ASSERT_TRUE(pid.has_value());
  EXPECT_EQ(pid.value(), 200);

  EXPECT_EQ(cache.size(), 2u);
}

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, LookupMissingNode) {
  PidCache cache(std::chrono::seconds{60});
  cache.refresh(tmpdir_.string());

  auto pid = cache.lookup("/demo/nonexistent", tmpdir_.string());
  EXPECT_FALSE(pid.has_value());
}

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, AutoRefreshOnTTLExpiry) {
  PidCache cache(std::chrono::milliseconds{1});  // 1ms TTL
  cache.refresh(tmpdir_.string());

  // Wait for TTL to expire
  std::this_thread::sleep_for(std::chrono::milliseconds{5});

  // Add a new node
  fs::create_directories(tmpdir_ / "proc" / "300");
  {
    std::ofstream f(tmpdir_ / "proc" / "300" / "cmdline");
    std::string cmdline = std::string("/usr/bin/chatter") + '\0' + std::string("--ros-args") + '\0' +
                          std::string("__node:=chatter") + '\0' + std::string("__ns:=/demo") + '\0';
    f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
  }

  // lookup should trigger refresh and find the new node
  auto pid = cache.lookup("/demo/chatter", tmpdir_.string());
  ASSERT_TRUE(pid.has_value());
  EXPECT_EQ(pid.value(), 300);
}

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, NoRefreshWithinTTL) {
  PidCache cache(std::chrono::seconds{60});
  cache.refresh(tmpdir_.string());
  EXPECT_EQ(cache.size(), 2u);

  // Add a new node - should NOT be picked up since TTL hasn't expired
  fs::create_directories(tmpdir_ / "proc" / "300");
  {
    std::ofstream f(tmpdir_ / "proc" / "300" / "cmdline");
    std::string cmdline = std::string("/usr/bin/chatter") + '\0' + std::string("--ros-args") + '\0' +
                          std::string("__node:=chatter") + '\0' + std::string("__ns:=/demo") + '\0';
    f.write(cmdline.data(), static_cast<std::streamsize>(cmdline.size()));
  }

  auto pid = cache.lookup("/demo/chatter", tmpdir_.string());
  EXPECT_FALSE(pid.has_value());  // Not found - cache still has old data
  EXPECT_EQ(cache.size(), 2u);
}

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, EmptyProcDir) {
  auto empty_dir = fs::temp_directory_path() / "test_pid_cache_empty";
  fs::create_directories(empty_dir / "proc");

  PidCache cache(std::chrono::seconds{60});
  cache.refresh(empty_dir.string());
  EXPECT_EQ(cache.size(), 0u);

  auto pid = cache.lookup("/any/node", empty_dir.string());
  EXPECT_FALSE(pid.has_value());

  fs::remove_all(empty_dir);
}

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, NonexistentProcDir) {
  auto bad_dir = fs::temp_directory_path() / "test_pid_cache_nonexistent";
  fs::remove_all(bad_dir);  // Ensure it doesn't exist

  PidCache cache(std::chrono::seconds{60});
  cache.refresh(bad_dir.string());
  EXPECT_EQ(cache.size(), 0u);
}

// @verifies REQ_INTEROP_003
TEST_F(PidCacheTest, ConcurrentLookupDoesNotCrash) {
  PidCache cache(std::chrono::milliseconds{1});

  constexpr int kNumThreads = 8;
  constexpr int kIterations = 100;
  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);

  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&cache, this]() {
      for (int j = 0; j < kIterations; ++j) {
        cache.lookup("/demo/talker", tmpdir_.string());
        cache.lookup("/demo/listener", tmpdir_.string());
        cache.lookup("/demo/nonexistent", tmpdir_.string());
      }
    });
  }

  for (auto & t : threads) {
    t.join();
  }

  // No crash or deadlock = pass
  EXPECT_GE(cache.size(), 0u);
}
