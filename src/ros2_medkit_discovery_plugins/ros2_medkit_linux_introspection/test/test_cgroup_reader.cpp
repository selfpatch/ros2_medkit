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
#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"

namespace fs = std::filesystem;
using namespace ros2_medkit_linux_introspection;

TEST(CgroupReader, ExtractDockerContainerId) {
  std::string path =
      "/system.slice/"
      "docker-a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2.scope";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2");
}

TEST(CgroupReader, ExtractPodmanContainerId) {
  std::string path =
      "/user.slice/user-1000.slice/user@1000.service/"
      "libpod-aabbccddee112233aabbccddee112233aabbccddee112233aabbccddee112233.scope";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "aabbccddee112233aabbccddee112233aabbccddee112233aabbccddee112233");
}

TEST(CgroupReader, ExtractContainerdContainerId) {
  std::string path =
      "/system.slice/containerd.service/"
      "cri-containerd-deadbeef12345678deadbeef12345678deadbeef12345678deadbeef12345678.scope";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "deadbeef12345678deadbeef12345678deadbeef12345678deadbeef12345678");
}

TEST(CgroupReader, ExtractNoContainerId) {
  EXPECT_TRUE(extract_container_id("/user.slice/user-1000.slice/session-1.scope").empty());
  EXPECT_TRUE(extract_container_id("/").empty());
}

TEST(CgroupReader, DetectRuntime) {
  EXPECT_EQ(detect_runtime("/system.slice/docker-abc123.scope"), "docker");
  EXPECT_EQ(detect_runtime("/user.slice/libpod-abc123.scope"), "podman");
  EXPECT_EQ(detect_runtime("/system.slice/cri-containerd-abc123.scope"), "containerd");
  EXPECT_TRUE(detect_runtime("/user.slice/session-1.scope").empty());
}

TEST(CgroupReader, IsContainerizedSyntheticProc) {
  auto tmpdir = fs::temp_directory_path() / "test_cgroup";
  fs::create_directories(tmpdir / "proc" / "42");

  // Process inside Docker container
  {
    std::ofstream f(tmpdir / "proc" / "42" / "cgroup");
    f << "0::/system.slice/"
         "docker-a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2.scope\n";
  }

  EXPECT_TRUE(is_containerized(42, tmpdir.string()));

  // Process on host
  fs::create_directories(tmpdir / "proc" / "43");
  {
    std::ofstream f(tmpdir / "proc" / "43" / "cgroup");
    f << "0::/user.slice/user-1000.slice/session-1.scope\n";
  }

  EXPECT_FALSE(is_containerized(43, tmpdir.string()));

  fs::remove_all(tmpdir);
}

TEST(CgroupReader, ReadCgroupInfoWithResourceLimits) {
  auto tmpdir = fs::temp_directory_path() / "test_cgroup_limits";
  fs::create_directories(tmpdir / "proc" / "42");
  fs::create_directories(tmpdir / "sys" / "fs" / "cgroup" / "system.slice" /
                         "docker-aabb112233445566aabb112233445566aabb112233445566aabb112233445566.scope");

  // cgroup file
  {
    std::ofstream f(tmpdir / "proc" / "42" / "cgroup");
    f << "0::/system.slice/"
         "docker-aabb112233445566aabb112233445566aabb112233445566aabb112233445566.scope\n";
  }

  auto cgroup_dir = tmpdir / "sys" / "fs" / "cgroup" / "system.slice" /
                    "docker-aabb112233445566aabb112233445566aabb112233445566aabb112233445566.scope";

  // memory.max
  {
    std::ofstream f(cgroup_dir / "memory.max");
    f << "1073741824\n";
  }

  // cpu.max (quota period)
  {
    std::ofstream f(cgroup_dir / "cpu.max");
    f << "100000 100000\n";
  }

  auto result = read_cgroup_info(42, tmpdir.string());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto & info = result.value();
  EXPECT_EQ(info.container_id, "aabb112233445566aabb112233445566aabb112233445566aabb112233445566");
  EXPECT_EQ(info.container_runtime, "docker");
  ASSERT_TRUE(info.memory_limit_bytes.has_value());
  EXPECT_EQ(info.memory_limit_bytes.value(), 1073741824u);
  ASSERT_TRUE(info.cpu_quota_us.has_value());
  EXPECT_EQ(info.cpu_quota_us.value(), 100000);
  ASSERT_TRUE(info.cpu_period_us.has_value());
  EXPECT_EQ(info.cpu_period_us.value(), 100000);

  fs::remove_all(tmpdir);
}

TEST(CgroupReader, ReadCgroupInfoUnlimitedResources) {
  auto tmpdir = fs::temp_directory_path() / "test_cgroup_unlimited";
  fs::create_directories(tmpdir / "proc" / "42");
  fs::create_directories(tmpdir / "sys" / "fs" / "cgroup" / "system.slice" /
                         "docker-aabb112233445566aabb112233445566aabb112233445566aabb112233445566.scope");

  // cgroup file
  {
    std::ofstream f(tmpdir / "proc" / "42" / "cgroup");
    f << "0::/system.slice/"
         "docker-aabb112233445566aabb112233445566aabb112233445566aabb112233445566.scope\n";
  }

  auto cgroup_dir = tmpdir / "sys" / "fs" / "cgroup" / "system.slice" /
                    "docker-aabb112233445566aabb112233445566aabb112233445566aabb112233445566.scope";

  // memory.max = "max" means unlimited
  {
    std::ofstream f(cgroup_dir / "memory.max");
    f << "max\n";
  }

  // cpu.max = "max 100000" means unlimited quota
  {
    std::ofstream f(cgroup_dir / "cpu.max");
    f << "max 100000\n";
  }

  auto result = read_cgroup_info(42, tmpdir.string());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto & info = result.value();
  // memory_limit_bytes should not be set when "max"
  EXPECT_FALSE(info.memory_limit_bytes.has_value());
  // cpu_quota_us should not be set when "max", but period should be
  EXPECT_FALSE(info.cpu_quota_us.has_value());
  ASSERT_TRUE(info.cpu_period_us.has_value());
  EXPECT_EQ(info.cpu_period_us.value(), 100000);

  fs::remove_all(tmpdir);
}

TEST(CgroupReader, ReadCgroupInfoMissingResourceFiles) {
  auto tmpdir = fs::temp_directory_path() / "test_cgroup_nores";
  fs::create_directories(tmpdir / "proc" / "42");

  // cgroup file points to path with no resource files
  {
    std::ofstream f(tmpdir / "proc" / "42" / "cgroup");
    f << "0::/user.slice/user-1000.slice/session-1.scope\n";
  }

  auto result = read_cgroup_info(42, tmpdir.string());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto & info = result.value();
  EXPECT_TRUE(info.container_id.empty());
  EXPECT_TRUE(info.container_runtime.empty());
  EXPECT_FALSE(info.memory_limit_bytes.has_value());
  EXPECT_FALSE(info.cpu_quota_us.has_value());
  EXPECT_FALSE(info.cpu_period_us.has_value());

  fs::remove_all(tmpdir);
}

TEST(CgroupReader, ReadCgroupInfoMissingCgroupFile) {
  auto tmpdir = fs::temp_directory_path() / "test_cgroup_nocg";
  fs::create_directories(tmpdir / "proc" / "42");
  // No cgroup file at all

  auto result = read_cgroup_info(42, tmpdir.string());
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());

  fs::remove_all(tmpdir);
}

TEST(CgroupReader, ExtractDockerOldStyleContainerId) {
  std::string path = "/docker/a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2";
  auto id = extract_container_id(path);
  EXPECT_EQ(id, "a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2");
}

TEST(CgroupReader, CgroupV1FormatNotSupported) {
  // cgroup v1 uses "hierarchy-ID:controller-list:cgroup-path" format.
  // Our reader only supports cgroup v2 ("0::<path>").
  // This test documents the intentional limitation.
  auto tmpdir = fs::temp_directory_path() / "test_cgroup_v1";
  fs::create_directories(tmpdir / "proc" / "42");
  {
    std::ofstream f(tmpdir / "proc" / "42" / "cgroup");
    f << "12:memory:/docker/a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2\n"
      << "11:cpu:/docker/a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2\n";
  }
  // cgroup v1 format is not supported - returns false
  EXPECT_FALSE(is_containerized(42, tmpdir.string()));
  fs::remove_all(tmpdir);
}
