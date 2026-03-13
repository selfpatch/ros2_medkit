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

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <sys/types.h>
#include <tl/expected.hpp>

namespace ros2_medkit_linux_introspection {

struct CgroupInfo {
  std::string cgroup_path;
  std::string container_id;       // 64-char hex from cgroup path, or empty
  std::string container_runtime;  // "docker", "podman", "containerd", or empty
  std::optional<uint64_t> memory_limit_bytes;
  std::optional<int64_t> cpu_quota_us;
  std::optional<int64_t> cpu_period_us;
};

/// Detect if PID runs inside a container (based on cgroup path)
bool is_containerized(pid_t pid, const std::string & root = "/");

/// Read cgroup info for a PID
tl::expected<CgroupInfo, std::string> read_cgroup_info(pid_t pid, const std::string & root = "/");

/// Extract container ID from a cgroup path string
/// Supports Docker (/docker/<hash>), podman (/libpod-<hash>.scope), containerd
/// (/cri-containerd-<hash>.scope)
std::string extract_container_id(const std::string & cgroup_path);

/// Detect container runtime from cgroup path
std::string detect_runtime(const std::string & cgroup_path);

}  // namespace ros2_medkit_linux_introspection
