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

#include <chrono>
#include <cstdint>
#include <optional>
#include <shared_mutex>
#include <string>
#include <sys/types.h>
#include <tl/expected.hpp>
#include <unordered_map>

namespace ros2_medkit_linux_introspection {

struct ProcessInfo {
  pid_t pid{0};
  pid_t ppid{0};
  std::string state;  // Process state: R=running, S=sleeping, D=disk, Z=zombie, T=stopped
  std::string cmdline;
  std::string exe_path;
  uint64_t rss_bytes{0};
  uint64_t vm_size_bytes{0};
  uint64_t cpu_user_ticks{0};
  uint64_t cpu_system_ticks{0};
  uint64_t start_time_ticks{0};
  uint32_t num_threads{0};
};

/// Read process info from /proc/{pid}
tl::expected<ProcessInfo, std::string> read_process_info(pid_t pid, const std::string & root = "/");

/// Scan /proc for ROS 2 node processes, return PID for matching node
tl::expected<pid_t, std::string> find_pid_for_node(const std::string & node_name, const std::string & node_namespace,
                                                   const std::string & root = "/");

/// Cache for node-to-PID mappings with TTL-based refresh
class PidCache {
 public:
  explicit PidCache(std::chrono::steady_clock::duration ttl = std::chrono::seconds{10});

  /// Lookup PID for a node FQN (e.g. "/namespace/node_name"). Refreshes if TTL expired.
  std::optional<pid_t> lookup(const std::string & node_fqn, const std::string & root = "/");

  /// Force refresh the cache by rescanning /proc
  void refresh(const std::string & root = "/");

  /// Number of cached entries
  size_t size() const;

 private:
  std::unordered_map<std::string, pid_t> node_to_pid_;
  std::chrono::steady_clock::time_point last_refresh_;
  std::chrono::steady_clock::duration ttl_;
  mutable std::shared_mutex mutex_;
};

}  // namespace ros2_medkit_linux_introspection
