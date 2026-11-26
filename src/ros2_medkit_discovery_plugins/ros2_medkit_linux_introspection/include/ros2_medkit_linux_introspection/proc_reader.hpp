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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <shared_mutex>
#include <string>
#include <sys/types.h>
#include <tl/expected.hpp>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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

/// Read /proc/{pid}/environ: the NUL-separated NAME=value entries the process started with.
/// Error when the file cannot be read, e.g. for another user's process.
tl::expected<std::string, std::string> read_environ(pid_t pid, const std::string & root = "/");

/// ROS domain ID in an environ image, parsed the way rcl parses ROS_DOMAIN_ID: 0 when the
/// variable is unset or empty, nullopt when its value is not a domain ID.
std::optional<size_t> ros_domain_id_from_environ(const std::string & env_block);

/// Scan /proc for ROS 2 node processes, return PID for matching node.
/// With domain_id, a process of another ROS domain is skipped. A process whose environ
/// cannot be read is matched on its name alone, only when no process of the domain matches.
tl::expected<pid_t, std::string> find_pid_for_node(const std::string & node_name, const std::string & node_namespace,
                                                   const std::string & root = "/",
                                                   std::optional<size_t> domain_id = std::nullopt);

/// Read system uptime in seconds from /proc/uptime
tl::expected<double, std::string> read_system_uptime(const std::string & root = "/");

/// Cache for node-to-PID mappings with TTL-based refresh
class PidCache {
 public:
  /// Called with the PID and FQN of a process matched on its name alone.
  using UnverifiedCallback = std::function<void(pid_t, const std::string &)>;

  /// Called with the PID, FQN and ROS domain of a process skipped for its domain. The
  /// domain is nullopt when the process's ROS_DOMAIN_ID is not a domain ID.
  using OtherDomainCallback = std::function<void(pid_t, const std::string &, std::optional<size_t>)>;

  /// What lookup_and_read returns: the PID and the result of reading it.
  template <typename T>
  using ReadResult = std::optional<std::pair<pid_t, T>>;

  explicit PidCache(std::chrono::steady_clock::duration ttl = std::chrono::seconds{10});

  /// Keep only node processes of ROS domain domain_id. A process whose environ cannot be
  /// read is kept on its name alone, only for an FQN no process of the domain has.
  /// on_unverified gets such a PID once, when a lookup first returns it. on_other_domain
  /// gets each skipped process in every scan. Neither is called with the cache locked.
  void set_domain_filter(size_t domain_id, UnverifiedCallback on_unverified = {},
                         OtherDomainCallback on_other_domain = {});

  /// Lookup PID for a node FQN (e.g. "/namespace/node_name"). Refreshes if TTL expired.
  std::optional<pid_t> lookup(const std::string & node_fqn, const std::string & root = "/");

  /// Look up the PID of a node and call read(pid). When the read fails and the PID no longer
  /// runs as that node, rescan /proc once and read again. Returns the PID and the read result,
  /// or nullopt when no process matches.
  template <typename Read>
  ReadResult<std::invoke_result_t<Read &, pid_t>> lookup_and_read(const std::string & node_fqn,
                                                                  const std::string & root, Read && read) {
    auto pid = lookup(node_fqn, root);
    if (!pid) {
      return std::nullopt;
    }
    auto result = read(*pid);
    if (!result && !runs_as(*pid, node_fqn, root)) {
      invalidate();
      pid = lookup(node_fqn, root);
      if (!pid) {
        return std::nullopt;
      }
      result = read(*pid);
    }
    return std::make_pair(*pid, std::move(result));
  }

  /// Rescan /proc on the next lookup or refresh, whatever the TTL.
  void invalidate();

  /// Rescan /proc when the TTL has expired or the cache was invalidated.
  void refresh(const std::string & root = "/");

  /// Number of cached entries
  size_t size() const;

 private:
  /// True when the process pid still carries the node FQN on its command line.
  static bool runs_as(pid_t pid, const std::string & node_fqn, const std::string & root);

  /// A process a scan skipped because its ROS domain is not the gateway's.
  struct OtherDomainProcess {
    pid_t pid;
    std::string fqn;
    std::optional<size_t> process_domain;
  };

  /// Rebuild the cache from root/proc. Needs mutex_ held exclusively.
  void scan(const std::string & root, std::vector<OtherDomainProcess> & other_domain);

  /// Cached PID of a node FQN, and whether it is still to be reported. Needs mutex_ held.
  std::pair<std::optional<pid_t>, bool> cached(const std::string & node_fqn) const;

  /// Pass pid to on_unverified_ unless it was reported before. Takes mutex_ itself.
  void report_unverified(pid_t pid, const std::string & node_fqn);

  std::unordered_map<std::string, pid_t> node_to_pid_;
  std::chrono::steady_clock::time_point last_refresh_;
  std::chrono::steady_clock::duration ttl_;
  bool stale_{true};
  std::optional<size_t> domain_id_;
  UnverifiedCallback on_unverified_;
  OtherDomainCallback on_other_domain_;
  /// PIDs in node_to_pid_ matched on their name alone.
  std::unordered_set<pid_t> unverified_;
  /// PIDs of unverified_ already passed to on_unverified_.
  std::unordered_set<pid_t> unverified_reported_;
  mutable std::shared_mutex mutex_;
};

}  // namespace ros2_medkit_linux_introspection
