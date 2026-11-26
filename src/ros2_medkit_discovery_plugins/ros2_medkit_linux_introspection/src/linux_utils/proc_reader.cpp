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

#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

namespace ros2_medkit_linux_introspection {

namespace {

struct DirCloser {
  void operator()(DIR * d) const {
    if (d) {
      closedir(d);
    }
  }
};
using UniqueDir = std::unique_ptr<DIR, DirCloser>;

std::string read_file_contents(const std::string & path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    return {};
  }
  // Read through the stream buffer rather than a pair of istreambuf_iterators:
  // the iterator form makes GCC lose track of the buffer pointers and report a
  // null dereference in <streambuf> that cannot happen on an open stream.
  std::ostringstream contents;
  contents << f.rdbuf();
  return contents.str();
}

// Parse VmRSS and VmSize from /proc/{pid}/status
void parse_status_file(const std::string & path, uint64_t & rss_bytes, uint64_t & vm_size_bytes) {
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      uint64_t val = 0;
      if (sscanf(line.c_str(), "VmRSS: %" SCNu64, &val) == 1) {  // NOLINT(runtime/printf)
        rss_bytes = val * 1024;                                  // kB to bytes
      }
    } else if (line.rfind("VmSize:", 0) == 0) {
      uint64_t val = 0;
      if (sscanf(line.c_str(), "VmSize: %" SCNu64, &val) == 1) {  // NOLINT(runtime/printf)
        vm_size_bytes = val * 1024;
      }
    }
  }
}

// Parse cmdline from /proc/{pid}/cmdline (null-separated arguments)
std::string parse_cmdline(const std::string & path) {
  auto content = read_file_contents(path);
  // Replace null bytes with spaces for display
  std::replace(content.begin(), content.end(), '\0', ' ');
  // Trim trailing space
  if (!content.empty() && content.back() == ' ') {
    content.pop_back();
  }
  return content;
}

}  // namespace

// Parse node name and namespace from /proc/{pid}/cmdline
// Not in anonymous namespace - used by both find_pid_for_node and PidCache::refresh
static bool parse_ros_args(const std::string & cmdline_path, std::string & node_name, std::string & node_namespace) {
  auto content = read_file_contents(cmdline_path);
  // cmdline is null-separated
  std::vector<std::string> args;
  std::string current;
  for (char c : content) {
    if (c == '\0') {
      if (!current.empty()) {
        args.push_back(std::move(current));
      }
      current.clear();
    } else {
      current += c;
    }
  }
  if (!current.empty()) {
    args.push_back(std::move(current));
  }

  for (const auto & arg : args) {
    if (arg.rfind("__node:=", 0) == 0) {
      node_name = arg.substr(8);
    } else if (arg.rfind("__ns:=", 0) == 0) {
      node_namespace = arg.substr(6);
    }
  }
  return !node_name.empty();
}

// FQN as the PID cache keys it: "/ns/name", or "/name" in the root namespace.
static std::string make_fqn(const std::string & node_name, const std::string & node_ns) {
  std::string fqn;
  if (node_ns != "/" && !node_ns.empty()) {
    fqn = node_ns;
  }
  fqn += "/";
  fqn += node_name;
  return fqn;
}

namespace {

enum class DomainMatch { kSame, kOther, kUnreadable };

struct DomainCheck {
  DomainMatch match;
  /// The domain the process's environ names; nullopt when unreadable or not a domain ID.
  std::optional<size_t> process_domain;
};

DomainCheck match_domain(pid_t pid, const std::string & root, size_t domain_id) {
  auto env_block = read_environ(pid, root);
  if (!env_block) {
    return {DomainMatch::kUnreadable, std::nullopt};
  }
  auto process_domain = ros_domain_id_from_environ(*env_block);
  auto match = process_domain && *process_domain == domain_id ? DomainMatch::kSame : DomainMatch::kOther;
  return {match, process_domain};
}

}  // namespace

tl::expected<std::string, std::string> read_environ(pid_t pid, const std::string & root) {
  auto path = root + "/proc/" + std::to_string(pid) + "/environ";
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return tl::make_unexpected("Cannot read " + path);
  }
  std::ostringstream contents;
  contents << f.rdbuf();
  return contents.str();
}

std::optional<size_t> ros_domain_id_from_environ(const std::string & env_block) {
  static const std::string kPrefix = "ROS_DOMAIN_ID=";
  std::string value;
  size_t start = 0;
  while (start < env_block.size()) {
    auto end = env_block.find('\0', start);
    if (end == std::string::npos) {
      end = env_block.size();
    }
    // The first entry wins, as for getenv().
    if (env_block.compare(start, kPrefix.size(), kPrefix) == 0) {
      value = env_block.substr(start + kPrefix.size(), end - start - kPrefix.size());
      break;
    }
    start = end + 1;
  }
  if (value.empty()) {
    return 0;
  }
  // The checks of rcl_get_default_domain_id(): strtoul with base 0, so "0x1A" is 26 and
  // trailing text after a non-zero number is accepted.
  static_assert(sizeof(unsigned long) <= sizeof(size_t), "a domain ID must fit size_t");  // NOLINT(runtime/int)
  errno = 0;
  char * end = nullptr;
  unsigned long number = std::strtoul(value.c_str(), &end, 0);  // NOLINT(runtime/int)
  if (number == 0UL && *end != '\0') {
    return std::nullopt;
  }
  if (number == ULONG_MAX && errno == ERANGE) {
    return std::nullopt;
  }
  return static_cast<size_t>(number);
}

tl::expected<ProcessInfo, std::string> read_process_info(pid_t pid, const std::string & root) {
  auto proc_dir = root + "/proc/" + std::to_string(pid);

  if (!fs::exists(proc_dir)) {
    return tl::make_unexpected("Process " + std::to_string(pid) + " not found");
  }

  ProcessInfo info;
  info.pid = pid;

  // Parse /proc/{pid}/stat
  auto stat_content = read_file_contents(proc_dir + "/stat");
  if (stat_content.empty()) {
    return tl::make_unexpected("Cannot read " + proc_dir + "/stat");
  }

  // Find the closing paren of (comm) to skip process names with spaces
  auto comm_end = stat_content.rfind(')');
  if (comm_end == std::string::npos || comm_end + 2 >= stat_content.size()) {
    return tl::make_unexpected("Malformed stat file for PID " + std::to_string(pid));
  }

  // Fields after (comm): state ppid pgrp session tty_nr tpgid flags
  //   minflt cminflt majflt cmajflt utime stime cutime cstime priority nice
  //   num_threads itrealvalue starttime vsize rss ...
  std::istringstream ss(stat_content.substr(comm_end + 2));
  std::string state;
  int64_t ppid = 0;
  int64_t pgrp = 0;
  int64_t session = 0;
  int64_t tty_nr = 0;
  int64_t tpgid = 0;
  uint64_t flags = 0;
  uint64_t minflt = 0;
  uint64_t cminflt = 0;
  uint64_t majflt = 0;
  uint64_t cmajflt = 0;
  uint64_t utime = 0;
  uint64_t stime = 0;
  uint64_t cutime = 0;
  uint64_t cstime = 0;
  int64_t priority = 0;
  int64_t nice = 0;
  uint32_t num_threads = 0;
  int64_t itrealvalue = 0;
  uint64_t starttime = 0;

  ss >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags;
  ss >> minflt >> cminflt >> majflt >> cmajflt;
  ss >> utime >> stime >> cutime >> cstime;
  ss >> priority >> nice >> num_threads >> itrealvalue >> starttime;

  if (ss.fail()) {
    return tl::make_unexpected("Truncated stat file for PID " + std::to_string(pid));
  }

  // Suppress unused variable warnings for positional fields we don't need
  (void)pgrp;
  (void)session;
  (void)tty_nr;
  (void)tpgid;
  (void)flags;
  (void)minflt;
  (void)cminflt;
  (void)majflt;
  (void)cmajflt;
  (void)cutime;
  (void)cstime;
  (void)priority;
  (void)nice;
  (void)itrealvalue;

  info.ppid = static_cast<pid_t>(ppid);
  info.state = state;
  info.cpu_user_ticks = utime;
  info.cpu_system_ticks = stime;
  info.num_threads = num_threads;
  info.start_time_ticks = starttime;

  // Parse /proc/{pid}/status for VmRSS, VmSize
  parse_status_file(proc_dir + "/status", info.rss_bytes, info.vm_size_bytes);

  // Parse /proc/{pid}/cmdline
  info.cmdline = parse_cmdline(proc_dir + "/cmdline");

  // Read /proc/{pid}/exe symlink
  std::error_code ec;
  auto exe = fs::read_symlink(proc_dir + "/exe", ec);
  if (!ec) {
    info.exe_path = exe.string();
  }

  return info;
}

tl::expected<pid_t, std::string> find_pid_for_node(const std::string & node_name, const std::string & node_namespace,
                                                   const std::string & root, std::optional<size_t> domain_id) {
  auto proc_dir = root + "/proc";
  UniqueDir dir(opendir(proc_dir.c_str()));
  if (!dir) {
    return tl::make_unexpected("Cannot open " + proc_dir);
  }

  std::optional<pid_t> unverified;
  struct dirent * entry = nullptr;
  while ((entry = readdir(dir.get())) != nullptr) {
    if (entry->d_type != DT_DIR && entry->d_type != DT_UNKNOWN) {
      continue;
    }
    if (entry->d_type == DT_UNKNOWN) {
      struct stat st {};
      auto full_path = proc_dir + "/" + entry->d_name;
      if (stat(full_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        continue;
      }
    }
    bool all_digits = true;
    for (const char * p = entry->d_name; *p; ++p) {
      if (*p < '0' || *p > '9') {
        all_digits = false;
        break;
      }
    }
    if (!all_digits) {
      continue;
    }

    auto cmdline_path = proc_dir + "/" + entry->d_name + "/cmdline";
    std::string found_node;
    std::string found_ns;
    if (parse_ros_args(cmdline_path, found_node, found_ns)) {
      if (found_node == node_name && (node_namespace.empty() || found_ns == node_namespace)) {
        char * end = nullptr;
        long pid_val = std::strtol(entry->d_name, &end, 10);
        if (end == entry->d_name || *end != '\0' || pid_val <= 0) {
          continue;
        }
        auto pid = static_cast<pid_t>(pid_val);
        if (domain_id) {
          auto match = match_domain(pid, root, *domain_id).match;
          if (match == DomainMatch::kOther) {
            continue;
          }
          if (match == DomainMatch::kUnreadable) {
            // A process of the domain found later still wins.
            if (!unverified) {
              unverified = pid;
            }
            continue;
          }
        }
        return pid;
      }
    }
  }

  if (unverified) {
    return *unverified;
  }
  return tl::make_unexpected("No process found for node " + node_namespace + "/" + node_name);
}

tl::expected<double, std::string> read_system_uptime(const std::string & root) {
  auto path = root + "/proc/uptime";
  std::ifstream f(path);
  if (!f.is_open()) {
    return tl::make_unexpected("Cannot open " + path);
  }
  double uptime = 0.0;
  if (!(f >> uptime)) {
    return tl::make_unexpected("Cannot parse " + path);
  }
  return uptime;
}

// --- PidCache implementation ---

PidCache::PidCache(std::chrono::steady_clock::duration ttl) : ttl_(ttl) {
}

void PidCache::set_domain_filter(size_t domain_id, UnverifiedCallback on_unverified,
                                 OtherDomainCallback on_other_domain) {
  std::unique_lock lock(mutex_);
  domain_id_ = domain_id;
  on_unverified_ = std::move(on_unverified);
  on_other_domain_ = std::move(on_other_domain);
  stale_ = true;
}

void PidCache::invalidate() {
  std::unique_lock lock(mutex_);
  stale_ = true;
}

bool PidCache::runs_as(pid_t pid, const std::string & fqn, const std::string & root) {
  std::string node_name;
  std::string node_ns;
  return parse_ros_args(root + "/proc/" + std::to_string(pid) + "/cmdline", node_name, node_ns) &&
         make_fqn(node_name, node_ns) == fqn;
}

std::pair<std::optional<pid_t>, bool> PidCache::cached(const std::string & node_fqn) const {
  auto it = node_to_pid_.find(node_fqn);
  if (it == node_to_pid_.end()) {
    return {std::nullopt, false};
  }
  bool to_report = on_unverified_ && unverified_.count(it->second) != 0 && unverified_reported_.count(it->second) == 0;
  return {it->second, to_report};
}

void PidCache::report_unverified(pid_t pid, const std::string & node_fqn) {
  UnverifiedCallback on_unverified;
  {
    std::unique_lock lock(mutex_);
    // A scan or another lookup may have run since the PID was read.
    if (unverified_.count(pid) == 0 || !unverified_reported_.insert(pid).second) {
      return;
    }
    on_unverified = on_unverified_;
  }
  if (on_unverified) {
    on_unverified(pid, node_fqn);
  }
}

std::optional<pid_t> PidCache::lookup(const std::string & node_fqn, const std::string & root) {
  std::pair<std::optional<pid_t>, bool> found;
  bool fresh = false;
  {
    std::shared_lock lock(mutex_);
    fresh = !stale_ && std::chrono::steady_clock::now() - last_refresh_ < ttl_;
    if (fresh) {
      found = cached(node_fqn);
    }
  }
  if (!fresh) {
    refresh(root);
    std::shared_lock lock(mutex_);
    found = cached(node_fqn);
  }
  if (found.second) {
    report_unverified(*found.first, node_fqn);
  }
  return found.first;
}

void PidCache::refresh(const std::string & root) {
  std::vector<OtherDomainProcess> other_domain;
  OtherDomainCallback on_other_domain;
  {
    std::unique_lock lock(mutex_);
    // Double-check under exclusive lock to avoid redundant scans
    auto now = std::chrono::steady_clock::now();
    if (!stale_ && now - last_refresh_ < ttl_) {
      return;
    }
    stale_ = false;
    scan(root, other_domain);
    last_refresh_ = std::chrono::steady_clock::now();
    on_other_domain = on_other_domain_;
  }
  if (on_other_domain) {
    for (const auto & process : other_domain) {
      on_other_domain(process.pid, process.fqn, process.process_domain);
    }
  }
}

void PidCache::scan(const std::string & root, std::vector<OtherDomainProcess> & other_domain) {
  node_to_pid_.clear();
  unverified_.clear();
  auto proc_dir = root + "/proc";
  UniqueDir dir(opendir(proc_dir.c_str()));
  if (!dir) {
    unverified_reported_.clear();
    return;
  }

  struct dirent * entry = nullptr;
  while ((entry = readdir(dir.get())) != nullptr) {
    if (entry->d_type != DT_DIR && entry->d_type != DT_UNKNOWN) {
      continue;
    }
    if (entry->d_type == DT_UNKNOWN) {
      struct stat st {};
      auto full_path = proc_dir + "/" + entry->d_name;
      if (stat(full_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        continue;
      }
    }
    bool all_digits = true;
    for (const char * p = entry->d_name; *p; ++p) {
      if (*p < '0' || *p > '9') {
        all_digits = false;
        break;
      }
    }
    if (!all_digits) {
      continue;
    }

    auto cmdline_path = proc_dir + "/" + entry->d_name + "/cmdline";
    std::string node_name;
    std::string node_ns;
    if (parse_ros_args(cmdline_path, node_name, node_ns)) {
      auto fqn = make_fqn(node_name, node_ns);
      char * end = nullptr;
      long pid_val = std::strtol(entry->d_name, &end, 10);
      if (end == entry->d_name || *end != '\0' || pid_val <= 0) {
        continue;
      }
      auto pid = static_cast<pid_t>(pid_val);
      bool verified = true;
      if (domain_id_) {
        auto check = match_domain(pid, root, *domain_id_);
        if (check.match == DomainMatch::kOther) {
          other_domain.push_back({pid, fqn, check.process_domain});
          continue;
        }
        verified = check.match == DomainMatch::kSame;
      }
      auto [it, inserted] = node_to_pid_.try_emplace(fqn, pid);
      if (!inserted) {
        // The last match wins, but a name-only match never replaces a verified one.
        if (!verified && unverified_.count(it->second) == 0) {
          continue;
        }
        unverified_.erase(it->second);
        it->second = pid;
      }
      if (!verified) {
        unverified_.insert(pid);
      }
    }
  }
  // Only PIDs still served unverified stay reported, so the set does not grow.
  for (auto it = unverified_reported_.begin(); it != unverified_reported_.end();) {
    it = unverified_.count(*it) != 0 ? std::next(it) : unverified_reported_.erase(it);
  }
}

size_t PidCache::size() const {
  std::shared_lock lock(mutex_);
  return node_to_pid_.size();
}

}  // namespace ros2_medkit_linux_introspection
