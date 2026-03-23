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

#include "ros2_medkit_linux_introspection/cgroup_reader.hpp"

#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

namespace ros2_medkit_linux_introspection {

namespace {

std::string read_first_line(const std::string & path) {
  std::ifstream f(path);
  std::string line;
  if (f.is_open()) {
    std::getline(f, line);
  }
  return line;
}

// Read cgroup path from /proc/{pid}/cgroup (cgroup v2: single line "0::<path>")
std::string read_cgroup_path(pid_t pid, const std::string & root) {
  auto path = root + "/proc/" + std::to_string(pid) + "/cgroup";
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    // cgroup v2 format: "0::<path>"
    if (line.rfind("0::", 0) == 0) {
      return line.substr(3);
    }
  }
  return {};
}

}  // namespace

std::string extract_container_id(const std::string & cgroup_path) {
  // Docker: /docker-<64hex>.scope or /docker/<64hex>
  // Podman: /libpod-<64hex>.scope
  // Containerd: /cri-containerd-<64hex>.scope
  static const std::regex re("(?:docker-|libpod-|cri-containerd-|docker/)([0-9a-f]{64})(?:\\.scope)?");
  std::smatch match;
  if (std::regex_search(cgroup_path, match, re)) {
    return match[1].str();
  }
  return {};
}

std::string detect_runtime(const std::string & cgroup_path) {
  if (cgroup_path.find("docker") != std::string::npos) {
    return "docker";
  }
  if (cgroup_path.find("libpod") != std::string::npos) {
    return "podman";
  }
  if (cgroup_path.find("cri-containerd") != std::string::npos) {
    return "containerd";
  }
  return {};
}

bool is_containerized(pid_t pid, const std::string & root) {
  auto cgroup_path = read_cgroup_path(pid, root);
  return !extract_container_id(cgroup_path).empty();
}

tl::expected<CgroupInfo, std::string> read_cgroup_info(pid_t pid, const std::string & root) {
  auto cgroup_path = read_cgroup_path(pid, root);
  if (cgroup_path.empty()) {
    return tl::make_unexpected("Cannot read cgroup for PID " + std::to_string(pid));
  }

  CgroupInfo info;
  info.cgroup_path = cgroup_path;
  info.container_id = extract_container_id(cgroup_path);
  info.container_runtime = detect_runtime(cgroup_path);

  // Read resource limits from cgroup v2 filesystem
  auto cgroup_fs_path = root + "/sys/fs/cgroup" + cgroup_path;

  // memory.max
  auto mem_max = read_first_line(cgroup_fs_path + "/memory.max");
  if (!mem_max.empty() && mem_max != "max") {
    try {
      info.memory_limit_bytes = std::stoull(mem_max);
    } catch (const std::exception & e) {
      std::cerr << "[ros2_medkit_linux_introspection] Failed to parse memory.max value '" << mem_max
                << "': " << e.what() << std::endl;
    }
  }

  // cpu.max (format: "quota period" or "max period")
  auto cpu_max = read_first_line(cgroup_fs_path + "/cpu.max");
  if (!cpu_max.empty()) {
    std::istringstream ss(cpu_max);
    std::string quota_str;
    int64_t period = 0;
    ss >> quota_str >> period;
    if (quota_str != "max") {
      try {
        info.cpu_quota_us = std::stoll(quota_str);
      } catch (const std::exception & e) {
        std::cerr << "[ros2_medkit_linux_introspection] Failed to parse cpu.max quota '" << quota_str
                  << "': " << e.what() << std::endl;
      }
    }
    if (period > 0) {
      info.cpu_period_us = period;
    }
  }

  return info;
}

}  // namespace ros2_medkit_linux_introspection
