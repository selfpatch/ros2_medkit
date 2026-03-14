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

#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <nlohmann/json.hpp>

#include <unistd.h>

namespace ros2_medkit_linux_introspection {

/// Convert ProcessInfo to JSON for the procfs plugin HTTP response.
/// @param info Process information from read_process_info()
/// @param system_uptime System uptime in seconds from read_system_uptime()
/// @return JSON object with all process fields
inline nlohmann::json process_info_to_json(const ProcessInfo & info, double system_uptime) {
  long ticks_per_sec = sysconf(_SC_CLK_TCK);
  double uptime_sec = 0.0;
  if (ticks_per_sec > 0 && info.start_time_ticks > 0 && system_uptime > 0.0) {
    double start_sec = static_cast<double>(info.start_time_ticks) / static_cast<double>(ticks_per_sec);
    uptime_sec = system_uptime - start_sec;
    if (uptime_sec < 0.0) {
      uptime_sec = 0.0;
    }
  }

  double cpu_user_sec = 0.0;
  double cpu_system_sec = 0.0;
  if (ticks_per_sec > 0) {
    cpu_user_sec = static_cast<double>(info.cpu_user_ticks) / static_cast<double>(ticks_per_sec);
    cpu_system_sec = static_cast<double>(info.cpu_system_ticks) / static_cast<double>(ticks_per_sec);
  }

  return {{"pid", info.pid},
          {"ppid", info.ppid},
          {"state", info.state},
          {"exe", info.exe_path},
          {"cmdline", info.cmdline},
          {"rss_bytes", info.rss_bytes},
          {"vm_size_bytes", info.vm_size_bytes},
          {"threads", info.num_threads},
          {"cpu_user_ticks", info.cpu_user_ticks},
          {"cpu_system_ticks", info.cpu_system_ticks},
          {"cpu_user_seconds", cpu_user_sec},
          {"cpu_system_seconds", cpu_system_sec},
          {"uptime_seconds", uptime_sec}};
}

}  // namespace ros2_medkit_linux_introspection
