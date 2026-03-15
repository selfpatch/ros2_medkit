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

#include <chrono>
#include <memory>
#include <string>

namespace ros2_medkit_linux_introspection {

struct IntrospectionConfig {
  std::unique_ptr<PidCache> pid_cache;
  std::string proc_root{"/"};
};

inline IntrospectionConfig parse_introspection_config(const nlohmann::json & config) {
  IntrospectionConfig result;

  std::chrono::seconds ttl{10};
  if (config.contains("pid_cache_ttl_seconds")) {
    auto val = config["pid_cache_ttl_seconds"].get<int>();
    if (val < 1) {
      val = 1;
    }
    ttl = std::chrono::seconds{val};
  }
  if (config.contains("proc_root")) {
    result.proc_root = config["proc_root"].get<std::string>();
    if (result.proc_root.empty() || result.proc_root[0] != '/') {
      result.proc_root = "/";
    }
  }
  result.pid_cache = std::make_unique<PidCache>(ttl);

  return result;
}

}  // namespace ros2_medkit_linux_introspection
