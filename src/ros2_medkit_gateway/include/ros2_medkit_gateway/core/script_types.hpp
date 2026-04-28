// Copyright 2026 Bartlomiej Burda
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

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {

/// Configuration for a single pre-defined script loaded from manifest YAML.
struct ScriptEntryConfig {
  std::string id;
  std::string name;
  std::string description;
  std::string path;    // filesystem path to the script file
  std::string format;  // python, bash, sh
  int timeout_sec = 300;
  std::vector<std::string> entity_filter;  // globs like "components/*", "apps/*"
  std::map<std::string, std::string> env;  // environment variables
  nlohmann::json args;                     // array of {name, type, flag}
  std::optional<nlohmann::json> parameters_schema;
};

/// Top-level scripts configuration.
struct ScriptsConfig {
  std::string scripts_dir;
  int max_file_size_mb = 10;
  int max_concurrent_executions = 5;
  int default_timeout_sec = 300;
  int max_execution_history = 100;
  bool allow_uploads = true;
  std::vector<std::string> supported_execution_types = {"now"};
  std::vector<ScriptEntryConfig> entries;
};

}  // namespace ros2_medkit_gateway
