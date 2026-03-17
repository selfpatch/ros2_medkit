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

#include <filesystem>
#include <map>
#include <mutex>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/providers/script_provider.hpp"

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
  std::vector<std::string> supported_execution_types = {"now"};
  std::vector<ScriptEntryConfig> entries;
};

/// Tracks the state of a running script execution (stub for Task 7).
struct ExecutionState {
  std::string id;
  std::string script_id;
  std::string entity_id;
  std::string status;
  int pid = -1;
  std::optional<std::string> started_at;
  std::optional<std::string> completed_at;
  std::optional<int> progress;
  std::optional<nlohmann::json> output_parameters;
  std::optional<nlohmann::json> error;
};

/// Built-in ScriptProvider that loads manifest entries and supports filesystem CRUD for uploaded
/// scripts. Execution methods are stubbed (implemented in Task 7).
class DefaultScriptProvider : public ScriptProvider {
 public:
  explicit DefaultScriptProvider(const ScriptsConfig & config);

  tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo> list_scripts(const std::string & entity_id) override;

  tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & entity_id,
                                                              const std::string & script_id) override;

  tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                const std::optional<nlohmann::json> & metadata) override;

  tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & entity_id,
                                                           const std::string & script_id) override;

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> start_execution(const std::string & entity_id,
                                                                      const std::string & script_id,
                                                                      const ExecutionRequest & request) override;

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> get_execution(const std::string & entity_id,
                                                                    const std::string & script_id,
                                                                    const std::string & execution_id) override;

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & entity_id,
                                                                        const std::string & script_id,
                                                                        const std::string & execution_id,
                                                                        const std::string & action) override;

  tl::expected<void, ScriptBackendErrorInfo> delete_execution(const std::string & entity_id,
                                                              const std::string & script_id,
                                                              const std::string & execution_id) override;

 private:
  /// Returns the filesystem path for a given entity/script pair.
  std::filesystem::path script_dir_path(const std::string & entity_id, const std::string & script_id) const;

  /// Generates a unique script ID.
  std::string generate_id();

  /// Returns true if the given script_id is a manifest-defined script.
  bool is_manifest_script(const std::string & script_id) const;

  /// Returns true if an entity_id matches any of the given filter globs.
  /// An empty filter list matches everything.
  static bool matches_entity_filter(const std::string & entity_id, const std::vector<std::string> & filters);

  /// Detects script format from filename extension.
  static std::string detect_format(const std::string & filename);

  /// Returns the current time as an ISO 8601 string.
  static std::string now_iso8601();

  ScriptsConfig config_;
  std::map<std::string, ScriptEntryConfig> manifest_scripts_;
  mutable std::mutex fs_mutex_;

  // Execution tracking (populated by Task 7)
  std::unordered_map<std::string, std::unique_ptr<ExecutionState>> executions_;
  std::mutex exec_mutex_;
  int active_execution_count_ = 0;

  // Counter for ID generation
  int id_counter_ = 0;
};

}  // namespace ros2_medkit_gateway
