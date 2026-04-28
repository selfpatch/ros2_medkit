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

#include <atomic>
#include <filesystem>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/providers/script_provider.hpp"
#include "ros2_medkit_gateway/core/script_types.hpp"

namespace ros2_medkit_gateway {

/// Tracks the state of a running script execution.
struct ExecutionState {
  std::string id;
  std::string script_id;
  std::string entity_id;
  std::string status;  // running, completed, failed, terminated
  std::atomic<pid_t> pid{-1};
  std::optional<std::string> started_at;
  std::optional<std::string> completed_at;
  std::optional<int> progress;
  std::optional<nlohmann::json> output_parameters;
  std::optional<nlohmann::json> error;

  // Eviction ordering - monotonic counter set at creation time.
  // Used instead of completed_at timestamps to guarantee FIFO eviction.
  uint64_t creation_seq = 0;

  // Subprocess I/O
  std::string stdout_data;
  std::string stderr_data;
  int exit_code = -1;

  // Thread management
  std::atomic<bool> timed_out{false};
  std::thread monitor_thread;
  std::thread timeout_thread;

  ~ExecutionState();

  // Non-copyable, non-movable (due to atomic + threads)
  ExecutionState() = default;
  ExecutionState(const ExecutionState &) = delete;
  ExecutionState & operator=(const ExecutionState &) = delete;
  ExecutionState(ExecutionState &&) = delete;
  ExecutionState & operator=(ExecutionState &&) = delete;
};

/// Built-in ScriptProvider that loads manifest entries and supports filesystem CRUD for uploaded
/// scripts. Includes POSIX subprocess execution with timeout support.
class DefaultScriptProvider : public ScriptProvider {
 public:
  explicit DefaultScriptProvider(const ScriptsConfig & config);
  ~DefaultScriptProvider() override;

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

  /// Resolves the script path and format for execution.
  /// Checks manifest first, then filesystem uploads.
  struct ResolvedScript {
    std::string path;
    std::string format;
    int timeout_sec;
    std::map<std::string, std::string> env;
    nlohmann::json args;
  };
  std::optional<ResolvedScript> resolve_script(const std::string & entity_id, const std::string & script_id);

  /// Determines the interpreter command for a given format.
  static std::string interpreter_for_format(const std::string & format);

  /// Builds the command-line arguments from manifest args config and request parameters.
  static std::vector<std::string> build_command_args(const nlohmann::json & args_config,
                                                     const std::optional<nlohmann::json> & parameters);

  /// Converts an ExecutionState to an ExecutionInfo return value.
  static ExecutionInfo state_to_info(const ExecutionState & state);

  /// Generates a unique execution ID.
  std::string generate_execution_id();

  ScriptsConfig config_;
  mutable std::mutex fs_mutex_;
  std::map<std::string, ScriptEntryConfig> manifest_scripts_;

  // Execution tracking - exec_mutex_ declared before executions_ so the mutex
  // outlives the map (ExecutionState destructors join threads that lock it).
  mutable std::mutex exec_mutex_;
  std::unordered_map<std::string, std::unique_ptr<ExecutionState>> executions_;
  int active_execution_count_ = 0;

  /// Evicts oldest completed executions when the history exceeds max_execution_history.
  /// Called with exec_mutex_ already held. Moves evicted states into `to_evict` so
  /// the caller can let them destruct outside the lock.
  void evict_old_executions(std::vector<std::unique_ptr<ExecutionState>> & to_evict);

  // Counter for ID generation
  std::atomic<int> id_counter_{0};
  std::atomic<int> exec_id_counter_{0};
};

}  // namespace ros2_medkit_gateway
