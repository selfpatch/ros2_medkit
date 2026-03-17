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

#include "ros2_medkit_gateway/default_script_provider.hpp"

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

namespace ros2_medkit_gateway {

// --- ExecutionState destructor ---

ExecutionState::~ExecutionState() {
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }
  if (timeout_thread.joinable()) {
    timeout_thread.join();
  }
}

// --- Constructor ---

DefaultScriptProvider::DefaultScriptProvider(const ScriptsConfig & config) : config_(config) {
  // Populate manifest_scripts_ from config entries
  for (const auto & entry : config_.entries) {
    manifest_scripts_[entry.id] = entry;
  }

  // Ensure the scripts directory exists
  if (!config_.scripts_dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(config_.scripts_dir, ec);
  }
}

// --- CRUD methods (unchanged from Task 6) ---

tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
DefaultScriptProvider::list_scripts(const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(fs_mutex_);

  std::vector<ScriptInfo> result;

  // 1. Collect manifest scripts matching the entity filter
  for (const auto & [id, entry] : manifest_scripts_) {
    if (matches_entity_filter(entity_id, entry.entity_filter)) {
      ScriptInfo info;
      info.id = entry.id;
      info.name = entry.name;
      info.description = entry.description;
      info.managed = true;
      info.proximity_proof_required = false;
      info.parameters_schema = entry.parameters_schema;
      result.push_back(std::move(info));
    }
  }

  // 2. Scan filesystem for uploaded scripts: scripts_dir/{entity_id}/*/metadata.json
  if (!config_.scripts_dir.empty()) {
    auto entity_dir = std::filesystem::path(config_.scripts_dir) / entity_id;
    auto canonical_base = std::filesystem::weakly_canonical(config_.scripts_dir);
    auto canonical_entity_dir = std::filesystem::weakly_canonical(entity_dir);
    auto base_str = canonical_base.string();
    if (base_str.back() != '/') {
      base_str += '/';
    }
    auto entity_str = canonical_entity_dir.string();
    if (entity_str.size() < base_str.size() || entity_str.compare(0, base_str.size(), base_str) != 0) {
      return result;  // Path escaped scripts_dir - return manifest-only results
    }
    std::error_code ec;
    if (std::filesystem::is_directory(entity_dir, ec)) {
      for (const auto & dir_entry : std::filesystem::directory_iterator(entity_dir, ec)) {
        if (!dir_entry.is_directory()) {
          continue;
        }
        auto meta_path = dir_entry.path() / "metadata.json";
        if (!std::filesystem::exists(meta_path, ec)) {
          continue;
        }
        try {
          std::ifstream meta_file(meta_path);
          auto meta = nlohmann::json::parse(meta_file);
          ScriptInfo info;
          info.id = dir_entry.path().filename().string();
          info.name = meta.value("name", info.id);
          info.description = meta.value("description", "");
          info.managed = false;
          info.proximity_proof_required = false;
          if (meta.contains("parameters_schema") && !meta["parameters_schema"].is_null()) {
            info.parameters_schema = meta["parameters_schema"];
          }
          result.push_back(std::move(info));
        } catch (const std::exception &) {
          // Skip malformed metadata files
          continue;
        }
      }
    }
  }

  return result;
}

tl::expected<ScriptInfo, ScriptBackendErrorInfo> DefaultScriptProvider::get_script(const std::string & entity_id,
                                                                                   const std::string & script_id) {
  // Check manifest first
  auto manifest_it = manifest_scripts_.find(script_id);
  if (manifest_it != manifest_scripts_.end()) {
    const auto & entry = manifest_it->second;
    if (matches_entity_filter(entity_id, entry.entity_filter)) {
      ScriptInfo info;
      info.id = entry.id;
      info.name = entry.name;
      info.description = entry.description;
      info.managed = true;
      info.proximity_proof_required = false;
      info.parameters_schema = entry.parameters_schema;
      return info;
    }
  }

  // Check filesystem
  if (!config_.scripts_dir.empty()) {
    auto dir = script_dir_path(entity_id, script_id);
    if (dir.empty()) {
      return tl::make_unexpected(
          ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
    }
    auto meta_path = dir / "metadata.json";
    std::error_code ec;
    if (std::filesystem::exists(meta_path, ec)) {
      try {
        std::ifstream meta_file(meta_path);
        auto meta = nlohmann::json::parse(meta_file);
        ScriptInfo info;
        info.id = script_id;
        info.name = meta.value("name", script_id);
        info.description = meta.value("description", "");
        info.managed = false;
        info.proximity_proof_required = false;
        if (meta.contains("parameters_schema") && !meta["parameters_schema"].is_null()) {
          info.parameters_schema = meta["parameters_schema"];
        }
        return info;
      } catch (const std::exception &) {
        return tl::make_unexpected(
            ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to read script metadata"});
      }
    }
  }

  return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
}

tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
DefaultScriptProvider::upload_script(const std::string & entity_id, const std::string & filename,
                                     const std::string & content, const std::optional<nlohmann::json> & metadata) {
  std::lock_guard<std::mutex> lock(fs_mutex_);

  if (config_.scripts_dir.empty()) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, "Scripts directory not configured"});
  }

  // Check file size
  auto max_bytes = static_cast<size_t>(config_.max_file_size_mb) * 1024 * 1024;
  if (content.size() > max_bytes) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::FileTooLarge, "Script file exceeds maximum size of " +
                                                                     std::to_string(config_.max_file_size_mb) + " MB"});
  }

  // Generate a unique script ID
  auto script_id = generate_id();

  // Detect format from filename
  auto format = detect_format(filename);

  // Create directory
  auto dir = script_dir_path(entity_id, script_id);
  if (dir.empty()) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::InvalidInput, "Invalid script ID"});
  }
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to create script directory: " + ec.message()});
  }

  // Determine extension from format
  std::string ext;
  if (format == "python") {
    ext = ".py";
  } else if (format == "bash") {
    ext = ".bash";
  } else {
    ext = ".sh";
  }

  // Write script file
  auto script_path = dir / ("script" + ext);
  {
    std::ofstream script_file(script_path, std::ios::binary);
    if (!script_file) {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to write script file"});
    }
    script_file.write(content.data(), static_cast<std::streamsize>(content.size()));
  }

  // Build metadata JSON
  std::string name = filename;
  std::string description;
  std::optional<nlohmann::json> parameters_schema;

  if (metadata.has_value()) {
    const auto & meta = metadata.value();
    if (meta.contains("name") && meta["name"].is_string()) {
      name = meta["name"].get<std::string>();
    }
    if (meta.contains("description") && meta["description"].is_string()) {
      description = meta["description"].get<std::string>();
    }
    if (meta.contains("parameters_schema") && !meta["parameters_schema"].is_null()) {
      parameters_schema = meta["parameters_schema"];
    }
  }

  nlohmann::json meta_json;
  meta_json["name"] = name;
  meta_json["description"] = description;
  meta_json["filename"] = filename;
  meta_json["format"] = format;
  meta_json["created_at"] = now_iso8601();
  if (parameters_schema.has_value()) {
    meta_json["parameters_schema"] = parameters_schema.value();
  } else {
    meta_json["parameters_schema"] = nullptr;
  }

  // Write metadata.json
  auto meta_path = dir / "metadata.json";
  {
    std::ofstream meta_file(meta_path);
    if (!meta_file) {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to write metadata file"});
    }
    meta_file << meta_json.dump(2);
  }

  return ScriptUploadResult{script_id, name};
}

tl::expected<void, ScriptBackendErrorInfo> DefaultScriptProvider::delete_script(const std::string & entity_id,
                                                                                const std::string & script_id) {
  // Check if it's a manifest script - cannot delete those
  if (is_manifest_script(script_id)) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::ManagedScript, "Cannot delete managed script: " + script_id});
  }

  std::lock_guard<std::mutex> lock(fs_mutex_);

  auto dir = script_dir_path(entity_id, script_id);
  if (dir.empty()) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
  }
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
  }

  std::filesystem::remove_all(dir, ec);
  if (ec) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to delete script: " + ec.message()});
  }

  return {};
}

// --- Execution methods ---

tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
DefaultScriptProvider::start_execution(const std::string & entity_id, const std::string & script_id,
                                       const ExecutionRequest & request) {
  // Validate execution type
  auto & types = config_.supported_execution_types;
  if (std::find(types.begin(), types.end(), request.execution_type) == types.end()) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::UnsupportedType,
                                                      "Unsupported execution type: " + request.execution_type});
  }

  // Check concurrency limit
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (active_execution_count_ >= config_.max_concurrent_executions) {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::ConcurrencyLimit,
                                                        "Maximum concurrent executions reached (" +
                                                            std::to_string(config_.max_concurrent_executions) + ")"});
    }
  }

  // Resolve script
  auto resolved = resolve_script(entity_id, script_id);
  if (!resolved) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
  }

  // Build command
  auto interpreter = interpreter_for_format(resolved->format);
  std::string script_path = resolved->path;

  // Check if args config is present to decide arg passing strategy
  bool has_args_config = resolved->args.is_array() && !resolved->args.empty();
  std::vector<std::string> cmd_args;
  bool pass_stdin_json = false;

  if (has_args_config) {
    cmd_args = build_command_args(resolved->args, request.parameters);
  } else if (request.parameters.has_value()) {
    // No args config - pass JSON on stdin
    pass_stdin_json = true;
  }

  // Prepare stdin data if needed
  std::string stdin_data;
  if (pass_stdin_json && request.parameters.has_value()) {
    stdin_data = request.parameters.value().dump();
  }

  // Create pipes for stdout, stderr, and optionally stdin
  int stdout_pipe[2];
  int stderr_pipe[2];
  int stdin_pipe[2];

  if (pipe(stdout_pipe) != 0 || pipe(stderr_pipe) != 0) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to create pipes"});
  }

  if (pass_stdin_json) {
    if (pipe(stdin_pipe) != 0) {
      close(stdout_pipe[0]);
      close(stdout_pipe[1]);
      close(stderr_pipe[0]);
      close(stderr_pipe[1]);
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to create stdin pipe"});
    }
  }

  // Build argv for execvp
  std::vector<std::string> argv_strings;
  argv_strings.push_back(interpreter);
  argv_strings.push_back(script_path);
  for (const auto & arg : cmd_args) {
    argv_strings.push_back(arg);
  }

  std::vector<char *> argv;
  argv.reserve(argv_strings.size() + 1);
  for (auto & s : argv_strings) {
    argv.push_back(s.data());
  }
  argv.push_back(nullptr);

  // Collect env vars to set in child process
  auto env_vars = resolved->env;

  // Fork
  pid_t child_pid = fork();
  if (child_pid < 0) {
    close(stdout_pipe[0]);
    close(stdout_pipe[1]);
    close(stderr_pipe[0]);
    close(stderr_pipe[1]);
    if (pass_stdin_json) {
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to fork process"});
  }

  if (child_pid == 0) {
    // --- Child process ---

    // Create a new process group so we can kill all child processes on timeout
    setpgid(0, 0);

    // Redirect stdin if passing JSON
    if (pass_stdin_json) {
      close(stdin_pipe[1]);  // Close write end
      dup2(stdin_pipe[0], STDIN_FILENO);
      close(stdin_pipe[0]);
    }

    // Redirect stdout
    close(stdout_pipe[0]);  // Close read end
    dup2(stdout_pipe[1], STDOUT_FILENO);
    close(stdout_pipe[1]);

    // Redirect stderr
    close(stderr_pipe[0]);  // Close read end
    dup2(stderr_pipe[1], STDERR_FILENO);
    close(stderr_pipe[1]);

    // Set custom environment variables in the child
    for (const auto & [env_key, env_val] : env_vars) {
      // NOLINTNEXTLINE(concurrency-mt-unsafe)
      setenv(env_key.c_str(), env_val.c_str(), 1);
    }

    // Execute (uses PATH lookup)
    execvp(argv[0], argv.data());

    // If exec fails, write error and exit
    const char * err_msg = "Failed to execute script\n";
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    auto written = write(STDERR_FILENO, err_msg, strlen(err_msg));
    (void)written;
    _exit(127);
  }

  // --- Parent process ---

  // Close pipe ends we don't use
  close(stdout_pipe[1]);
  close(stderr_pipe[1]);

  // Write stdin data and close
  if (pass_stdin_json) {
    close(stdin_pipe[0]);  // Close read end in parent
    auto written = write(stdin_pipe[1], stdin_data.data(), stdin_data.size());
    (void)written;
    close(stdin_pipe[1]);
  }

  // Create execution state
  auto exec_id = generate_execution_id();
  auto state = std::make_unique<ExecutionState>();
  state->id = exec_id;
  state->script_id = script_id;
  state->entity_id = entity_id;
  state->status = "running";
  state->pid = child_pid;
  state->started_at = now_iso8601();

  int timeout_sec = resolved->timeout_sec;

  // Get raw pointer for thread lambdas (safe: pointer stability via unique_ptr in map)
  auto * state_ptr = state.get();

  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    active_execution_count_++;
    executions_[exec_id] = std::move(state);
  }

  // Start I/O monitoring thread
  int stdout_fd = stdout_pipe[0];
  int stderr_fd = stderr_pipe[0];

  state_ptr->monitor_thread = std::thread([this, state_ptr, stdout_fd, stderr_fd, child_pid]() {
    // Read stdout and stderr until EOF
    constexpr int buf_size = 4096;
    char buf[buf_size];
    bool stdout_open = true;
    bool stderr_open = true;

    // Use select() to read from both pipes
    while (stdout_open || stderr_open) {
      fd_set read_fds;
      FD_ZERO(&read_fds);

      int max_fd = -1;
      if (stdout_open) {
        FD_SET(stdout_fd, &read_fds);
        max_fd = std::max(max_fd, stdout_fd);
      }
      if (stderr_open) {
        FD_SET(stderr_fd, &read_fds);
        max_fd = std::max(max_fd, stderr_fd);
      }

      struct timeval tv;
      tv.tv_sec = 1;
      tv.tv_usec = 0;

      int ready = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }

      if (ready == 0) {
        continue;  // timeout, loop again
      }

      if (stdout_open && FD_ISSET(stdout_fd, &read_fds)) {
        auto n = read(stdout_fd, buf, sizeof(buf));
        if (n > 0) {
          state_ptr->stdout_data.append(buf, static_cast<size_t>(n));
        } else {
          stdout_open = false;
        }
      }

      if (stderr_open && FD_ISSET(stderr_fd, &read_fds)) {
        auto n = read(stderr_fd, buf, sizeof(buf));
        if (n > 0) {
          state_ptr->stderr_data.append(buf, static_cast<size_t>(n));
        } else {
          stderr_open = false;
        }
      }
    }

    close(stdout_fd);
    close(stderr_fd);

    // Reap the child process
    int wstatus = 0;
    waitpid(child_pid, &wstatus, 0);

    // Update state under lock
    std::lock_guard<std::mutex> lock(exec_mutex_);

    if (state_ptr->status == "running") {
      if (state_ptr->timed_out.load()) {
        state_ptr->status = "terminated";
        state_ptr->error = nlohmann::json{{"message", "Script execution timed out"}};
      } else if (WIFEXITED(wstatus)) {
        state_ptr->exit_code = WEXITSTATUS(wstatus);
        if (state_ptr->exit_code == 0) {
          state_ptr->status = "completed";
          // Try to parse stdout as JSON for output_parameters
          if (!state_ptr->stdout_data.empty()) {
            try {
              state_ptr->output_parameters = nlohmann::json::parse(state_ptr->stdout_data);
            } catch (const std::exception &) {
              // Not JSON - store as raw string
              state_ptr->output_parameters = nlohmann::json{{"stdout", state_ptr->stdout_data}};
            }
          }
        } else {
          state_ptr->status = "failed";
          std::string err_msg = state_ptr->stderr_data.empty()
                                    ? "Script exited with code " + std::to_string(state_ptr->exit_code)
                                    : state_ptr->stderr_data;
          state_ptr->error = nlohmann::json{{"message", err_msg}, {"exit_code", state_ptr->exit_code}};
        }
      } else if (WIFSIGNALED(wstatus)) {
        state_ptr->status = "terminated";
        int sig = WTERMSIG(wstatus);
        state_ptr->error = nlohmann::json{{"message", "Script killed by signal " + std::to_string(sig)}};
      }
    }

    state_ptr->completed_at = now_iso8601();
    active_execution_count_--;
  });

  // Start timeout thread if configured
  if (timeout_sec > 0) {
    state_ptr->timeout_thread = std::thread([state_ptr, timeout_sec, child_pid]() {
      // Sleep for timeout period (in 100ms increments to allow early exit)
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
      while (std::chrono::steady_clock::now() < deadline) {
        // Check if process already finished (without reaping - leave that to monitor thread)
        if (kill(child_pid, 0) != 0) {
          return;  // Process no longer exists, no timeout needed
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      // Timeout reached - mark and send SIGTERM to process group
      state_ptr->timed_out.store(true);
      kill(-child_pid, SIGTERM);  // Negative PID = process group

      // Grace period (2 seconds), then SIGKILL
      auto grace_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (std::chrono::steady_clock::now() < grace_deadline) {
        if (kill(child_pid, 0) != 0) {
          return;  // Process exited after SIGTERM
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      // Still running after grace period - force kill the group
      kill(-child_pid, SIGKILL);
    });
  }

  // Return initial ExecutionInfo
  ExecutionInfo info;
  info.id = exec_id;
  info.status = "running";
  info.started_at = state_ptr->started_at;
  return info;
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
DefaultScriptProvider::get_execution(const std::string & /*entity_id*/, const std::string & /*script_id*/,
                                     const std::string & execution_id) {
  std::lock_guard<std::mutex> lock(exec_mutex_);

  auto it = executions_.find(execution_id);
  if (it == executions_.end()) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Execution not found: " + execution_id});
  }

  return state_to_info(*it->second);
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
DefaultScriptProvider::control_execution(const std::string & /*entity_id*/, const std::string & /*script_id*/,
                                         const std::string & execution_id, const std::string & action) {
  std::lock_guard<std::mutex> lock(exec_mutex_);

  auto it = executions_.find(execution_id);
  if (it == executions_.end()) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Execution not found: " + execution_id});
  }

  auto & state = *it->second;

  if (state.status != "running") {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotRunning,
                                                      "Execution is not running (status: " + state.status + ")"});
  }

  if (action == "stop") {
    kill(-state.pid, SIGTERM);  // Kill the process group
    state.status = "terminated";
    state.error = nlohmann::json{{"message", "Execution stopped by user"}};
    return state_to_info(state);
  }

  if (action == "forced_termination") {
    kill(-state.pid, SIGKILL);  // Kill the process group
    state.status = "terminated";
    state.error = nlohmann::json{{"message", "Execution forcefully terminated"}};
    return state_to_info(state);
  }

  return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::InvalidInput, "Unknown action: " + action});
}

tl::expected<void, ScriptBackendErrorInfo> DefaultScriptProvider::delete_execution(const std::string & /*entity_id*/,
                                                                                   const std::string & /*script_id*/,
                                                                                   const std::string & execution_id) {
  std::lock_guard<std::mutex> lock(exec_mutex_);

  auto it = executions_.find(execution_id);
  if (it == executions_.end()) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Execution not found: " + execution_id});
  }

  auto & state = *it->second;
  if (state.status == "running") {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::AlreadyRunning, "Cannot delete running execution. Stop it first."});
  }

  // Detach threads before erasing so the unique_ptr destructor doesn't join
  // (they should already be finished since status is not "running")
  if (state.monitor_thread.joinable()) {
    state.monitor_thread.join();
  }
  if (state.timeout_thread.joinable()) {
    state.timeout_thread.join();
  }

  executions_.erase(it);
  return {};
}

// --- Private helpers ---

std::filesystem::path DefaultScriptProvider::script_dir_path(const std::string & entity_id,
                                                             const std::string & script_id) const {
  auto path = std::filesystem::path(config_.scripts_dir) / entity_id / script_id;
  auto canonical_base = std::filesystem::weakly_canonical(config_.scripts_dir);
  auto canonical_path = std::filesystem::weakly_canonical(path);
  // Safety check: verify path is strictly inside scripts_dir (not equal to it)
  auto base_str = canonical_base.string();
  if (base_str.back() != '/') {
    base_str += '/';
  }
  auto path_str = canonical_path.string();
  if (path_str.size() < base_str.size() || path_str.compare(0, base_str.size(), base_str) != 0) {
    return {};  // Path escaped scripts_dir or equals scripts_dir
  }
  // Verify path is exactly 2 levels deep (entity_id/script_id)
  auto relative = canonical_path.lexically_relative(canonical_base);
  auto it = relative.begin();
  int depth = 0;
  for (; it != relative.end(); ++it) {
    if (*it == "..") {
      return {};  // Should not happen after canonicalization, but be safe
    }
    depth++;
  }
  if (depth != 2) {
    return {};  // Must be exactly entity_id/script_id
  }
  return canonical_path;
}

std::string DefaultScriptProvider::generate_id() {
  return "script_" + std::to_string(id_counter_++);
}

std::string DefaultScriptProvider::generate_execution_id() {
  return "exec_" + std::to_string(exec_id_counter_++);
}

bool DefaultScriptProvider::is_manifest_script(const std::string & script_id) const {
  return manifest_scripts_.find(script_id) != manifest_scripts_.end();
}

bool DefaultScriptProvider::matches_entity_filter(const std::string & entity_id,
                                                  const std::vector<std::string> & filters) {
  // Empty filter list matches everything
  if (filters.empty()) {
    return true;
  }

  for (const auto & filter : filters) {
    // Wildcard matches everything
    if (filter == "*") {
      return true;
    }

    // Check for glob pattern ending with /* (e.g. "components/*")
    if (filter.size() >= 2 && filter.substr(filter.size() - 2) == "/*") {
      // The prefix before /* is informational (entity type hint) - since entity_id
      // alone does not carry type info, a trailing wildcard matches any entity
      return true;
    }

    // Exact match
    if (filter == entity_id) {
      return true;
    }
  }

  return false;
}

std::string DefaultScriptProvider::detect_format(const std::string & filename) {
  auto dot_pos = filename.rfind('.');
  if (dot_pos == std::string::npos) {
    return "sh";
  }

  auto ext = filename.substr(dot_pos);
  if (ext == ".py") {
    return "python";
  }
  if (ext == ".bash") {
    return "bash";
  }
  return "sh";
}

std::string DefaultScriptProvider::now_iso8601() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  gmtime_r(&time_t_now, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

std::optional<DefaultScriptProvider::ResolvedScript>
DefaultScriptProvider::resolve_script(const std::string & entity_id, const std::string & script_id) {
  // Check manifest first
  auto manifest_it = manifest_scripts_.find(script_id);
  if (manifest_it != manifest_scripts_.end()) {
    const auto & entry = manifest_it->second;
    if (matches_entity_filter(entity_id, entry.entity_filter)) {
      ResolvedScript resolved;
      resolved.path = entry.path;
      resolved.format = entry.format;
      resolved.timeout_sec = entry.timeout_sec;
      resolved.env = entry.env;
      resolved.args = entry.args;
      return resolved;
    }
  }

  // Check filesystem uploads
  if (!config_.scripts_dir.empty()) {
    auto dir = script_dir_path(entity_id, script_id);
    if (dir.empty()) {
      return std::nullopt;
    }
    auto meta_path = dir / "metadata.json";
    std::error_code ec;
    if (std::filesystem::exists(meta_path, ec)) {
      try {
        std::ifstream meta_file(meta_path);
        auto meta = nlohmann::json::parse(meta_file);
        auto format = meta.value("format", "sh");

        // Find the script file
        std::string ext;
        if (format == "python") {
          ext = ".py";
        } else if (format == "bash") {
          ext = ".bash";
        } else {
          ext = ".sh";
        }
        auto script_path = dir / ("script" + ext);
        if (!std::filesystem::exists(script_path, ec)) {
          return std::nullopt;
        }

        ResolvedScript resolved;
        resolved.path = script_path.string();
        resolved.format = format;
        resolved.timeout_sec = config_.default_timeout_sec;
        return resolved;
      } catch (const std::exception &) {
        return std::nullopt;
      }
    }
  }

  return std::nullopt;
}

std::string DefaultScriptProvider::interpreter_for_format(const std::string & format) {
  if (format == "python") {
    return "python3";
  }
  if (format == "bash") {
    return "bash";
  }
  return "sh";
}

std::vector<std::string> DefaultScriptProvider::build_command_args(const nlohmann::json & args_config,
                                                                   const std::optional<nlohmann::json> & parameters) {
  std::vector<std::string> result;

  if (!args_config.is_array()) {
    return result;
  }

  for (const auto & arg_def : args_config) {
    auto name = arg_def.value("name", "");
    auto type = arg_def.value("type", "positional");
    auto flag = arg_def.value("flag", "");

    // Get parameter value if available
    std::string value_str;
    bool has_value = false;
    if (parameters.has_value() && parameters->contains(name)) {
      const auto & val = (*parameters)[name];
      if (val.is_string()) {
        value_str = val.get<std::string>();
      } else if (val.is_boolean()) {
        value_str = val.get<bool>() ? "true" : "false";
        has_value = val.get<bool>();
      } else {
        value_str = val.dump();
      }
      if (!val.is_boolean()) {
        has_value = true;
      }
    }

    if (type == "positional") {
      if (has_value || !value_str.empty()) {
        result.push_back(value_str);
      }
    } else if (type == "named") {
      if ((has_value || !value_str.empty()) && !flag.empty()) {
        result.push_back(flag);
        result.push_back(value_str);
      }
    } else if (type == "flag") {
      if (has_value && !flag.empty()) {
        result.push_back(flag);
      }
    }
  }

  return result;
}

ExecutionInfo DefaultScriptProvider::state_to_info(const ExecutionState & state) {
  ExecutionInfo info;
  info.id = state.id;
  info.status = state.status;
  info.progress = state.progress;
  info.started_at = state.started_at;
  info.completed_at = state.completed_at;
  info.output_parameters = state.output_parameters;
  info.error = state.error;
  return info;
}

}  // namespace ros2_medkit_gateway
