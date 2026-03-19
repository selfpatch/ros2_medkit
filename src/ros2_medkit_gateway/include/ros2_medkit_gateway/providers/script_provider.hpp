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

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <vector>

namespace ros2_medkit_gateway {

struct ScriptInfo {
  std::string id;
  std::string name;
  std::string description;
  bool managed = false;
  bool proximity_proof_required = false;
  std::optional<nlohmann::json> parameters_schema;
};

struct ScriptUploadResult {
  std::string id;
  std::string name;
};

struct ExecutionInfo {
  std::string id;
  std::string status;  // prepared, running, completed, failed, terminated
  std::optional<int> progress;
  std::optional<std::string> started_at;
  std::optional<std::string> completed_at;
  std::optional<nlohmann::json> output_parameters;
  std::optional<nlohmann::json> error;
};

struct ExecutionRequest {
  std::string execution_type;  // now, on_restart, etc.
  std::optional<nlohmann::json> parameters;
  std::optional<std::string> proximity_response;
};

enum class ScriptBackendError {
  NotFound,
  AlreadyExists,  // Reserved for plugin implementations (e.g., SQLite registry duplicate IDs)
  ManagedScript,
  AlreadyRunning,
  NotRunning,
  ConcurrencyLimit,
  UnsupportedType,
  InvalidInput,
  FileTooLarge,
  Internal
};

struct ScriptBackendErrorInfo {
  ScriptBackendError code;
  std::string message;
};

/**
 * @brief Abstract interface for script storage and execution backends.
 *
 * Implementations are provided by the built-in DefaultScriptProvider or by
 * external plugins via dlopen. The ScriptManager wraps this interface with
 * null-safety, exception isolation, and logging.
 *
 * @par Thread Safety
 * All methods may be called concurrently from multiple HTTP handler threads.
 * Implementations MUST be thread-safe. The gateway does not serialize calls
 * to the provider. The ScriptManager's try/catch wrapper ensures that
 * exceptions thrown by implementations do not crash the gateway.
 *
 * @see ScriptManager for the subsystem that uses this
 * @see DefaultScriptProvider for the built-in implementation
 */
class ScriptProvider {
 public:
  virtual ~ScriptProvider() = default;

  /// @brief List scripts available for a given entity.
  /// @param entity_id The entity to list scripts for.
  /// @return A vector of ScriptInfo on success, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo> list_scripts(const std::string & entity_id) = 0;

  /// @brief Get metadata for a specific script.
  /// @param entity_id The entity that owns the script.
  /// @param script_id The script identifier.
  /// @return ScriptInfo on success, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & entity_id,
                                                                      const std::string & script_id) = 0;

  /// @brief Upload a new script file with optional metadata.
  /// @param entity_id The entity to associate the script with.
  /// @param filename Original filename (used for format detection).
  /// @param content Raw script file content.
  /// @param metadata Optional JSON metadata (name, description, parameters_schema).
  /// @return ScriptUploadResult with the generated ID, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                const std::optional<nlohmann::json> & metadata) = 0;

  /// @brief Delete an uploaded script. Manifest-managed scripts cannot be deleted.
  /// @param entity_id The entity that owns the script.
  /// @param script_id The script identifier.
  /// @return void on success, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & entity_id,
                                                                   const std::string & script_id) = 0;

  /// @brief Start executing a script as a subprocess.
  /// @param entity_id The entity context for execution.
  /// @param script_id The script to execute.
  /// @param request Execution parameters (type, input parameters, proximity proof).
  /// @return ExecutionInfo with initial status, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  start_execution(const std::string & entity_id, const std::string & script_id, const ExecutionRequest & request) = 0;

  /// @brief Get the current status of a script execution.
  /// @param entity_id The entity context.
  /// @param script_id The script that was executed.
  /// @param execution_id The execution identifier.
  /// @return ExecutionInfo with current status, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  get_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id) = 0;

  /// @brief Control a running execution (stop or force-terminate).
  /// @param entity_id The entity context.
  /// @param script_id The script that was executed.
  /// @param execution_id The execution identifier.
  /// @param action The control action ("stop" or "forced_termination").
  /// @return Updated ExecutionInfo, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & entity_id,
                                                                                const std::string & script_id,
                                                                                const std::string & execution_id,
                                                                                const std::string & action) = 0;

  /// @brief Delete a completed execution record. Running executions cannot be deleted.
  /// @param entity_id The entity context.
  /// @param script_id The script that was executed.
  /// @param execution_id The execution identifier.
  /// @return void on success, or ScriptBackendErrorInfo on failure.
  virtual tl::expected<void, ScriptBackendErrorInfo>
  delete_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id) = 0;
};

}  // namespace ros2_medkit_gateway
