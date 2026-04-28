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

#include "ros2_medkit_gateway/core/providers/script_provider.hpp"

namespace ros2_medkit_gateway {

/// @brief Central manager for diagnostic script operations.
///
/// Acts as the integration point between HTTP handlers and the pluggable
/// ScriptProvider backend. Provides null-safety, exception isolation for
/// plugin-provided backends, and logging. The backend is set once during
/// gateway initialization via set_backend() and must not be changed
/// while HTTP threads are active.
///
/// Thread safety: All methods are safe to call concurrently from multiple
/// HTTP handler threads. The manager itself is stateless (delegates to backend).
class ScriptManager {
 public:
  ScriptManager() = default;
  ~ScriptManager() = default;
  ScriptManager(const ScriptManager &) = delete;
  ScriptManager & operator=(const ScriptManager &) = delete;
  ScriptManager(ScriptManager &&) = delete;
  ScriptManager & operator=(ScriptManager &&) = delete;

  /// Set the script backend provider. Must be called during initialization,
  /// before any HTTP threads start. Not thread-safe with concurrent method calls.
  void set_backend(ScriptProvider * backend);

  /// @return true if a backend provider is configured
  bool has_backend() const;

  /// List all scripts available for the given entity.
  /// @param entity_id The entity to list scripts for
  /// @return Vector of ScriptInfo or error
  tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo> list_scripts(const std::string & entity_id);

  /// Get detailed information about a specific script.
  /// @param entity_id The owning entity
  /// @param script_id The script to retrieve
  /// @return ScriptInfo or error (e.g., NotFound)
  tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & entity_id,
                                                              const std::string & script_id);

  /// Upload a new user-provided script.
  /// @param entity_id The target entity
  /// @param filename Original filename of the uploaded script
  /// @param content Raw script content
  /// @param metadata Optional JSON metadata to associate with the script
  /// @return Upload result containing the assigned script ID, or error
  tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                const std::optional<nlohmann::json> & metadata);

  /// Delete a user-uploaded script. Managed (built-in) scripts cannot be deleted.
  /// @param entity_id The owning entity
  /// @param script_id The script to delete
  /// @return void on success, or error (e.g., NotFound, ManagedScript)
  tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & entity_id,
                                                           const std::string & script_id);

  /// Start a new script execution.
  /// @param entity_id The target entity
  /// @param script_id The script to execute
  /// @param request Execution parameters (type, input parameters, proximity proof)
  /// @return ExecutionInfo with the new execution ID, or error
  tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  start_execution(const std::string & entity_id, const std::string & script_id, const ExecutionRequest & request);

  /// Get the current status of a script execution.
  /// @param entity_id The owning entity
  /// @param script_id The parent script
  /// @param execution_id The execution to query
  /// @return ExecutionInfo or error (e.g., NotFound)
  tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  get_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id);

  /// Send a control action to a running execution (e.g., terminate).
  /// @param entity_id The owning entity
  /// @param script_id The parent script
  /// @param execution_id The execution to control
  /// @param action The control action (e.g., "terminate")
  /// @return Updated ExecutionInfo or error
  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & entity_id,
                                                                        const std::string & script_id,
                                                                        const std::string & execution_id,
                                                                        const std::string & action);

  /// Delete an execution record. Only completed/failed/terminated executions can be deleted.
  /// @param entity_id The owning entity
  /// @param script_id The parent script
  /// @param execution_id The execution to delete
  /// @return void on success, or error
  tl::expected<void, ScriptBackendErrorInfo>
  delete_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id);

 private:
  ScriptProvider * backend_ = nullptr;
};

}  // namespace ros2_medkit_gateway
