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

class ScriptProvider {
 public:
  virtual ~ScriptProvider() = default;

  virtual tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo> list_scripts(const std::string & entity_id) = 0;

  virtual tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & entity_id,
                                                                      const std::string & script_id) = 0;

  virtual tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                const std::optional<nlohmann::json> & metadata) = 0;

  virtual tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & entity_id,
                                                                   const std::string & script_id) = 0;

  virtual tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  start_execution(const std::string & entity_id, const std::string & script_id, const ExecutionRequest & request) = 0;

  virtual tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  get_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id) = 0;

  virtual tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & entity_id,
                                                                                const std::string & script_id,
                                                                                const std::string & execution_id,
                                                                                const std::string & action) = 0;

  virtual tl::expected<void, ScriptBackendErrorInfo>
  delete_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id) = 0;
};

}  // namespace ros2_medkit_gateway
