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

#include "ros2_medkit_gateway/providers/script_provider.hpp"

namespace ros2_medkit_gateway {

class ScriptManager {
 public:
  ScriptManager() = default;

  void set_backend(ScriptProvider * backend);
  bool has_backend() const;

  tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo> list_scripts(const std::string & entity_id);
  tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & entity_id,
                                                              const std::string & script_id);
  tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                const std::optional<nlohmann::json> & metadata);
  tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & entity_id,
                                                           const std::string & script_id);
  tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  start_execution(const std::string & entity_id, const std::string & script_id, const ExecutionRequest & request);
  tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
  get_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id);
  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & entity_id,
                                                                        const std::string & script_id,
                                                                        const std::string & execution_id,
                                                                        const std::string & action);
  tl::expected<void, ScriptBackendErrorInfo>
  delete_execution(const std::string & entity_id, const std::string & script_id, const std::string & execution_id);

 private:
  ScriptProvider * backend_ = nullptr;
};

}  // namespace ros2_medkit_gateway
