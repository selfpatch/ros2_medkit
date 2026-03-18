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

#include "ros2_medkit_gateway/script_manager.hpp"

namespace ros2_medkit_gateway {

void ScriptManager::set_backend(ScriptProvider * backend) {
  backend_ = backend;
}

bool ScriptManager::has_backend() const {
  return backend_ != nullptr;
}

tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
ScriptManager::list_scripts(const std::string & entity_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->list_scripts(entity_id);
}

tl::expected<ScriptInfo, ScriptBackendErrorInfo> ScriptManager::get_script(const std::string & entity_id,
                                                                           const std::string & script_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->get_script(entity_id, script_id);
}

tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
ScriptManager::upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                             const std::optional<nlohmann::json> & metadata) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->upload_script(entity_id, filename, content, metadata);
}

tl::expected<void, ScriptBackendErrorInfo> ScriptManager::delete_script(const std::string & entity_id,
                                                                        const std::string & script_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->delete_script(entity_id, script_id);
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo> ScriptManager::start_execution(const std::string & entity_id,
                                                                                   const std::string & script_id,
                                                                                   const ExecutionRequest & request) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->start_execution(entity_id, script_id, request);
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo> ScriptManager::get_execution(const std::string & entity_id,
                                                                                 const std::string & script_id,
                                                                                 const std::string & execution_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->get_execution(entity_id, script_id, execution_id);
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo> ScriptManager::control_execution(const std::string & entity_id,
                                                                                     const std::string & script_id,
                                                                                     const std::string & execution_id,
                                                                                     const std::string & action) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->control_execution(entity_id, script_id, execution_id, action);
}

tl::expected<void, ScriptBackendErrorInfo> ScriptManager::delete_execution(const std::string & entity_id,
                                                                           const std::string & script_id,
                                                                           const std::string & execution_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  return backend_->delete_execution(entity_id, script_id, execution_id);
}

}  // namespace ros2_medkit_gateway
