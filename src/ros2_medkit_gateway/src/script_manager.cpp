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

#include "ros2_medkit_gateway/core/managers/script_manager.hpp"

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

namespace {
auto logger() {
  return rclcpp::get_logger("script_manager");
}
}  // namespace

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
  try {
    return backend_->list_scripts(entity_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "list_scripts failed for entity '%s': %s", entity_id.c_str(), e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "list_scripts failed for entity '%s': unknown exception", entity_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<ScriptInfo, ScriptBackendErrorInfo> ScriptManager::get_script(const std::string & entity_id,
                                                                           const std::string & script_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->get_script(entity_id, script_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "get_script failed for entity '%s', script '%s': %s", entity_id.c_str(), script_id.c_str(),
                 e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "get_script failed for entity '%s', script '%s': unknown exception", entity_id.c_str(),
                 script_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
ScriptManager::upload_script(const std::string & entity_id, const std::string & filename, const std::string & content,
                             const std::optional<nlohmann::json> & metadata) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->upload_script(entity_id, filename, content, metadata);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "upload_script failed for entity '%s', file '%s': %s", entity_id.c_str(), filename.c_str(),
                 e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "upload_script failed for entity '%s', file '%s': unknown exception", entity_id.c_str(),
                 filename.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<void, ScriptBackendErrorInfo> ScriptManager::delete_script(const std::string & entity_id,
                                                                        const std::string & script_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->delete_script(entity_id, script_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "delete_script failed for entity '%s', script '%s': %s", entity_id.c_str(),
                 script_id.c_str(), e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "delete_script failed for entity '%s', script '%s': unknown exception", entity_id.c_str(),
                 script_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo> ScriptManager::start_execution(const std::string & entity_id,
                                                                                   const std::string & script_id,
                                                                                   const ExecutionRequest & request) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->start_execution(entity_id, script_id, request);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "start_execution failed for entity '%s', script '%s': %s", entity_id.c_str(),
                 script_id.c_str(), e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "start_execution failed for entity '%s', script '%s': unknown exception", entity_id.c_str(),
                 script_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo> ScriptManager::get_execution(const std::string & entity_id,
                                                                                 const std::string & script_id,
                                                                                 const std::string & execution_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->get_execution(entity_id, script_id, execution_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "get_execution failed for entity '%s', script '%s', execution '%s': %s", entity_id.c_str(),
                 script_id.c_str(), execution_id.c_str(), e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "get_execution failed for entity '%s', script '%s', execution '%s': unknown exception",
                 entity_id.c_str(), script_id.c_str(), execution_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo> ScriptManager::control_execution(const std::string & entity_id,
                                                                                     const std::string & script_id,
                                                                                     const std::string & execution_id,
                                                                                     const std::string & action) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->control_execution(entity_id, script_id, execution_id, action);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "control_execution('%s') failed for entity '%s', script '%s', execution '%s': %s",
                 action.c_str(), entity_id.c_str(), script_id.c_str(), execution_id.c_str(), e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(),
                 "control_execution('%s') failed for entity '%s', script '%s', execution '%s': unknown exception",
                 action.c_str(), entity_id.c_str(), script_id.c_str(), execution_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

tl::expected<void, ScriptBackendErrorInfo> ScriptManager::delete_execution(const std::string & entity_id,
                                                                           const std::string & script_id,
                                                                           const std::string & execution_id) {
  if (!backend_) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "No script backend configured"});
  }
  try {
    return backend_->delete_execution(entity_id, script_id, execution_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "delete_execution failed for entity '%s', script '%s', execution '%s': %s",
                 entity_id.c_str(), script_id.c_str(), execution_id.c_str(), e.what());
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, std::string("Backend error: ") + e.what()});
  } catch (...) {
    RCLCPP_ERROR(logger(), "delete_execution failed for entity '%s', script '%s', execution '%s': unknown exception",
                 entity_id.c_str(), script_id.c_str(), execution_id.c_str());
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Unknown backend error"});
  }
}

}  // namespace ros2_medkit_gateway
