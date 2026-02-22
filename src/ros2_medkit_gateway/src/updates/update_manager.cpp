// Copyright 2026 bburda
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

#include "ros2_medkit_gateway/updates/update_manager.hpp"

#include <dlfcn.h>

namespace ros2_medkit_gateway {

UpdateManager::UpdateManager(std::unique_ptr<UpdateBackend> backend, void * plugin_handle)
  : backend_(std::move(backend)), plugin_handle_(plugin_handle) {
}

UpdateManager::~UpdateManager() {
  // Wait for all active tasks to finish
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [id, state] : states_) {
      if (state->active_task.valid()) {
        state->active_task.wait();
      }
    }
  }
  // Destroy backend before closing plugin handle
  backend_.reset();
  if (plugin_handle_) {
    dlclose(plugin_handle_);
  }
}

bool UpdateManager::has_backend() const {
  return backend_ != nullptr;
}

tl::expected<std::vector<std::string>, std::string> UpdateManager::list_updates(const UpdateFilter & filter) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }
  return backend_->list_updates(filter);
}

tl::expected<nlohmann::json, std::string> UpdateManager::get_update(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }
  return backend_->get_update(id);
}

tl::expected<void, std::string> UpdateManager::register_update(const nlohmann::json & metadata) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }
  return backend_->register_update(metadata);
}

tl::expected<void, std::string> UpdateManager::delete_update(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }

  // Check if an operation is in progress
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && is_task_active(id)) {
      return tl::make_unexpected("Cannot delete update while operation is in progress");
    }
    // Wait for completed task before erasing (clean up future)
    if (it != states_.end() && it->second->active_task.valid()) {
      it->second->active_task.wait();
    }
  }

  auto result = backend_->delete_update(id);
  if (result) {
    std::lock_guard<std::mutex> lock(mutex_);
    states_.erase(id);
  }
  return result;
}

tl::expected<void, std::string> UpdateManager::start_prepare(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }

  // Verify package exists
  auto pkg = backend_->get_update(id);
  if (!pkg) {
    return tl::make_unexpected(pkg.error());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_task_active(id)) {
    return tl::make_unexpected("An operation is already in progress for this package");
  }

  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
  }
  state_ptr->phase = UpdatePhase::Preparing;
  state_ptr->status = UpdateStatusInfo{UpdateStatus::Pending, std::nullopt, std::nullopt, std::nullopt};
  state_ptr->active_task = std::async(std::launch::async, &UpdateManager::run_prepare, this, id);
  return {};
}

tl::expected<void, std::string> UpdateManager::start_execute(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }

  // Verify package exists
  auto pkg = backend_->get_update(id);
  if (!pkg) {
    return tl::make_unexpected(pkg.error());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(id);
  if (it == states_.end() || !it->second || it->second->phase != UpdatePhase::Prepared) {
    return tl::make_unexpected("Package must be prepared before execution");
  }
  if (is_task_active(id)) {
    return tl::make_unexpected("An operation is already in progress for this package");
  }

  auto & state = *it->second;
  state.phase = UpdatePhase::Executing;
  state.status = UpdateStatusInfo{UpdateStatus::Pending, std::nullopt, std::nullopt, std::nullopt};
  state.active_task = std::async(std::launch::async, &UpdateManager::run_execute, this, id);
  return {};
}

tl::expected<void, std::string> UpdateManager::start_automated(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected("No update backend loaded");
  }

  // Verify package exists and supports automated
  auto supported = backend_->supports_automated(id);
  if (!supported) {
    return tl::make_unexpected(supported.error());
  }
  if (!*supported) {
    return tl::make_unexpected("Package does not support automated updates");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (is_task_active(id)) {
    return tl::make_unexpected("An operation is already in progress for this package");
  }

  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
  }
  state_ptr->phase = UpdatePhase::Preparing;
  state_ptr->status = UpdateStatusInfo{UpdateStatus::Pending, std::nullopt, std::nullopt, std::nullopt};
  state_ptr->active_task = std::async(std::launch::async, &UpdateManager::run_automated, this, id);
  return {};
}

tl::expected<UpdateStatusInfo, std::string> UpdateManager::get_status(const std::string & id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(id);
  if (it == states_.end() || !it->second) {
    return tl::make_unexpected("No status available for package '" + id + "'");
  }
  return it->second->status;
}

bool UpdateManager::is_task_active(const std::string & id) const {
  auto it = states_.find(id);
  if (it == states_.end() || !it->second) {
    return false;
  }
  if (!it->second->active_task.valid()) {
    return false;
  }
  return it->second->active_task.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
}

void UpdateManager::run_prepare(const std::string & id) {
  PackageState * state = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = states_[id].get();  // stable pointer (unique_ptr)
    state->status.status = UpdateStatus::InProgress;
  }

  UpdateProgressReporter reporter(state->status, mutex_);
  auto result = backend_->prepare(id, reporter);

  std::lock_guard<std::mutex> lock(mutex_);
  if (result) {
    state->status.status = UpdateStatus::Completed;
    state->phase = UpdatePhase::Prepared;
  } else {
    state->status.status = UpdateStatus::Failed;
    state->status.error_message = result.error();
    state->phase = UpdatePhase::Failed;
  }
}

void UpdateManager::run_execute(const std::string & id) {
  PackageState * state = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = states_[id].get();
    state->status.status = UpdateStatus::InProgress;
  }

  UpdateProgressReporter reporter(state->status, mutex_);
  auto result = backend_->execute(id, reporter);

  std::lock_guard<std::mutex> lock(mutex_);
  if (result) {
    state->status.status = UpdateStatus::Completed;
    state->phase = UpdatePhase::Executed;
  } else {
    state->status.status = UpdateStatus::Failed;
    state->status.error_message = result.error();
    state->phase = UpdatePhase::Failed;
  }
}

void UpdateManager::run_automated(const std::string & id) {
  PackageState * state = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = states_[id].get();
    state->status.status = UpdateStatus::InProgress;
  }

  // Phase 1: Prepare
  UpdateProgressReporter reporter(state->status, mutex_);
  auto prep_result = backend_->prepare(id, reporter);

  if (!prep_result) {
    std::lock_guard<std::mutex> lock(mutex_);
    state->status.status = UpdateStatus::Failed;
    state->status.error_message = prep_result.error();
    state->phase = UpdatePhase::Failed;
    return;
  }

  // Phase 2: Execute (reset progress for execute phase)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state->phase = UpdatePhase::Executing;
    state->status.progress = std::nullopt;
    state->status.sub_progress = std::nullopt;
  }

  auto exec_result = backend_->execute(id, reporter);

  std::lock_guard<std::mutex> lock(mutex_);
  if (exec_result) {
    state->status.status = UpdateStatus::Completed;
    state->phase = UpdatePhase::Executed;
  } else {
    state->status.status = UpdateStatus::Failed;
    state->status.error_message = exec_result.error();
    state->phase = UpdatePhase::Failed;
  }
}

}  // namespace ros2_medkit_gateway
