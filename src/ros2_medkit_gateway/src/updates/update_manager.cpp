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
  // Signal background tasks to stop accepting new work
  stopped_ = true;

  // Collect all valid futures, then wait OUTSIDE the lock to avoid
  // deadlock (async tasks also acquire mutex_ during execution).
  std::vector<std::future<void>> futures;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [id, state] : states_) {
      if (state && state->active_task.valid()) {
        futures.push_back(std::move(state->active_task));
      }
    }
  }
  for (auto & f : futures) {
    f.wait();
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

tl::expected<std::vector<std::string>, UpdateError> UpdateManager::list_updates(const UpdateFilter & filter) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  auto result = backend_->list_updates(filter);
  if (!result) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, result.error().message});
  }
  return *result;
}

tl::expected<nlohmann::json, UpdateError> UpdateManager::get_update(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  auto result = backend_->get_update(id);
  if (!result) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, result.error().message});
  }
  return *result;
}

tl::expected<void, UpdateError> UpdateManager::register_update(const nlohmann::json & metadata) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  auto result = backend_->register_update(metadata);
  if (!result) {
    const auto & err = result.error();
    switch (err.code) {
      case UpdateBackendError::AlreadyExists:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::AlreadyExists, err.message});
      case UpdateBackendError::InvalidInput:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::InvalidRequest, err.message});
      default:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, err.message});
    }
  }
  return {};
}

tl::expected<void, UpdateError> UpdateManager::delete_update(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }

  bool had_state = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end()) {
      had_state = true;
      if (is_task_active(id)) {
        return tl::make_unexpected(
            UpdateError{UpdateErrorCode::InProgress, "Cannot delete update while operation is in progress"});
      }
      // Wait for completed task before proceeding (clean up future)
      if (it->second->active_task.valid()) {
        it->second->active_task.wait();
      }
      // Mark as deleting so no new operations can start on this package
      it->second->phase = UpdatePhase::Deleting;
    } else {
      // Create sentinel to prevent concurrent start_prepare
      states_[id] = std::make_unique<PackageState>();
      states_[id]->phase = UpdatePhase::Deleting;
    }
  }

  auto result = backend_->delete_update(id);
  if (result) {
    std::lock_guard<std::mutex> lock(mutex_);
    states_.erase(id);
  } else {
    // Rollback sentinel on failure
    std::lock_guard<std::mutex> lock(mutex_);
    if (had_state) {
      auto it = states_.find(id);
      if (it != states_.end() && it->second) {
        it->second->phase = UpdatePhase::Failed;
      }
    } else {
      // Remove the sentinel we created - package never had state before
      states_.erase(id);
    }
    const auto & err = result.error();
    switch (err.code) {
      case UpdateBackendError::NotFound:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, err.message});
      default:
        return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, err.message});
    }
  }
  return {};
}

tl::expected<void, UpdateError> UpdateManager::start_prepare(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  if (stopped_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, "UpdateManager is shutting down"});
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Verify package exists while holding lock to prevent concurrent deletion
  auto pkg = backend_->get_update(id);
  if (!pkg) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, pkg.error().message});
  }

  if (is_task_active(id)) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::InProgress, "An operation is already in progress for this package"});
  }

  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
  }

  if (state_ptr->phase == UpdatePhase::Deleting) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Deleting, "Package is being deleted"});
  }

  state_ptr->phase = UpdatePhase::Preparing;
  state_ptr->status = UpdateStatusInfo{UpdateStatus::Pending, std::nullopt, std::nullopt, std::nullopt};
  state_ptr->active_task = std::async(std::launch::async, &UpdateManager::run_prepare, this, id);
  return {};
}

tl::expected<void, UpdateError> UpdateManager::start_execute(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  if (stopped_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, "UpdateManager is shutting down"});
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto pkg = backend_->get_update(id);
  if (!pkg) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, pkg.error().message});
  }

  auto it = states_.find(id);
  if (it == states_.end() || !it->second || it->second->phase != UpdatePhase::Prepared) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotPrepared, "Package must be prepared before execution"});
  }
  if (is_task_active(id)) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::InProgress, "An operation is already in progress for this package"});
  }

  auto & state = *it->second;
  state.phase = UpdatePhase::Executing;
  state.status = UpdateStatusInfo{UpdateStatus::Pending, std::nullopt, std::nullopt, std::nullopt};
  state.active_task = std::async(std::launch::async, &UpdateManager::run_execute, this, id);
  return {};
}

tl::expected<void, UpdateError> UpdateManager::start_automated(const std::string & id) {
  if (!backend_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NoBackend, "No update backend loaded"});
  }
  if (stopped_) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Internal, "UpdateManager is shutting down"});
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto supported = backend_->supports_automated(id);
  if (!supported) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, supported.error().message});
  }
  if (!*supported) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::NotAutomated, "Package does not support automated updates"});
  }

  if (is_task_active(id)) {
    return tl::make_unexpected(
        UpdateError{UpdateErrorCode::InProgress, "An operation is already in progress for this package"});
  }

  auto & state_ptr = states_[id];
  if (!state_ptr) {
    state_ptr = std::make_unique<PackageState>();
  }

  if (state_ptr->phase == UpdatePhase::Deleting) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::Deleting, "Package is being deleted"});
  }

  state_ptr->phase = UpdatePhase::Preparing;
  state_ptr->status = UpdateStatusInfo{UpdateStatus::Pending, std::nullopt, std::nullopt, std::nullopt};
  state_ptr->active_task = std::async(std::launch::async, &UpdateManager::run_automated, this, id);
  return {};
}

tl::expected<UpdateStatusInfo, UpdateError> UpdateManager::get_status(const std::string & id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = states_.find(id);
  if (it == states_.end() || !it->second) {
    return tl::make_unexpected(UpdateError{UpdateErrorCode::NotFound, "No status available for package '" + id + "'"});
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
  try {
    PackageState * state = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = states_[id].get();
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
      state->status.error_message = result.error().message;
      state->phase = UpdatePhase::Failed;
    }
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && it->second) {
      it->second->status.status = UpdateStatus::Failed;
      it->second->status.error_message = std::string("Exception: ") + e.what();
      it->second->phase = UpdatePhase::Failed;
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && it->second) {
      it->second->status.status = UpdateStatus::Failed;
      it->second->status.error_message = "Unknown exception during prepare";
      it->second->phase = UpdatePhase::Failed;
    }
  }
}

void UpdateManager::run_execute(const std::string & id) {
  try {
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
      state->status.error_message = result.error().message;
      state->phase = UpdatePhase::Failed;
    }
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && it->second) {
      it->second->status.status = UpdateStatus::Failed;
      it->second->status.error_message = std::string("Exception: ") + e.what();
      it->second->phase = UpdatePhase::Failed;
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && it->second) {
      it->second->status.status = UpdateStatus::Failed;
      it->second->status.error_message = "Unknown exception during execute";
      it->second->phase = UpdatePhase::Failed;
    }
  }
}

void UpdateManager::run_automated(const std::string & id) {
  try {
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
      state->status.error_message = prep_result.error().message;
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
      state->status.error_message = exec_result.error().message;
      state->phase = UpdatePhase::Failed;
    }
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && it->second) {
      it->second->status.status = UpdateStatus::Failed;
      it->second->status.error_message = std::string("Exception: ") + e.what();
      it->second->phase = UpdatePhase::Failed;
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = states_.find(id);
    if (it != states_.end() && it->second) {
      it->second->status.status = UpdateStatus::Failed;
      it->second->status.error_message = "Unknown exception during automated update";
      it->second->phase = UpdatePhase::Failed;
    }
  }
}

}  // namespace ros2_medkit_gateway
