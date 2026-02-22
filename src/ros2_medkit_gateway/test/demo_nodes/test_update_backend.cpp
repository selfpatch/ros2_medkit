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

#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/updates/update_backend.hpp"

using json = nlohmann::json;
using namespace ros2_medkit_gateway;

/**
 * @brief In-memory update backend for integration testing.
 *
 * Stores package metadata in a map. Simulates prepare/execute with
 * short delays (100ms per step, 4 steps) and progress reporting.
 */
class TestUpdateBackend : public UpdateBackend {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter & filter) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> ids;
    for (const auto & [id, meta] : packages_) {
      if (filter.origin.has_value()) {
        auto origins = meta.value("origins", std::vector<std::string>{});
        bool found = false;
        for (const auto & o : origins) {
          if (o == *filter.origin) {
            found = true;
            break;
          }
        }
        if (!found) {
          continue;
        }
      }
      if (filter.target_version.has_value()) {
        auto tv = meta.value("target_version", std::string{});
        if (tv != *filter.target_version) {
          continue;
        }
      }
      ids.push_back(id);
    }
    return ids;
  }

  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::NotFound, "Update package '" + id + "' not found"});
    }
    return it->second;
  }

  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & metadata) override {
    if (!metadata.contains("id") || !metadata["id"].is_string()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "Missing required field: id"});
    }
    if (!metadata.contains("update_name") || !metadata["update_name"].is_string()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "Missing required field: update_name"});
    }
    if (!metadata.contains("automated") || !metadata["automated"].is_boolean()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "Missing required field: automated"});
    }
    if (!metadata.contains("origins") || !metadata["origins"].is_array()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "Missing required field: origins"});
    }

    std::string id = metadata["id"].get<std::string>();
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.count(id) > 0) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::AlreadyExists, "Update package '" + id + "' already exists"});
    }
    packages_[id] = metadata;
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::NotFound, "Update package '" + id + "' not found"});
    }
    packages_.erase(it);
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & id,
                                                     UpdateProgressReporter & reporter) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (packages_.find(id) == packages_.end()) {
        return tl::make_unexpected(
            UpdateBackendErrorInfo{UpdateBackendError::NotFound, "Update package '" + id + "' not found"});
      }
    }

    // Simulate preparation with 4 steps
    std::vector<std::string> steps = {"Downloading package", "Verifying checksum", "Checking dependencies",
                                      "Ready to install"};
    for (int i = 0; i < 4; ++i) {
      int pct = (i + 1) * 25;
      reporter.set_progress(pct);
      std::vector<UpdateSubProgress> sub;
      for (int j = 0; j <= i; ++j) {
        sub.push_back({steps[static_cast<size_t>(j)], 100});
      }
      if (i < 3) {
        sub.push_back({steps[static_cast<size_t>(i + 1)], 0});
      }
      reporter.set_sub_progress(std::move(sub));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & id,
                                                     UpdateProgressReporter & reporter) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (packages_.find(id) == packages_.end()) {
        return tl::make_unexpected(
            UpdateBackendErrorInfo{UpdateBackendError::NotFound, "Update package '" + id + "' not found"});
      }
    }

    // Simulate execution with 4 steps
    std::vector<std::string> steps = {"Stopping services", "Installing update", "Migrating data",
                                      "Restarting services"};
    for (int i = 0; i < 4; ++i) {
      int pct = (i + 1) * 25;
      reporter.set_progress(pct);
      std::vector<UpdateSubProgress> sub;
      for (int j = 0; j <= i; ++j) {
        sub.push_back({steps[static_cast<size_t>(j)], 100});
      }
      if (i < 3) {
        sub.push_back({steps[static_cast<size_t>(i + 1)], 0});
      }
      reporter.set_sub_progress(std::move(sub));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return {};
  }

  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(
          UpdateBackendErrorInfo{UpdateBackendError::NotFound, "Update package '" + id + "' not found"});
    }
    return it->second.value("automated", false);
  }

 private:
  std::mutex mutex_;
  std::unordered_map<std::string, json> packages_;
};

// Plugin factory function - exported for dlopen/dlsym loading
extern "C" UpdateBackend * create_update_backend() {
  return new TestUpdateBackend();
}
