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

#pragma once

#include "ros2_medkit_gateway/updates/update_types.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Provider interface for software update backends
 *
 * Typed provider interface that plugins implement alongside GatewayPlugin
 * via multiple inheritance. Replaces the old UpdateBackend abstract class.
 *
 * Reuses all types from update_types.hpp (UpdateFilter, UpdateBackendErrorInfo,
 * UpdateProgressReporter, etc.).
 *
 * @see GatewayPlugin for the base class
 * @see UpdateManager for the subsystem that uses this
 */
class UpdateProvider {
 public:
  virtual ~UpdateProvider() = default;

  // ---- CRUD (metadata storage owned by plugin) ----
  virtual tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter & filter) = 0;
  virtual tl::expected<nlohmann::json, UpdateBackendErrorInfo> get_update(const std::string & id) = 0;
  virtual tl::expected<void, UpdateBackendErrorInfo> register_update(const nlohmann::json & metadata) = 0;
  virtual tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & id) = 0;

  // ---- Async operations (called in background threads) ----
  virtual tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & id,
                                                             UpdateProgressReporter & reporter) = 0;
  virtual tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & id,
                                                             UpdateProgressReporter & reporter) = 0;

  // ---- Capability queries ----
  virtual tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & id) = 0;
};

}  // namespace ros2_medkit_gateway
