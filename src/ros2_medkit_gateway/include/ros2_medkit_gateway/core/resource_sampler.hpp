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

#include <functional>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Callback: (entity_id, resource_path) -> JSON payload or error string.
using ResourceSamplerFn = std::function<tl::expected<nlohmann::json, std::string>(const std::string & entity_id,
                                                                                  const std::string & resource_path)>;

/// Thread-safe registry mapping collection names to sampling functions.
class ResourceSamplerRegistry {
 public:
  void register_sampler(const std::string & collection, ResourceSamplerFn fn, bool is_builtin = false);
  std::optional<ResourceSamplerFn> get_sampler(const std::string & collection) const;
  bool has_sampler(const std::string & collection) const;

 private:
  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, ResourceSamplerFn> samplers_;
};

}  // namespace ros2_medkit_gateway
