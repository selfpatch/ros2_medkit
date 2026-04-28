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

#include "ros2_medkit_gateway/core/resource_sampler.hpp"

#include <mutex>
#include <stdexcept>

namespace ros2_medkit_gateway {

void ResourceSamplerRegistry::register_sampler(const std::string & collection, ResourceSamplerFn fn, bool is_builtin) {
  std::unique_lock lock(mutex_);

  if (!is_builtin) {
    if (collection.size() < 2 || collection.substr(0, 2) != "x-") {
      throw std::runtime_error("Plugin sampler collection must use 'x-' prefix: " + collection);
    }
    if (samplers_.count(collection) > 0) {
      throw std::runtime_error("Sampler already registered for collection: " + collection);
    }
  }

  samplers_[collection] = std::move(fn);
}

std::optional<ResourceSamplerFn> ResourceSamplerRegistry::get_sampler(const std::string & collection) const {
  std::shared_lock lock(mutex_);
  auto it = samplers_.find(collection);
  if (it == samplers_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool ResourceSamplerRegistry::has_sampler(const std::string & collection) const {
  std::shared_lock lock(mutex_);
  return samplers_.count(collection) > 0;
}

}  // namespace ros2_medkit_gateway
