// Copyright 2026 Selfpatch
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

#include "ros2_medkit_serialization/type_cache.hpp"

#include <regex>
#include <shared_mutex>

namespace ros2_medkit_serialization {

TypeCache & TypeCache::instance() {
  static TypeCache instance;
  return instance;
}

const TypeInfo_Cpp * TypeCache::get_message_type_info(const std::string & package_name, const std::string & type_name) {
  // This overload assumes "msg" interface type for backward compatibility
  return get_message_type_info(package_name, "msg", type_name);
}

const TypeInfo_Cpp * TypeCache::get_message_type_info(const std::string & package_name,
                                                      const std::string & interface_type,
                                                      const std::string & type_name) {
  const std::string key = make_key(package_name, interface_type, type_name);

  // Try read lock first (fast path for cache hits)
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      return it->second;
    }
  }

  // Load the type info using dynmsg with full interface type
  FullInterfaceTypeName full_interface_type{package_name, interface_type, type_name};
  const TypeInfo_Cpp * type_info = dynmsg::cpp::get_type_info(full_interface_type);

  if (type_info != nullptr) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    cache_[key] = type_info;
  }

  return type_info;
}

const TypeInfo_Cpp * TypeCache::get_message_type_info(const std::string & full_type) {
  auto parsed = parse_type_string(full_type);
  if (!parsed.has_value()) {
    return nullptr;
  }
  // Now we have (package, interface_type, type_name)
  return get_message_type_info(std::get<0>(*parsed), std::get<1>(*parsed), std::get<2>(*parsed));
}

std::optional<std::tuple<std::string, std::string, std::string>>
TypeCache::parse_type_string(const std::string & full_type) {
  // Pattern: package/msg/TypeName or package/srv/TypeName or package/action/TypeName
  static const std::regex type_regex(R"(^([a-zA-Z_][a-zA-Z0-9_]*)/(msg|srv|action)/(\w+)$)");

  std::smatch match;
  if (!std::regex_match(full_type, match, type_regex)) {
    return std::nullopt;
  }

  // Return (package, interface_type, type_name)
  std::string package = match[1].str();
  std::string interface_type = match[2].str();
  std::string type_name = match[3].str();

  return std::make_tuple(package, interface_type, type_name);
}

bool TypeCache::is_cached(const std::string & package_name, const std::string & type_name) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return cache_.find(make_key(package_name, type_name)) != cache_.end();
}

void TypeCache::clear() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  cache_.clear();
}

size_t TypeCache::size() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return cache_.size();
}

std::string TypeCache::make_key(const std::string & package_name, const std::string & type_name) {
  return package_name + "/msg/" + type_name;
}

std::string TypeCache::make_key(const std::string & package_name, const std::string & interface_type,
                                const std::string & type_name) {
  return package_name + "/" + interface_type + "/" + type_name;
}

}  // namespace ros2_medkit_serialization
