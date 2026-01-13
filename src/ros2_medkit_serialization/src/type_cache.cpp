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

namespace ros2_medkit_serialization {

TypeCache & TypeCache::instance() {
  static TypeCache instance;
  return instance;
}

const TypeInfo_Cpp * TypeCache::get_message_type_info(const std::string & package_name, const std::string & type_name) {
  const std::string key = make_key(package_name, type_name);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      return it->second;
    }
  }

  // Load the type info using dynmsg
  InterfaceTypeName interface_type{package_name, type_name};
  const TypeInfo_Cpp * type_info = dynmsg::cpp::get_type_info(interface_type);

  if (type_info != nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_[key] = type_info;
  }

  return type_info;
}

const TypeInfo_Cpp * TypeCache::get_message_type_info(const std::string & full_type) {
  auto parsed = parse_type_string(full_type);
  if (!parsed.has_value()) {
    return nullptr;
  }
  return get_message_type_info(parsed->first, parsed->second);
}

std::optional<std::pair<std::string, std::string>> TypeCache::parse_type_string(const std::string & full_type) {
  // Pattern: package/msg/TypeName or package/srv/TypeName or package/action/TypeName
  static const std::regex type_regex(R"(^([a-zA-Z_][a-zA-Z0-9_]*)/(msg|srv|action)/(\w+)$)");

  std::smatch match;
  if (!std::regex_match(full_type, match, type_regex)) {
    return std::nullopt;
  }

  // Return package and type name (include interface type in the name for srv/action)
  std::string package = match[1].str();
  std::string interface_type = match[2].str();
  std::string type_name = match[3].str();

  // For messages, we just need the type name
  // For services/actions, the type is already specialized (e.g., SetBool_Request)
  return std::make_pair(package, type_name);
}

bool TypeCache::is_cached(const std::string & package_name, const std::string & type_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.find(make_key(package_name, type_name)) != cache_.end();
}

void TypeCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  cache_.clear();
}

size_t TypeCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

std::string TypeCache::make_key(const std::string & package_name, const std::string & type_name) {
  return package_name + "/" + type_name;
}

}  // namespace ros2_medkit_serialization
