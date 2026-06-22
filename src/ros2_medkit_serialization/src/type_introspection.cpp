// Copyright 2025 mfaferek93
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

#include "ros2_medkit_serialization/type_introspection.hpp"

#include <sstream>
#include <stdexcept>

#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_serialization {

TypeIntrospection::TypeIntrospection(const std::string & /* scripts_path */)
  : serializer_(std::make_shared<JsonSerializer>()) {
  // scripts_path is deprecated and ignored - we use native serialization now
}

nlohmann::json TypeIntrospection::get_type_template(const std::string & type_name) {
  try {
    return serializer_->get_defaults(type_name);
  } catch (const TypeNotFoundError & e) {
    throw std::runtime_error("Unknown type: " + type_name + " (" + e.what() + ")");
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to get type template: " + std::string(e.what()));
  }
}

nlohmann::json TypeIntrospection::get_type_schema(const std::string & type_name) {
  try {
    return serializer_->get_schema(type_name);
  } catch (const TypeNotFoundError & e) {
    throw std::runtime_error("Unknown type: " + type_name + " (" + e.what() + ")");
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to get type schema: " + std::string(e.what()));
  }
}

TopicTypeInfo TypeIntrospection::get_type_info(const std::string & type_name) {
  // Check cache first
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = type_cache_.find(type_name);
    if (it != type_cache_.end()) {
      return it->second;
    }
  }

  // Build type info using native serializer
  TopicTypeInfo info;
  info.name = type_name;

  // Get template (default values)
  try {
    info.default_value = get_type_template(type_name);
  } catch (const std::exception & e) {
    // If template fails, use empty object
    info.default_value = nlohmann::json::object();
  }

  // Get schema (type structure)
  bool schema_resolved = true;
  try {
    info.schema = get_type_schema(type_name);
  } catch (const std::exception & e) {
    // If schema fails, use empty object and do NOT memoize: resolution may be
    // transient (e.g. the type support library is not yet loadable), and caching
    // the empty result would pin it empty for the process lifetime.
    info.schema = nlohmann::json::object();
    schema_resolved = false;
  }

  if (!schema_resolved) {
    return info;  // return empty result without poisoning the cache; retry later
  }

  // Cache it (use try_emplace to avoid overwriting if another thread added it first)
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return type_cache_.try_emplace(type_name, info).first->second;  // may be another thread's
  }
}

std::shared_ptr<const nlohmann::json> TypeIntrospection::get_service_type_info(const std::string & service_type) {
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = service_type_info_cache_.find(service_type);
    if (it != service_type_info_cache_.end()) {
      return it->second;
    }
  }
  // get_type_info is itself cached and returns empty schemas for unknown types
  // (it does not throw); build the wrapper once.
  auto assembled = std::make_shared<nlohmann::json>();
  (*assembled)["request"] = get_type_info(service_type + "_Request").schema;
  (*assembled)["response"] = get_type_info(service_type + "_Response").schema;
  // Memoize only a resolved wrapper. If neither side resolved, the type support
  // was not available; return without caching so a later request can retry
  // instead of pinning an empty wrapper for the process lifetime. (A genuinely
  // empty service still has a non-empty schema object on at least one side.)
  const bool resolved = !(*assembled)["request"].empty() || !(*assembled)["response"].empty();
  std::shared_ptr<const nlohmann::json> shared = std::move(assembled);
  if (!resolved) {
    return shared;
  }
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    // try_emplace keeps the first cached instance if another thread raced us.
    return service_type_info_cache_.try_emplace(service_type, shared).first->second;
  }
}

std::shared_ptr<const nlohmann::json> TypeIntrospection::get_action_type_info(const std::string & action_type) {
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = action_type_info_cache_.find(action_type);
    if (it != action_type_info_cache_.end()) {
      return it->second;
    }
  }
  auto assembled = std::make_shared<nlohmann::json>();
  (*assembled)["goal"] = get_type_info(action_type + "_Goal").schema;
  (*assembled)["result"] = get_type_info(action_type + "_Result").schema;
  (*assembled)["feedback"] = get_type_info(action_type + "_Feedback").schema;
  // Memoize only a resolved wrapper (see get_service_type_info); if none of the
  // three sides resolved, return without caching so a later request can retry.
  const bool resolved =
      !(*assembled)["goal"].empty() || !(*assembled)["result"].empty() || !(*assembled)["feedback"].empty();
  std::shared_ptr<const nlohmann::json> shared = std::move(assembled);
  if (!resolved) {
    return shared;
  }
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    // try_emplace keeps the first cached instance if another thread raced us.
    return action_type_info_cache_.try_emplace(action_type, shared).first->second;
  }
}

}  // namespace ros2_medkit_serialization
