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

#include <nlohmann/json.hpp>
#include <string>
#include <tl/expected.hpp>
#include <type_traits>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

/// A single request-body validation failure.
struct FieldError {
  std::string field;
  std::string message;
};

template <class T>
struct JsonReader;  // forward declaration

/// Decode one JSON value into `out`. Appends a FieldError on failure.
template <class U>
void decode_value(const nlohmann::json & j, U & out, const std::string & path, std::vector<FieldError> & errs) {
  if constexpr (is_dto_v<U>) {
    auto nested = JsonReader<U>::read(j);
    if (nested.has_value()) {
      out = std::move(nested.value());
    } else {
      for (auto & e : nested.error()) {
        errs.push_back({path + "." + e.field, e.message});
      }
    }
  } else if constexpr (is_vector_v<U>) {
    if (!j.is_array()) {
      errs.push_back({path, "expected an array"});
      return;
    }
    out.clear();
    std::size_t idx = 0;
    for (const auto & elem : j) {
      typename U::value_type item{};
      decode_value(elem, item, path + "[" + std::to_string(idx++) + "]", errs);
      out.push_back(std::move(item));
    }
  } else if constexpr (std::is_same_v<U, nlohmann::json>) {
    out = j;
  } else if constexpr (std::is_same_v<U, std::string>) {
    if (j.is_string()) {
      out = j.get<std::string>();
    } else {
      errs.push_back({path, "expected a string"});
    }
  } else if constexpr (std::is_same_v<U, bool>) {
    if (j.is_boolean()) {
      out = j.get<bool>();
    } else {
      errs.push_back({path, "expected a boolean"});
    }
  } else if constexpr (std::is_integral_v<U>) {
    if (j.is_number_integer() || j.is_number_unsigned()) {
      out = j.get<U>();
    } else {
      errs.push_back({path, "expected an integer"});
    }
  } else if constexpr (std::is_floating_point_v<U>) {
    if (j.is_number()) {
      out = j.get<U>();
    } else {
      errs.push_back({path, "expected a number"});
    }
  } else {
    static_assert(sizeof(U) == 0, "decode_value: unsupported field type");
  }
}

/// Check an already-decoded string member against the field's enum vocabulary.
template <class FieldT>
void check_enum(const FieldT & f, const std::string & value, const std::string & key, std::vector<FieldError> & errs) {
  if (f.enum_count == 0) {
    return;
  }
  for (std::size_t i = 0; i < f.enum_count; ++i) {
    if (value == f.enum_values[i]) {
      return;
    }
  }
  errs.push_back({key, "value not in allowed set"});
}

/// Parses + validates a JSON object into a DTO. Collects every error.
template <class T>
struct JsonReader {
  static tl::expected<T, std::vector<FieldError>> read(const nlohmann::json & j) {
    std::vector<FieldError> errs;
    T obj{};
    if (!j.is_object()) {
      errs.push_back({"", "expected a JSON object"});
      return tl::make_unexpected(errs);
    }
    for_each_field<T>([&](const auto & f) {
      using FieldT = std::decay_t<decltype(f)>;
      const std::string key(f.key);
      if constexpr (is_opaque_object_field_v<FieldT>) {
        // Opaque any-object: required field, must be a JSON object.
        // Absent / null leaves the member at its default (empty object).
        const auto it = j.find(key);
        if (it == j.end() || it->is_null()) {
          return;
        }
        if (!it->is_object()) {
          errs.push_back({key, "expected object"});
          return;
        }
        obj.*(f.ptr) = *it;
        return;
      } else {
        auto & member = obj.*(f.ptr);
        using MemberT = std::decay_t<decltype(member)>;
        const auto it = j.find(key);
        // A present-but-null JSON value is a legitimate value for free-form json
        // members (e.g. PUT .../data/{id} or .../configurations/{id} with
        // {"data": null} to set a parameter to null). For every other member type
        // null cannot be coerced, so it is treated like an absent field.
        const bool present = (it != j.end());
        bool treat_as_absent = !present;
        if (present && it->is_null()) {
          if constexpr (is_optional_v<MemberT>) {
            treat_as_absent = !std::is_same_v<typename MemberT::value_type, nlohmann::json>;
          } else {
            treat_as_absent = !std::is_same_v<MemberT, nlohmann::json>;
          }
        }
        if (treat_as_absent) {
          if (f.presence == Presence::kRequired) {
            errs.push_back({key, "missing required field"});
          }
          return;  // unknown/extra fields are never looked for (lenient)
        }
        if constexpr (is_optional_v<MemberT>) {
          typename MemberT::value_type val{};
          decode_value(*it, val, key, errs);
          if constexpr (std::is_same_v<typename MemberT::value_type, std::string>) {
            check_enum(f, val, key, errs);
          }
          member = std::move(val);
        } else {
          decode_value(*it, member, key, errs);
          if constexpr (std::is_same_v<MemberT, std::string>) {
            check_enum(f, member, key, errs);
          }
        }
      }
    });
    if (!errs.empty()) {
      return tl::make_unexpected(errs);
    }
    return obj;
  }
};

}  // namespace dto
}  // namespace ros2_medkit_gateway
