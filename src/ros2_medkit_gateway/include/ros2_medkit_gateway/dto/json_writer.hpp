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
#include <type_traits>
#include <variant>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

template <class T>
struct JsonWriter;  // forward declaration for nested-DTO recursion

/// Encode a single field value to JSON.
template <class U>
nlohmann::json encode_value(const U & v) {
  if constexpr (is_dto_v<U>) {
    return JsonWriter<U>::write(v);
  } else if constexpr (is_vector_v<U>) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto & e : v) {
      arr.push_back(encode_value(e));
    }
    return arr;
  } else if constexpr (is_variant_v<U>) {
    return std::visit(
        [](const auto & alt) {
          return encode_value(alt);
        },
        v);
  } else if constexpr (std::is_same_v<U, std::string> || std::is_same_v<U, bool> || std::is_integral_v<U> ||
                       std::is_floating_point_v<U> || std::is_same_v<U, nlohmann::json>) {
    // string / bool / integral / floating / nlohmann::json passthrough
    return nlohmann::json(v);
  } else {
    // Mirror decode_value / schema_of: fail loud if a field type leaks here that
    // is neither a DTO, container, variant, nor a supported scalar (e.g. a DTO
    // sentinel used in a TU that never saw its dto_fields specialization).
    static_assert(sizeof(U) == 0, "encode_value: unsupported field type");
  }
}

/// Serializes a DTO instance to its wire JSON object.
template <class T>
struct JsonWriter {
  static nlohmann::json write(const T & obj) {
    nlohmann::json out = nlohmann::json::object();
    for_each_field<T>([&](const auto & f) {
      using FieldT = std::decay_t<decltype(f)>;
      if constexpr (is_opaque_object_field_v<FieldT>) {
        // Opaque any-object: pass the nlohmann::json member through as-is.
        out[std::string(f.key)] = obj.*(f.ptr);
      } else {
        const auto & val = obj.*(f.ptr);
        using MemberT = std::decay_t<decltype(val)>;
        if constexpr (is_optional_v<MemberT>) {
          if (val.has_value()) {
            out[std::string(f.key)] = encode_value(*val);
          }
        } else {
          out[std::string(f.key)] = encode_value(val);
        }
      }
    });
    return out;
  }
};

}  // namespace dto
}  // namespace ros2_medkit_gateway
