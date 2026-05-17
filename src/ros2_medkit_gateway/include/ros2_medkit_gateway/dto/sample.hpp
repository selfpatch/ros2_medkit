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

#include <string>
#include <type_traits>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

/// Canonical value for a non-DTO field type. DTO-typed members never reach
/// here - dto_sample::make handles them directly.
template <class U>
U sample_value() {
  if constexpr (is_optional_v<U>) {
    return typename U::value_type{};  // present, default-filled
  } else if constexpr (is_vector_v<U>) {
    return U{};  // empty vector is a valid array
  } else if constexpr (std::is_same_v<U, std::string>) {
    return std::string{"sample"};
  } else if constexpr (std::is_same_v<U, bool>) {
    return true;
  } else if constexpr (std::is_arithmetic_v<U>) {
    return U{1};
  } else {
    return U{};  // nlohmann::json and any other type
  }
}

/// Build a canonical sample instance of a DTO by filling each field.
/// Specialize dto_sample<T> to override for DTOs the generic path cannot cover
/// (e.g. variant members - the variant's first alternative must be
/// default-constructible for the generic path to work).
template <class T>
struct dto_sample {
  static T make() {
    T obj{};
    for_each_field<T>([&](const auto & f) {
      using MemberT = std::decay_t<decltype(obj.*(f.ptr))>;
      if constexpr (is_dto_v<MemberT>) {
        obj.*(f.ptr) = dto_sample<MemberT>::make();
      } else if constexpr (is_optional_v<MemberT>) {
        if constexpr (is_dto_v<typename MemberT::value_type>) {
          obj.*(f.ptr) = dto_sample<typename MemberT::value_type>::make();
        } else {
          obj.*(f.ptr) = sample_value<MemberT>();
        }
      } else {
        obj.*(f.ptr) = sample_value<MemberT>();
      }
      // enum fields (string or optional<string>): use the first allowed value.
      // NESTED if constexpr - a single `&&` would substitute
      // `typename MemberT::value_type` for non-optional scalar members and
      // hard-fail to compile.
      if (f.enum_count > 0) {
        if constexpr (std::is_same_v<MemberT, std::string>) {
          obj.*(f.ptr) = std::string(f.enum_values[0]);
        } else if constexpr (is_optional_v<MemberT>) {
          if constexpr (std::is_same_v<typename MemberT::value_type, std::string>) {
            obj.*(f.ptr) = std::string(f.enum_values[0]);
          }
        }
      }
    });
    return obj;
  }
};

template <class T>
T make_sample() {
  return dto_sample<T>::make();
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
