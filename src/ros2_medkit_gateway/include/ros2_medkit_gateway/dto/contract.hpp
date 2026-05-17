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

#include <cstddef>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <variant>
#include <vector>

namespace ros2_medkit_gateway {
namespace dto {

/// Whether a DTO field is mandatory in the OpenAPI schema / request body.
enum class Presence { kRequired, kOptional };

// --- type traits -----------------------------------------------------------

template <class T>
struct is_optional : std::false_type {};
template <class U>
struct is_optional<std::optional<U>> : std::true_type {};
template <class T>
inline constexpr bool is_optional_v = is_optional<T>::value;

template <class T>
struct is_vector : std::false_type {};
template <class U, class A>
struct is_vector<std::vector<U, A>> : std::true_type {};
template <class T>
inline constexpr bool is_vector_v = is_vector<T>::value;

template <class T>
struct is_variant : std::false_type {};
template <class... U>
struct is_variant<std::variant<U...>> : std::true_type {};
template <class T>
inline constexpr bool is_variant_v = is_variant<T>::value;

/// Optional members default to kOptional, everything else to kRequired.
template <class M>
constexpr Presence default_presence() {
  return is_optional_v<M> ? Presence::kOptional : Presence::kRequired;
}

// --- Field -----------------------------------------------------------------

/// Binds a JSON key to a struct member plus OpenAPI metadata.
/// NEVER brace-initialize Field directly: aggregate CTAD is C++20-only.
/// Always construct via the field() / field_enum() factories below.
template <class Class, class Member>
struct Field {
  std::string_view key;
  Member Class::*ptr;
  Presence presence;
  std::string_view description;
  const std::string_view * enum_values;  // (ptr,count) into an inline constexpr array
  std::size_t enum_count;
};

template <class C, class M>
constexpr Field<C, M> field(std::string_view key, M C::*ptr, std::string_view desc = std::string_view{}) {
  return Field<C, M>{key, ptr, default_presence<M>(), desc, nullptr, 0};
}

template <class C, class M>
constexpr Field<C, M> field(std::string_view key, M C::*ptr, Presence p, std::string_view desc = std::string_view{}) {
  return Field<C, M>{key, ptr, p, desc, nullptr, 0};
}

/// Enum-constrained field: `values` must be an inline constexpr std::string_view array.
template <class C, class M, std::size_t N>
constexpr Field<C, M> field_enum(std::string_view key, M C::*ptr, const std::string_view (&values)[N],
                                 std::string_view desc = std::string_view{}) {
  return Field<C, M>{key, ptr, default_presence<M>(), desc, values, N};
}

// --- dto_fields / dto_name / is_dto -----------------------------------------

namespace detail {
/// Per-type sentinel: the value of the primary (unspecialized) dto_fields.
template <class T>
struct not_a_dto {};
}  // namespace detail

/// Primary template: a sentinel. Specialize per DTO with std::make_tuple(field(...), ...).
/// IMPORTANT: every dto_fields<X> / dto_name<X> specialization MUST be co-located
/// in the header that declares X. A TU that instantiates a visitor before seeing
/// the specialization silently binds this sentinel (latent ODR-adjacent bug).
/// IMPORTANT: a DTO must not transitively contain itself BY VALUE (infinite
/// template recursion) - use std::optional / std::vector / nlohmann::json for any
/// recursive shape.
template <class T>
inline constexpr auto dto_fields = detail::not_a_dto<T>{};

/// True iff dto_fields<T> was specialized (detected by sentinel type identity -
/// the primary is well-formed for any T, so std::void_t cannot probe it).
template <class T>
inline constexpr bool is_dto_v = !std::is_same_v<std::decay_t<decltype(dto_fields<T>)>, detail::not_a_dto<T>>;

/// Schema name in components/schemas. Specialize per DTO with a string literal.
template <class T>
inline constexpr std::string_view dto_name = std::string_view{};

// --- field fold -------------------------------------------------------------

/// Invoke `v` once per field of T, in declaration order.
template <class T, class V>
void for_each_field(V && v) {
  static_assert(is_dto_v<T>, "for_each_field requires a DTO; specialize dto_fields<T>");
  std::apply(
      [&](auto &&... f) {
        (v(f), ...);
      },
      dto_fields<T>);
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
