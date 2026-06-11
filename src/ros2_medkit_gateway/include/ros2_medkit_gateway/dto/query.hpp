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

// Query-parameter contract: the fourth descriptor-driven visitor.
//
// A "query DTO" is a plain struct with a `dto_fields<T>` specialization whose
// members are query-parameter scalars (string / bool, optionally wrapped in
// std::optional). The same descriptor drives two sides so they cannot drift:
//
//   * QueryParamWriter<T> - emits the OpenAPI `parameters` array (all
//     `in: query`) for the route, mirroring SchemaWriter.
//   * TypedRequest::query<T>() (see http/typed_router.hpp) - parses the request
//     query string into a typed T, using `assign_query_field` below.
//
// Because a handler can only read fields that exist on T, and those same fields
// are what the spec advertises, a handler cannot read an undeclared query
// parameter. Adding a parameter means adding a member - which appears in the
// spec automatically.

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"

namespace ros2_medkit_gateway {
namespace dto {

/// Unwrap std::optional<U> -> U, identity otherwise. Query-parameter optionality
/// is expressed via `required:false`, not a nullable schema, so the emitted
/// `schema` uses the unwrapped scalar type.
template <class M>
struct query_value {
  using type = M;
};
template <class U>
struct query_value<std::optional<U>> {
  using type = U;
};
template <class M>
using query_value_t = typename query_value<M>::type;

/// Emits the OpenAPI `parameters` array (all `in: query`) for a query DTO `T`,
/// derived from the same `dto_fields<T>` descriptor the typed reader consumes.
template <class T>
struct QueryParamWriter {
  static nlohmann::json parameters() {
    nlohmann::json params = nlohmann::json::array();
    for_each_field<T>([&](const auto & f) {
      using FieldT = std::decay_t<decltype(f)>;
      static_assert(!is_opaque_object_field_v<FieldT>, "query DTOs must not contain opaque-object fields");
      using MemberT = std::decay_t<decltype(std::declval<T>().*(f.ptr))>;

      nlohmann::json param;
      param["name"] = std::string(f.key);
      param["in"] = "query";
      param["required"] = (f.presence == Presence::kRequired);
      if (!f.description.empty()) {
        param["description"] = std::string(f.description);
      }
      nlohmann::json schema = schema_of<query_value_t<MemberT>>();
      if (f.enum_count > 0) {
        nlohmann::json values = nlohmann::json::array();
        for (std::size_t i = 0; i < f.enum_count; ++i) {
          values.push_back(std::string(f.enum_values[i]));
        }
        schema["enum"] = std::move(values);
      }
      param["schema"] = std::move(schema);
      params.push_back(std::move(param));
    });
    return params;
  }
};

/// Assigns a raw query-string value to a typed query-DTO member. Only the scalar
/// shapes a query parameter can take are supported; any other field type is a
/// compile error so unsupported types fail loudly at the route, not silently at
/// runtime.
template <class M>
void assign_query_field(M & member, const std::string & raw) {
  if constexpr (std::is_same_v<M, std::string> || std::is_same_v<M, std::optional<std::string>>) {
    member = raw;
  } else if constexpr (std::is_same_v<M, bool> || std::is_same_v<M, std::optional<bool>>) {
    member = (raw == "true");
  } else {
    static_assert(sizeof(M) == 0, "assign_query_field: unsupported query field type");
  }
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
