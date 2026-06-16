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
//
// Scope of the guarantee: the descriptor binds parameter NAMES and PRESENCE -
// those cannot drift between the spec and the parser. It does NOT police a
// parameter's VALUE. The emitted `schema` (enum membership, type/format) is
// advisory metadata; enforcing it (e.g. rejecting `?status=bogus`) is the
// handler's job, where the richer 400 payload lives. To keep the two type
// universes aligned, query fields are constrained at compile time to the scalar
// shapes the parser supports (string / bool, optionally std::optional), and a
// query parameter can never be `required` (the parser always tolerates absence,
// leaving the member at its default).

#include <cctype>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tuple>
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

/// The scalar shapes a query parameter may take (after std::optional unwrapping).
/// This is the single source of truth shared by both sides of the contract: the
/// reader (`assign_query_field`) parses exactly these, and the writer
/// (`QueryParamWriter`) static_asserts every field against it - so the emitted
/// schema can never advertise a type the parser would reject.
template <class U>
inline constexpr bool is_query_scalar_v = std::is_same_v<U, std::string> || std::is_same_v<U, bool>;

/// True iff every field of the query DTO `T` is declared optional (never
/// `Presence::kRequired`). The parser leaves absent parameters at their default
/// and never rejects a missing query parameter, so a `required:true` parameter
/// would be advertised but silently unenforced; this check makes that
/// unrepresentable. Evaluated in a `static_assert` inside `QueryParamWriter`.
template <class T>
constexpr bool query_dto_all_optional() {
  bool all_optional = true;
  std::apply(
      [&all_optional](const auto &... f) {
        ((all_optional = all_optional && (f.presence != Presence::kRequired)), ...);
      },
      dto_fields<T>);
  return all_optional;
}

/// Emits the OpenAPI `parameters` array (all `in: query`) for a query DTO `T`,
/// derived from the same `dto_fields<T>` descriptor the typed reader consumes.
template <class T>
struct QueryParamWriter {
  static nlohmann::json parameters() {
    static_assert(query_dto_all_optional<T>(),
                  "query DTO fields must be optional: the parser tolerates absence and never enforces "
                  "presence, so a required query parameter would be advertised but silently unenforced. "
                  "Declare the member as std::optional<...> or pass Presence::kOptional.");
    nlohmann::json params = nlohmann::json::array();
    for_each_field<T>([&](const auto & f) {
      using FieldT = std::decay_t<decltype(f)>;
      static_assert(!is_opaque_object_field_v<FieldT>, "query DTOs must not contain opaque-object fields");
      using MemberT = std::decay_t<decltype(std::declval<T>().*(f.ptr))>;
      static_assert(is_query_scalar_v<query_value_t<MemberT>>,
                    "query DTO fields must be std::string or bool (optionally std::optional). The reader "
                    "(assign_query_field) parses only these, so any other type would advertise a schema the "
                    "parser cannot honour.");

      nlohmann::json param;
      param["name"] = std::string(f.key);
      param["in"] = "query";
      param["required"] = (f.presence == Presence::kRequired);
      if (!f.description.empty()) {
        param["description"] = std::string(f.description);
      }
      nlohmann::json schema = schema_of<query_value_t<MemberT>>();
      if (f.enum_count > 0) {
        // Advisory only: the parser does not reject off-enum values (see the
        // file header). The handler validates membership and owns the 400.
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

/// Parses a raw query-string value as a boolean, matching the `type: boolean`
/// the schema advertises. `true`/`1` (case-insensitive) are truthy; every other
/// value - including an empty flag-style `?x`, `false`, and `0` - is false. The
/// parser does not reject malformed values (query parsing never 400s; see the
/// file header), so a typo simply reads as false.
inline bool parse_query_bool(const std::string & raw) {
  std::string lowered;
  lowered.reserve(raw.size());
  for (char c : raw) {
    lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return lowered == "true" || lowered == "1";
}

/// Assigns a raw query-string value to a typed query-DTO member. Only the scalar
/// shapes a query parameter can take are supported; any other field type is a
/// compile error so unsupported types fail loudly at the route, not silently at
/// runtime. `QueryParamWriter` static_asserts the same type set, so the schema
/// and the parser can never advertise different shapes.
template <class M>
void assign_query_field(M & member, const std::string & raw) {
  // Gate on the SAME trait QueryParamWriter enforces, so the reader and the
  // schema writer can never advertise different type sets - is_query_scalar_v is
  // the one source of truth, consulted by both sides.
  static_assert(is_query_scalar_v<query_value_t<M>>,
                "assign_query_field: unsupported query field type (expected std::string or bool, optionally "
                "std::optional). is_query_scalar_v is the shared source of truth QueryParamWriter also enforces.");
  if constexpr (std::is_same_v<query_value_t<M>, std::string>) {
    member = raw;
  } else {  // query_value_t<M> is bool, guaranteed by the static_assert above
    member = parse_query_bool(raw);
  }
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
