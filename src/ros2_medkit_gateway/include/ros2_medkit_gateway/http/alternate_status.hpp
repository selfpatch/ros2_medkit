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

#include "ros2_medkit_gateway/http/handler_result.hpp"

namespace ros2_medkit_gateway {
namespace http {

/// Per-type HTTP status mapping used by `RouteRegistry::post_alternates` /
/// `RouteRegistry::del_alternates`. The framework picks the status from the
/// active alternative of the handler's returned `std::variant<TAlt...>`.
///
/// Default is `200`. Specialize for any DTO whose default should differ:
///
/// ```
/// template <> struct dto_alternate_status<MyDto> {
///   static constexpr int value = 202;
/// };
/// ```
///
/// `NoContent` is specialized below because it is a framework type. Other
/// specializations should live next to the DTO they describe (per-domain
/// header) so the wire-shape mapping stays close to the DTO definition.
template <class T>
struct dto_alternate_status {
  static constexpr int value = 200;
};

/// `NoContent` marker always maps to 204 No Content.
template <>
struct dto_alternate_status<NoContent> {
  static constexpr int value = 204;
};

}  // namespace http
}  // namespace ros2_medkit_gateway
