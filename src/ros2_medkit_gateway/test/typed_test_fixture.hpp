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

// TypedTestFixture - shared boilerplate for handler unit tests that drive a
// TypedRequest directly (no live HTTP server).
//
// Every typed handler takes a `TypedRequest` which wraps an `httplib::Request`.
// Tests typically need to:
//   1. set req.path,
//   2. populate req.matches by running the route regex against the path so
//      `path_param("1")` returns the first capture group as production code
//      does,
//   3. add query params and headers,
//   4. wrap the resulting raw request in a TypedRequest and invoke the handler.
//
// Steps 1+2 were duplicated across handler test files; this header centralises
// them so future typed-handler tests don't have to re-derive the pattern. The
// fixture owns the underlying `httplib::Request` so the returned `TypedRequest`
// stays valid for the test body.

#include <httplib.h>

#include <regex>
#include <string>
#include <string_view>
#include <utility>

#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway::test {

// Owns an httplib::Request and exposes typed accessors. Returned references
// stay valid for the fixture's lifetime.
class TypedTestFixture {
 public:
  TypedTestFixture() = default;

  TypedTestFixture(const TypedTestFixture &) = delete;
  TypedTestFixture & operator=(const TypedTestFixture &) = delete;
  TypedTestFixture(TypedTestFixture &&) = delete;
  TypedTestFixture & operator=(TypedTestFixture &&) = delete;
  ~TypedTestFixture() = default;

  // Set the request path (used by the handler when reading req.path).
  TypedTestFixture & with_path(std::string path) {
    req_.path = std::move(path);
    return *this;
  }

  // Apply a route regex against the current path so path_param("1") works.
  // No-op if the regex does not match - the caller can still drive the
  // handler to exercise the "missing capture" branch deliberately.
  TypedTestFixture & with_route_pattern(const std::string & pattern) {
    std::regex re(pattern);
    std::regex_match(req_.path, req_.matches, re);
    return *this;
  }

  // Add a single query parameter (HTTP ?name=value).
  TypedTestFixture & with_query(std::string name, std::string value) {
    req_.params.emplace(std::move(name), std::move(value));
    return *this;
  }

  // Add a single HTTP header.
  TypedTestFixture & with_header(std::string name, std::string value) {
    req_.headers.emplace(std::move(name), std::move(value));
    return *this;
  }

  // Set the request body.
  TypedTestFixture & with_body(std::string body) {
    req_.body = std::move(body);
    return *this;
  }

  // Build the TypedRequest wrapping the underlying httplib::Request. The
  // returned wrapper holds a reference to the fixture-owned request, so the
  // fixture must outlive the TypedRequest.
  http::TypedRequest build() {
    return http::TypedRequest(req_);
  }

  // Direct access to the underlying request for advanced setups not yet
  // covered by the builder API.
  httplib::Request & raw() {
    return req_;
  }
  const httplib::Request & raw() const {
    return req_;
  }

 private:
  httplib::Request req_;
};

// Convenience free function for the common "request with path + regex"
// shape. Returns a TypedRequest wrapping the caller-owned httplib::Request
// reference so the lifetime is explicit at the call site.
//
// Usage:
//   httplib::Request raw;
//   auto typed = make_typed_request(raw, "/api/v1/areas/sensors",
//                                   R"(/api/v1/areas/([^/]+))");
inline http::TypedRequest make_typed_request(httplib::Request & req, const std::string & path,
                                             std::string_view pattern = {}) {
  req.path = path;
  if (!pattern.empty()) {
    std::regex re(std::string{pattern});
    std::regex_match(req.path, req.matches, re);
  }
  return http::TypedRequest(req);
}

}  // namespace ros2_medkit_gateway::test
