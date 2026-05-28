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

#include <httplib.h>

namespace ros2_medkit_gateway {
namespace http {
namespace detail {

/**
 * @brief Framework-internal thread-local channel that lets the typed
 *        `validate_entity_for_route` overload stream the proxied response
 *        body to the underlying cpp-httplib response when an entity belongs
 *        to a remote peer.
 *
 * The typed router installs a `ForwardResponseScope` around every typed
 * handler invocation so the validator can write the proxy response to the
 * real cpp-httplib response object without taking `httplib::Response &` as a
 * parameter.
 *
 * The framework guarantees one in-flight call per thread, so a thread_local
 * pointer is safe. Handlers MUST NOT touch this header - it is part of the
 * internal routing layer.
 */
extern thread_local httplib::Response * tl_forward_response;

/**
 * @brief RAII helper that installs a thread-local response pointer used by
 *        the typed validator's peer-forwarding path.
 *
 * Non-copyable, non-movable. The previous value is restored on destruction
 * so nested scopes (legacy validator delegating to the typed validator, or
 * tests that install their own response) compose correctly.
 */
class ForwardResponseScope {
 public:
  explicit ForwardResponseScope(httplib::Response * res) : prev_(tl_forward_response) {
    tl_forward_response = res;
  }
  ~ForwardResponseScope() {
    tl_forward_response = prev_;
  }
  ForwardResponseScope(const ForwardResponseScope &) = delete;
  ForwardResponseScope & operator=(const ForwardResponseScope &) = delete;
  ForwardResponseScope(ForwardResponseScope &&) = delete;
  ForwardResponseScope & operator=(ForwardResponseScope &&) = delete;

 private:
  httplib::Response * prev_;
};

}  // namespace detail
}  // namespace http
}  // namespace ros2_medkit_gateway
