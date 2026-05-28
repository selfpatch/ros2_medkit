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

#include <optional>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"

namespace ros2_medkit_gateway {
namespace http {

/// Handler return type. The success branch carries the DTO (or marker like
/// NoContent) the handler wants to send; the error branch carries a fully
/// formed ErrorInfo the framework can serialize as a SOVD GenericError.
template <class T>
using Result = tl::expected<T, ErrorInfo>;

/// Empty success marker used by handlers that complete with no body (typically
/// DELETE -> 204 No Content). Distinct from `Result<void>` so that the
/// framework can statically dispatch the response writer.
struct NoContent {};

/// Sentinel returned by validators when the entity belongs to a remote peer
/// and the validator has already committed the wire response by proxying the
/// request to that peer. Returning Forwarded tells the caller "the response
/// has been written, do not write anything else".
struct Forwarded {};

namespace detail {
/// Variant ordering invariant: ErrorInfo must be the first alternative so a
/// default-constructed `std::variant<ErrorInfo, Forwarded>` (the error state
/// callers may inadvertently produce) denotes an "unknown error", not the
/// misleading "request already proxied" Forwarded state. If a future refactor
/// reorders the alternatives, this static_assert breaks compilation.
template <class T>
inline constexpr bool kValidatorVariantOrderingOk =
    std::is_same_v<std::variant_alternative_t<0, std::variant<ErrorInfo, Forwarded>>, ErrorInfo>;
static_assert(kValidatorVariantOrderingOk<void>, "ErrorInfo must be the first variant alternative");
}  // namespace detail

/// Return type for HandlerContext::validate_entity_for_route and similar
/// validators that may either succeed locally, fail with an ErrorInfo, or
/// short-circuit by proxying the request to a remote peer (Forwarded).
template <class T>
using ValidatorResult = tl::expected<T, std::variant<ErrorInfo, Forwarded>>;

/// Side-channel a handler can attach to its successful response when the
/// default "200 OK + DTO body" is not enough. Examples:
///   - 201 Created with a `Location` header for POST creating a resource.
///   - 202 Accepted for asynchronously processed requests.
///   - 207 Multi-Status for aggregated fan-out responses.
///   - Vendor headers such as `X-Medkit-Local-Only`.
struct ResponseAttachments {
  /// HTTP status to use instead of the default 200. Leave empty for 200.
  std::optional<int> status_override;
  /// Additional headers to append. Order is preserved; duplicate names are
  /// allowed (cpp-httplib delivers them as separate Set-Cookie-style headers).
  std::vector<std::pair<std::string, std::string>> headers;

  /// Fluent setter for the status override.
  ResponseAttachments & with_status(int status) {
    status_override = status;
    return *this;
  }

  /// Fluent setter that appends a header entry.
  ResponseAttachments & with_header(std::string name, std::string value) {
    headers.emplace_back(std::move(name), std::move(value));
    return *this;
  }
};

/// Thin, type-safe wrapper around `const httplib::Request &` so handlers do
/// not see raw cpp-httplib types. A future commit will extend the constructor
/// with a RouteEntry parameter to support named path-parameter lookup; for now
/// path_param() relies on regex group ordering through the placeholder path
/// below.
class TypedRequest {
 public:
  /// Wraps the framework-provided request reference. The TypedRequest does
  /// not own the request; the request must outlive the wrapper.
  explicit TypedRequest(const httplib::Request & req) : req_(req) {
  }

  /// Returns the value of the named path parameter, or an `ErrorInfo` if the
  /// lookup fails. This method never throws.
  ///
  /// PLACEHOLDER BEHAVIOR (until commit 4 wires named-lookup via `RouteEntry`):
  /// `name` must be a base-10 unsigned integer literal interpreted as a
  /// 0-based literal index into `req.matches` (matching `std::smatch` indexing
  /// where `matches[0]` is the full match, `matches[1]` is the first capture
  /// group, etc.). Empty `name`, non-numeric `name`, or out-of-range index all
  /// produce `tl::unexpected(ErrorInfo{ERR_INVALID_PARAMETER, ..., 400, {}})`.
  /// Handlers that need a real named lookup should wait for commit 4; this
  /// method is intentionally restrictive so misuse fails loudly rather than
  /// silently returning the wrong capture group.
  tl::expected<std::string, ErrorInfo> path_param(std::string_view name) const {
    auto make_err = [](std::string message) {
      ErrorInfo info;
      info.code = ERR_INVALID_PARAMETER;
      info.message = std::move(message);
      info.http_status = 400;
      return tl::unexpected(std::move(info));
    };
    if (name.empty()) {
      return make_err("path_param: empty name");
    }
    std::size_t idx = 0;
    for (char c : name) {
      if (c < '0' || c > '9') {
        return make_err("path_param: non-numeric name '" + std::string(name) + "' (named lookup not wired yet)");
      }
      idx = idx * 10 + static_cast<std::size_t>(c - '0');
    }
    if (idx >= req_.matches.size()) {
      return make_err("path_param: index " + std::to_string(idx) + " out of range");
    }
    return req_.matches[idx].str();
  }

  /// Returns the value of the named query parameter, or std::nullopt if the
  /// request does not carry that parameter at all.
  std::optional<std::string> query_param(std::string_view name) const {
    // Note: Humble's cpp-httplib is older and only accepts const char*; Jazzy
    // accepts std::string_view too. Keep the c_str() conversion for Humble
    // compatibility (clang-tidy on Jazzy flags it as redundant - the warning
    // is suppressed at the call site).
    const std::string name_str(name);
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    if (!req_.has_param(name_str.c_str())) {
      return std::nullopt;
    }
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    return req_.get_param_value(name_str.c_str());
  }

  /// Returns the value of the named header, or std::nullopt if absent.
  std::optional<std::string> header(std::string_view name) const {
    // Note: see query_param() comment - Humble compatibility requires c_str().
    const std::string name_str(name);
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    if (!req_.has_header(name_str.c_str())) {
      return std::nullopt;
    }
    // NOLINTNEXTLINE(readability-redundant-string-cstr)
    return req_.get_header_value(name_str.c_str());
  }

  /// True if the client explicitly opted out of fan-out (header
  /// `X-Medkit-No-Fan-Out` present, value irrelevant).
  bool fan_out_disabled() const {
    return req_.has_header("X-Medkit-No-Fan-Out");
  }

  /// Returns the request path (post-routing, post-prefix-strip). Handlers
  /// occasionally need this to build a `Location` header for resources they
  /// just created via POST (e.g. `Location: <request-path>/<new-id>`). This
  /// is the only path-shaped read most handlers need; routes that need to
  /// inspect path segments should use `path_param` instead.
  const std::string & path() const {
    return req_.path;
  }

  /// Framework-only escape hatch back to the raw cpp-httplib request. Do not
  /// use this from handler bodies - it exists for the routing layer and for
  /// helpers that need access to fields not yet wrapped by TypedRequest. The
  /// `[[deprecated]]` attribute is load-bearing: it fires a warning at every
  /// caller, ensuring any premature handler use is surfaced at build time.
  /// The framework's own internal use sites (none yet; commit 4 will add the
  /// first) are expected to suppress the warning locally.
  [[deprecated("framework-internal; handlers must not call this")]]
  const httplib::Request & raw_for_framework() const {
    return req_;
  }

 private:
  const httplib::Request & req_;
};

}  // namespace http
}  // namespace ros2_medkit_gateway
