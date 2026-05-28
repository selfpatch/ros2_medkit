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

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace http {

/// Response shape returned by `RouteRegistry::binary_download` handlers.
/// The framework wires the `provider` callback into cpp-httplib's range-aware
/// content provider machinery. Range support is opt-in via `supports_ranges`
/// because not every backend can serve byte ranges efficiently.
///
/// Minimum viable shape - real-world callers (bulkdata, snapshot download)
/// will tune the contract in their migration commit.
struct BinaryResponse {
  /// Range-aware content provider. `offset` and `length` are byte offsets
  /// into the logical resource; the callback writes the requested slice into
  /// `sink` and returns `true` to continue or `false` to abort the stream.
  std::function<bool(uint64_t offset, uint64_t length, httplib::DataSink & sink)> provider;
  /// MIME type (e.g. `application/octet-stream`, `application/gzip`).
  std::string content_type;
  /// Optional download filename; rendered as `Content-Disposition` if set.
  std::optional<std::string> filename;
  /// True iff the provider honours `offset`/`length`. False -> framework
  /// serves the full body in one shot.
  bool supports_ranges{false};
  /// Total size in bytes; cpp-httplib uses this for `Content-Length`.
  uint64_t total_size{0};
};

/// Thin typed view of a `multipart/form-data` request body.
/// Minimum viable shape - the migration commit that lands a real multipart
/// route (typically bulk-data upload) will add per-field typed accessors
/// (typed file part, typed JSON part) on top of `parts`.
struct MultipartBody {
  /// Raw parts as parsed by cpp-httplib. Each part carries name, filename,
  /// content type, and the part bytes.
  httplib::MultipartFormDataItems parts;
};

/// Response shape returned by `RouteRegistry::static_asset` handlers.
/// Used for serving small bundled assets - HTML, JS, CSS, images - whose
/// content is already in memory.
struct StaticAsset {
  /// Asset bytes (already in memory).
  std::vector<std::uint8_t> bytes;
  /// MIME type (e.g. `text/html; charset=utf-8`, `application/javascript`).
  std::string content_type;
  /// Optional extra headers (Cache-Control, ETag, etc.).
  std::vector<std::pair<std::string, std::string>> headers;
};

/// Server-Sent Events stream shape returned by `RouteRegistry::sse` handlers.
/// Each call to `next_event` writes one event into the cpp-httplib data sink
/// (formatted `data: <json>\n\n`) and returns `true` to continue or `false`
/// to terminate the stream.
///
/// Minimum viable shape - real-world callers (fault stream, log stream) will
/// likely move to a typed `TEvent`-aware variant on top of this in their
/// migration commit.
struct SseStream {
  std::function<bool(httplib::DataSink & sink)> next_event;
};

}  // namespace http
}  // namespace ros2_medkit_gateway
