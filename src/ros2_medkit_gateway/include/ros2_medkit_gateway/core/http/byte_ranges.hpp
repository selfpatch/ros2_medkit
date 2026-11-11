// Copyright 2026 mfaferek93
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

#include <cstdint>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace http {

/// One range of a `Range` header as cpp-httplib parses it: first and last byte,
/// each -1 when the header leaves it out (`bytes=1000-`, `bytes=-500`).
using RequestedByteRange = std::pair<int64_t, int64_t>;

/// An absolute, inclusive first-last byte range inside the representation.
using ByteRange = std::pair<uint64_t, uint64_t>;

/// Resolve @p requested against a representation of @p size bytes, per RFC 9110
/// section 14.1.2.
///
/// A last byte at or past the end means "to the end", so it is clamped to
/// `size - 1`. A suffix range `-N` is the last N bytes, or the whole representation
/// when it is shorter. A range whose first byte is at or past the end, and a suffix
/// of zero bytes, are unsatisfiable and dropped. The others keep their order.
///
/// @return The satisfiable ranges. Empty when none is, which is the only case a
///         server answers 416 (RFC 9110 section 15.5.17).
std::vector<ByteRange> satisfiable_byte_ranges(const std::vector<RequestedByteRange> & requested, uint64_t size);

}  // namespace http
}  // namespace ros2_medkit_gateway
