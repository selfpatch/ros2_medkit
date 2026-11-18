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

#include "ros2_medkit_gateway/core/http/byte_ranges.hpp"

namespace ros2_medkit_gateway {
namespace http {

std::vector<ByteRange> satisfiable_byte_ranges(const std::vector<RequestedByteRange> & requested, uint64_t size) {
  std::vector<ByteRange> satisfiable;
  if (size == 0) {
    return satisfiable;
  }
  const uint64_t last_byte = size - 1;
  for (const auto & [first, last] : requested) {
    if (first < 0) {
      // Suffix range. cpp-httplib also reads a bare "-" this way, with no length,
      // and serves it as the whole representation.
      if (last == 0) {
        continue;
      }
      const uint64_t length = last < 0 ? size : static_cast<uint64_t>(last);
      satisfiable.emplace_back(length >= size ? 0 : size - length, last_byte);
      continue;
    }
    const auto start = static_cast<uint64_t>(first);
    if (start >= size) {
      continue;
    }
    const uint64_t end =
        (last < 0 || static_cast<uint64_t>(last) > last_byte) ? last_byte : static_cast<uint64_t>(last);
    satisfiable.emplace_back(start, end);
  }
  return satisfiable;
}

}  // namespace http
}  // namespace ros2_medkit_gateway
