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

#include "ros2_medkit_gateway/core/aggregation/peer_json.hpp"

namespace ros2_medkit_gateway {

bool json_nesting_exceeds(const std::string & text, size_t max_depth) {
  size_t depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (const char c : text) {
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }
    switch (c) {
      case '"':
        in_string = true;
        break;
      case '[':
      case '{':
        if (++depth > max_depth) {
          return true;
        }
        break;
      case ']':
      case '}':
        if (depth > 0) {
          --depth;
        }
        break;
      default:
        break;
    }
  }
  return false;
}

}  // namespace ros2_medkit_gateway
