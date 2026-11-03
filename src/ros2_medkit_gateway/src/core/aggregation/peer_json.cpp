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

PeerJsonShape judge_peer_json(const std::string & text, size_t max_depth, size_t max_values) {
  size_t depth = 0;
  size_t values = 0;
  bool in_string = false;
  bool escaped = false;
  // Set by `[` or `{`: the next character that is not white space either
  // closes the container or starts its first value.
  bool opened = false;
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
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
      continue;
    }
    if (opened) {
      opened = false;
      if (c != ']' && c != '}' && ++values > max_values) {
        return PeerJsonShape::kTooManyValues;
      }
    }
    switch (c) {
      case '"':
        in_string = true;
        break;
      case '[':
      case '{':
        if (++depth > max_depth) {
          return PeerJsonShape::kTooDeep;
        }
        opened = true;
        break;
      case ']':
      case '}':
        if (depth > 0) {
          --depth;
        }
        break;
      case ',':
        if (++values > max_values) {
          return PeerJsonShape::kTooManyValues;
        }
        break;
      default:
        break;
    }
  }
  return PeerJsonShape::kAcceptable;
}

std::string describe_peer_json_shape(PeerJsonShape shape) {
  switch (shape) {
    case PeerJsonShape::kTooDeep:
      return "nests deeper than " + std::to_string(kMaxPeerJsonDepth) + " levels";
    case PeerJsonShape::kTooManyValues:
      return "holds more than " + std::to_string(kMaxPeerJsonValues) + " values";
    case PeerJsonShape::kAcceptable:
      break;
  }
  return {};
}

}  // namespace ros2_medkit_gateway
