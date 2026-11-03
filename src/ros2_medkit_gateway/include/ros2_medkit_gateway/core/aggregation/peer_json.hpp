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

#include <cstddef>
#include <string>

namespace ros2_medkit_gateway {

/// Deepest nesting of arrays and objects accepted in a JSON body a peer sends.
///
/// Parsing a body is safe at any depth - the parser and the destructor of the
/// JSON library this gateway builds against (nlohmann/json 3.11.3) keep their
/// own stacks on the heap - but copying, comparing and serializing a value
/// recurse once per level, and so does anything here that walks one. A peer
/// answering with a few hundred thousand nested arrays would overflow the
/// stack of the thread handling the request and take the gateway down. A body
/// a gateway serves nests a few levels - an entity detail five - and the
/// deepest carry ROS message values, which nest as deep as their message
/// types do; 64 is far above any of them and keeps every walk shallow. The text
/// is judged before it is parsed (json_nesting_exceeds), so no deep value is
/// ever built either.
inline constexpr size_t kMaxPeerJsonDepth = 64;

/// True when `text` nests arrays and objects deeper than `max_depth`.
///
/// Scans the text - outside string literals, counting `[` and `{` against
/// `]` and `}` - so a body is judged before anything is built from it. Text
/// that is not valid JSON is judged the same way and then fails to parse.
bool json_nesting_exceeds(const std::string & text, size_t max_depth = kMaxPeerJsonDepth);

}  // namespace ros2_medkit_gateway
