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
#include <cstdint>
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
/// is judged before it is parsed (judge_peer_json), so no deep value is ever
/// built either.
inline constexpr size_t kMaxPeerJsonDepth = 64;

/// Most values - array elements and object members, counted at every level -
/// accepted in a JSON body a peer sends.
///
/// The JSON library builds a 16-byte value for each, and an object or a
/// string costs more on the heap, so a parsed body costs many times its text:
/// a list of three million empty objects, 9 MB, parsed to some 0.6 GB, and the
/// size limit alone lets a body hold five million values. The largest answers
/// a gateway gives are its collections, at most MAX_ENTITIES_PER_COLLECTION
/// (1000) items a peer, and the largest item of a gateway serving the demo
/// nodes holds about 160 values (a data item with its message schema), the
/// largest body (225 parameters) about 3000. 1000 items of 1000 values each
/// is far above both. A body at the bound made of the costliest values there
/// are, a million empty objects, parses to some 250 MB, against some 600 MB
/// for the 9 MB list. Judged on the text before it is parsed
/// (judge_peer_json), like the depth.
inline constexpr size_t kMaxPeerJsonValues = 1000 * 1000;

/// What the text of a peer's JSON body was judged to be, before it is parsed.
enum class PeerJsonShape : uint8_t {
  kAcceptable,     ///< Within both bounds; it may still fail to parse.
  kTooDeep,        ///< Nests deeper than the depth bound.
  kTooManyValues,  ///< Holds more values than the value bound.
};

/// Judge `text`, a JSON body a peer sent, before anything is built from it.
///
/// Scans the text once, outside string literals: `[` and `{` against `]` and
/// `}` for the depth, and every array element and object member - the first
/// in a container and each one after a `,` - for the count. Text that is not
/// valid JSON is judged the same way and then fails to parse.
PeerJsonShape judge_peer_json(const std::string & text, size_t max_depth = kMaxPeerJsonDepth,
                              size_t max_values = kMaxPeerJsonValues);

/// What a body judge_peer_json refused does, for an error message that
/// follows the body's name: "nests deeper than 64 levels" or "holds more than
/// 1000000 values". Empty for kAcceptable.
std::string describe_peer_json_shape(PeerJsonShape shape);

}  // namespace ros2_medkit_gateway
