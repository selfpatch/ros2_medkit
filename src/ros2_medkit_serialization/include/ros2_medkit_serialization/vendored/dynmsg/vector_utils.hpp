// Copyright 2021 Christophe Bedard
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

#ifndef ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__VECTOR_UTILS_HPP_
#define ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__VECTOR_UTILS_HPP_

#include <cstdint>
#include <cstddef>

namespace dynmsg
{

/// Get the number of elements in/size of a vector given a pointer to it and its element size.
/**
 * This uses some knowledge/assumption about the std::vector implementation.
 * See the `get_vector_size` implementation for more info on how it works.
 *
 * Note that std::vector<bool> is a special optimized std::vector<T>
 * specialization and the "trick" used in get_vector_size() does not work.
 */
size_t get_vector_size(const uint8_t * vector, size_t element_size);

}  // namespace dynmsg

#endif  // ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__VECTOR_UTILS_HPP_
