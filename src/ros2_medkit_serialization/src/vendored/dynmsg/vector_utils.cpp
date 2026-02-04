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

#include <vector>

#include "ros2_medkit_serialization/vendored/dynmsg/vector_utils.hpp"

/// Memory layout of an std::vector.
/**
 * https://en.cppreference.com/w/cpp/container/vector
 * https://stackoverflow.com/a/52337100
 */
struct fake_vector
{
  void * begin;
  void * end;
  void * end_capacity;
};

union vector_union {
  // The <Type> here doesn't matter because the memory
  // layout (see fake_vector) is always the same.
  std::vector<int> * std;
  fake_vector * fake;
};

namespace dynmsg
{

/**
 * This uses the fact that std::vector<T> is a contiguous container, therefore
 *    vector_size = (end - begin) / element_size
 *
 * This doesn't work for std::vector<bool> because that specialization
 * is space-optimized and doesn't store its elements in a contiguous array,
 * so that above gives a wildly invalid result.
 */
size_t get_vector_size(const uint8_t * vector, size_t element_size)
{
  // Should not get an element size of 0, but we want to avoid errors
  if (0ul == element_size) {
    return 0ul;
  }
  vector_union v = {reinterpret_cast<std::vector<int> *>(const_cast<uint8_t *>(vector))};
  return (
    reinterpret_cast<uint64_t>(v.fake->end) - reinterpret_cast<uint64_t>(v.fake->begin)
  ) / element_size;
}

}  // namespace dynmsg
