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
#include <iterator>
#include <map>
#include <utility>

namespace ros2_medkit_gateway {

/// Fixed-capacity least-recently-used cache.
///
/// Backs per-key caches that would otherwise grow one entry per distinct key
/// for a process lifetime - for example the parameter transport's per-node
/// AsyncParametersClient cache and default-value cache, each of which gains one
/// entry per node name ever queried and, unbounded, leaks memory over a long
/// run against a churning graph. When an insert pushes the entry count past
/// @c max_size, the least-recently-used entry is evicted.
///
/// Recency is tracked with a monotonic access counter, not a wall or steady
/// clock: every find() hit and every put() stamps the entry with the next
/// counter value, and eviction removes the entry with the smallest stamp. This
/// gives a strict, tie-free ordering that is deterministic and test-friendly,
/// and needs no clock. The counter is 64-bit; at one access per nanosecond it
/// would take ~585 years to wrap, so overflow is not a practical concern.
///
/// NOT thread-safe. A shared instance must be guarded by the caller's own mutex
/// on every call. Ros2ParameterTransport holds clients_mutex_ around its client
/// cache and defaults_mutex_ around its default-value cache for exactly this
/// reason.
///
/// @tparam Key   map key type (must be usable as a std::map key)
/// @tparam Value cached value type
template <typename Key, typename Value>
class BoundedLruCache {
 public:
  /// @param max_size Maximum number of entries retained. Values below 1 are
  ///                 clamped to 1 (a zero-capacity LRU cache is meaningless and
  ///                 would evict the entry it just inserted).
  explicit BoundedLruCache(std::size_t max_size) : max_size_(max_size == 0 ? 1 : max_size) {
  }

  /// Look up @p key, marking it most-recently-used on a hit.
  /// @return pointer to the stored value, or nullptr if absent. The pointer is
  ///         valid only until the next mutating call (put/clear) on this cache.
  Value * find(const Key & key) {
    auto it = entries_.find(key);
    if (it == entries_.end()) {
      return nullptr;
    }
    it->second.stamp = ++counter_;
    return &it->second.value;
  }

  /// Report whether @p key is present WITHOUT changing its recency.
  bool contains(const Key & key) const {
    return entries_.find(key) != entries_.end();
  }

  /// Insert or replace the value for @p key, mark it most-recently-used, then
  /// evict the least-recently-used entry if the cache is now over capacity.
  ///
  /// The just-stored entry always carries the largest stamp, so it is never the
  /// eviction victim; erasing a different entry from the underlying std::map
  /// does not invalidate the reference returned here.
  ///
  /// @return reference to the stored value (valid until the next mutating call).
  Value & put(const Key & key, Value value) {
    auto & entry = entries_[key];
    entry.value = std::move(value);
    entry.stamp = ++counter_;
    if (entries_.size() > max_size_) {
      evict_lru();
    }
    return entry.value;
  }

  /// Remove all entries. The access counter is intentionally not reset - stamps
  /// only need to be monotonic, not dense.
  void clear() {
    entries_.clear();
  }

  std::size_t size() const {
    return entries_.size();
  }
  std::size_t max_size() const {
    return max_size_;
  }

 private:
  struct Entry {
    Value value;
    std::uint64_t stamp{0};
  };

  /// Erase the entry with the smallest access stamp (the least-recently-used).
  /// Only called right after an over-capacity insert, so the map holds at least
  /// two entries and the just-inserted one (largest stamp) is never the victim.
  void evict_lru() {
    auto victim = entries_.begin();
    for (auto it = std::next(entries_.begin()); it != entries_.end(); ++it) {
      if (it->second.stamp < victim->second.stamp) {
        victim = it;
      }
    }
    entries_.erase(victim);
  }

  std::size_t max_size_;
  std::uint64_t counter_{0};
  std::map<Key, Entry> entries_;
};

}  // namespace ros2_medkit_gateway
