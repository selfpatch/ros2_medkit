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
#include <vector>

namespace ros2_medkit_gateway {

/// Fixed-capacity object pool with stable slot indices.
///
/// Slots are never shifted; removal tombstones a slot onto a free-list so the
/// next alloc_slot() reuses it without growing the underlying vectors.
/// This keeps capacity() bounded under add/remove churn - the embedded crux:
/// structural arrays are reserved at init, no realloc in steady state.
template <typename T>
class SlotStore {
 public:
  SlotStore() = default;

  explicit SlotStore(size_t cap) {
    reserve(cap);
  }

  ~SlotStore() = default;
  SlotStore(const SlotStore &) = default;
  SlotStore & operator=(const SlotStore &) = default;
  SlotStore(SlotStore &&) = default;
  SlotStore & operator=(SlotStore &&) = default;

  /// Reserve storage for `cap` slots without marking any live.
  void reserve(size_t cap) {
    items_.reserve(cap);
    live_.reserve(cap);
    free_list_.reserve(cap);
  }

  /// Allocate a slot index: pop from free-list if available, else append.
  /// Appending past items_.capacity() causes a vector grow and sets grew_.
  uint32_t alloc_slot() {
    if (!free_list_.empty()) {
      uint32_t slot = free_list_.back();
      free_list_.pop_back();
      return slot;
    }
    size_t before = items_.capacity();
    items_.emplace_back();
    live_.push_back(0u);
    if (items_.capacity() != before) {
      grew_ = true;
    }
    return static_cast<uint32_t>(items_.size() - 1u);
  }

  /// Copy value into the slot and mark it live. Assigning over an already-live
  /// slot updates the payload in place and does NOT change live_count_; only a
  /// dead -> live transition bumps the count. This makes assign() safe for the
  /// incremental-reconcile update path that overwrites existing live slots.
  void assign(uint32_t slot, const T & value) {
    items_[static_cast<size_t>(slot)] = value;
    if (live_[static_cast<size_t>(slot)] == 0u) {
      live_[static_cast<size_t>(slot)] = 1u;
      ++live_count_;
    }
  }

  /// Move value into the slot and mark it live. Assigning over an already-live
  /// slot updates the payload in place and does NOT change live_count_; only a
  /// dead -> live transition bumps the count.
  void assign(uint32_t slot, T && value) {
    items_[static_cast<size_t>(slot)] = std::move(value);
    if (live_[static_cast<size_t>(slot)] == 0u) {
      live_[static_cast<size_t>(slot)] = 1u;
      ++live_count_;
    }
  }

  /// Release the slot: reset payload to T{}, mark dead, push to free-list.
  void free(uint32_t slot) {
    items_[static_cast<size_t>(slot)] = T{};
    live_[static_cast<size_t>(slot)] = 0u;
    --live_count_;
    free_list_.push_back(slot);
  }

  bool is_live(uint32_t slot) const noexcept {
    return live_[static_cast<size_t>(slot)] != 0u;
  }

  T & operator[](size_t slot) noexcept {
    return items_[slot];
  }
  const T & operator[](size_t slot) const noexcept {
    return items_[slot];
  }

  /// Number of slots allocated (including dead). Use for index bounds checks.
  size_t slot_count() const noexcept {
    return items_.size();
  }

  size_t live_count() const noexcept {
    return live_count_;
  }

  /// Current storage capacity (= items_.capacity()).
  size_t capacity() const noexcept {
    return items_.capacity();
  }

  bool grew() const noexcept {
    return grew_;
  }
  void clear_grew() noexcept {
    grew_ = false;
  }

  /// Call fn(uint32_t slot, const T&) for every live slot.
  template <typename Fn>
  void for_each_live(Fn && fn) const {
    for (size_t i = 0; i < items_.size(); ++i) {
      if (live_[i] != 0u) {
        fn(static_cast<uint32_t>(i), items_[i]);
      }
    }
  }

  /// Compact copy of all live values (order: increasing slot index).
  std::vector<T> collect_live() const {
    std::vector<T> result;
    result.reserve(live_count_);
    for (size_t i = 0; i < items_.size(); ++i) {
      if (live_[i] != 0u) {
        result.push_back(items_[i]);
      }
    }
    return result;
  }

 private:
  std::vector<T> items_;
  std::vector<uint8_t> live_;  // NOT vector<bool>
  std::vector<uint32_t> free_list_;
  size_t live_count_{0};
  bool grew_{false};
};

}  // namespace ros2_medkit_gateway
