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
#include <functional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

// ---------------------------------------------------------------------------
// flat_reset_value: clears container-valued entries in place (retains capacity)
// Generic overload is a no-op (scalars, integral types, etc.)
// ---------------------------------------------------------------------------

template <typename V>
inline void flat_reset_value(V & /*value*/) noexcept {
}

template <typename T, typename A>
inline void flat_reset_value(std::vector<T, A> & value) noexcept {
  value.clear();
}

inline void flat_reset_value(std::string & value) noexcept {
  value.clear();
}

// ---------------------------------------------------------------------------
// FlatHashMap<K,V,Hash>
//
// Open-addressed hash table with linear probing, tombstone deletion, and
// ZERO structural allocation in steady state after reserve().
//
// Invariants:
//   - Table size is always a power of two.
//   - max_load = 0.70: (occupied + deleted) / capacity triggers a grow.
//   - On grow, a brand-new table is allocated, all OCCUPIED entries rehashed,
//     tombstones are vacuumed, and grew_ is set to true.
//   - reset() retains the backing array (no deallocation) and calls
//     flat_reset_value() on each entry's value so container-valued maps
//     retain their heap buffers across refills.
//   - used_ tracks (occupied + deleted) in O(1) so ensure_capacity() never
//     scans the table.
// ---------------------------------------------------------------------------

template <typename K, typename V, typename Hash = std::hash<K>>
class FlatHashMap {
 public:
  FlatHashMap() = default;

  explicit FlatHashMap(size_t capacity) {
    reserve(capacity);
  }

  ~FlatHashMap() = default;
  FlatHashMap(const FlatHashMap &) = default;
  FlatHashMap & operator=(const FlatHashMap &) = default;
  FlatHashMap(FlatHashMap &&) noexcept = default;
  FlatHashMap & operator=(FlatHashMap &&) noexcept = default;

  // -------------------------------------------------------------------------
  // Capacity management
  // -------------------------------------------------------------------------

  /// Ensure the table can hold at least `capacity` entries at max_load.
  ///
  /// reserve() establishes the baseline capacity and is NOT a structural growth:
  /// the post-reserve grew_ flag is cleared so grew() reports only reallocations
  /// that occur because the live set later outgrew the reserved capacity. This
  /// matches SlotStore, whose reserve() likewise never sets grew().
  void reserve(size_t capacity) {
    size_t needed = next_pow2(static_cast<size_t>(static_cast<double>(capacity) / kMaxLoad + 1.0));
    if (needed > table_.size()) {
      rehash(needed);
      grew_ = false;  // baseline allocation, not a growth
    }
  }

  // -------------------------------------------------------------------------
  // Lookup
  // -------------------------------------------------------------------------

  V * find(const K & key) {
    return const_cast<V *>(const_cast<const FlatHashMap *>(this)->find(key));
  }

  const V * find(const K & key) const {
    if (table_.empty()) {
      return nullptr;
    }
    const uint64_t h = hash_of(key);
    size_t idx = slot(h);
    const size_t cap = table_.size();
    for (size_t probe = 0; probe < cap; ++probe) {
      const Entry & e = table_[idx];
      if (e.state == kEmpty) {
        return nullptr;
      }
      if (e.state == kOccupied && e.hash == h && e.key == key) {
        return &e.value;
      }
      idx = (idx + 1) & (cap - 1);
    }
    return nullptr;
  }

  bool contains(const K & key) const {
    return find(key) != nullptr;
  }

  // -------------------------------------------------------------------------
  // Insertion / assignment
  // -------------------------------------------------------------------------

  /// Insert key with default V if absent; return reference to value.
  /// If key already exists (OCCUPIED), return its existing value.
  /// On insertion reuses any available DELETED slot (or an EMPTY slot).
  V & get_or_create(const K & key) {
    ensure_capacity();
    const uint64_t h = hash_of(key);
    size_t idx = slot(h);
    const size_t cap = table_.size();
    size_t first_tombstone = cap;  // sentinel: no tombstone seen yet

    for (size_t probe = 0; probe < cap; ++probe) {
      Entry & e = table_[idx];
      if (e.state == kOccupied) {
        if (e.hash == h && e.key == key) {
          return e.value;
        }
      } else if (e.state == kDeleted) {
        if (first_tombstone == cap) {
          first_tombstone = idx;
        }
      } else {
        // kEmpty - key is absent; use tombstone if we saw one, else this slot
        size_t insert_at = (first_tombstone != cap) ? first_tombstone : idx;
        Entry & slot_ref = table_[insert_at];
        // Tombstone reuse: used_ already counted this slot; only bump if EMPTY
        if (slot_ref.state == kEmpty) {
          ++used_;
        }
        slot_ref.state = kOccupied;
        slot_ref.hash = h;
        slot_ref.key = key;
        flat_reset_value(slot_ref.value);
        ++size_;
        return slot_ref.value;
      }
      idx = (idx + 1) & (cap - 1);
    }

    // All slots DELETED, no EMPTY found - reuse tombstone (used_ already counted)
    if (first_tombstone != cap) {
      Entry & slot_ref = table_[first_tombstone];
      slot_ref.state = kOccupied;
      slot_ref.hash = h;
      slot_ref.key = key;
      flat_reset_value(slot_ref.value);
      ++size_;
      return slot_ref.value;
    }

    // Should be unreachable due to load factor guard.
    rehash(table_.size() * 2);
    return get_or_create(key);
  }

  /// Assign value to key. Returns true if a NEW key was inserted (false if
  /// key already existed and was updated).
  bool insert_or_assign(const K & key, V value) {
    ensure_capacity();
    const uint64_t h = hash_of(key);
    size_t idx = slot(h);
    const size_t cap = table_.size();
    size_t first_tombstone = cap;

    for (size_t probe = 0; probe < cap; ++probe) {
      Entry & e = table_[idx];
      if (e.state == kOccupied) {
        if (e.hash == h && e.key == key) {
          e.value = std::move(value);
          return false;  // updated, not new
        }
      } else if (e.state == kDeleted) {
        if (first_tombstone == cap) {
          first_tombstone = idx;
        }
      } else {
        // kEmpty
        size_t insert_at = (first_tombstone != cap) ? first_tombstone : idx;
        Entry & slot_ref = table_[insert_at];
        // Tombstone reuse: used_ already counted this slot; only bump if EMPTY
        if (slot_ref.state == kEmpty) {
          ++used_;
        }
        slot_ref.state = kOccupied;
        slot_ref.hash = h;
        slot_ref.key = key;
        slot_ref.value = std::move(value);
        ++size_;
        return true;
      }
      idx = (idx + 1) & (cap - 1);
    }

    // All slots DELETED, no EMPTY found - reuse tombstone (used_ already counted)
    if (first_tombstone != cap) {
      Entry & slot_ref = table_[first_tombstone];
      slot_ref.state = kOccupied;
      slot_ref.hash = h;
      slot_ref.key = key;
      slot_ref.value = std::move(value);
      ++size_;
      return true;
    }

    rehash(table_.size() * 2);
    return insert_or_assign(key, std::move(value));
  }

  // -------------------------------------------------------------------------
  // Deletion
  // -------------------------------------------------------------------------

  /// Erase key. Returns true if the key was present and removed.
  bool erase(const K & key) {
    if (table_.empty()) {
      return false;
    }
    const uint64_t h = hash_of(key);
    size_t idx = slot(h);
    const size_t cap = table_.size();
    for (size_t probe = 0; probe < cap; ++probe) {
      Entry & e = table_[idx];
      if (e.state == kEmpty) {
        return false;
      }
      if (e.state == kOccupied && e.hash == h && e.key == key) {
        e.state = kDeleted;
        --size_;
        // used_ stays unchanged: DELETED still counts toward load
        return true;
      }
      idx = (idx + 1) & (cap - 1);
    }
    return false;
  }

  // -------------------------------------------------------------------------
  // Reset (logical clear, retain allocation)
  // -------------------------------------------------------------------------

  /// Clear all entries in place. Backing array is NOT shrunk.
  /// For container-valued entries, calls flat_reset_value() to retain capacity.
  void reset() {
    for (Entry & e : table_) {
      if (e.state == kOccupied) {
        flat_reset_value(e.value);
      }
      e.state = kEmpty;
    }
    size_ = 0;
    used_ = 0;
    // Note: grew_ is NOT cleared by reset - only clear_grew() does that.
    // Note: grew_ is NOT set by reset - only actual reallocation sets it.
  }

  // -------------------------------------------------------------------------
  // Observers
  // -------------------------------------------------------------------------

  size_t size() const noexcept {
    return size_;
  }

  bool grew() const noexcept {
    return grew_;
  }

  void clear_grew() noexcept {
    grew_ = false;
  }

  // -------------------------------------------------------------------------
  // Iteration
  // -------------------------------------------------------------------------

  template <typename Fn>
  void for_each(Fn && fn) const {
    for (const Entry & e : table_) {
      if (e.state == kOccupied) {
        fn(e.key, e.value);
      }
    }
  }

 private:
  // -------------------------------------------------------------------------
  // Entry state constants
  // -------------------------------------------------------------------------
  static constexpr uint8_t kEmpty = 0;
  static constexpr uint8_t kOccupied = 1;
  static constexpr uint8_t kDeleted = 2;

  // Maximum load factor: (occupied + deleted) / capacity
  static constexpr double kMaxLoad = 0.7;

  // -------------------------------------------------------------------------
  // Entry structure
  // -------------------------------------------------------------------------
  struct Entry {
    uint64_t hash{0};
    K key{};
    V value{};
    uint8_t state{kEmpty};
  };

  // -------------------------------------------------------------------------
  // Internal helpers
  // -------------------------------------------------------------------------

  uint64_t hash_of(const K & key) const {
    return static_cast<uint64_t>(Hash{}(key));
  }

  size_t slot(uint64_t h) const {
    return static_cast<size_t>(h) & (table_.size() - 1);
  }

  static size_t next_pow2(size_t n) {
    if (n == 0) {
      return 1;
    }
    size_t p = 1;
    while (p < n) {
      p <<= 1;
    }
    return p;
  }

  /// Keep the table below max load. When (occupied+deleted) load is too high we
  /// distinguish two causes, which matters for steady-state add/remove churn:
  ///   - If the LIVE entries (size_) still fit at the current capacity, the load
  ///     is driven by accumulated tombstones - rehash to the SAME capacity to
  ///     vacuum them. This does not grow the table, so grew() stays false and
  ///     churn over a bounded set never reallocates.
  ///   - Otherwise the live set genuinely outgrew the table - double capacity.
  /// O(1): uses the tracked size_/used_ counters rather than scanning the table.
  void ensure_capacity() {
    if (table_.empty()) {
      rehash(4);
      return;
    }
    const double cap = static_cast<double>(table_.size());
    if (static_cast<double>(used_ + 1) <= kMaxLoad * cap) {
      return;  // within load - nothing to do
    }
    if (static_cast<double>(size_ + 1) <= kMaxLoad * cap) {
      rehash(table_.size());  // tombstone-driven: vacuum in place, no growth
    } else {
      rehash(table_.size() * 2);  // live set outgrew capacity: grow
    }
  }

  /// Rehash into a new table of the given size (must be power-of-two).
  /// Vacuums tombstones (used_ resets to size_ after rehash). Sets grew_ only
  /// when the new capacity exceeds the old one - a same-size vacuum preserves
  /// capacity and must not be reported as a structural growth.
  void rehash(size_t new_cap) {
    new_cap = next_pow2(new_cap);
    const bool grew_capacity = new_cap > table_.size();
    std::vector<Entry> new_table(new_cap);
    for (Entry & old_e : table_) {
      if (old_e.state != kOccupied) {
        continue;
      }
      size_t idx = static_cast<size_t>(old_e.hash) & (new_cap - 1);
      while (new_table[idx].state == kOccupied) {
        idx = (idx + 1) & (new_cap - 1);
      }
      new_table[idx].hash = old_e.hash;
      new_table[idx].key = std::move(old_e.key);
      new_table[idx].value = std::move(old_e.value);
      new_table[idx].state = kOccupied;
    }
    table_ = std::move(new_table);
    used_ = size_;  // tombstones vacuumed
    if (grew_capacity) {
      grew_ = true;
    }
  }

  // -------------------------------------------------------------------------
  // Data members
  // -------------------------------------------------------------------------
  std::vector<Entry> table_;
  size_t size_{0};  // number of OCCUPIED entries
  size_t used_{0};  // number of OCCUPIED + DELETED entries (load denominator)
  bool grew_{false};
};

}  // namespace ros2_medkit_gateway
