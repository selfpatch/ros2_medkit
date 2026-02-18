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

#include <atomic>
#include <cstddef>

namespace ros2_medkit_gateway {

/// Shared counter for SSE client connections across all SSE handlers.
/// Thread-safe via atomic compare-exchange.
class SSEClientTracker {
 public:
  explicit SSEClientTracker(size_t max_clients) : max_clients_(max_clients) {
  }

  /// Try to register a new SSE client. Returns true if under limit.
  bool try_connect() {
    size_t current = count_.load();
    while (current < max_clients_) {
      if (count_.compare_exchange_weak(current, current + 1)) {
        return true;
      }
    }
    return false;
  }

  /// Unregister an SSE client.
  void disconnect() {
    count_.fetch_sub(1);
  }

  size_t connected_clients() const {
    return count_.load();
  }

  size_t max_clients() const {
    return max_clients_;
  }

 private:
  std::atomic<size_t> count_{0};
  size_t max_clients_;
};

}  // namespace ros2_medkit_gateway
