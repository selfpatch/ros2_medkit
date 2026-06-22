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

#include <gtest/gtest.h>

#include "ros2_medkit_gateway/core/discovery/refresh_debounce.hpp"

using ros2_medkit_gateway::decide_graph_refresh;

// No graph event and nothing pending -> idle, no refresh, latch stays clear.
TEST(RefreshDebounce, IdleDoesNotRefresh) {
  auto d = decide_graph_refresh(/*event_fired=*/false, /*currently_dirty=*/false, /*elapsed_ms=*/100000, 1000);
  EXPECT_FALSE(d.refresh);
  EXPECT_FALSE(d.dirty);
}

// An event inside the debounce window latches dirty but does not refresh yet.
TEST(RefreshDebounce, EventWithinWindowLatchesButDefers) {
  auto d = decide_graph_refresh(/*event_fired=*/true, /*currently_dirty=*/false, /*elapsed_ms=*/10, 1000);
  EXPECT_FALSE(d.refresh);
  EXPECT_TRUE(d.dirty);
}

// A burst of events inside the window coalesces: still pending, still no refresh.
TEST(RefreshDebounce, CoalescesPendingWithinWindow) {
  auto d = decide_graph_refresh(/*event_fired=*/true, /*currently_dirty=*/true, /*elapsed_ms=*/500, 1000);
  EXPECT_FALSE(d.refresh);
  EXPECT_TRUE(d.dirty);
}

// Once the window elapses, a pending change is serviced and the latch clears.
TEST(RefreshDebounce, PendingPastWindowRefreshesAndClears) {
  auto d = decide_graph_refresh(/*event_fired=*/false, /*currently_dirty=*/true, /*elapsed_ms=*/1500, 1000);
  EXPECT_TRUE(d.refresh);
  EXPECT_FALSE(d.dirty);
}

// debounce_ms == 0 disables debouncing: any pending change refreshes immediately.
TEST(RefreshDebounce, ZeroDebounceRefreshesImmediately) {
  auto d = decide_graph_refresh(/*event_fired=*/true, /*currently_dirty=*/false, /*elapsed_ms=*/0, 0);
  EXPECT_TRUE(d.refresh);
  EXPECT_FALSE(d.dirty);
}

// Cold start (last refresh seeded to the epoch) yields a huge elapsed, so the
// first event refreshes immediately rather than waiting out a window.
TEST(RefreshDebounce, ColdStartRefreshesOnFirstEvent) {
  auto d = decide_graph_refresh(/*event_fired=*/true, /*currently_dirty=*/false, /*elapsed_ms=*/9'000'000'000LL, 1000);
  EXPECT_TRUE(d.refresh);
  EXPECT_FALSE(d.dirty);
}

// A spurious tick (no event) past the window with nothing pending does nothing.
TEST(RefreshDebounce, NoEventPastWindowStaysIdle) {
  auto d = decide_graph_refresh(/*event_fired=*/false, /*currently_dirty=*/false, /*elapsed_ms=*/5000, 1000);
  EXPECT_FALSE(d.refresh);
  EXPECT_FALSE(d.dirty);
}
