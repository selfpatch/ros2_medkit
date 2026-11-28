// Copyright 2026 ahmedsleem109
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

#include <string>
#include <vector>

#include "ros2_medkit_policy_bridge/core/fault_sync.hpp"

using ros2_medkit_policy_bridge::core::FaultSync;

namespace {

const std::vector<std::string> kCodes{"P_CHUNK_DISAGREEMENT", "P_SLOW_INFERENCE"};

// Send everything that is due, as the node does while the FaultManager is up.
std::vector<FaultSync::Report> flush(FaultSync & sync) {
  auto due = sync.due();
  for (const auto & r : due) {
    sync.mark_sent(r);
  }
  return due;
}

}  // namespace

// A bridge restarted while its fault was CONFIRMED has no memory of it: the
// first sync passes every code, so a stale fault heals.
TEST(FaultSync, StartPassesEveryCodeOnce) {
  FaultSync sync(kCodes);
  auto first = flush(sync);
  ASSERT_EQ(first.size(), 2u);
  EXPECT_EQ(first[0].code, "P_CHUNK_DISAGREEMENT");
  EXPECT_FALSE(first[0].raise);
  EXPECT_EQ(first[1].code, "P_SLOW_INFERENCE");
  EXPECT_FALSE(first[1].raise);
  EXPECT_TRUE(sync.due().empty());
}

TEST(FaultSync, RaiseAndPassAreSentOnceEach) {
  FaultSync sync(kCodes);
  flush(sync);
  sync.set("P_SLOW_INFERENCE", true, 2, "slow");
  auto raise = flush(sync);
  ASSERT_EQ(raise.size(), 1u);
  EXPECT_TRUE(raise[0].raise);
  EXPECT_EQ(raise[0].severity, 2);
  EXPECT_EQ(raise[0].description, "slow");
  sync.set("P_SLOW_INFERENCE", true, 2, "slow again");  // same state: nothing new
  EXPECT_TRUE(sync.due().empty());
  sync.set("P_SLOW_INFERENCE", false, 0, "");
  auto pass = flush(sync);
  ASSERT_EQ(pass.size(), 1u);
  EXPECT_FALSE(pass[0].raise);
}

// While the FaultManager is missing nothing is marked sent, so the raise is
// still due when it appears.
TEST(FaultSync, RaiseWhileUnreachableStaysDue) {
  FaultSync sync(kCodes);
  flush(sync);
  sync.set("P_CHUNK_DISAGREEMENT", true, 1, "unsure");
  ASSERT_EQ(sync.due().size(), 1u);
  ASSERT_EQ(sync.due().size(), 1u);  // reading due() does not consume it
  auto sent = flush(sync);
  ASSERT_EQ(sent.size(), 1u);
  EXPECT_TRUE(sent[0].raise);
}

// A FaultManager that restarted may have lost the raise: forget_sent() makes
// the active raise and the passes due again.
TEST(FaultSync, ForgetSentResendsTheCurrentState) {
  FaultSync sync(kCodes);
  flush(sync);
  sync.set("P_CHUNK_DISAGREEMENT", true, 1, "unsure");
  flush(sync);
  sync.forget_sent();
  auto again = flush(sync);
  ASSERT_EQ(again.size(), 2u);
  EXPECT_TRUE(again[0].raise);
  EXPECT_EQ(again[0].description, "unsure");
  EXPECT_FALSE(again[1].raise);
}

// Only the latest state is kept: a fault that came and went while nothing
// could be sent leaves nothing to send.
TEST(FaultSync, RaiseThenPassWhileUnreachableSendsNothing) {
  FaultSync sync(kCodes);
  flush(sync);
  sync.set("P_CHUNK_DISAGREEMENT", true, 1, "unsure");
  sync.set("P_CHUNK_DISAGREEMENT", false, 0, "");
  EXPECT_TRUE(sync.due().empty());
}

TEST(FaultSync, UnknownCodeIsIgnored) {
  FaultSync sync(kCodes);
  flush(sync);
  sync.set("OTHER", true, 1, "x");
  EXPECT_TRUE(sync.due().empty());
  EXPECT_FALSE(sync.raised("OTHER"));
  sync.mark_sent({"OTHER", true, 1, "x"});
  EXPECT_TRUE(sync.due().empty());
}
