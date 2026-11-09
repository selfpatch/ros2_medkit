// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/fault_code_lanes.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace {

// A fake service: records what was sent and keeps each call's ``done``.
struct Wire {
  std::vector<std::string> sent;
  std::vector<std::function<void()>> pending;
  FaultCodeLanes::Send call(const std::string & name) {
    return [this, name](std::function<void()> done) {
      sent.push_back(name);
      pending.push_back(std::move(done));
    };
  }
};

// An alarm that came and went inside one publishing interval: the clear must
// not go out before the report was answered, or it can overtake it.
TEST(FaultCodeLanes, ClearWaitsForTheReportOfTheSameCode) {
  FaultCodeLanes lanes;
  Wire wire;
  lanes.submit("IMM_02_ALARM", wire.call("report"));
  lanes.submit("IMM_02_ALARM", wire.call("clear"));
  EXPECT_EQ(wire.sent, std::vector<std::string>{"report"});
  wire.pending[0]();
  EXPECT_EQ(wire.sent, (std::vector<std::string>{"report", "clear"}));
  wire.pending[1]();
  EXPECT_EQ(lanes.busy_count(), 0u);
}

TEST(FaultCodeLanes, OtherCodesDoNotWait) {
  FaultCodeLanes lanes;
  Wire wire;
  lanes.submit("A", wire.call("report A"));
  lanes.submit("B", wire.call("report B"));
  EXPECT_EQ(wire.sent, (std::vector<std::string>{"report A", "report B"}));
  EXPECT_EQ(lanes.busy_count(), 2u);
}

TEST(FaultCodeLanes, OrderIsKeptAcrossSeveralCalls) {
  FaultCodeLanes lanes;
  Wire wire;
  for (const char * name : {"report 1", "clear 1", "report 2", "clear 2"}) {
    lanes.submit("X", wire.call(name));
  }
  for (size_t i = 0; i < 4; ++i) {
    ASSERT_EQ(wire.sent.size(), i + 1);
    wire.pending[i]();
  }
  EXPECT_EQ(wire.sent, (std::vector<std::string>{"report 1", "clear 1", "report 2", "clear 2"}));
  EXPECT_EQ(lanes.busy_count(), 0u);
}

// fault_manager restarted under a call: the code is not held forever, and the
// late answer to the abandoned call does not release the one now in flight.
TEST(FaultCodeLanes, AnUnansweredCallStopsHoldingItsCodeAfterStaleAfter) {
  FaultCodeLanes lanes(std::chrono::seconds(5));
  Wire wire;
  const auto t0 = FaultCodeLanes::Clock::now();
  lanes.submit("X", wire.call("report"), t0);
  lanes.submit("X", wire.call("clear"), t0 + std::chrono::seconds(1));
  EXPECT_EQ(wire.sent, std::vector<std::string>{"report"});
  lanes.submit("X", wire.call("report again"), t0 + std::chrono::seconds(6));
  EXPECT_EQ(wire.sent, (std::vector<std::string>{"report", "clear"}));
  wire.pending[0]();  // late answer to the abandoned report
  EXPECT_EQ(wire.sent.size(), 2u);
  wire.pending[1]();
  EXPECT_EQ(wire.sent, (std::vector<std::string>{"report", "clear", "report again"}));
}

TEST(FaultCodeLanes, ASynchronousAnswerDrainsTheQueue) {
  FaultCodeLanes lanes;
  std::vector<std::string> sent;
  auto immediate = [&sent](const std::string & name) {
    return [&sent, name](std::function<void()> done) {
      sent.push_back(name);
      done();
    };
  };
  lanes.submit("X", immediate("report"));
  lanes.submit("X", immediate("clear"));
  EXPECT_EQ(sent, (std::vector<std::string>{"report", "clear"}));
  EXPECT_EQ(lanes.busy_count(), 0u);
}

}  // namespace
}  // namespace ros2_medkit_gateway
