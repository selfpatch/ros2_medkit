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

#include <unistd.h>
#include "ros2_medkit_gateway/core/fault_trigger_engine.hpp"

#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using ros2_medkit_gateway::FaultTriggerEngine;
using ros2_medkit_gateway::FaultTriggerRule;

namespace {

// Records the report/clear calls the engine emits so edge detection is
// observable without a ROS graph.
struct Recorder {
  std::vector<std::string> reports;  // "app_id/fault_code"
  std::vector<std::string> clears;
};

nlohmann::json make_body(const std::string & data_name, const std::string & op, double threshold,
                         const std::string & fault_code, const std::string & severity) {
  return nlohmann::json{{"data_name", data_name},
                        {"operator", op},
                        {"threshold", threshold},
                        {"fault_code", fault_code},
                        {"severity", severity}};
}

}  // namespace

TEST(FaultTriggerEngineTest, OperatorAndSeverityValidation) {
  for (const char * op : {">", "<", ">=", "<=", "=="}) {
    EXPECT_TRUE(FaultTriggerEngine::valid_operator(op));
  }
  EXPECT_FALSE(FaultTriggerEngine::valid_operator("!="));
  EXPECT_FALSE(FaultTriggerEngine::valid_operator("=>"));

  for (const char * s : {"INFO", "WARNING", "ERROR", "CRITICAL"}) {
    EXPECT_TRUE(FaultTriggerEngine::valid_severity(s));
  }
  EXPECT_FALSE(FaultTriggerEngine::valid_severity("WARN"));
  EXPECT_FALSE(FaultTriggerEngine::valid_severity("fatal"));
}

TEST(FaultTriggerEngineTest, Compare) {
  EXPECT_TRUE(FaultTriggerEngine::compare(5.0, ">", 4.0));
  EXPECT_FALSE(FaultTriggerEngine::compare(4.0, ">", 4.0));
  EXPECT_TRUE(FaultTriggerEngine::compare(4.0, ">=", 4.0));
  EXPECT_TRUE(FaultTriggerEngine::compare(3.0, "<", 4.0));
  EXPECT_TRUE(FaultTriggerEngine::compare(4.0, "<=", 4.0));
  EXPECT_TRUE(FaultTriggerEngine::compare(4.0, "==", 4.0));
  EXPECT_FALSE(FaultTriggerEngine::compare(4.1, "==", 4.0));
}

TEST(FaultTriggerEngineTest, CreateRejectsInvalidBodies) {
  FaultTriggerEngine engine("", nullptr, nullptr, nullptr, nullptr);

  EXPECT_FALSE(engine.create("app", make_body("", ">", 1.0, "F", "ERROR")));    // empty data_name
  EXPECT_FALSE(engine.create("app", make_body("d", "!=", 1.0, "F", "ERROR")));  // bad operator
  EXPECT_FALSE(engine.create("app", make_body("d", ">", 1.0, "", "ERROR")));    // empty fault_code
  EXPECT_FALSE(engine.create("app", make_body("d", ">", 1.0, "F", "WARN")));    // bad severity
  auto no_threshold = nlohmann::json{{"data_name", "d"}, {"operator", ">"}, {"fault_code", "F"}, {"severity", "ERROR"}};
  EXPECT_FALSE(engine.create("app", no_threshold));  // missing threshold

  auto ok = engine.create("app", make_body("d", ">", 1.0, "F", "ERROR"));
  ASSERT_TRUE(ok);
  EXPECT_FALSE(ok->id.empty());
  EXPECT_EQ(ok->app_id, "app");
  EXPECT_TRUE(ok->active);
}

TEST(FaultTriggerEngineTest, CreateRejectsNonexistentDataPoint) {
  // A rule on a data point the app does not expose can never fire (a silently
  // dead alarm); with the app's points enumerable, create() must 400 it.
  auto names = [](const std::string & app_id) -> std::optional<std::vector<std::string>> {
    if (app_id == "tank") {
      return std::vector<std::string>{"level", "trigger", "trigger2"};
    }
    return std::nullopt;  // unknown app: points not enumerable
  };
  FaultTriggerEngine engine("", nullptr, nullptr, nullptr, nullptr, names);

  auto bogus = engine.create("tank", make_body("/plc/bogus_point", ">", 10.0, "F", "ERROR"));
  ASSERT_FALSE(bogus);
  EXPECT_EQ(bogus.error().first, 400);
  EXPECT_NE(bogus.error().second.find("'/plc/bogus_point' does not exist"), std::string::npos);
  EXPECT_NE(bogus.error().second.find("level"), std::string::npos) << "message should list the available points";
  EXPECT_TRUE(engine.list("tank").empty());

  // Existing point passes.
  EXPECT_TRUE(engine.create("tank", make_body("level", ">", 50.0, "F", "ERROR")));

  // Points not enumerable right now (nullopt): creation must not be blocked.
  EXPECT_TRUE(engine.create("other_app", make_body("anything", ">", 1.0, "F2", "ERROR")));
}

TEST(FaultTriggerEngineTest, CreateWithoutEnumeratorSkipsExistenceCheck) {
  // No DataPointNamesFn injected (legacy wiring): behavior unchanged.
  FaultTriggerEngine engine("", nullptr, nullptr, nullptr, nullptr);
  EXPECT_TRUE(engine.create("tank", make_body("whatever", ">", 1.0, "F", "ERROR")));
}

TEST(FaultTriggerEngineTest, ListIsScopedPerApp) {
  FaultTriggerEngine engine("", nullptr, nullptr, nullptr, nullptr);
  ASSERT_TRUE(engine.create("app_a", make_body("x", ">", 1.0, "FA", "ERROR")));
  ASSERT_TRUE(engine.create("app_b", make_body("y", "<", 2.0, "FB", "WARNING")));

  EXPECT_EQ(engine.list("app_a").size(), 1u);
  EXPECT_EQ(engine.list("app_b").size(), 1u);
  EXPECT_TRUE(engine.list("app_c").empty());
}

TEST(FaultTriggerEngineTest, EvaluateFiresOnCrossAndClearsOnRecover) {
  Recorder rec;
  double live = 10.0;  // starts below threshold
  auto fetcher = [&live](const std::string &, const std::string &) -> std::optional<double> {
    return live;
  };
  auto report = [&rec](const std::string & app, const std::string & code, const std::string &, const std::string &) {
    rec.reports.push_back(app + "/" + code);
  };
  auto clear = [&rec](const std::string & app, const std::string & code) {
    rec.clears.push_back(app + "/" + code);
  };

  FaultTriggerEngine engine("", fetcher, report, clear, nullptr);
  ASSERT_TRUE(engine.create("tank", make_body("level", ">", 80.0, "TANK_OVERFILL", "ERROR")));

  // Below threshold: no fault.
  engine.evaluate_once();
  EXPECT_TRUE(rec.reports.empty());
  EXPECT_TRUE(rec.clears.empty());

  // Cross the threshold: fault reported (each poll while crossed = level-triggered).
  live = 95.0;
  engine.evaluate_once();
  engine.evaluate_once();
  EXPECT_EQ(rec.reports.size(), 2u);
  EXPECT_EQ(rec.reports.front(), "tank/TANK_OVERFILL");
  EXPECT_TRUE(rec.clears.empty());

  // Recover: auto-clear exactly once on the falling edge.
  live = 50.0;
  engine.evaluate_once();
  engine.evaluate_once();
  EXPECT_EQ(rec.clears.size(), 1u);
  EXPECT_EQ(rec.clears.front(), "tank/TANK_OVERFILL");
}

TEST(FaultTriggerEngineTest, UnreadableValueHoldsState) {
  Recorder rec;
  std::optional<double> live = 95.0;
  auto fetcher = [&live](const std::string &, const std::string &) -> std::optional<double> {
    return live;
  };
  auto report = [&rec](const std::string & app, const std::string & code, const std::string &, const std::string &) {
    rec.reports.push_back(app + "/" + code);
  };
  auto clear = [&rec](const std::string & app, const std::string & code) {
    rec.clears.push_back(app + "/" + code);
  };

  FaultTriggerEngine engine("", fetcher, report, clear, nullptr);
  ASSERT_TRUE(engine.create("tank", make_body("level", ">", 80.0, "TANK_OVERFILL", "ERROR")));

  engine.evaluate_once();  // crossed -> report
  ASSERT_EQ(rec.reports.size(), 1u);

  // PLC unreadable: must NOT auto-clear a live fault.
  live = std::nullopt;
  engine.evaluate_once();
  EXPECT_TRUE(rec.clears.empty());
}

TEST(FaultTriggerEngineTest, PersistsAcrossReload) {
  // mkstemp, not std::tmpnam: tmpnam is TOCTOU-racy and collides under
  // parallel test runs. The fd is closed immediately - only the unique path
  // matters; the engine reopens it by name.
  char tmpl[] = "/tmp/ftr_store_XXXXXX";
  const int fd = mkstemp(tmpl);
  ASSERT_NE(fd, -1);
  close(fd);
  const std::string path = std::string(tmpl) + "_ftr.json";
  {
    FaultTriggerEngine engine(path, nullptr, nullptr, nullptr, nullptr);
    ASSERT_TRUE(engine.create("tank", make_body("level", ">=", 80.0, "TANK_OVERFILL", "CRITICAL")));
  }
  {
    FaultTriggerEngine reloaded(path, nullptr, nullptr, nullptr, nullptr);
    auto rules = reloaded.list("tank");
    ASSERT_EQ(rules.size(), 1u);
    EXPECT_EQ(rules[0].data_name, "level");
    EXPECT_EQ(rules[0].op, ">=");
    EXPECT_DOUBLE_EQ(rules[0].threshold, 80.0);
    EXPECT_EQ(rules[0].fault_code, "TANK_OVERFILL");
    EXPECT_EQ(rules[0].severity, "CRITICAL");

    // Deleting removes it from the store too.
    EXPECT_TRUE(reloaded.remove("tank", rules[0].id));
  }
  {
    FaultTriggerEngine after_delete(path, nullptr, nullptr, nullptr, nullptr);
    EXPECT_TRUE(after_delete.list("tank").empty());
  }
  std::remove(path.c_str());
}
