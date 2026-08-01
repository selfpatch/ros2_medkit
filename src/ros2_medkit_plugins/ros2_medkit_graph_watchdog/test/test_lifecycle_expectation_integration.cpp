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
//
// Drives the REAL mechanism: the detector's own LifecycleExpectationTracker state, a
// snapshot-driven presence diff (via set_apps()), and a REAL ReliabilityGate to arm the
// global bringup grace and to feed the detector its lifecycle labels via
// ReliabilityGate::lifecycle_state_of() - raising/clearing through a real ReportFault
// service round-trip to a fake fault_manager. NOT a full-gateway e2e; this proves the
// detector end to end within its own scope (the e2e tier proves the real
// lifecycle-transition pipeline).
//
// Two ordering rules are load-bearing here, or the test silently proves nothing:
//   1. Arm the global gate FIRST. The aggregated raise goes out under
//      source_id="graph_watchdog", unknown to the WarmupTracker, so allows_raise() takes
//      the global-bringup path and stays suppressed until a non-empty gate.update() has
//      run and the global grace has elapsed: gate.update(snapshot, warmup) then
//      gate.update(snapshot, warmup + 3).
//   2. Inject the lifecycle label AFTER the last gate.update(), and never call
//      gate.update() again. LifecycleWatcher::update() rebuilds its tracked set from
//      snapshot.apps SERVICES and ERASES any id it cannot find a get-state path for; the
//      fixture's apps carry no services, so any gate.update() after
//      set_lifecycle_state_for_test() wipes the injected label (state_of() then returns
//      nullopt = no expectation = no raise).
//
// Alongside the fixture, plain configure()-level TESTs pin the config contract (unknown
// keys, validation warnings, the prune clamp): this binary links the detector .cpp, so
// REGISTER_DETECTOR is in-image and the registry hands out instances without any ROS
// scaffolding.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/lifecycle_expectation_tracker.hpp"  // kDefaultAbsenceGrace
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_graph_watchdog::DetectorContext;
using ros2_medkit_graph_watchdog::DetectorMode;
using ros2_medkit_graph_watchdog::kDefaultAbsenceGrace;
using ros2_medkit_graph_watchdog::ReliabilityGate;
using ros2_medkit_graph_watchdog::graph_fault_codes::kNodeInactive;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
using namespace std::chrono_literals;

namespace {
// The detector class is file-local in lifecycle_expectation_detector.cpp but
// self-registers via REGISTER_DETECTOR, which runs when that .cpp is linked into this
// test. Pull an instance from the registry (no production factory needed).
std::unique_ptr<ros2_medkit_graph_watchdog::Detector> make_lifecycle_expectation() {
  for (auto & d : ros2_medkit_graph_watchdog::DetectorRegistry::instance().create_all()) {
    if (d->id() == "lifecycle_expectation") {
      return std::move(d);
    }
  }
  return nullptr;
}
// Aggregated fault source: no Component in the test snapshot, so graph_source_id()
// falls back to this literal (see aggregated_fault.hpp).
constexpr const char * kGraphSource = "graph_watchdog";
constexpr int kWarmupCycles = 3;
// Mirrors the detector's kUnmeasuredHoldTicks (file-local there by design): consecutive
// matched-but-never-read ticks a required node may hold back the clear before the typo
// unblock releases it.
constexpr int kHoldTicks = 60;

/// Captures rcutils log output for as long as it is alive and restores the console handler on every
/// exit path. Leaving the process-global handler installed would swallow the log output of every
/// later case in this binary, so the restore cannot be left to the success path.
///
/// One implementation for the whole file. The rcutils handler CONTRACT hands over a caller-supplied
/// format string and a va_list - it is not a choice the handler makes - so `vsnprintf` here is the
/// one place in this file that cannot use a literal format.
///
/// The handler is a plain C function pointer with no user-data slot, so the live capture is reached
/// through a file-static. It is written by the test thread and read by whichever thread logs,
/// hence the atomic.
class LogCapture {
 public:
  LogCapture() {
    active().store(this);
    rcutils_logging_set_output_handler(&LogCapture::handler);
  }
  ~LogCapture() {
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
    active().store(nullptr);
  }
  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;
  LogCapture(LogCapture &&) = delete;
  LogCapture & operator=(LogCapture &&) = delete;

  /// How many captured lines contain `needle`.
  int count(const std::string & needle) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(std::count_if(lines_.begin(), lines_.end(), [&needle](const std::string & line) {
      return line.find(needle) != std::string::npos;
    }));
  }

 private:
  static std::atomic<LogCapture *> & active() {
    static std::atomic<LogCapture *> current{nullptr};
    return current;
  }

  static void handler(const rcutils_log_location_t * /*location*/, int /*severity*/, const char * /*name*/,
                      rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args) {
    char buf[1024];
    va_list copy;
    va_copy(copy, *args);
    // The format string arrives from the logging call site through the handler signature, so there
    // is no literal to write here and no other way to reach the arguments than the va_list. The
    // build runs -Werror=format=2; GCC exempts va_list-taking formatters from -Wformat-nonliteral,
    // clang does not, so the CI clang-tidy job fails on this line alone. A pragma rather than a
    // lint-suppression comment, because the diagnostic is raised as an ERROR and clang-tidy does
    // not honour suppression comments for those. Scoped to the single call.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    vsnprintf(buf, sizeof(buf), format, copy);
#pragma GCC diagnostic pop
    va_end(copy);
    LogCapture * capture = active().load();
    if (capture == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lk(capture->mutex_);
    capture->lines_.emplace_back(buf);
  }

  mutable std::mutex mutex_;
  std::vector<std::string> lines_;
};

/// Adapts a body that wants the request and the response by reference to the shape rclcpp requires.
///
/// create_service deduces the callback by comparing its argument tuple against
/// `std::function<void(std::shared_ptr<Request>, std::shared_ptr<Response>)>` EXACTLY, so a handler
/// cannot declare those parameters by reference however little it needs a copy of them. Stating
/// that shape once here keeps it out of the fake service below (and keeps the by-value shared_ptr
/// copies out of clang-tidy's sight - see test_param_drift_integration.cpp's identical adapter).
template <typename ServiceT, typename BodyT>
std::function<void(std::shared_ptr<typename ServiceT::Request>, std::shared_ptr<typename ServiceT::Response>)>
service_callback(BodyT body) {
  return [body = std::move(body)](std::shared_ptr<typename ServiceT::Request> req,
                                  std::shared_ptr<typename ServiceT::Response> resp) {
    body(*req, *resp);
  };
}

/// Snapshot builder for the configure()-level TESTs below (the fixture has its own).
IntrospectionInput snapshot_of(const std::vector<std::string> & ids) {
  IntrospectionInput snapshot;
  for (const auto & id : ids) {
    App a;
    a.id = id;
    a.bound_fqn = "/" + id;
    snapshot.apps.push_back(a);
  }
  return snapshot;
}

/// One app under an EXPLICIT (id, fqn), for the identity-churn sweep below: every
/// respawn arrives as a brand-new fqn under a namespace, matched by the same bare entry
/// through its leaf.
IntrospectionInput namespaced_snapshot_of(const std::string & ns, const std::string & leaf) {
  IntrospectionInput snapshot;
  App a;
  a.id = ns + "_" + leaf;
  a.bound_fqn = "/" + ns + "/" + leaf;
  snapshot.apps.push_back(a);
  return snapshot;
}
}  // namespace

class LifecycleExpectationIntegrationTest : public ::testing::Test {
 protected:
  // Must run before the FIRST test's fixture is constructed - see the identical
  // rationale in test_param_drift_integration.cpp.
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override {
    gateway_ = std::make_shared<rclcpp::Node>("le_it_gateway");
    sink_ = std::make_shared<rclcpp::Node>("le_it_sink");
    srv_ = sink_->create_service<ReportFault>(
        "/fault_manager/report_fault",
        service_callback<ReportFault>([this](ReportFault::Request & req, ReportFault::Response & resp) {
          {
            std::lock_guard<std::mutex> lk(mtx_);
            received_.push_back(req);
          }
          resp.accepted = true;  // ReportFault.srv response field is `bool accepted`
        }));
    client_ = gateway_->create_client<ReportFault>("/fault_manager/report_fault");
    exec_.add_node(gateway_);
    exec_.add_node(sink_);
    spin_ = std::thread([this]() {
      exec_.spin();
    });
    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  void TearDown() override {
    exec_.cancel();
    if (spin_.joinable()) {
      spin_.join();
    }
    exec_.remove_node(gateway_);
    exec_.remove_node(sink_);
    client_.reset();
    srv_.reset();
    sink_.reset();
    gateway_.reset();
  }

  DetectorContext make_ctx(DetectorMode mode, ReliabilityGate * gate) {
    DetectorContext ctx;
    ctx.gateway_node = gateway_.get();
    ctx.node_mutex = &node_mutex_;
    ctx.mode = mode;
    ctx.gate = gate;  // the detector reads ctx.gate->lifecycle_state_of() - must be real, not null
    ctx.fault_client = client_;
    ctx.snapshot = &snapshot_;  // tick() early-returns without a snapshot; each test seeds set_apps()
    return ctx;
  }

  // (Re)populate the owned entity snapshot the detector reads each tick - the
  // fixture's presence-diff seam. components stays empty, so the aggregated fault's
  // source_id falls back to the literal "graph_watchdog".
  void set_apps(const std::vector<std::string> & ids) {
    snapshot_.apps.clear();
    for (const auto & id : ids) {
      App a;
      a.id = id;
      a.bound_fqn = "/" + id;
      snapshot_.apps.push_back(a);
    }
  }

  // Seed apps with EXPLICIT (id, fqn) pairs - to reproduce App::id instability, where the
  // gateway gives a bare-name-colliding node a namespaced id while its fqn stays stable.
  void set_apps_raw(const std::vector<std::pair<std::string, std::string>> & id_fqns) {
    snapshot_.apps.clear();
    for (const auto & [id, fqn] : id_fqns) {
      App a;
      a.id = id;
      a.bound_fqn = fqn;
      snapshot_.apps.push_back(a);
    }
  }

  // Seed the snapshot with ONE managed lifecycle app under an EXPLICIT (id, fqn): the
  // GetState + ChangeState services are what make the gate's internal lifecycle tracking
  // keep the id across gate.update() calls (the service-less apps above are erased by
  // every update - ordering rule 2). No live node answers at `fqn`, so seeds fail into
  // the unknown (empty) label.
  void set_managed_app(const std::string & id, const std::string & fqn) {
    snapshot_.apps.clear();
    ros2_medkit_gateway::ServiceInfo get_state;
    get_state.full_path = fqn + "/get_state";
    get_state.type = "lifecycle_msgs/srv/GetState";
    ros2_medkit_gateway::ServiceInfo change_state;
    change_state.full_path = fqn + "/change_state";
    change_state.type = "lifecycle_msgs/srv/ChangeState";
    App a;
    a.id = id;
    a.bound_fqn = fqn;
    a.services = {get_state, change_state};
    snapshot_.apps.push_back(a);
  }

  // Arm the global bringup grace the aggregated "graph_watchdog" source needs (see the
  // file-level ordering-rule doc comment, rule 1). Must be the LAST gate.update() call
  // in a test before any set_lifecycle_state_for_test() injection (rule 2).
  void arm_global_grace(ReliabilityGate & gate) {
    gate.update(snapshot_, 5);
    gate.update(snapshot_, 5 + kWarmupCycles);
  }

  // How many matching faults are currently in the log (for absence checks + snapshots).
  std::size_t count_faults(const std::string & source_id, uint8_t event_type) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::size_t n = 0;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == kNodeInactive && r.event_type == event_type) {
        ++n;
      }
    }
    return n;
  }

  // EVERY request the fake fault_manager has received, of ANY source/code/event kind.
  // The zero-config claim is "the fault_manager hears nothing at all", so the instrument
  // has to count everything, not just the matching FAILED events.
  std::size_t count_all_requests() {
    std::lock_guard<std::mutex> lk(mtx_);
    return received_.size();
  }

  // Description of the LAST matching FAILED request (empty if none arrived).
  std::string last_failed_description(const std::string & source_id) {
    std::lock_guard<std::mutex> lk(mtx_);
    std::string desc;
    for (const auto & r : received_) {
      if (r.source_id == source_id && r.fault_code == kNodeInactive &&
          r.event_type == ReportFault::Request::EVENT_FAILED) {
        desc = r.description;
      }
    }
    return desc;
  }

  // True if any recorded FAILED for `source_id` carries a description mentioning `needle`.
  bool any_failed_desc_contains(const std::string & source_id, const std::string & needle) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto & r : received_) {
      if (r.source_id != source_id || r.fault_code != kNodeInactive ||
          r.event_type != ReportFault::Request::EVENT_FAILED) {
        continue;
      }
      if (r.description.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  // Tick until a NEW matching fault appears beyond `baseline_count` (arrived AFTER the
  // trigger), or timeout. NEVER calls gate.update() - only the detector is ticked, per
  // ordering rule 2 (a further gate.update() would erase the injected lifecycle label).
  bool poll_for_new(const std::string & source_id, uint8_t event_type, std::size_t baseline_count,
                    ros2_medkit_graph_watchdog::Detector & det, DetectorContext & ctx) {
    for (int i = 0; i < 130; ++i) {
      det.tick(ctx);
      std::this_thread::sleep_for(50ms);
      if (count_faults(source_id, event_type) > baseline_count) {
        return true;
      }
    }
    return false;
  }

  // Wait (WITHOUT ticking) until at least `target` matching requests have arrived - for
  // the exactly-N assertions, where further ticks would legitimately emit more.
  bool wait_for_count(const std::string & source_id, uint8_t event_type, std::size_t target) {
    for (int i = 0; i < 150; ++i) {
      if (count_faults(source_id, event_type) >= target) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return count_faults(source_id, event_type) >= target;
  }

  rclcpp::Node::SharedPtr gateway_, sink_;
  rclcpp::Service<ReportFault>::SharedPtr srv_;
  rclcpp::Client<ReportFault>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::thread spin_;
  std::mutex mtx_, node_mutex_;
  std::vector<ReportFault::Request> received_;
  IntrospectionInput snapshot_;  // owned entity snapshot the detector reads each tick
};

// The full story: "a" present, armed via a REAL wired gate (arming the global bringup
// grace the aggregated "graph_watchdog" source needs), then injected as
// lifecycle-inactive (AFTER the last gate.update(), per ordering rule 2) - ticking the
// detector past grace raises GRAPH_NODE_INACTIVE naming "a"; driving "a" to "active"
// clears it.
TEST_F(LifecycleExpectationIntegrationTest, RequiredNodeStuckInactivePastGraceRaisesThenActiveClears) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Inject the label AFTER the last gate.update() (rule 2) - never call gate.update()
  // again from here on.
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "a"));

  // "a" reaches active -> the aggregate clears.
  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx));
}

// Positive control: a required node that is active from the moment it is observed must
// never raise, however many ticks pass.
TEST_F(LifecycleExpectationIntegrationTest, RequiredNodeActiveFromArmingNeverRaises) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "a required node reported active from the start was flagged GRAPH_NODE_INACTIVE";
}

// Absence control: once "a" vanishes from the snapshot entirely, GRAPH_NODE_INACTIVE
// must clear (not stay raised, not re-raise) - a vanished node belongs to the presence
// class, not to lifecycle_expectation. Absence is driven via set_apps({}), NEVER
// gate.update({}) (which would reset the gate's own graph_seen_ global-bringup marker,
// see the file-level ordering-rule doc comment).
TEST_F(LifecycleExpectationIntegrationTest, AbsenceAfterRaiseClearsNotNodeDeathsDomain) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx));

  // "a" vanishes from the graph entirely (a crash, not a lifecycle transition).
  set_apps({});
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "GRAPH_NODE_INACTIVE did not clear once the required node vanished from the graph - "
         "absence must be left to GRAPH_NODE_DISAPPEARED, not kept flagged by lifecycle_expectation";
}

// Regression: App::id is recomputed each sweep and gets namespaced on a same-bare-name
// collision, so a bare-name require_active entry must still match a live node via its stable
// fqn leaf - otherwise the check silently stops on a multi-robot graph. Here the app carries a
// namespaced id ("robot1_a") but a stable fqn "/robot1/a"; require_active: ["a"] (bare) must
// match it by leaf and still enforce the expectation.
TEST_F(LifecycleExpectationIntegrationTest, BareNameRequireActiveMatchesNamespacedAppByLeaf) {
  set_apps_raw({{"robot1_a", "/robot1/a"}});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Lifecycle state is keyed by the node's current App::id ("robot1_a"); the require_active
  // entry "a" matches it via the fqn leaf.
  gate.set_lifecycle_state_for_test("robot1_a", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a bare-name require_active entry stopped matching a node whose App::id was namespaced by a "
         "collision - the check must match via the stable fqn leaf instead of silently stopping";
}

// A FULL-FQN entry must match through the fqn arm (id != fqn), not by luck of the App::id.
// The app carries an id that equals neither the entry nor its leaf, so only the fqn
// comparison can bind them.
TEST_F(LifecycleExpectationIntegrationTest, FullFqnRequireActiveMatchesByFqnWhenIdDiffers) {
  set_apps_raw({{"nsb", "/ns/b"}});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"/ns/b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("nsb", "inactive");
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a full-FQN require_active entry did not match an app whose id differs from its fqn";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/ns/b"));
}

// The shipped default: an empty detector config checks nothing and CONTACTS nothing. The
// instrument counts every request of any kind, so a stray clear-spam (or any other
// chatter) fails this, not just a wrong raise. The inactive label makes the claim sharp:
// even with a would-be-offending node in view, an unconfigured detector stays silent.
TEST_F(LifecycleExpectationIntegrationTest, EmptyConfigNeverContactsTheFaultManager) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json::object());
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(200ms);  // let any in-flight request land before counting
  EXPECT_EQ(count_all_requests(), 0u)
      << "an unconfigured lifecycle_expectation sent the fault_manager a request; the zero-config "
         "default must emit neither raises nor clears";
}

// Same claim for the explicit `require_active: []` spelling of "nothing configured".
TEST_F(LifecycleExpectationIntegrationTest, ExplicitEmptyRequireActiveNeverContactsTheFaultManager) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array()}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(count_all_requests(), 0u) << "require_active: [] must behave exactly like no config at all";
}

// An entry that matches a present node with NO tracked lifecycle state is probably a typo
// (or a plain node). The warning must reach the log after kUnmanagedWarnTicks consecutive
// such ticks, exactly once per entry per configuration - and a reconfigure that removes
// and re-adds the entry must warn AGAIN, because that is exactly when the operator wants
// to hear the entry still names nothing managed.
TEST_F(LifecycleExpectationIntegrationTest, UnmanagedEntryWarnsOncePerConfiguration) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  const nlohmann::json config{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}};
  det->configure(config);
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // Deliberately NO set_lifecycle_state_for_test: "a" is matched but has no tracked
  // lifecycle state, which is the unmanaged case under test.

  const std::string needle = "require_active entry 'a' has matched a present node";
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1) << "expected the unmanaged-entry warning exactly once over ten ticks "
                                     "(0 = never surfaced, >1 = the once-per-entry guard is gone)";

  // Remove the entry, then re-add it: the warn latch is scoped to the CURRENT config, so
  // the re-added entry must warn again.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array()}});
  det->configure(config);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 2) << "an entry removed and re-added by reconfigure() never warned again - "
                                     "the latch must reset with the config";
}

// Withheld-clear guard, the never-measured half (see the detector's withheld-clear design
// note). A gateway restart re-instantiates the detector with everything unread; the fault
// it raised BEFORE the restart is still in the store. Emitting the level-triggered clear
// before a single label has been read would spuriously HEAL a still-real inactive fault -
// so through the bounded hold window the detector must emit NOTHING, and the clear must
// flow only after a label is actually read.
TEST_F(LifecycleExpectationIntegrationTest, RestartStandInWithholdsTheClearUntilALabelIsRead) {
  set_apps({"a"});

  // Phase 1 (pre-restart): a configured + labeled run raises the prior fault.
  ReliabilityGate gate1(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate1);
  auto det1 = make_lifecycle_expectation();
  ASSERT_TRUE(det1);
  det1->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx1 = make_ctx(DetectorMode::Raise, &gate1);
  gate1.set_lifecycle_state_for_test("a", "inactive");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det1, ctx1))
      << "phase 1 never raised, so phase 2 would prove nothing";
  std::this_thread::sleep_for(200ms);  // let phase-1 in-flight requests land before baselining

  // Phase 2 (restart stand-in): fresh gate, fresh detector, NO labels read yet.
  ReliabilityGate gate2(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate2);
  auto det2 = make_lifecycle_expectation();
  ASSERT_TRUE(det2);
  det2->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx2 = make_ctx(DetectorMode::Raise, &gate2);

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kHoldTicks; ++i) {
    det2->tick(ctx2);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);  // any withheld-but-actually-sent clear must have landed by now
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a freshly restarted detector cleared GRAPH_NODE_INACTIVE before reading a single lifecycle "
         "label - the restart spuriously heals a still-real fault";

  // The label is finally read (still-inactive would re-raise; active releases the clear).
  gate2.set_lifecycle_state_for_test("a", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det2, ctx2))
      << "once the label IS read and healthy, the clear must flow";
}

// Withheld-clear guard, the typo unblock: an entry that matches a node whose label is
// NEVER read - an unmanaged node, or a typo that happens to match - may hold the clear
// back only for the bounded hold window. Past it the clear must flow, or one such entry
// blocks healing forever.
TEST_F(LifecycleExpectationIntegrationTest, NeverReadNodePastTheHoldBoundStopsBlockingTheClear) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // NO label injection, ever: "a" stays unread for the whole test.

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear stayed withheld past the bounded hold - an unmanaged or typo'd entry must not "
         "block healing forever";
}

// Withheld-clear guard, the window that actually matters: a restart whose labels seed
// FAST. The plugin seeds lifecycle labels BEFORE it ticks the detectors, so on a
// responsive stack the first tick of a fresh detector already reads the real label -
// "unread" is the rare case, not the common one. What IS fresh after a restart is the
// violation streak, and while it climbs back through grace the tracker reports nothing
// affected. Emitting the level-triggered clear there asserts "nothing is stuck" about a
// node this detector's own last read found stuck, and heals the fault the restart was
// supposed to preserve.
TEST_F(LifecycleExpectationIntegrationTest, RestartStandInWithholdsTheClearWhileTheNodeReadsNotActiveBelowGrace) {
  set_apps({"a"});

  // Phase 1 (pre-restart): a configured + labeled run raises the prior fault.
  ReliabilityGate gate1(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate1);
  auto det1 = make_lifecycle_expectation();
  ASSERT_TRUE(det1);
  det1->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx1 = make_ctx(DetectorMode::Raise, &gate1);
  gate1.set_lifecycle_state_for_test("a", "inactive");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det1, ctx1))
      << "phase 1 never raised, so phase 2 would prove nothing";
  std::this_thread::sleep_for(200ms);  // let phase-1 in-flight requests land before baselining

  // Phase 2 (restart stand-in): fresh gate, fresh detector - but the label is seeded
  // BEFORE the first tick, and the node is still stuck exactly where it was.
  ReliabilityGate gate2(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate2);
  auto det2 = make_lifecycle_expectation();
  ASSERT_TRUE(det2);
  constexpr int kGrace = 4;
  det2->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kGrace}});
  auto ctx2 = make_ctx(DetectorMode::Raise, &gate2);
  gate2.set_lifecycle_state_for_test("a", "inactive");

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < kGrace; ++i) {  // the whole below-grace window, every tick measured stuck
    det2->tick(ctx2);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // any emitted request must have landed by now
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the restarted detector cleared GRAPH_NODE_INACTIVE while its own reads said the node was "
         "still not active - a young streak is not health";
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the raise must still wait for grace; only the clear is withheld";

  // Past grace the raise resumes, so the hold is not a silence that never ends.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det2, ctx2))
      << "the streak never reached grace + 1, so the still-real fault was never re-raised";
}

// The grace contract, through the REAL config path: "grace" is the number of consecutive
// not-active ticks a required NODE tolerates. An operator combining the two documented
// require_active forms (bare name = fleet-wide, full FQN = pin one robot) has one node
// named by two entries, and the detector pushes one match per (entry, node) - so counting
// per match raises after ceil((grace+1)/2) ticks instead of grace+1, a transient false
// positive on a node that is simply still coming up.
TEST_F(LifecycleExpectationIntegrationTest, TwoEntriesMatchingOneNodeDoNotHalveTheConfiguredGrace) {
  set_apps({"a"});  // App::id "a", fqn "/a" - matched by BOTH entries below
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  constexpr int kGrace = 4;
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "/a"})}, {"grace", kGrace}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < kGrace; ++i) {  // exactly the tolerated window
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "the node was reported inactive INSIDE its grace because two require_active entries named "
         "it - the streak must advance once per node per tick, not once per match";

  // And the expectation is still enforced once the streak really does pass grace.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "the raise never came at all - de-duplicating the matches must not disable the check";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a"));
}

// The churn story the absence grace exists for, driven through the detector: a stuck node
// blinks out of the snapshot and comes back with the label LifecycleWatcher seeds when its
// GetState has not answered yet (""). Treating that re-seed tick as a healthy read restarts
// the violation count, so a node that blinks once every few ticks never accumulates
// grace + 1 consecutive stuck ticks and is never reported at all.
TEST_F(LifecycleExpectationIntegrationTest, BlinkAndUnreadReseedDoNotRestartTheViolationCount) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 3}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // Tick-exact on purpose. "It raises eventually" is true whether or not the streak
  // survives - a reset only delays the raise - so an unbounded poll would pass against a
  // detector that restarts the count on every blink. The claim is that the streak reaches
  // grace + 1 on THIS tick, so the run is counted out and nothing ticks afterwards.
  gate.set_lifecycle_state_for_test("a", "inactive");
  det->tick(ctx);  // stuck 1
  det->tick(ctx);  // stuck 2

  set_apps({});    // blink: out of the snapshot for one tick
  det->tick(ctx);  // absent 1 - within the absence grace, streak preserved

  set_apps({"a"});                             // back, but the re-seed has not answered yet
  gate.set_lifecycle_state_for_test("a", "");  // exactly what a missed GetState seed leaves
  det->tick(ctx);                              // unread: absence-like, streak preserved
  gate.set_lifecycle_state_for_test("a", "inactive");
  det->tick(ctx);  // stuck 3 == grace, not past it yet

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // stuck 4 > grace - the raise is due on this tick and no later
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1))
      << "a node stuck through a blink and an unread re-seed was not reported on the tick its streak "
         "passed grace - the blink or the re-seed restarted its violation count, which is how a stuck "
         "node on a churning graph stays silent forever";
}

// Guard bookkeeping is per NODE, not per (entry, node) pair. An operator combining the
// documented bare-name and pinned-FQN forms has one node named by two entries; burning
// its unread hold twice a tick halves the documented bound and lets the clear out at half
// the horizon the design promises.
TEST_F(LifecycleExpectationIntegrationTest, TwoEntriesMatchingOneNodeDoNotHalveTheUnreadHold) {
  set_apps({"a"});  // App::id "a", fqn "/a" - matched by BOTH entries below
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "/a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  // NO label injection, ever: "/a" stays unread, so only the hold can release the clear.

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kHoldTicks / 2 + 5; ++i) {  // comfortably past a halved hold, short of the real one
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the unread hold released at half its documented length because two entries named the same "
         "node - the guard must count per node, not per match";

  // The bound itself still applies: past it, the clear flows.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the hold never released at all - it must stay bounded";
}

// Mixed set, direction 1: node "a" was raised and has now healed, while sibling "b" has
// never been measured. Within the hold the clear waits - a run in which "b" was never
// read is not a run in which "b" was found healthy.
TEST_F(LifecycleExpectationIntegrationTest, MixedSetClearWaitsForTheUnreadSiblingWithinTheHold) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // "b" is never read in this test
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, 0, *det, ctx))
      << "the raise never happened, so the heal leg would prove nothing";

  gate.set_lifecycle_state_for_test("a", "active");  // the raised node heals
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 20; ++i) {  // well inside "b"'s hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the clear flowed while a required sibling had never been measured once";

  gate.set_lifecycle_state_for_test("b", "active");  // now everything really is measured healthy
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "once every required node has been read healthy the clear must flow";
}

// Mixed set, direction 2: the hold is a bound on the WAIT, not a promise that every node
// was measured. "b"'s budget burns on every matched tick, including the ticks a raise
// about "a" was flowing, so a raise that outlives the hold leaves the clear free to flow
// the moment "a" heals - deliberately, or one unmanaged sibling would block healing for
// as long as the raise lasted.
TEST_F(LifecycleExpectationIntegrationTest, MixedSetClearFlowsOnceTheUnreadSiblingsHoldIsSpent) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // "b" is never read in this test
  for (int i = 0; i < kHoldTicks + 5; ++i) {           // the raise outlives "b"'s hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, 1))
      << "the raise never happened, so the heal leg would prove nothing";

  gate.set_lifecycle_state_for_test("a", "active");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear never came: past its hold an unread sibling must stop blocking, or one unmanaged "
         "entry would keep a healed fault raised for as long as the raise lasted";
}

// The guard's "a label was read" latch is keyed by fqn and outlives the node: a process
// that dies and respawns under the same name (systemd respawn) re-enters with the latch
// still set from the dead incarnation, so its brand-new, never-measured labels cannot
// withhold anything. That is the shipped behaviour, and it is what makes the streak-based
// half of the guard load-bearing rather than decorative.
TEST_F(LifecycleExpectationIntegrationTest, ReadLatchSurvivesARespawnUnderTheSameFqn) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // the latch is set by this read
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the healthy baseline never cleared, so nothing below can be attributed to the respawn";

  set_apps({});  // the process dies
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  // Respawned under the same fqn with an unseeded label ("" is what LifecycleWatcher
  // seeds for a tracked node whose GetState has not answered yet).
  set_apps({"a"});
  gate.set_lifecycle_state_for_test("a", "");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 10; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before + 1))
      << "the clear was withheld after the respawn, so the read latch is per-incarnation after all - "
         "the design note and the docs say it survives, and the streak guard is written on that "
         "assumption";
}

// The other half of the same story: the surviving latch does NOT leave a respawned node
// unguarded, because the streak the respawn starts is itself a hold. A node that comes
// back stuck must not have its still-real fault healed while its streak climbs to grace.
TEST_F(LifecycleExpectationIntegrationTest, RespawnedNodeStuckBelowGraceStillWithholdsTheClear) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  constexpr int kGrace = 5;
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", kGrace}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // read once: the latch is set
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx));

  set_apps({});  // the process dies, past the absence grace
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);

  // It comes back - stuck. The latch is still set from the dead incarnation, so only the
  // streak can hold the clear back.
  set_apps({"a"});
  gate.set_lifecycle_state_for_test("a", "inactive");
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < kGrace; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "a node that respawned still stuck had its fault cleared while the detector's own reads "
         "said it was not active";
}

// The window BEFORE the first match, which neither the read latch nor the streak can see.
// A restarted gateway does not have the graph yet: for the first ticks after the plugin
// comes up the entity snapshot has not caught up, so a require_active entry matches
// nothing at all. Both other halves of the guard are keyed by a MATCHED node, so both are
// empty, the tracker reports nothing affected, and the level-triggered clear flows - about
// a node the detector has never once looked at. The fault the restart was supposed to
// preserve heals in that gap.
//
// Distinct from a node that VANISHES: an entry that has matched before and stops matching
// is a presence problem (pinned by AbsenceAfterRaiseClearsNotNodeDeathsDomain, which must
// stay green). This is an entry that has never matched anything since configure().
TEST_F(LifecycleExpectationIntegrationTest, ClearIsWithheldUntilARequiredEntryHasMatchedAtLeastOnce) {
  set_apps({});  // the required node has not reached the snapshot yet
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 20; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the detector asserted GRAPH_NODE_INACTIVE was healthy before it had ever matched the node "
         "it was told to check - after a restart that heals a fault that is still real";

  // Once the node is actually there and reads healthy, the clear flows.
  set_apps({"a"});
  gate.set_lifecycle_state_for_test("a", "active");
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "the clear never flowed once the required node appeared and read active";
}

// The bound on that hold, same shape as the unread one: an entry that matches nothing
// because it is misspelt would otherwise block healing for the process lifetime.
TEST_F(LifecycleExpectationIntegrationTest, NeverMatchedEntryPastTheHoldBoundStopsBlockingTheClear) {
  set_apps({"something_else"});  // "a" matches nothing, and never will
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before, *det, ctx))
      << "a misspelt entry that can never match blocked healing forever - the never-matched hold "
         "must be bounded exactly like the unread one";
}

// A withhold is silence, and silence is indistinguishable from a detector that is working
// and finding nothing. The two reasons release on different conditions, so the log has to
// say which one is in effect - here the measured-but-below-grace one, whose hold ends when
// the node reads active or its streak passes grace, NOT after a fixed number of ticks.
TEST_F(LifecycleExpectationIntegrationTest, WithholdingForAYoungStreakSaysSoInTheLog) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  // Grace wide enough that the whole run stays inside the pending window.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 200}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");  // read every tick: only the streak can hold

  for (int i = 0; i < 30; ++i) {  // well past the 10-tick reporting horizon
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("withheld from clearing"), 1)
      << "a hold that outlives the reporting horizon must be explained exactly once per episode";
  EXPECT_EQ(log.count("measured not-active but still within grace"), 1)
      << "the log named the wrong reason: nothing here is unread, the streak is simply young";
  EXPECT_EQ(log.count("no lifecycle label read this run"), 0)
      << "the unread reason was reported for a node whose label was read on every tick";
}

// Withheld-clear guard boundary: a raise is NEVER withheld. A violation read from the
// nodes that did answer is real regardless of how many other required nodes are unread.
TEST_F(LifecycleExpectationIntegrationTest, RaiseIsNeverWithheldByAnUnreadNode) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "inactive");  // "b" stays unread
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "an unread sibling node held back a raise about a node that WAS read inactive";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/a"));
}

// Scale: 25 required nodes stuck inactive in one tick still produce exactly ONE
// aggregated FAILED, and its description honours the 480-char cap without ending in a
// mangled entry. This is the test that actually sits past the cap - a description
// enumerating 25 nodes is several times over it.
TEST_F(LifecycleExpectationIntegrationTest, TwentyFiveStuckNodesRaiseOneCappedFault) {
  std::vector<std::string> ids;
  nlohmann::json require = nlohmann::json::array();
  for (int i = 0; i < 25; ++i) {
    const std::string id = "a" + std::string(i < 10 ? "0" : "") + std::to_string(i);
    ids.push_back(id);
    require.push_back(id);
  }
  set_apps(ids);
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", require}, {"grace", 0}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  for (const auto & id : ids) {
    gate.set_lifecycle_state_for_test(id, "inactive");
  }

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  det->tick(ctx);  // ONE tick: grace 0 puts all 25 past grace at once
  ASSERT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before + 1));
  std::this_thread::sleep_for(200ms);  // a second request from the same tick would land by now
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before + 1)
      << "25 affected nodes in one tick must aggregate into exactly one FAILED";

  const std::string desc = last_failed_description(kGraphSource);
  const std::string marker = "...";
  EXPECT_LE(desc.size(), ros2_medkit_graph_watchdog::AggregatedFault::kMaxDescriptionChars)
      << "the aggregated description exceeded the cap";
  ASSERT_GE(desc.size(), marker.size());
  EXPECT_EQ(desc.compare(desc.size() - marker.size(), marker.size(), marker), 0)
      << "a description this far past the cap must end in the truncation marker, got tail: "
      << desc.substr(desc.size() - std::min<std::size_t>(desc.size(), 20));
  EXPECT_EQ(desc.find("node /a00 expected active"), 0u)
      << "the first (lexicographically) affected node must open the description intact";
}

// Change over time: a required node that APPEARS mid-run (graph growth) and then sticks
// inactive must be raised - the entry was configured before the node ever existed, so
// this pins enforcement of late arrivals, not just nodes present from arming.
TEST_F(LifecycleExpectationIntegrationTest, NodeAppearingMidRunThenStuckInactiveRaises) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "active");
  gate.set_lifecycle_state_for_test("b", "inactive");  // label parked before "b" exists - harmless

  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "raised about a node that is not in the graph yet";

  // "b" joins the graph and sits inactive.
  set_apps({"a", "b"});
  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "a required node appearing mid-run and sticking inactive was never raised";
  EXPECT_TRUE(any_failed_desc_contains(kGraphSource, "/b"));
}

// A re-bind under the SAME App::id must not keep enforcing the OLD node's label: "k" is
// bound to one managed node, read inactive there, then the id moves to a DIFFERENT node
// whose state nothing has read yet. Whatever label survives that move is what the
// detector enforces - and an unread new binding is unknown (benign), so no raise may
// flow. Uses managed-service snapshots (not the service-less fixture apps) because the
// re-bind is expressed through gate.update() itself.
TEST_F(LifecycleExpectationIntegrationTest, RebindUnderSameAppIdDoesNotEnforceTheOldNodesLabel) {
  set_managed_app("k", "/rb_old");
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"k"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  // The OLD binding's label, injected while "k" is still bound to /rb_old.
  gate.set_lifecycle_state_for_test("k", "inactive");

  // The re-bind: same App::id, different node. This gate.update() AFTER the injection is
  // deliberate (the one exception to ordering rule 2): with lifecycle services in the
  // snapshot the update keeps tracking "k", the injected label stands in for the OLD
  // binding's last read, and the NEW binding's own GetState fails into unknown - so
  // whatever label survives this update is exactly what the detector will enforce.
  set_managed_app("k", "/rb_new");
  gate.update(snapshot_, 9);

  for (int i = 0; i < 15; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // let any in-flight raise land before counting
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), 0u)
      << "the OLD binding's inactive label was enforced against the re-bound node - a re-bind must "
         "re-seed the id, not keep the departed node's label";
}

// The withheld-clear guard hands a node over to the presence class the moment it is absent
// past the absence grace, on BOTH node-keyed counts. Without that hand-off an unread node
// that crashes keeps blocking the clear until its guard entry is pruned - a whole minute at
// the shipped horizons instead of three ticks, so a GRAPH_NODE_INACTIVE about a DIFFERENT
// node takes that long to heal after the blocking node dies.
TEST_F(LifecycleExpectationIntegrationTest, UnreadNodeThatVanishesStopsBlockingTheClearAtTheAbsenceGrace) {
  set_apps({"a", "b"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", "b"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");  // "b" is matched but NEVER read
  const auto passed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED);
  for (int i = 0; i < 5; ++i) {  // far inside "b"'s 60-tick unread hold
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_PASSED), passed_before)
      << "the clear was never withheld in the first place, so the release below would prove nothing";

  // "b" crashes out of the graph. Sustained absence belongs to GRAPH_NODE_DISAPPEARED, so
  // the hold must end at the absence grace - NOT at the 60-tick prune horizon.
  set_apps({"a"});
  for (int i = 0; i < kDefaultAbsenceGrace + 3; ++i) {
    det->tick(ctx);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(wait_for_count(kGraphSource, ReportFault::Request::EVENT_PASSED, passed_before + 1))
      << "a never-read node that VANISHED kept blocking the clear past the absence grace - absence is "
         "the presence class's concern, so it must stop counting as unmeasured health within "
      << kDefaultAbsenceGrace << " ticks, not at the prune horizon";
}

// A withhold is silence, so it is explained once per episode - and the episode bookkeeping
// is what makes "once" survive a hold that lives for minutes AND lets a LATER hold speak up
// again. Nothing measured the log line itself: deleting the whole reporting body, or just
// its latch, left every hold test green.
TEST_F(LifecycleExpectationIntegrationTest, WithheldClearIsExplainedOncePerEpisodeNotOncePerProcess) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  // Grace wide enough that the second episode below never leaves the pending window.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 200}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const std::string needle = "withheld from clearing";
  // Episode 1: "a" is matched and never read. The horizon is 10 ticks, so 12 crosses it.
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1) << "a hold that outlived the 10-tick reporting horizon was never explained";
  EXPECT_EQ(log.count("no lifecycle label read this run"), 1) << "the log named the wrong reason for an unread node";

  for (int i = 0; i < 20; ++i) {  // the SAME episode, twice as long
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1) << "the reason was repeated within one episode - it is once per episode, or "
                                     "a hold that lasts minutes fills the log with the same line";

  // The episode ends: the label reads active, everything is measured healthy, the clear flows.
  gate.set_lifecycle_state_for_test("a", "active");
  ASSERT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_PASSED, 0, *det, ctx))
      << "the hold never released, so no second episode can start";

  // Episode 2, a different reason: the node reads not-active again and its fresh streak is
  // young, which withholds on its own.
  gate.set_lifecycle_state_for_test("a", "inactive");
  for (int i = 0; i < 12; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 2) << "the second withhold episode was never explained - the latch is scoped to "
                                     "the process, so an operator only ever hears about the first one";
  EXPECT_EQ(log.count("measured not-active but still within grace"), 1)
      << "the second episode named the first episode's reason";
}

// The detector's own no-match warning, at the tier that can see it: an entry that matches
// nothing at all is silent in every other path (it never reaches the violation branch, and
// the presence class never tracks a node that was never present), so this log line is the
// only signal a misspelt entry produces.
TEST_F(LifecycleExpectationIntegrationTest, EntryThatNeverMatchesAnythingIsWarnedAboutInTheLog) {
  const LogCapture log;
  set_apps({"something_else"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"typoed_name"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  const std::string needle = "has matched no node at all since startup";
  for (int i = 0; i < 12; ++i) {  // the default no-match horizon is 10 ticks
    det->tick(ctx);
  }
  EXPECT_EQ(log.count(needle), 1)
      << "a require_active entry that can never match produced no fault and no log line either, which is "
         "the whole failure mode this warning exists for (0 = never fired at detector level; >1 = the "
         "once-per-entry latch is gone)";
}

// The other side of that warning: it must not be said about an entry whose node WAS there
// and left. The tracker counts CONSECUTIVE no-match ticks, so a departed node surfaces the
// same way a misspelt entry does - but for it both halves of the sentence are false, and
// the presence class (GRAPH_NODE_DISAPPEARED) owns it, exactly as the never-matched hold
// distinguishes the two cases.
TEST_F(LifecycleExpectationIntegrationTest, EntryWhoseNodeVanishedIsNotAccusedOfNeverComingUp) {
  const LogCapture log;
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);

  gate.set_lifecycle_state_for_test("a", "active");
  det->tick(ctx);  // "a" matches here, and never again

  set_apps({});
  for (int i = 0; i < 15; ++i) {  // well past the 10-tick no-match horizon
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("has matched no node at all since startup"), 0)
      << "a node that was present and then departed was reported as one that never came up - and as one "
         "the presence class cannot see, which is exactly backwards: a departed node is what "
         "GRAPH_NODE_DISAPPEARED tracks";
}

// `grace` arrives from operator YAML as a 64-bit ROS integer parameter, so a value past the
// int range reaches configure() intact and is only truncated by the narrowing read - 2^32
// becomes 0, passes a >= 0 check, and silently installs a hair-trigger that reports a node
// on its very first not-active tick. The documented contract is that anything invalid warns
// and keeps the default, so the DEFAULT (5) must be what is actually in force here.
TEST_F(LifecycleExpectationIntegrationTest, WideGraceIsRejectedInsteadOfTruncatedIntoAHairTrigger) {
  set_apps({"a"});
  ReliabilityGate gate(kWarmupCycles, gateway_.get(), &node_mutex_);
  arm_global_grace(gate);

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4294967296}});
  auto ctx = make_ctx(DetectorMode::Raise, &gate);
  gate.set_lifecycle_state_for_test("a", "inactive");

  const auto failed_before = count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED);
  for (int i = 0; i < 3; ++i) {  // a grace truncated to 0 raises on the FIRST of these
    det->tick(ctx);
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count_faults(kGraphSource, ReportFault::Request::EVENT_FAILED), failed_before)
      << "a grace past the int range was truncated to 0 and reported a node that had been not-active for "
         "three ticks - the default grace of 5 is what an invalid value must leave in force";

  // And the default really is in force, so the expectation is still enforced past it.
  EXPECT_TRUE(poll_for_new(kGraphSource, ReportFault::Request::EVENT_FAILED, failed_before, *det, ctx))
      << "rejecting the invalid grace disabled the check instead of falling back to the default";
}

// ---- configure()-level contract (no ROS scaffolding beyond a logger node) ----

// A key the detector does not read must be reported, or the README's "unknown keys are
// reported" promise is false for this detector. The misspelling chosen is the worst case:
// with `require_activ` the detector is ALSO unconfigured, so the warning must be logged
// before the zero-config early return, not after it.
TEST(LifecycleExpectationConfig, MisspeltKeyIsWarnedAboutOnce) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_misspelt");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_activ", nlohmann::json::array({"a"})}});  // misspelt on purpose
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("unknown config key 'require_activ'"), 1)
      << "expected the misspelt key logged exactly once across three ticks (0 = the sweep is missing "
         "or logged after the zero-config early return; >1 = the once-guard is gone)";
}

// The known-key set must not drift from what configure() reads: a full valid config with
// every documented key plus the plugin-injected ones produces ZERO warnings.
TEST(LifecycleExpectationConfig, EveryKnownKeyIsAcceptedWithoutWarning) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_known");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{
      {"require_active", nlohmann::json::array({"a"})}, {"grace", 3}, {"prune_grace", 10}, {"tick_interval_ms", 100}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  for (int i = 0; i < 3; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "a fully valid config (all known keys + the plugin-injected ones) produced a warning";
}

TEST(LifecycleExpectationConfig, NegativeGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_neg_grace");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", -1}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be a non-negative integer; keeping the default (5)"), 1)
      << "grace: -1 must warn and name the default that stays in effect";
}

TEST(LifecycleExpectationConfig, NonIntegerGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_str_grace");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", "three"}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be a non-negative integer"), 1) << "a non-integer grace must warn, not vanish";
}

// prune_grace is plugin-injected and exempt from the unknown-key sweep, so a bad value
// must still be validated here - and on the WIDE integer: 4294967296 truncated through
// get<int>() would arrive as 0 and reclaim bookkeeping on the first absent tick.
TEST(LifecycleExpectationConfig, InvalidPruneGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", -1}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'prune_grace' must be an integer in 0..3600; keeping 60"), 1);

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", 4294967296}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'prune_grace' must be an integer in 0..3600; keeping 60"), 2)
      << "an out-of-int-range prune_grace must be rejected on the wide value, not truncated into "
         "a tiny prune horizon";
}

// `grace` is read off the same 64-bit JSON integer prune_grace is, so it needs the same
// wide range check: get<int>() truncates FIRST, and 4294967296 arrives as 0 - a value that
// passes the >= 0 test and installs the most dangerous setting this key has.
TEST(LifecycleExpectationConfig, WideGraceWarnsAndKeepsTheDefault) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_wide_grace");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4294967296}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be a non-negative integer; keeping the default (5)"), 1)
      << "a grace past the int range was truncated to 0 and accepted in silence";

  // The negative twin truncates to 0 as well, so the sign check alone cannot catch it.
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", -4294967296}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'grace' must be a non-negative integer; keeping the default (5)"), 2)
      << "a negative grace past the int range truncated to 0 and passed the >= 0 check";
}

// The documented prune_grace range endpoints. 3600 is inside the range and must be silent;
// 3601 is the first value outside it and must warn. Without both, an off-by-one at either
// end of the bound goes unnoticed.
TEST(LifecycleExpectationConfig, PruneGraceRangeEndpoints) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_prune_endpoints");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  DetectorContext ctx;
  ctx.gateway_node = node.get();

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", 3600}});
  det->tick(ctx);
  EXPECT_EQ(log.count("graph_watchdog lifecycle_expectation"), 0)
      << "prune_grace 3600 is the documented upper endpoint and must be accepted without a word";

  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"prune_grace", 3601}});
  det->tick(ctx);
  EXPECT_EQ(log.count("'prune_grace' must be an integer in 0..3600; keeping 60"), 1)
      << "the first value past the documented range was accepted";
}

// A require_active array of non-strings is what a ROS integer_array parameter delivers
// (`require_active: [1, 2]` in YAML). Every entry takes the warn-and-skip arm, and nothing
// may throw out of configure() - the plugin catches a throwing configure() and drops the
// detector entirely, so a typed-wrong list would silently disable the check.
TEST(LifecycleExpectationConfig, NonStringRequireActiveEntriesWarnAndAreSkipped) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_nonstring_entries");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  ASSERT_NO_THROW(det->configure(nlohmann::json{{"require_active", nlohmann::json::array({1, 2})}}));
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'require_active' entries must be non-empty node names; skipping one"), 2)
      << "a non-string entry was dropped without a word, so the operator believes a node is covered";
}

// Bounded tracking, through the detector's own config: prune_grace is allowed to undercut
// the tracker's absence grace (prune_grace 2 with grace 1 clamps to prune_ticks 2, one under
// the default absence grace of 3). Under identity churn - a bare entry matched by an
// ever-new namespaced respawn - nothing may be retained per name ever seen.
TEST(LifecycleExpectationConfig, ChurningIdentitiesStayBoundedWhenPruneGraceUndercutsTheAbsenceGrace) {
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 1}, {"prune_grace", 2}});
  IntrospectionInput snapshot;
  DetectorContext ctx;
  ctx.snapshot = &snapshot;  // no gate, no client: the tracker bookkeeping is the observable

  for (int i = 0; i < 50; ++i) {
    snapshot = namespaced_snapshot_of("ns" + std::to_string(i), "a");
    det->tick(ctx);
    EXPECT_LE(det->tracked_count_for_test(), 3u)
        << "iteration " << i
        << ": one bookkeeping entry is retained for every fqn ever seen, so a graph "
           "that churns node identities grows the detector's maps without bound";
  }
  snapshot.apps.clear();
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);
  }
  EXPECT_EQ(det->tracked_count_for_test(), 0u) << "every churned identity must be reclaimed once it is gone";
}

TEST(LifecycleExpectationConfig, NonArrayRequireActiveWarnsAndIsIgnored) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_nonarray");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", "a"}});  // a plain string, not an array
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'require_active' must be a string array of node names; ignoring it"), 1)
      << "a non-array require_active was dropped without a word";
}

TEST(LifecycleExpectationConfig, EmptyRequireActiveEntryWarnsAndIsSkipped) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("le_cfg_empty_entry");
  const LogCapture log;

  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a", ""})}});
  DetectorContext ctx;
  ctx.gateway_node = node.get();
  det->tick(ctx);
  EXPECT_EQ(log.count("'require_active' entries must be non-empty node names; skipping one"), 1)
      << "an empty require_active entry was dropped silently - the operator believes a node is covered";
}

// The prune clamp, grace+1 branch: with grace 0, prune_grace 0 clamps up to prune_ticks 1,
// so bookkeeping is reclaimed on the SECOND absent tick and never earlier - a reported
// entry can therefore never be pruned out from under its own fault.
TEST(LifecycleExpectationConfig, PruneClampGracePlusOneBranchAtPruneGraceZero) {
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 0}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;  // no gate, no client: the tracker bookkeeping is the observable
  det->tick(ctx);
  EXPECT_EQ(det->tracked_count_for_test(), 1u);
  snapshot.apps.clear();
  det->tick(ctx);  // absent 1 == prune_ticks(1): kept
  EXPECT_EQ(det->tracked_count_for_test(), 1u);
  det->tick(ctx);  // absent 2 > prune_ticks: reclaimed
  EXPECT_EQ(det->tracked_count_for_test(), 0u) << "prune_grace 0 with grace 0 must clamp to prune_ticks = grace + 1";
}

// The same clamp endpoint one step in: prune_grace 1 with grace 0 is exactly prune_ticks 1.
TEST(LifecycleExpectationConfig, PruneClampEndpointPruneGraceOne) {
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 1}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  det->tick(ctx);
  EXPECT_EQ(det->tracked_count_for_test(), 1u);
  snapshot.apps.clear();
  det->tick(ctx);  // absent 1 == prune_ticks(1): kept
  EXPECT_EQ(det->tracked_count_for_test(), 1u);
  det->tick(ctx);  // absent 2 > prune_ticks: reclaimed
  EXPECT_EQ(det->tracked_count_for_test(), 0u);
}

// The clamp's other branch: a prune_grace beyond grace+1 wins, so the entry survives
// absence up to prune_grace ticks.
TEST(LifecycleExpectationConfig, PruneClampPruneGraceWinsWhenLarger) {
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 0}, {"prune_grace", 4}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  det->tick(ctx);
  EXPECT_EQ(det->tracked_count_for_test(), 1u);
  snapshot.apps.clear();
  for (int i = 0; i < 4; ++i) {
    det->tick(ctx);  // absent 1..4, none past prune_ticks(4)
  }
  EXPECT_EQ(det->tracked_count_for_test(), 1u) << "pruned before prune_grace absent ticks elapsed";
  det->tick(ctx);  // absent 5 > prune_ticks: reclaimed
  EXPECT_EQ(det->tracked_count_for_test(), 0u);
}

// And with a large grace the clamp floors prune_ticks at grace+1 even when prune_grace
// says 0 - the branch that protects a slow-grace config from a hair-trigger prune.
TEST(LifecycleExpectationConfig, PruneClampGracePlusOneWinsWhenGraceIsLarge) {
  auto det = make_lifecycle_expectation();
  ASSERT_TRUE(det);
  det->configure(nlohmann::json{{"require_active", nlohmann::json::array({"a"})}, {"grace", 4}, {"prune_grace", 0}});
  auto snapshot = snapshot_of({"a"});
  DetectorContext ctx;
  ctx.snapshot = &snapshot;
  det->tick(ctx);
  EXPECT_EQ(det->tracked_count_for_test(), 1u);
  snapshot.apps.clear();
  for (int i = 0; i < 5; ++i) {
    det->tick(ctx);  // absent 1..5, none past prune_ticks(grace+1 = 5)
  }
  EXPECT_EQ(det->tracked_count_for_test(), 1u) << "prune_ticks must be floored at grace + 1";
  det->tick(ctx);  // absent 6 > prune_ticks: reclaimed
  EXPECT_EQ(det->tracked_count_for_test(), 0u);
}
