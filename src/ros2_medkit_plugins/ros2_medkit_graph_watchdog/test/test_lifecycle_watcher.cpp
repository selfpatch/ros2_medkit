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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"  // App, IntrospectionInput, ServiceInfo
#include "ros2_medkit_graph_watchdog/lifecycle_watcher.hpp"

class LifecycleWatcherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("lifecycle_watcher_test_node");
  }
  void TearDown() override {
    node_.reset();
  }
  rclcpp::Node::SharedPtr node_;
  std::mutex mtx_;
};

TEST_F(LifecycleWatcherTest, UntrackedNodeIsAlwaysOk) {
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  EXPECT_TRUE(w.node_ok("/not_a_lifecycle_node"));  // non-managed -> never gated
  EXPECT_FALSE(w.state_of("/not_a_lifecycle_node").has_value());
}

TEST_F(LifecycleWatcherTest, ActiveAllowedInactiveSuppressed) {
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  w.set_state_for_test("/nav/planner", "active");
  w.set_state_for_test("/nav/controller", "inactive");
  EXPECT_TRUE(w.node_ok("/nav/planner"));
  EXPECT_FALSE(w.node_ok("/nav/controller"));
  EXPECT_EQ(w.state_of("/nav/planner").value(), "active");
}

TEST_F(LifecycleWatcherTest, EmptyStateIsTreatedAsUnknownNotGated) {
  // A tracked node whose one-shot GetState seed failed has an empty label; since
  // transition_event is volatile and never backfills an already-active node,
  // gating on empty would suppress its faults forever. node_ok must NOT gate it.
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  w.set_state_for_test("/seed_failed", "");
  EXPECT_TRUE(w.node_ok("/seed_failed"));
}

namespace {
// Build a snapshot whose single App looks like a managed lifecycle node (GetState +
// ChangeState services), so update() discovers it, seeds it, and subscribes to its
// ~/transition_event.
ros2_medkit_gateway::IntrospectionInput managed_node_snapshot(const std::string & id) {
  ros2_medkit_gateway::ServiceInfo get_state;
  get_state.full_path = id + "/get_state";
  get_state.type = "lifecycle_msgs/srv/GetState";
  ros2_medkit_gateway::ServiceInfo change_state;
  change_state.full_path = id + "/change_state";
  change_state.type = "lifecycle_msgs/srv/ChangeState";

  ros2_medkit_gateway::App app;
  app.id = id;
  app.bound_fqn = id;  // `id` already looks like a full fqn in every caller (e.g. "/tnode")
  app.services = {get_state, change_state};

  ros2_medkit_gateway::IntrospectionInput in;
  in.apps.push_back(app);
  return in;
}

// Same shape, but with App::id decoupled from the node it is bound to - the re-bind
// cases below move `fqn` between snapshots while `id` stays put.
ros2_medkit_gateway::IntrospectionInput managed_app_snapshot(const std::string & id, const std::string & fqn) {
  auto in = managed_node_snapshot(fqn);
  in.apps.front().id = id;
  return in;
}

// Same App::id and same fqn, but the lifecycle services live somewhere else - a
// `~/get_state` remap in the launch file, or a manifest-bound app re-pointed at another
// node's services. The ~/transition_event topic is DERIVED from that path, so this is a
// different node behind an unchanged name: the half of the binding identity that no other
// test moves on its own.
ros2_medkit_gateway::IntrospectionInput remapped_service_snapshot(const std::string & fqn,
                                                                  const std::string & service_prefix) {
  auto in = managed_node_snapshot(service_prefix);
  in.apps.front().id = fqn;
  in.apps.front().bound_fqn = fqn;
  return in;
}

// N managed nodes in one snapshot, so a single update() queues more GetState work than the
// per-tick blocking budget can possibly run.
ros2_medkit_gateway::IntrospectionInput managed_nodes_snapshot(const std::vector<std::string> & ids) {
  ros2_medkit_gateway::IntrospectionInput in;
  for (const auto & id : ids) {
    in.apps.push_back(managed_node_snapshot(id).apps.front());
  }
  return in;
}
}  // namespace

// Exercises the REAL mechanism (not the set_state_for_test shim): update() discovers a
// managed node, derives its ~/transition_event topic, subscribes, and the callback writes
// the cached label from a live TransitionEvent - then the drop path forgets a vanished
// node. A broken topic derivation or a callback that never updates would fail here where
// the shim-based tests stay green.
TEST_F(LifecycleWatcherTest, UpdateSubscribesAndCallbackUpdatesState) {
  const std::string id = "/tnode";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  // Publisher matching the subscriber QoS (reliable, volatile), created on the gateway
  // node so its own subscription receives the message.
  auto pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // No GetState service exists, so the seed times out to an empty label (ungated). The
  // subscription is what we are testing.
  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());  // now tracked
  EXPECT_TRUE(w.node_ok(id));               // empty seed -> ungated

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  // Drive an "inactive" transition. Retry because a volatile publisher only delivers to a
  // subscription that has finished endpoint matching.
  lifecycle_msgs::msg::TransitionEvent msg;
  msg.goal_state.label = "inactive";
  bool flipped = false;
  for (int i = 0; i < 100 && !flipped; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flipped = (w.state_of(id).value_or("") == "inactive");
  }
  exec.cancel();
  spin.join();

  EXPECT_TRUE(flipped);         // callback wrote the label from the live message
  EXPECT_FALSE(w.node_ok(id));  // known non-active -> gated

  // Drop path: the node vanishes from a later snapshot and is forgotten.
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  EXPECT_FALSE(w.state_of(id).has_value());
}

// The failed-on_error death, over a live TransitionEvent rather than the injection seam:
// on_activate fails, on_error fails, the node lands in `finalized` and the process exits.
// The departure record must carry that it went through `errorprocessing`, otherwise the
// last label alone reads exactly like a lifecycle-manager shutdown.
TEST_F(LifecycleWatcherTest, DepartureThroughErrorProcessingIsRecordedAsErrorTerminated) {
  const std::string id = "/failing_driver";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  auto pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  // errorprocessing -> finalized is the ON_ERROR_FAILURE / ON_ERROR_ERROR edge.
  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "errorprocessing";
  msg.goal_state.label = "finalized";
  bool arrived = false;
  for (int i = 0; i < 100 && !arrived; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    arrived = (w.state_of(id).value_or("") == "finalized");
  }
  exec.cancel();
  spin.join();
  ASSERT_TRUE(arrived);

  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  const auto departed = w.departed_state_of(id);
  ASSERT_TRUE(departed.has_value());
  EXPECT_EQ(departed->label, "finalized");
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_TRUE(departed->error_terminated) << "the errorprocessing edge must survive into the departure record";
}

// The ordinary shutdown, same path: active -> shuttingdown -> finalized never touches the
// error branch, so the departure stays classifiable as clean.
TEST_F(LifecycleWatcherTest, OrdinaryShutdownDepartureIsNotErrorTerminated) {
  const std::string id = "/managed_ok";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  auto pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.start_state.label = "shuttingdown";
  msg.goal_state.label = "finalized";
  bool arrived = false;
  for (int i = 0; i < 100 && !arrived; ++i) {
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    arrived = (w.state_of(id).value_or("") == "finalized");
  }
  exec.cancel();
  spin.join();
  ASSERT_TRUE(arrived);

  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/2);
  const auto departed = w.departed_state_of(id);
  ASSERT_TRUE(departed.has_value());
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_FALSE(departed->error_terminated);
}

// The self-heal budget must be charged for reads that HAPPENED, not for jobs that were
// merely selected. None of these nodes has a real GetState service, so every read blocks to
// the reader's timeout (~500ms) while the per-tick blocking budget is 150ms: the very first
// read overruns it and the loop breaks, leaving the rest of the queue unread. Charging at
// selection time spent an attempt on every one of them. Two ticks of that and the whole
// budget was gone with almost no reads issued - and both outcomes are permanent for the
// process, because transition_event only fires on a FUTURE transition.
TEST_F(LifecycleWatcherTest, ReseedBudgetIsNotChargedForAJobTheTickBudgetSkipped) {
  const std::vector<std::string> ids{"/m0", "/m1", "/m2", "/m3", "/m4", "/m5", "/m6", "/m7"};
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  // Tick 1: all are new -> subscribed, each with a full self-heal budget, label "" (seeds
  // either timed out or never ran).
  w.update(managed_nodes_snapshot(ids), /*tick=*/1);
  int initial_total = 0;
  for (const auto & id : ids) {
    ASSERT_TRUE(w.state_of(id).has_value()) << id << " must be tracked";
    initial_total += w.reseeds_remaining_for_test(id);
  }
  ASSERT_GT(initial_total, 0) << "every newly tracked node starts with a self-heal budget";

  // Tick 2: every node is tracked and non-active, so all of them are selected as re-seed
  // jobs - but the budget only allows the first read.
  w.update(managed_nodes_snapshot(ids), /*tick=*/2);
  int remaining_total = 0;
  for (const auto & id : ids) {
    remaining_total += w.reseeds_remaining_for_test(id);
  }
  EXPECT_GT(remaining_total, initial_total - static_cast<int>(ids.size()))
      << "charging every selected job would leave " << initial_total - static_cast<int>(ids.size())
      << "; a job the per-tick budget skipped must keep its attempts";
}

// fqn capture + departed-lifecycle retention: a tracked node's stable fqn
// (App::effective_fqn(), captured at first sighting) is what departed_state_of() is
// keyed by, and its last-known label stays retrievable through that fqn for exactly
// `retention_ticks` ticks past its departure tick, then ages out.
TEST_F(LifecycleWatcherTest, DepartedNodeRetainsLastLabelByFqnThenPrunes) {
  const std::string fqn = "/managed_departed";
  constexpr int kRetentionTicks = 3;
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_, kRetentionTicks);

  // Track it (managed_node_snapshot() sets bound_fqn == id, so effective_fqn() == fqn).
  w.update(managed_node_snapshot(fqn), /*tick=*/1);
  ASSERT_TRUE(w.state_of(fqn).has_value());
  EXPECT_FALSE(w.departed_state_of(fqn).has_value())  // still present -> never "departed"
      << "a still-tracked node must never be reported by departed_state_of()";

  w.set_state_for_test(fqn, "finalized");

  // Vanishes at tick 10: recorded into recently_departed_ with departed_tick=10.
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/10);
  ASSERT_TRUE(w.departed_state_of(fqn).has_value());
  EXPECT_EQ(w.departed_state_of(fqn).value().label, "finalized");
  EXPECT_FALSE(w.state_of(fqn).has_value());  // no longer "currently tracked"

  // Still within retention (tick - departed_tick == retention_ticks, not yet pruned -
  // pruning only fires on strictly GREATER than retention_ticks).
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/10 + kRetentionTicks);
  EXPECT_TRUE(w.departed_state_of(fqn).has_value())
      << "an entry exactly at the retention boundary must still be retrievable";

  // One tick past retention -> pruned.
  w.update(ros2_medkit_gateway::IntrospectionInput{}, /*tick=*/10 + kRetentionTicks + 1);
  EXPECT_FALSE(w.departed_state_of(fqn).has_value());
}

// A tracked id whose BINDING moves (same App::id, different fqn / get_state path) is a
// different node wearing the same name: the old node's label and ~/transition_event
// subscription must be dropped and the id re-seeded through the new binding's GetState.
// The old label here is a KNOWN "inactive" with the self-heal budget already spent, so
// nothing else can heal it - an entry kept across the re-bind would keep gating the id
// on a node that is no longer behind it.
TEST_F(LifecycleWatcherTest, RebindToALiveNodeDropsTheOldLabelAndSeedsTheNewBinding) {
  const std::string id = "/appk";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  // The OLD binding: no GetState service (seeds fail), label driven over a live
  // ~/transition_event. The NEW binding: a GetState service that answers "active".
  auto old_pub = node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      "/rb_old/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  auto new_srv = node_->create_service<lifecycle_msgs::srv::GetState>(
      "/rb_new/get_state", [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*req*/,
                              const std::shared_ptr<lifecycle_msgs::srv::GetState::Response> & resp) {
        resp->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        resp->current_state.label = "active";
      });

  w.update(managed_app_snapshot(id, "/rb_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());
  const int fresh_budget = w.reseeds_remaining_for_test(id);
  ASSERT_GT(fresh_budget, 0);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  std::thread spin([&exec]() {
    exec.spin();
  });

  lifecycle_msgs::msg::TransitionEvent msg;
  msg.goal_state.label = "inactive";
  bool flipped = false;
  for (int i = 0; i < 100 && !flipped; ++i) {
    old_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flipped = (w.state_of(id).value_or("") == "inactive");
  }
  ASSERT_TRUE(flipped);

  // Drain the self-heal budget against the OLD (serviceless) binding, so the assertion
  // below cannot be satisfied by a leftover re-seed happening to hit the new path - the
  // re-bind handling itself has to do the healing.
  for (std::uint64_t tick = 2; w.reseeds_remaining_for_test(id) > 0 && tick < 10; ++tick) {
    w.update(managed_app_snapshot(id, "/rb_old"), tick);
  }
  ASSERT_EQ(w.reseeds_remaining_for_test(id), 0);
  ASSERT_EQ(w.state_of(id).value_or(""), "inactive");

  // The re-bind: same id, now bound to the live "/rb_new" node.
  w.update(managed_app_snapshot(id, "/rb_new"), /*tick=*/10);
  EXPECT_EQ(w.state_of(id).value_or(""), "active")
      << "after a re-bind the id must carry the NEW binding's seeded state, not the old node's label";
  EXPECT_TRUE(w.node_ok(id)) << "the old binding's non-active label must stop gating the id";
  EXPECT_EQ(w.reseeds_remaining_for_test(id), fresh_budget)
      << "a re-bound id must be re-seeded as a NEW entry (fresh self-heal budget), not healed in place";

  // The old subscription must be gone with the old entry: the old node's transitions
  // must no longer reach this id.
  for (int i = 0; i < 10; ++i) {
    old_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  EXPECT_EQ(w.state_of(id).value_or(""), "active")
      << "a transition published by the OLD binding after the re-bind overwrote the new binding's state";

  exec.cancel();
  spin.join();
}

// Re-bind to a binding with NO live node behind it: the id follows the new-binding
// seeding semantics - tracked with an empty (unknown, ungated) label after the failed
// GetState - and must NOT keep the old node's label.
TEST_F(LifecycleWatcherTest, RebindToADeadBindingLeavesStateUnknownNotTheOldLabel) {
  const std::string id = "/appd";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(managed_app_snapshot(id, "/rbd_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());
  w.set_state_for_test(id, "inactive");  // the OLD binding's observed label

  w.update(managed_app_snapshot(id, "/rbd_new"), /*tick=*/2);
  EXPECT_EQ(w.state_of(id).value_or("<untracked>"), "")
      << "a re-bind to a dead binding must leave the id unknown (empty seed), not wearing the old label";
  EXPECT_TRUE(w.node_ok(id)) << "unknown must not gate - only a KNOWN non-active label does";
}

// The guard against overcorrection: an UNCHANGED binding across updates must never be
// treated as a re-bind. The instrument is the self-heal budget, which counts what the
// remote actually pays (GetState round trips): a drop + re-create would reset it to the
// fresh value on every tick and record a phantom departure.
TEST_F(LifecycleWatcherTest, SameBindingAcrossUpdatesIsNeverTreatedAsARebind) {
  const std::string fqn = "/stable";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(managed_node_snapshot(fqn), /*tick=*/1);
  const int fresh_budget = w.reseeds_remaining_for_test(fqn);
  ASSERT_GT(fresh_budget, 0);

  // Non-active label + budget left -> exactly one charged re-seed per update.
  w.update(managed_node_snapshot(fqn), /*tick=*/2);
  EXPECT_EQ(w.reseeds_remaining_for_test(fqn), fresh_budget - 1)
      << "one update with the same binding must charge exactly one re-seed - a re-created entry "
         "would reset the budget to "
      << fresh_budget;
  w.update(managed_node_snapshot(fqn), /*tick=*/3);
  w.update(managed_node_snapshot(fqn), /*tick=*/4);
  EXPECT_EQ(w.reseeds_remaining_for_test(fqn), 0)
      << "the budget must drain monotonically across same-binding updates and stay drained";
  EXPECT_TRUE(w.state_of(fqn).has_value()) << "the entry itself must survive every same-binding update";
  EXPECT_FALSE(w.departed_state_of(fqn).has_value()) << "an unchanged binding must never be recorded as a departure";
}

// The OTHER half of the binding identity, on its own. Every re-bind test above moves the
// fqn and the get_state path together (the snapshot builder derives one from the other), so
// the path arm of the drop condition is never the thing that fires. Here only the SERVICE
// path moves - the id and the fqn are unchanged - and it is still a different node: the
// ~/transition_event subscription is derived from that path, so an entry kept across this
// would keep enforcing the old node's label and listening on the old node's topic.
TEST_F(LifecycleWatcherTest, MovingOnlyTheServicePathIsStillARebind) {
  const std::string fqn = "/remapped";
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);

  w.update(remapped_service_snapshot(fqn, "/svc_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(fqn).has_value());
  const int fresh_budget = w.reseeds_remaining_for_test(fqn);
  ASSERT_GT(fresh_budget, 0);
  w.set_state_for_test(fqn, "inactive");  // the OLD binding's observed label

  // Same id, same fqn - only the services moved.
  w.update(remapped_service_snapshot(fqn, "/svc_new"), /*tick=*/2);
  EXPECT_EQ(w.state_of(fqn).value_or("<untracked>"), "")
      << "a node whose lifecycle services moved kept the old binding's label - the get_state path is half "
         "the binding identity, and the transition_event topic is derived from it";
  EXPECT_EQ(w.reseeds_remaining_for_test(fqn), fresh_budget)
      << "the entry must be re-created (fresh self-heal budget), not healed in place";
}

// A re-seed is a BLOCKING read issued without any lock and applied afterwards, so a
// ~/transition_event that lands while the response is in flight is FRESHER than the answer
// about to overwrite it. This is the bringup moment the re-seed mechanism exists for - the
// node is mid-transition by construction - and when it happens on the LAST attempt of the
// bounded budget the stale label is permanent for the process: an "active" node stuck
// wearing "inactive" is a false GRAPH_NODE_INACTIVE forever AND has every other fault
// suppressed by node_ok(), the two outcomes this file's own comments say must not happen.
//
// Driven through the REAL machinery, not a shim: a live GetState service answers the
// watcher's own reader, and the answer is deliberately ordered AFTER a real TransitionEvent
// the same handler published - the handler publishes "active", then holds the response open
// long enough for the watcher's subscription to have processed it, then answers with the
// state the node has already left. The service lives on its own node so its callback group
// cannot serialise the subscription callback behind it.
TEST_F(LifecycleWatcherTest, AReSeedNeverOverwritesAFresherTransitionEvent) {
  const std::string id = "/rs";
  auto srv_node = std::make_shared<rclcpp::Node>("lifecycle_watcher_reseed_service_node");
  auto pub = srv_node->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
      id + "/transition_event", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // Off until the trap is armed: while it is off the service is an ordinary GetState that
  // answers "inactive", which is what proves the watcher's reader can reach it at all.
  std::atomic<bool> publish_active_before_answering{false};
  auto get_state = srv_node->create_service<lifecycle_msgs::srv::GetState>(
      id + "/get_state",
      [&pub, &publish_active_before_answering](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> & /*req*/,
                                               const std::shared_ptr<lifecycle_msgs::srv::GetState::Response> & resp) {
        if (publish_active_before_answering.load()) {
          lifecycle_msgs::msg::TransitionEvent event;
          event.start_state.label = "activating";
          event.goal_state.label = "active";
          pub->publish(event);
          // The node reached "active" here. The answer below is what the service had
          // already computed - stale by the time it lands. The hold is long enough for the
          // watcher's subscription to have processed the event (loopback delivery to an
          // already-matched endpoint is single-digit ms) and short enough to stay well
          // inside the reader's own 500 ms timeout: a read that TIMED OUT would be dropped
          // as an empty seed, which passes the assertion below for the wrong reason. That
          // is the one residual here, and it fails safe (a false green under extreme load,
          // never a false red); the calibration tick above is what proves the read path
          // works at all in this run.
          std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
        resp->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        resp->current_state.label = "inactive";
      });

  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_);
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node_);
  exec.add_node(srv_node);
  std::thread spin([&exec]() {
    exec.spin();
  });

  // Tick 1: new node -> subscribed to ~/transition_event, seeded (or not - the reader's own
  // client still has to discover the service).
  w.update(managed_node_snapshot(id), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());

  // Prove the subscription is endpoint-matched, or a "the event never arrived" run would
  // look exactly like the bug being absent. A volatile publisher delivers nothing until
  // matching completes, hence the retry.
  lifecycle_msgs::msg::TransitionEvent probe;
  probe.goal_state.label = "unconfigured";
  bool matched = false;
  for (int i = 0; i < 100 && !matched; ++i) {
    pub->publish(probe);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    matched = (w.state_of(id).value_or("") == "unconfigured");
  }
  ASSERT_TRUE(matched) << "the watcher's ~/transition_event subscription never matched the publisher, so "
                          "nothing below could deliver the fresher label";

  // Calibration tick (trap still off): a plain re-seed must reach the service and land its
  // answer. Without this, a run whose GetState never resolved would pass the assertion
  // below for the wrong reason.
  w.update(managed_node_snapshot(id), /*tick=*/2);
  ASSERT_EQ(w.state_of(id).value_or(""), "inactive")
      << "the re-seed never reached the live GetState service, so the ordering below is untested";
  ASSERT_GT(w.reseeds_remaining_for_test(id), 0) << "no self-heal budget left to arm the trap with";

  // Armed: the next re-seed's answer is computed BEFORE the node reaches "active" and
  // applied AFTER the watcher has already recorded it.
  publish_active_before_answering.store(true);
  w.update(managed_node_snapshot(id), /*tick=*/3);

  const std::string label = w.state_of(id).value_or("<untracked>");
  exec.cancel();
  spin.join();
  exec.remove_node(node_);
  exec.remove_node(srv_node);

  EXPECT_EQ(label, "active") << "a re-seed answer computed before the node activated overwrote the "
                                "~/transition_event that reported the activation - on the last attempt of the "
                                "bounded budget that stale label is permanent for the process";
}

// What a re-bind means for departed_state_of(): the OLD binding effectively departed, so
// its last observed state is recorded under ITS fqn - same record shape and same
// retention mechanics as a node vanishing from the snapshot.
TEST_F(LifecycleWatcherTest, RebindRecordsTheOldBindingAsDepartedUnderItsOwnFqn) {
  const std::string id = "/appr";
  constexpr int kRetentionTicks = 3;
  ros2_medkit_graph_watchdog::LifecycleWatcher w(node_.get(), &mtx_, kRetentionTicks);

  w.update(managed_app_snapshot(id, "/rbr_old"), /*tick=*/1);
  ASSERT_TRUE(w.state_of(id).has_value());
  w.set_state_for_test(id, "inactive");

  w.update(managed_app_snapshot(id, "/rbr_new"), /*tick=*/5);
  const auto departed = w.departed_state_of("/rbr_old");
  ASSERT_TRUE(departed.has_value()) << "a re-bind must record the OLD binding as departed under its fqn";
  EXPECT_EQ(departed->label, "inactive");
  EXPECT_TRUE(departed->saw_transition);
  EXPECT_FALSE(departed->error_terminated);
  EXPECT_FALSE(w.departed_state_of("/rbr_new").has_value())
      << "the NEW binding is present, not departed - only the old one may be recorded";

  // The record ages out through the ordinary retention prune.
  w.update(managed_app_snapshot(id, "/rbr_new"), /*tick=*/5 + kRetentionTicks + 1);
  EXPECT_FALSE(w.departed_state_of("/rbr_old").has_value());
}
