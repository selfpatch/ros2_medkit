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

#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>

namespace ros2_medkit_gateway::ros2_common {

namespace {

std::uint64_t steady_now_ns() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace

Ros2SubscriptionExecutor::Ros2SubscriptionExecutor(const std::shared_ptr<rclcpp::Node> & gateway_node, Config cfg)
  : cfg_(std::move(cfg)) {
  if (!gateway_node) {
    throw std::invalid_argument{"Ros2SubscriptionExecutor: gateway_node is null"};
  }

  const std::string suffixed_name = std::string{gateway_node->get_name()} + cfg_.subscription_node_name_suffix;
  rclcpp::NodeOptions opts;
  opts.start_parameter_services(false);
  opts.start_parameter_event_publisher(false);
  subscription_node_ = std::make_shared<rclcpp::Node>(suffixed_name, gateway_node->get_namespace(), opts);

  // Own executor owns the subscription node so subscription create/destroy
  // and callback dispatch all run on the same worker thread (this executor is
  // spun via spin_some() from worker_loop). Sharing the node with the gateway's
  // main MultiThreadedExecutor raced on the node's internal rcutils_hash_map
  // under TSan.
  sub_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  sub_executor_->add_node(subscription_node_);

  // Graph event - public rclcpp API, stable across distros.
  graph_event_ = subscription_node_->get_graph_event();

  worker_alive_.store(true);
  worker_ = std::thread([this] {
    worker_loop();
  });
  worker_thread_id_ = worker_.get_id();
  aux_thread_ = std::thread([this] {
    aux_loop();
  });
}

Ros2SubscriptionExecutor::~Ros2SubscriptionExecutor() {
  // 1. Signal shutdown. Worker will drain remaining queued tasks then exit.
  shutdown_flag_->store(true, std::memory_order_release);
  queue_cv_.notify_all();
  // aux_loop waits on its own condition variable; notifying here wakes it
  // immediately so destruction does not wait for the next graph-poll tick.
  { std::lock_guard<std::mutex> lk(aux_mtx_); }
  aux_cv_.notify_all();

  // 2. Join worker and aux - both loops check shutdown_flag_ and exit.
  if (worker_.joinable()) {
    worker_.join();
  }
  if (aux_thread_.joinable()) {
    aux_thread_.join();
  }
  worker_alive_.store(false);

  // 3. Detach subscription node from our own executor and destroy both.
  if (sub_executor_ && subscription_node_) {
    sub_executor_->remove_node(subscription_node_);
  }
  sub_executor_.reset();
  subscription_node_.reset();
  graph_event_.reset();
}

bool Ros2SubscriptionExecutor::post(std::function<void()> task) {
  {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    if (shutdown_flag_->load(std::memory_order_acquire)) {
      return false;
    }
    if (queue_.size() >= cfg_.max_queue_depth) {
      queue_dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    queue_.push_back(std::move(task));
    const std::size_t depth = queue_.size();
    std::size_t prev = queue_max_depth_observed_.load(std::memory_order_relaxed);
    while (depth > prev && !queue_max_depth_observed_.compare_exchange_weak(prev, depth, std::memory_order_relaxed)) {
    }
  }
  queue_cv_.notify_one();
  return true;
}

rclcpp::Node * Ros2SubscriptionExecutor::node() const noexcept {
  return subscription_node_.get();
}

std::size_t Ros2SubscriptionExecutor::on_graph_change(GraphCallback cb) {
  std::lock_guard<std::mutex> lk(graph_mtx_);
  for (std::size_t i = 0; i < kMaxGraphListeners; ++i) {
    if (!graph_slot_used_[i]) {
      graph_callbacks_[i] = std::move(cb);
      graph_slot_used_[i] = true;
      return i;
    }
  }
  return kMaxGraphListeners;  // no slot
}

void Ros2SubscriptionExecutor::remove_graph_change(std::size_t token) {
  if (token >= kMaxGraphListeners) {
    return;
  }
  std::lock_guard<std::mutex> lk(graph_mtx_);
  graph_slot_used_[token] = false;
  graph_callbacks_[token] = nullptr;
}

Ros2SubscriptionExecutor::Stats Ros2SubscriptionExecutor::stats() const {
  Stats s;
  s.worker_alive = worker_alive_.load();
  s.degraded = degraded_.load();
  {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    s.queue_depth = queue_.size();
  }
  s.queue_max_depth_observed = queue_max_depth_observed_.load();
  s.queue_dropped = queue_dropped_.load();
  s.tasks_completed = tasks_completed_.load();
  s.tasks_failed = tasks_failed_.load();
  s.last_task_latency_us = last_task_latency_us_.load();
  s.max_task_latency_us = max_task_latency_us_.load();

  const std::uint64_t started_ns = current_task_started_ns_.load();
  if (started_ns == 0) {
    s.current_task_age_ms = 0;
  } else {
    const std::uint64_t now_ns = steady_now_ns();
    s.current_task_age_ms = (now_ns > started_ns) ? (now_ns - started_ns) / 1'000'000U : 0U;
  }
  s.watchdog_trips = watchdog_trips_.load();
  s.graph_events_received = graph_events_received_.load();
  return s;
}

void Ros2SubscriptionExecutor::worker_loop() {
  constexpr std::chrono::milliseconds kSpinSlice{10};
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lk(queue_mtx_);
      // Short wait so we can interleave subscription-node callback dispatch.
      // Without spin_some() here, callbacks on the subscription node (real
      // topic data plus watchdog / graph-poll timers) would never fire.
      queue_cv_.wait_for(lk, kSpinSlice, [this] {
        return !queue_.empty() || shutdown_flag_->load();
      });
      if (queue_.empty()) {
        if (shutdown_flag_->load()) {
          break;
        }
        // No queued work and not shutting down: drive subscription callbacks.
        lk.unlock();
        if (sub_executor_) {
          try {
            sub_executor_->spin_some(kSpinSlice);
          } catch (const std::exception & ex) {
            RCLCPP_DEBUG(rclcpp::get_logger("ros2_subscription_executor"), "sub_executor.spin_some threw: %s",
                         ex.what());
          }
        }
        continue;
      }
      task = std::move(queue_.front());
      queue_.pop_front();
    }

    const std::uint64_t started_ns = steady_now_ns();
    current_task_started_ns_.store(started_ns, std::memory_order_release);
    try {
      task();
      tasks_completed_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception & ex) {
      tasks_failed_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"), "Task threw exception: %s", ex.what());
    } catch (...) {
      tasks_failed_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"), "Task threw unknown exception");
    }
    const std::uint64_t ended_ns = steady_now_ns();
    current_task_started_ns_.store(0, std::memory_order_release);

    const std::uint64_t latency_us = (ended_ns > started_ns) ? (ended_ns - started_ns) / 1000U : 0U;
    last_task_latency_us_.store(latency_us, std::memory_order_relaxed);
    std::uint64_t prev_max = max_task_latency_us_.load(std::memory_order_relaxed);
    while (latency_us > prev_max &&
           !max_task_latency_us_.compare_exchange_weak(prev_max, latency_us, std::memory_order_relaxed)) {
    }
  }
}

void Ros2SubscriptionExecutor::aux_loop() {
  // Independent thread driving watchdog + graph-event polling so they keep
  // running even when the worker thread is blocked inside a long-running
  // run_sync task. Both tick functions only touch this object's atomics and
  // the shared graph_event_; neither creates, destroys, or dispatches
  // callbacks on subscription_node_, so concurrency with the worker is safe.
  //
  // Sleep via condition_variable::wait_until(min(next_watchdog, next_graph))
  // with shutdown predicate so the destructor's notify wakes us within
  // microseconds rather than the next tick boundary.
  auto next_watchdog = std::chrono::steady_clock::now() + cfg_.watchdog_tick;
  auto next_graph = std::chrono::steady_clock::now() + cfg_.graph_poll_tick;
  while (!shutdown_flag_->load(std::memory_order_acquire)) {
    const auto deadline = std::min(next_watchdog, next_graph);
    {
      std::unique_lock<std::mutex> lk(aux_mtx_);
      aux_cv_.wait_until(lk, deadline, [this] {
        return shutdown_flag_->load(std::memory_order_acquire);
      });
    }
    if (shutdown_flag_->load(std::memory_order_acquire)) {
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_watchdog) {
      watchdog_tick();
      next_watchdog = now + cfg_.watchdog_tick;
    }
    if (now >= next_graph) {
      graph_poll_tick();
      next_graph = now + cfg_.graph_poll_tick;
    }
  }
}

void Ros2SubscriptionExecutor::watchdog_tick() {
  // Observational only: this detector flips `degraded_` when the in-flight
  // task on the worker has been running longer than `watchdog_threshold`. It
  // does not interrupt the worker, force-kill the task, or recreate the
  // executor; recovery is up to the operator (via /health visibility). Sticky
  // state: the flag stays true while a single task remains over threshold,
  // and only clears once the worker is observed idle (no in-flight task) so
  // a long-running task with brief sub-threshold dips does not cause flapping
  // between trips.
  const std::uint64_t started_ns = current_task_started_ns_.load(std::memory_order_acquire);
  if (started_ns == 0) {
    // Worker is idle - the previous task finished. Safe to clear.
    degraded_.store(false, std::memory_order_release);
    return;
  }
  const std::uint64_t now_ns = steady_now_ns();
  const std::uint64_t age_ms = (now_ns > started_ns) ? (now_ns - started_ns) / 1'000'000U : 0U;
  if (age_ms > static_cast<std::uint64_t>(cfg_.watchdog_threshold.count())) {
    if (!degraded_.exchange(true, std::memory_order_acq_rel)) {
      watchdog_trips_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"),
                   "Watchdog tripped: task age=%lums threshold=%lldms (observational only)",
                   static_cast<unsigned long>(age_ms), static_cast<long long>(cfg_.watchdog_threshold.count()));
    }
  }
  // Do NOT clear degraded_ while the SAME task is still in flight: the
  // previous implementation cleared on every tick where age < threshold, but
  // if the task takes 1500ms with threshold 1000ms, age oscillates around
  // the line and the flag flapped on/off between consecutive ticks. The
  // started_ns==0 branch above is the canonical recovery point.
}

void Ros2SubscriptionExecutor::graph_poll_tick() {
  if (!graph_event_) {
    return;
  }
  if (!graph_event_->check_and_clear()) {
    return;
  }
  graph_events_received_.fetch_add(1, std::memory_order_relaxed);
  fire_graph_callbacks();
}

void Ros2SubscriptionExecutor::fire_graph_callbacks() {
  // Snapshot under the lock, then post a SINGLE wrapper that fires the entire
  // snapshot. Posting one task per callback meant that a queue overflow mid
  // fan-out delivered the event to some listeners and silently dropped it for
  // others - the recovery path was the upstream provider's idle-safety-net
  // sweep, which can take 15 minutes by default. All-or-nothing delivery
  // means a single dropped post at most loses one whole graph event, which
  // graph_poll_tick() will refire on the next change.
  std::vector<GraphCallback> snapshot;
  snapshot.reserve(kMaxGraphListeners);
  {
    std::lock_guard<std::mutex> lk(graph_mtx_);
    for (std::size_t i = 0; i < kMaxGraphListeners; ++i) {
      if (graph_slot_used_[i] && graph_callbacks_[i]) {
        snapshot.push_back(graph_callbacks_[i]);
      }
    }
  }
  if (snapshot.empty()) {
    return;
  }
  (void)post([cbs = std::move(snapshot)] {
    for (const auto & cb : cbs) {
      try {
        cb();
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"), "Graph callback threw exception: %s", ex.what());
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"), "Graph callback threw unknown exception");
      }
    }
  });
}

}  // namespace ros2_medkit_gateway::ros2_common
