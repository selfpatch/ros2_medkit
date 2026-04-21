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

Ros2SubscriptionExecutor::Ros2SubscriptionExecutor(const std::shared_ptr<rclcpp::Node> & gateway_node,
                                                   rclcpp::Executor & main_executor, Config cfg)
  : cfg_(std::move(cfg)), main_executor_(&main_executor) {
  if (!gateway_node) {
    throw std::invalid_argument{"Ros2SubscriptionExecutor: gateway_node is null"};
  }

  const std::string suffixed_name = std::string{gateway_node->get_name()} + cfg_.subscription_node_name_suffix;
  rclcpp::NodeOptions opts;
  opts.start_parameter_services(false);
  opts.start_parameter_event_publisher(false);
  subscription_node_ = std::make_shared<rclcpp::Node>(suffixed_name, gateway_node->get_namespace(), opts);
  main_executor_->add_node(subscription_node_);

  // Graph event - public rclcpp API, stable across distros.
  graph_event_ = subscription_node_->get_graph_event();

  // Watchdog and graph-poll timers run on the subscription node via the main executor.
  watchdog_timer_ = subscription_node_->create_wall_timer(cfg_.watchdog_tick, [this] {
    watchdog_tick();
  });
  graph_poll_timer_ = subscription_node_->create_wall_timer(cfg_.graph_poll_tick, [this] {
    graph_poll_tick();
  });

  worker_alive_.store(true);
  worker_ = std::thread([this] {
    worker_loop();
  });
}

Ros2SubscriptionExecutor::~Ros2SubscriptionExecutor() {
  // 1. Signal shutdown. Worker will drain remaining queued tasks then exit.
  shutdown_flag_->store(true, std::memory_order_release);
  queue_cv_.notify_all();

  // 2. Join worker - this waits for the drain to complete.
  if (worker_.joinable()) {
    worker_.join();
  }
  worker_alive_.store(false);

  // 3. Cancel timers before releasing the node. Timer callbacks capture `this`;
  //    after timer cancel + node removal from executor, no more callbacks fire.
  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }
  if (graph_poll_timer_) {
    graph_poll_timer_->cancel();
    graph_poll_timer_.reset();
  }

  // 4. Detach subscription node from main executor.
  if (main_executor_ && subscription_node_) {
    main_executor_->remove_node(subscription_node_);
  }
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
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lk(queue_mtx_);
      queue_cv_.wait(lk, [this] {
        return !queue_.empty() || shutdown_flag_->load();
      });
      if (queue_.empty()) {
        // Shutdown requested and nothing to drain.
        break;
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

void Ros2SubscriptionExecutor::watchdog_tick() {
  const std::uint64_t started_ns = current_task_started_ns_.load(std::memory_order_acquire);
  if (started_ns == 0) {
    degraded_.store(false, std::memory_order_release);
    return;
  }
  const std::uint64_t now_ns = steady_now_ns();
  const std::uint64_t age_ms = (now_ns > started_ns) ? (now_ns - started_ns) / 1'000'000U : 0U;
  if (age_ms > static_cast<std::uint64_t>(cfg_.watchdog_threshold.count())) {
    if (!degraded_.exchange(true, std::memory_order_acq_rel)) {
      watchdog_trips_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"),
                   "Watchdog tripped: task age=%lums threshold=%lldms", static_cast<unsigned long>(age_ms),
                   static_cast<long long>(cfg_.watchdog_threshold.count()));
    }
  } else {
    degraded_.store(false, std::memory_order_release);
  }
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
  // Snapshot under the lock, fire outside, post onto worker.
  std::array<GraphCallback, kMaxGraphListeners> snapshot;
  std::size_t count = 0;
  {
    std::lock_guard<std::mutex> lk(graph_mtx_);
    for (std::size_t i = 0; i < kMaxGraphListeners; ++i) {
      if (graph_slot_used_[i] && graph_callbacks_[i]) {
        snapshot[count++] = graph_callbacks_[i];
      }
    }
  }
  for (std::size_t i = 0; i < count; ++i) {
    // If the queue is full we accept losing this graph fan-out for some
    // listeners; the idle-safety-net eviction in upstream providers compensates.
    (void)post([cb = std::move(snapshot[i])] {
      try {
        cb();
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"), "Graph callback threw exception: %s", ex.what());
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("ros2_subscription_executor"), "Graph callback threw unknown exception");
      }
    });
  }
}

}  // namespace ros2_medkit_gateway::ros2_common
