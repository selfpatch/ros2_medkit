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

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway::ros2_common {

/**
 * @brief Single-writer bridge that serializes all rcl-mutating calls on one worker thread.
 *
 * Owns a dedicated `subscription_node` (added to the main gateway executor) and a single
 * worker thread that processes a bounded queue of tasks. All subscribe / unsubscribe /
 * callback-group mutations must go through this executor to avoid the rcl hash-map race
 * that occurs when multiple threads create subscriptions on the same node concurrently.
 *
 * @par Thread model
 * - One worker thread, fed by a bounded task queue. Tasks run in FIFO order and
 *   interleave with `sub_executor_.spin_some()` so callback dispatch on the
 *   subscription node happens on the same thread as subscription creation /
 *   destruction.
 * - The subscription node is exclusively owned by an internal
 *   `SingleThreadedExecutor`; the gateway's main executor never sees it.
 * - One auxiliary thread drives the watchdog and graph-event polling ticks on
 *   their own cadence. They only touch atomics / the graph event, not the
 *   node's subscription list, so they cannot race the worker.
 * - Graph-change callbacks are posted onto the worker task queue so they run
 *   outside the main executor context.
 *
 * @par Bounded behavior
 * - Queue depth capped at `Config::max_queue_depth`. Excess posts return `false`
 *   (backpressure) and increment `queue_dropped`.
 * - `run_sync` has a deadline; on timeout returns `tl::unexpected` instead of blocking
 *   indefinitely.
 * - Graph-listener slots pre-allocated at `kMaxGraphListeners` (no heap allocation).
 *
 * @par Shutdown
 * Destruction drains the queue (all pending tasks run to completion so `run_sync` callers
 * get their promises fulfilled), then joins the worker, cancels timers, and tears down the
 * internal subscription executor. Bounded by longest remaining task execution time.
 * External deployment platform (systemd / k8s / Docker) should enforce hard timeout.
 *
 * @par Not for inheritance
 * Marked `final`. The abstraction boundary for alternate transports is at the provider
 * interface level (`DataProvider`, `OperationProvider`, etc.), not this class.
 */
class Ros2SubscriptionExecutor final {
 public:
  /// Pre-allocated graph listener slots. Size chosen to cover all domain providers plus headroom.
  static constexpr std::size_t kMaxGraphListeners = 16;

  struct Config {
    std::size_t max_queue_depth;
    std::chrono::milliseconds watchdog_threshold;
    std::chrono::milliseconds watchdog_tick;
    std::chrono::milliseconds graph_poll_tick;
    std::string subscription_node_name_suffix;

    // Explicit ctor needed because GCC does not allow default member initializers
    // of a nested class to be used by the enclosing class's member declarations.
    Config()
      : max_queue_depth{256}
      , watchdog_threshold{5000}
      , watchdog_tick{1000}
      , graph_poll_tick{100}
      , subscription_node_name_suffix{"_sub"} {
    }
  };

  struct Stats {
    bool worker_alive{false};
    bool degraded{false};
    std::size_t queue_depth{0};
    std::size_t queue_max_depth_observed{0};
    std::size_t queue_dropped{0};
    std::size_t tasks_completed{0};
    std::size_t tasks_failed{0};
    std::uint64_t last_task_latency_us{0};
    std::uint64_t max_task_latency_us{0};
    std::uint64_t current_task_age_ms{0};
    std::size_t watchdog_trips{0};
    std::size_t graph_events_received{0};
  };

  using GraphCallback = std::function<void()>;

  /**
   * @brief Construct and start the worker thread.
   *
   * The subscription node is owned by an internal `SingleThreadedExecutor` that is
   * pumped via `spin_some()` from the worker thread. The main gateway executor is
   * deliberately not wired up - sharing the subscription node with a multi-threaded
   * executor reintroduces the rcutils_hash_map race this class exists to eliminate.
   *
   * @param gateway_node Owning gateway node. Used only to derive the subscription
   *                     node name and namespace; no references retained after
   *                     construction.
   * @param cfg          Bounded resource configuration.
   */
  Ros2SubscriptionExecutor(const std::shared_ptr<rclcpp::Node> & gateway_node, Config cfg = Config());

  /// Idempotent shutdown: drains queue, joins worker, cancels timers, removes sub node.
  ~Ros2SubscriptionExecutor();

  Ros2SubscriptionExecutor(const Ros2SubscriptionExecutor &) = delete;
  Ros2SubscriptionExecutor & operator=(const Ros2SubscriptionExecutor &) = delete;
  Ros2SubscriptionExecutor(Ros2SubscriptionExecutor &&) = delete;
  Ros2SubscriptionExecutor & operator=(Ros2SubscriptionExecutor &&) = delete;

  /**
   * @brief Execute a task on the worker and wait for completion with a deadline.
   *
   * Serializes the task with all other rcl mutations. Exceptions thrown by the task
   * are caught and returned as `tl::unexpected(what_string)`. Queue full / deadline
   * exceeded / shutting down are also reported as `tl::unexpected`.
   *
   * @tparam R        Return type (may be `void`).
   * @param task      Callable executed on the worker thread.
   * @param deadline  Maximum wall time to wait for completion (default 5s).
   * @return The task's return value, or an error string on failure.
   */
  template <typename R>
  [[nodiscard]] tl::expected<R, std::string>
  run_sync(std::function<R()> task, std::chrono::milliseconds deadline = std::chrono::milliseconds{5000});

  /**
   * @brief Enqueue a fire-and-forget task on the worker.
   *
   * Used for graph-change callbacks and other cases where synchronous completion
   * is not required. Returns `false` if the executor is shutting down or the queue
   * is at capacity.
   */
  bool post(std::function<void()> task);

  /**
   * @brief Accessor for the subscription node.
   *
   * @par Allowed (any thread)
   * Read-only graph queries: `get_topic_names_and_types`, `count_publishers`,
   * `get_publishers_info_by_topic`, `get_node_names_*`, `get_logger`. These
   * acquire internal rmw locks but do not mutate the node's hash maps.
   *
   * @par Forbidden (any thread except the worker)
   * Anything that mutates the node: `create_subscription`, `create_publisher`,
   * `create_service`, `create_client`, timers, parameter set/declare. Use
   * `Ros2SubscriptionSlot::create_*` or post a task via `run_sync` instead.
   * The CI regression gate `scripts/check_no_naked_subscriptions.sh` rejects
   * direct `create_*` calls on the subscription node outside ros2_common/.
   *
   * Concurrent rcl mutations on other threads were the root cause of issue
   * #375 (rcutils_hash_map_set/get race) and are NOT prevented by this getter
   * - callers are expected to follow the rules above.
   */
  rclcpp::Node * node() const noexcept;

  /**
   * @brief Register a graph-change callback.
   *
   * Called on the worker thread when the ROS 2 graph changes (publishers appearing /
   * disappearing, types changing). Use to invalidate per-topic caches or evict stale
   * pool entries.
   *
   * @warning A registered callback must NOT call remove_graph_change() on its own
   *          token from inside the callback - graph_mtx_ is non-recursive and a
   *          self-removal attempt deadlocks. Drop the token through a separate task
   *          posted to the worker if dynamic deregistration is needed.
   *
   * @return Opaque token in range [0, kMaxGraphListeners). `kMaxGraphListeners` if
   *         all slots are taken.
   */
  [[nodiscard]] std::size_t on_graph_change(GraphCallback cb);

  /// Remove a previously-registered graph callback. Idempotent. See warning on on_graph_change.
  void remove_graph_change(std::size_t token);

  /// True after shutdown has started. Monotonic. Use to skip re-posting work on teardown.
  [[nodiscard]] bool is_shutting_down() const noexcept {
    return shutdown_flag_->load();
  }

  /**
   * @brief id of the worker thread that runs this executor's tasks.
   *
   * Use to detect "is the calling thread already the worker?" in destructors that
   * would otherwise post a task and wait via run_sync - posting from the worker
   * thread itself recursively deadlocks because the worker is the one that has
   * to drain the queue. Slot destructor uses this to drop subscriptions inline
   * when called from a graph callback running on the worker.
   */
  [[nodiscard]] std::thread::id worker_thread_id() const noexcept {
    return worker_thread_id_;
  }

  /**
   * @brief Shared pointer to the shutdown flag.
   *
   * Slots copy this shared_ptr at construction so their destructor can read
   * the flag even if the executor has already been destroyed (shutdown fast
   * path). Without shared ownership the slot would dereference freed memory.
   */
  [[nodiscard]] std::shared_ptr<std::atomic<bool>> shutdown_flag_ptr() const noexcept {
    return shutdown_flag_;
  }

  [[nodiscard]] Stats stats() const;

 private:
  void worker_loop();
  void watchdog_tick();
  void graph_poll_tick();
  void fire_graph_callbacks();

  Config cfg_;
  std::shared_ptr<rclcpp::Node> subscription_node_;
  // Dedicated executor owned by this Ros2SubscriptionExecutor. The subscription
  // node lives here, NOT on the gateway's main executor, so that callbacks and
  // subscription create/destroy all run on the same worker thread. Sharing the
  // node with the main MultiThreadedExecutor would race on the node's internal
  // hash map (rcutils_hash_map_set/get) under TSan.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> sub_executor_;

  // Task queue
  mutable std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<std::function<void()>> queue_;
  std::shared_ptr<std::atomic<bool>> shutdown_flag_ = std::make_shared<std::atomic<bool>>(false);
  std::thread worker_;
  std::thread::id worker_thread_id_{};

  // Stats
  std::atomic<bool> worker_alive_{false};
  std::atomic<bool> degraded_{false};
  std::atomic<std::size_t> queue_max_depth_observed_{0};
  std::atomic<std::size_t> queue_dropped_{0};
  std::atomic<std::size_t> tasks_completed_{0};
  std::atomic<std::size_t> tasks_failed_{0};
  std::atomic<std::uint64_t> last_task_latency_us_{0};
  std::atomic<std::uint64_t> max_task_latency_us_{0};
  std::atomic<std::uint64_t> current_task_started_ns_{0};
  std::atomic<std::size_t> watchdog_trips_{0};
  std::atomic<std::size_t> graph_events_received_{0};

  // Graph callbacks (pre-allocated, Tier 1)
  mutable std::mutex graph_mtx_;
  std::array<GraphCallback, kMaxGraphListeners> graph_callbacks_{};
  std::array<bool, kMaxGraphListeners> graph_slot_used_{};

  // Public rclcpp graph API - stable across Humble / Jazzy / Rolling
  rclcpp::Event::SharedPtr graph_event_;

  // Dedicated auxiliary thread driving the watchdog and graph-event polling.
  // These cannot ride the worker thread because the worker may be blocked
  // inside a long-running run_sync task; nor can they ride sub_executor_
  // because spin_some only runs when the worker is idle. They do not touch
  // subscription_node_ internals so running on a separate thread is safe.
  std::thread aux_thread_;
  // Shutdown-aware sleep for aux_loop: condition variable predicate-waits
  // on shutdown_flag_ instead of polling sleep_for(tick), so destruction
  // returns within microseconds rather than the next 100ms tick boundary.
  mutable std::mutex aux_mtx_;
  std::condition_variable aux_cv_;

  void aux_loop();
};

// ---------------------------------------------------------------------------
// Template implementation

template <typename R>
tl::expected<R, std::string> Ros2SubscriptionExecutor::run_sync(std::function<R()> task,
                                                                std::chrono::milliseconds deadline) {
  if (shutdown_flag_->load(std::memory_order_acquire)) {
    return tl::unexpected(std::string{"executor shutting down"});
  }
  auto promise = std::make_shared<std::promise<tl::expected<R, std::string>>>();
  auto future = promise->get_future();

  auto wrapper = [task = std::move(task), promise]() mutable {
    try {
      if constexpr (std::is_void_v<R>) {
        task();
        promise->set_value(tl::expected<void, std::string>{});
      } else {
        promise->set_value(tl::expected<R, std::string>{task()});
      }
    } catch (const std::exception & ex) {
      promise->set_value(tl::unexpected(std::string{ex.what()}));
    } catch (...) {
      promise->set_value(tl::unexpected(std::string{"unknown exception"}));
    }
  };

  if (!post(std::move(wrapper))) {
    return tl::unexpected(std::string{"queue full or shutting down"});
  }
  if (future.wait_for(deadline) != std::future_status::ready) {
    return tl::unexpected(std::string{"run_sync deadline exceeded"});
  }
  return future.get();
}

}  // namespace ros2_medkit_gateway::ros2_common
