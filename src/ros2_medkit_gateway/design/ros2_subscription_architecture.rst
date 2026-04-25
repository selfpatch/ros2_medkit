ROS 2 Subscription Architecture
================================

Background
----------

The gateway's original ``NativeTopicSampler`` created a short-lived
``rclcpp::GenericSubscription`` for every ``/data`` sample call. Under concurrent
HTTP load, multiple cpp-httplib handler threads invoked
``Node::create_generic_subscription()`` on the same node, racing inside rcl's
internal hash map and corrupting its linked entries. The symptom was a Rolling
SIGSEGV in ``test_data_read`` (issue #375) and intermittent non-deterministic
crashes on sample-heavy deployments.

This document describes the replacement architecture: a single-writer
subscription executor, a RAII slot handle, and a per-topic pool.

Goals
-----

1. Eliminate the rcl mutation race by serializing all subscription / callback
   group creation on one worker thread.
2. Amortize subscription cost for hot topics - subscribe once per topic, serve
   many samples from the cached latest message.
3. Stay embedded-friendly: no new threads beyond the serial worker, bounded
   queues and pools, deterministic shutdown bounded by the longest in-flight
   task.
4. Preserve SOVD REST contract; the changes are entirely below the handler
   layer.

Layering
--------

::

   +-----------------------------+      +----------------------------+
   | HTTP handlers (data, etc.)  |      | DiscoveryManager, etc.     |
   +-----------------------------+      +----------------------------+
                |  TopicDataProvider&  (pure C++ interface, no rclcpp)
                v
   +----------------------------------------------------------------+
   | Ros2TopicDataProvider            (data domain, ROS 2-specific) |
   |   per-topic pool, LRU, idle sweep, graph-change eviction       |
   +----------------------------------------------------------------+
                |  Ros2SubscriptionSlot  (RAII handle)
                v
   +----------------------------------------------------------------+
   | Ros2SubscriptionExecutor          (ros2_common, shared infra)  |
   |   one worker thread, bounded queue, watchdog, graph polling    |
   +----------------------------------------------------------------+
                |  rclcpp::Node (dedicated "_sub" subscription node)
                v
   +----------------------------------------------------------------+
   | rclcpp / rcl                                                   |
   +----------------------------------------------------------------+

- Handlers and business managers depend only on ``TopicDataProvider``
  (``include/ros2_medkit_gateway/data/topic_data_provider.hpp``). No
  ``#include <rclcpp/...>`` is required to consume the interface.
- ``Ros2TopicDataProvider`` is the statically-linked default implementation.
  Alternate transports (Zenoh, XRCE-DDS, mocks) can plug in with a different
  class implementing the same interface.

Ros2SubscriptionExecutor
------------------------

Owns:

- One dedicated ``std::thread`` (the worker).
- One ``rclcpp::Node`` (the subscription node, suffixed ``_sub``) exclusively
  owned by this executor's internal ``SingleThreadedExecutor``. The gateway's
  main ``MultiThreadedExecutor`` never sees this node; creation, destruction
  and callback dispatch of every subscription on it run on the single worker
  thread, preserving the single-writer invariant against rcl's hash-map.
- A bounded task queue guarded by ``queue_mtx_`` + ``queue_cv_``.
- One ``aux`` thread driving the watchdog and graph-event polling ticks on
  their own cadence. The aux thread only touches atomics and the
  subscription node's graph event; it never mutates the node's subscription
  list so it cannot race the worker's create / destroy path.

Public API (summary):

- ``run_sync<R>(std::function<R()>, deadline)`` - execute on the worker, wait
  with deadline, return ``tl::expected<R, string>``.
- ``post(std::function<void()>)`` - fire-and-forget. Returns ``false`` when the
  queue is at capacity (backpressure) or the executor is shutting down.
- ``on_graph_change(cb)`` / ``remove_graph_change(token)`` - register /
  deregister callbacks that fire on the worker when the ROS 2 graph changes.
  Pre-allocated slot array (16 slots, Tier 1 memory guarantee).
- ``is_shutting_down()`` / ``stats()`` - observability.

Graph change detection uses the public ``rclcpp::Node::get_graph_event()`` API
and a wall timer polling ``event->check_and_clear()`` on the subscription
node. Internal ``GraphListener`` is intentionally avoided - it is not a stable
public API across Humble / Jazzy / Rolling.

Shutdown drains the queue: the worker processes any already-enqueued tasks so
pending ``run_sync`` promises are fulfilled before the worker exits. Bounded
by the longest in-flight task (individual ``run_sync`` deadlines cap worst
case).

Ros2SubscriptionSlot
--------------------

RAII handle returned by ``create_typed<MessageT>`` or ``create_generic``. On
destruction:

- If the executor has entered shutdown (``is_shutting_down()``), the slot
  releases its ``SubscriptionBase::SharedPtr`` on the current thread. The
  subscription node's ``rcl_node_fini`` during executor teardown cleans up the
  rcl handle atomically. No ``run_sync`` call, no hang.
- Otherwise, the slot posts a destroy task via ``run_sync`` with a 30 s
  deadline. On timeout, the slot logs an ERROR and releases on the current
  thread anyway - we prefer a narrow rcl race risk over hanging gateway
  shutdown.

``release_without_executor`` is not exposed publicly. The destructor makes the
decision internally; shrinking the public surface is intentional for future
safety analysis.

Ros2TopicDataProvider (pool)
----------------------------

Per-entry structure::

    struct PoolEntry {
      std::string topic;
      std::string cached_type;
      std::unique_ptr<Ros2SubscriptionSlot> slot;
      std::mutex buf_mtx;                // protects latest, latest_ns
      std::condition_variable buf_cv;
      std::optional<rclcpp::SerializedMessage> latest;
      std::int64_t latest_ns;
      std::chrono::steady_clock::time_point last_sample_time;
      std::atomic<bool> shutdown;
    };

``sample(topic, timeout)`` flow:

1. Graph query for publisher count / type. If no publishers, return
   metadata-only immediately.
2. Pool lookup under ``pool_mtx_``. Hit: promote the topic to the MRU end of
   the LRU list and return the shared entry.
3. Miss: build the entry shell, create the slot via
   ``Ros2SubscriptionSlot::create_generic`` (this goes through the executor's
   ``run_sync``), then insert under ``pool_mtx_``. If the pool is at cap,
   evict the LRU entry first.
4. Lock ``entry->buf_mtx``. If ``latest`` present, deserialize and return. If
   cold and ``concurrent_cold_waits_ >= cold_wait_cap``, return ``tl::unexpected``
   with ``ERR_X_MEDKIT_COLD_WAIT_CAP_EXCEEDED`` (HTTP 503, retryable with
   backoff). Otherwise wait on ``buf_cv`` up to ``timeout``.
5. Deserialize the latest message via ``JsonSerializer`` and return.

The callback registered with the slot captures a
``std::weak_ptr<PoolEntry>``. If the entry is evicted, the callback no-ops
safely rather than touching freed memory.

Eviction
^^^^^^^^

Four independent eviction triggers keep the pool size bounded:

1. **Graph change** - topics that disappeared from the graph or whose type
   changed are evicted on the next graph event. Worker-scheduled callback.
2. **LRU** - at ``max_pool_size``, a new miss evicts the least-recently-used
   entry.
3. **Idle safety net** - a wall timer (every ``idle_sweep_tick``, default
   1 min) drops entries whose ``last_sample_time`` is older than
   ``idle_safety_net`` (default 15 min).
4. **Shutdown** - the provider destructor signals every per-entry CV, drops
   the pool map + LRU structures, then releases each slot.

All four paths release slots outside ``pool_mtx_`` so the slot destructor's
``run_sync`` can not deadlock against a sample thread waiting on the pool
mutex.

Cold-wait cap
^^^^^^^^^^^^^

cpp-httplib serves requests on a bounded worker pool (typically 4-8 threads).
Without protection, N concurrent cold-topic samples can saturate that pool and
starve ``/health``, ``/components``, and other endpoints.

``concurrent_cold_waits_`` tracks how many sample calls are currently blocked
on a cold entry's CV. When the count reaches ``cold_wait_cap`` (default 4),
further cold callers fail fast with ``ERR_X_MEDKIT_COLD_WAIT_CAP_EXCEEDED``
(HTTP 503, ``params.cold_wait_cap`` set, retry with backoff). Existing waiters
complete normally; warm topics are unaffected. Returning a structured error
instead of a metadata-only success lets clients distinguish back-pressure
("retry me") from "no publishers yet" ("nothing to wait for").

QoS matching
^^^^^^^^^^^^

The pool queries the publisher QoS via
``get_publishers_info_by_topic`` on each miss and derives a subscriber QoS
that is compatible:

- Reliable if any publisher is Reliable, else BestEffort.
- TransientLocal if any publisher is TransientLocal, else Volatile. (The
  latter is important for latched ``status`` topics - a volatile subscriber
  would miss the latched last value.)
- ``keep_last`` depth 1 always. The pool only keeps the newest serialized
  message; deeper queues would just allocate and discard.

Main assembly
-------------

``main.cpp`` constructs the chain after the gateway node is added to the
executor::

    auto node = std::make_shared<GatewayNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    auto sub_exec     = std::make_shared<Ros2SubscriptionExecutor>(node);
    auto serializer   = std::make_shared<JsonSerializer>();
    auto data_provider = std::make_shared<Ros2TopicDataProvider>(sub_exec, serializer);
    node->set_topic_data_provider(data_provider);

    executor.spin();

    data_provider.reset();       // drops pool entries through sub_exec
    sub_exec.reset();            // joins worker + aux, destroys subscription node
    executor.remove_node(node);  // detach before ~GatewayNode runs
    node.reset();                // ~GatewayNode with executor still alive
    rclcpp::shutdown();

``remove_node`` + explicit ``node.reset()`` are required on rolling / newer
jazzy: rclcpp aborts with ``Node ... needs to be associated with an executor``
if ``~GatewayNode`` touches a service client while stack-unwind has already
destroyed the executor.

Regression gate
---------------

``scripts/check_no_naked_subscriptions.sh`` fails CI if any ``.cpp`` or
``.hpp`` outside ``ros2_common/`` or ``test/`` introduces a new call to
``create_generic_subscription``, ``create_subscription<``, or
``create_callback_group``. New production code must route through
``Ros2SubscriptionSlot``.

Known-legacy call sites (fault subscribers, operation manager, log manager,
trigger_topic_subscriber) are explicitly allowlisted and tracked for
migration under Phase C of the provider architecture rollout.

Observability
-------------

``GET /health`` includes two vendor-extension sections populated from atomic
reads only (never blocks):

- ``x-medkit-subscription-executor`` - worker_alive, degraded, queue depth /
  dropped, tasks completed / failed, latency, watchdog trips.
- ``x-medkit-data-provider`` - pool_size / cap, pool hits / misses, evictions,
  graph_events_received, concurrent_cold_waits.

External monitors (k8s liveness probe, Docker HEALTHCHECK, systemd watchdog)
trigger on ``x-medkit-subscription-executor.degraded`` without deployment-
specific assumptions.
