Server Configuration
====================

This reference describes all server-related configuration options for the
ros2_medkit gateway.

Quick Start
-----------

The gateway can be configured via:

1. **Command line**: ``--ros-args -p server.port:=9000``
2. **Launch files**: ``parameters=[{'server.port': 9000}]``
3. **YAML file**: See ``src/ros2_medkit_gateway/config/gateway_params.yaml``

Out-of-range values
-------------------

Every ``server.*``, ``sse.*`` and ``docs.*`` parameter is read once, when the
gateway starts. A later ``ros2 param set`` does not change the running server.

Where a parameter documents a valid range, that range is enforced when the value
is read at startup, and an out-of-range value is never accepted in silence. What
happens next differs by parameter, in one of three ways.

**Clamped to the nearest endpoint.** The value is moved to the closest end of the
range and the gateway logs both values. This applies to the thread and pool knobs
(``server.executor_threads``, ``server.http_thread_pool_size``,
``server.keep_alive_timeout_sec``, ``sse.max_clients``,
``service_call_timeout_sec``, ``logs.buffer_size``, ``entity_cache.capacity``,
``aggregation.max_discovered_peers``, ``fault_triggers.poll_interval_ms``) and to
the ``subscription_executor.*`` and ``data_provider.*`` knobs:

.. code-block:: text

   [WARN] [rest_server]: server.http_thread_pool_size 0 clamped to 1

**Rejected, and the documented default is used instead.** Clamping makes no sense
for these, so the value is refused and the default takes its place. The warning
names the value that was refused. This applies to ``server.port``,
``refresh_interval_ms``, ``discovery.refresh_debounce_ms``,
``topic_sample_timeout_sec``, ``fault_manager.service_timeout_sec``,
``bulk_data.max_upload_size``, ``sse.max_subscriptions``,
``sse.max_duration_sec`` and ``triggers.max_triggers``:

.. code-block:: text

   [ERROR] [ros2_medkit_gateway]: Invalid port 70000. Must be between 1024-65535. Using default 8080.

**Refused at startup.** The parameters declared with a range descriptor
(``parameter_service_timeout_sec`` and ``parameter_service_negative_cache_sec``,
floating-point; ``server.bind_wait_sec``, integer) are rejected by rclcpp itself
when the value is out of range, and the gateway does not start:

.. code-block:: text

   [ros2_medkit_gateway] Fatal exception in main: parameter
   'parameter_service_timeout_sec' could not be set: Parameter
   {parameter_service_timeout_sec} doesn't comply with floating point range.

``.nan`` is the one value whose handling depends on the ROS 2 distribution. The
range check compares with ``<`` and ``>``, and every comparison against NaN is
false, so on Jazzy rclcpp reads NaN as in-range and the gateway starts; on
Lyrical it is refused like any other bad value and the gateway does not start.
Where it does get through, the gateway catches it when it reads the value, warns
and uses the default, so a NaN never reaches the code that would treat it as a
duration.

In every case the point is the same. The config file and the running process must
not disagree with nothing to show it, because the symptom of a mis-set value (a
slow or jittery gateway, a limit that never fires) surfaces far from the cause.
If the gateway behaves as though a parameter were different from what your YAML
says, its startup log names the parameter.

Note the logger name in the message. Most of these come from
``ros2_medkit_gateway``; the three that the REST server reads for itself -
``server.http_thread_pool_size``, ``server.keep_alive_timeout_sec`` and
``sse.max_duration_sec`` - come from ``rest_server``.

Network Settings
----------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.host``
     - string
     - ``"127.0.0.1"``
     - Host to bind the REST server. Without authentication only a loopback host is
       accepted (see :ref:`protected-mode`); use ``"0.0.0.0"`` for Docker or network access
       together with authentication or ``server.allow_unauthenticated_remote``.
   * - ``server.bind_wait_sec``
     - int
     - ``70``
     - Longest wait (seconds) at start for closing connections of an earlier process to release
       the bind address when no live listener holds it. TIME_WAIT normally ends 60 s after the
       close, so the default covers it; a late packet from the old peer can extend it. Range 0
       to 300; a value outside it stops the gateway at start. ``0`` means no wait. A stop (SIGINT or SIGTERM) during the wait ends
       it at once. See :ref:`protected-mode`.
   * - ``server.allow_unauthenticated_remote``
     - bool
     - ``false``
     - Let a gateway without authentication bind a non-loopback host. ``MEDKIT_AUTH_DISABLED=1``
       implies it. The gateway logs a warning at start. Launch argument:
       ``allow_unauthenticated_remote``.
   * - ``server.port``
     - int
     - ``8080``
     - Port for REST API. Valid range: 1024-65535.

.. _protected-mode:

Protected mode
^^^^^^^^^^^^^^

A gateway that answers every request anonymously (authentication off, or
``auth.require_auth_for`` ``none``) binds only the local machine. A loopback
``server.host`` (``127.0.0.0/8``, ``::1``, ``localhost``) starts as before; ``localhost`` counts
as loopback by its name, as RFC 6761 reserves it, and the gateway binds what the resolver
returns for it. Any other value (``0.0.0.0``, ``::``, a concrete address, a host name) makes the
gateway exit with status 1 and a message that names the address, unless authentication is in
force. The check uses the authentication the gateway enforces after the environment is
applied, so ``MEDKIT_JWT_SECRET`` counts as authentication on whatever ``auth.enabled`` says.
``auth.require_auth_for`` ``write`` and ``all`` count, in any letter case; ``write`` binds the
network and leaves reads anonymous. ``none`` does not count: authentication on with ``none`` is
refused like authentication off. Three ways out:

- Turn authentication on: ``MEDKIT_JWT_SECRET`` (with ``MEDKIT_CLIENTS``) or the ``auth``
  parameters with ``auth.require_auth_for`` ``write`` or ``all``; see
  :doc:`/tutorials/authentication`.
- Bind a loopback address.
- Opt out: ``server.allow_unauthenticated_remote: true``, the launch argument
  ``allow_unauthenticated_remote:=true``, or ``MEDKIT_AUTH_DISABLED=1``. Anyone who can reach
  the address then controls the robot; use it for demos and for networks you trust.

A gateway that cannot bind ``server.host`` and ``server.port`` (the address:port is held by a
live process, an address the machine does not own, a name that does not resolve, a bracketed form
such as ``[::1]``) also exits with status 1. The message names the host, the port and the
reason, and the gateway never logs its ready line. An address the machine does not own is
refused unless the kernel allows non-local binds (``net.ipv4.ip_nonlocal_bind``,
``net.ipv6.ip_nonlocal_bind``). A second gateway cannot bind an address:port a live process
holds, and the gateway never binds with ``SO_REUSEPORT``: two gateways never serve one
address:port. ``127.0.0.1:P`` and ``127.0.0.2:P`` are different addresses. A live listener on
the same address:port ends the start at once. When the bind fails with no live listener (the
first start after an upgrade from a release before this one, whose closed connections stay in
TIME_WAIT, or a process that holds the port without listening), the gateway logs one line naming
the address:port and the wait, retries the bind until it succeeds or ``server.bind_wait_sec``
ends, and then exits 1 with the time waited in the message. TIME_WAIT normally ends 60 s after
the close, which the default covers; a late packet from the old peer can extend it, and then the
wait ends with exit 1 (a restart policy starts the gateway again). A restart of this version
over its own TIME_WAIT needs no wait. A listener on ``::`` counts as holding the IPv4 addresses
of the port too, also when it is IPv6-only: the gateway then exits at once rather than risk
sharing the port.

Example:

.. code-block:: bash

   # Expose on all interfaces (Docker/network), without authentication
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p server.host:=0.0.0.0 \
       -p server.allow_unauthenticated_remote:=true \
       -p server.port:=8080

TLS/HTTPS Configuration
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.tls.enabled``
     - bool
     - ``false``
     - Enable HTTPS using OpenSSL.
   * - ``server.tls.cert_file``
     - string
     - ``""``
     - Path to PEM-encoded certificate file.
   * - ``server.tls.key_file``
     - string
     - ``""``
     - Path to PEM-encoded private key file. Restrict permissions (chmod 600).
   * - ``server.tls.ca_file``
     - string
     - ``""``
     - Path to CA certificate file (reserved for mutual TLS).
   * - ``server.tls.min_version``
     - string
     - ``"1.2"``
     - Minimum TLS version: ``"1.2"`` (compatible) or ``"1.3"`` (secure).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         tls:
           enabled: true
           cert_file: "/etc/ros2_medkit/certs/cert.pem"
           key_file: "/etc/ros2_medkit/certs/key.pem"
           min_version: "1.2"

See :doc:`/tutorials/https` for a complete HTTPS setup tutorial.

CORS Configuration
------------------

Cross-Origin Resource Sharing (CORS) settings for browser-based clients.

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``cors.allowed_origins``
     - list
     - ``[]``
     - List of allowed origins. Use ``["*"]`` for all (not recommended). CORS is
       off while this is empty, so the shipped ``gateway_params.yaml`` value of
       ``[""]`` also leaves it off: empty strings are stripped before the check.
   * - ``cors.allowed_methods``
     - list
     - ``["GET", "PUT", "POST", "DELETE", "OPTIONS"]``
     - Allowed HTTP methods.
   * - ``cors.allowed_headers``
     - list
     - ``["Content-Type", "Accept"]``
     - Allowed headers in requests.
   * - ``cors.allow_credentials``
     - bool
     - ``false``
     - Allow credentials (cookies, auth headers).
   * - ``cors.max_age_seconds``
     - int
     - ``86400``
     - Preflight response cache duration (24 hours).

Example for development with ros2_medkit_web_ui:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       cors:
         allowed_origins: ["http://localhost:5173"]
         allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
         allowed_headers: ["Content-Type", "Accept", "Authorization"]
         allow_credentials: true

Data Access Settings
--------------------

.. list-table::
   :header-rows: 1
   :widths: 35 10 10 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``data_provider.max_parallel_samples``
     - int
     - ``8``
     - Max concurrent topic samples. Higher values use more resources. Excess
       topics are processed in follow-up chunks. Range: 1-256.
   * - ``topic_sample_timeout_sec``
     - float
     - ``1.0``
     - Timeout for sampling topics with active publishers. Range: 0.1-30.0.
       The declared default is ``1.0``, which is what a bare ``ros2 run`` uses.
       ``gateway_params.yaml`` sets ``2.0``, so a launch that loads it gets that
       instead.
   * - ``parameter_service_timeout_sec``
     - float
     - ``2.0``
     - Timeout for ROS 2 parameter service calls (configurations endpoint).
       Nodes without parameter service (e.g., micro-ROS bridges) block for this
       duration before returning SERVICE_UNAVAILABLE. Range: 0.1-10.0.
   * - ``parameter_service_negative_cache_sec``
     - float
     - ``60.0``
     - After a node's parameter service fails to respond, subsequent requests
       return immediately with SERVICE_UNAVAILABLE for this duration.
       Set to 0 to disable. Range: 0-3600.
   * - ``service_call_timeout_sec``
     - int
     - ``10``
     - How long the gateway waits for a **response** to an operation RPC: a
       ROS 2 service call (``POST .../executions`` on a service-backed
       operation) and each of the three action RPCs - send goal, get result
       and cancel. Values outside the range are clamped with a warning at
       startup. Range: 1-3600.

       It is not the whole wall-clock cost, because each RPC first waits for
       its service to be discovered and the three do that differently:

       .. list-table::
          :header-rows: 1
          :widths: 30 35 35

          * - RPC
            - Discovery wait
            - Worst case in total
          * - Service call
            - none (bounded by the response wait)
            - ``service_call_timeout_sec``
          * - Action send goal
            - up to ``service_call_timeout_sec``
            - ``2 x service_call_timeout_sec``
          * - Action get result
            - up to 2 s, fixed
            - ``service_call_timeout_sec + 2 s``
          * - Action cancel
            - up to 2 s, fixed
            - ``service_call_timeout_sec + 2 s``

       The last row is the *cancel budget* referenced by
       ``DELETE .../executions/{id}`` and ``PUT .../executions/{id}`` (see
       :doc:`../api/rest`): a cancel that gets no answer within the response
       wait is reported as ``504 not-responding`` unless the action's status
       stream already shows the goal cancelling. The 2 s discovery waits are
       fixed and do not shrink with this parameter, so lowering it to the
       minimum of 1 s does not make an undiscovered cancel or get-result
       answer in under 2 s - size client timeouts off the "worst case in
       total" column, not off the parameter alone.

.. note::

   The defaults listed above are the ones the code declares. Where the shipped
   ``src/ros2_medkit_gateway/config/gateway_params.yaml`` sets a different value
   the row says so - ``topic_sample_timeout_sec`` is the one that differs today.
   If
   ``topic_sample_timeout_sec`` is out of range, ``DataAccessManager`` and the
   topic transport both fall back to ``1.0``.

Subscription Pool and Executor
------------------------------

The topic data provider keeps a pool of live subscriptions, and all subscription
create and destroy calls are funnelled through one serial worker. These knobs
bound both. Every value is clamped to its range on read, with a warning (see
`Out-of-range values`_).

.. list-table::
   :header-rows: 1
   :widths: 38 8 10 44

   * - Parameter
     - Type
     - Default
     - Description
   * - ``data_provider.max_pool_size``
     - int
     - ``256``
     - Live per-topic subscriptions the pool keeps. At the cap, sampling a new
       topic evicts the least recently used entry. Range: 1-4096.
   * - ``data_provider.cold_wait_cap``
     - int
     - ``4``
     - Concurrent HTTP workers allowed to block waiting for the first message on
       a freshly subscribed topic. Further callers get 503 and back off instead
       of saturating the request pool. Range: 0-1024.
   * - ``data_provider.idle_safety_net_sec``
     - int
     - ``900``
     - A pool entry unsampled for this long is evicted, releasing its
       subscription. ``0`` disables idle eviction. Range: 0-86400.
   * - ``data_provider.idle_sweep_tick_sec``
     - int
     - ``60``
     - How often the idle sweep runs. ``0`` disables idle eviction.
       Range: 0-3600.
   * - ``subscription_executor.max_queue_depth``
     - int
     - ``256``
     - Tasks queued on the serial worker before new posts are rejected with
       backpressure. Bounds peak memory when handler threads create
       subscriptions faster than the worker services them. Range: 16-4096.
   * - ``subscription_executor.watchdog_threshold_ms``
     - int
     - ``5000``
     - A worker task running longer than this marks the executor degraded
       (``x-medkit-subscription-executor.degraded`` on ``/health``). The flag
       clears when the task completes. Range: 100-60000.
   * - ``subscription_executor.watchdog_tick_ms``
     - int
     - ``1000``
     - How often the watchdog checks. Finer tick detects degradation sooner at
       the cost of more wakeups. Range: 10-10000.
   * - ``subscription_executor.graph_poll_tick_ms``
     - int
     - ``100``
     - How often graph events are polled (publisher appear or disappear, type
       changes). Range: 10-10000.

Fault Manager Integration
-------------------------

Configure how the gateway connects to the fault manager services and event topic.

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``fault_manager.namespace``
     - string
     - ``""``
     - Optional namespace prefix for fault manager service and event topic resolution.
       Examples: ``""`` -> ``/fault_manager/list_faults``, ``"robot1"`` -> ``/robot1/fault_manager/list_faults``.
   * - ``fault_manager.service_timeout_sec``
     - float
     - ``5.0``
     - Timeout for fault manager service calls such as ``list_faults`` and
       ``get_snapshots``. Must be finite and greater than ``0``; anything else
       is refused with a warning and ``5.0`` is used. Zero or negative would
       make every fault call report "service not available" instead of waiting,
       and a non-finite value would make it wait forever, holding an HTTP worker
       for the life of the process.
   * - ``entity_freeze_frame.enabled``
     - bool
     - ``true``
     - Zero-config freeze-frames for plugin-backed entities: when a fault from a
       plugin-owned entity confirms, snapshot the entity's data values as served
       by its plugin's DataProvider and merge them into the fault detail's
       ``environment_data.snapshots``. Faults already confirmed at gateway start
       are caught up at startup; those frames read the values at start time (not
       confirm time) and are marked ``x-medkit.capture_origin: startup``. A fault
       whose frame of the same occurrence was reloaded from the freeze-frame
       store (see ``entity_freeze_frame.storage.path`` below) is not read at
       start. The reloaded frame is served as captured. For an entity that
       reports its link down these are the plugin's last known values.
       ``connected: false`` marks them in the snapshot's ``x-medkit``
       block only when the plugin's payload carries a link flag; the OPC UA
       plugin's ``DataProvider`` payload carries none.
       Explicit snapshot config in the fault manager always wins when present.
       Only active when plugins are loaded.
       A program that hosts the gateway core without ROS sets the same switch
       as ``FreezeFrameCaptureConfig::enabled`` (see the gateway README).
   * - ``entity_freeze_frame.storage.path``
     - string
     - ``""``
     - SQLite file of the captured entity frames. A restart serves the values
       frozen when the fault confirmed. When empty, the frames go in
       ``entity_freeze_frames.db`` in the directory of ``triggers.storage.path``.
       With that empty too, or with ``triggers.storage.path`` set to
       ``:memory:``, the frames stay in memory and a restart loses them (logged
       at startup). A file that cannot be opened is logged as an error, and the
       frames stay in memory. Each row stores the fault's ``occurrence_count``
       and ``first_occurred``. At startup one fault list judges the reloaded
       frames, and none is served before that. While the fault manager does not
       answer, the list is retried every 5 s, and the first failure is logged
       as a warning. A frame of
       another occurrence, or of a fault the fault manager does not hold, is
       dropped with a warning. When the fault manager lists no fault, every
       frame is kept. A kept frame is served as captured, with its
       ``captured_at`` and ``capture_origin``. The 256-fault bound counts
       reloaded and captured frames together. One file belongs to one gateway.
       See :doc:`/tutorials/snapshots` for each case.

When ``fault_manager.namespace`` is set, the gateway also subscribes to the matching
fault event topic (for example ``/robot1/fault_manager/events`` instead of the default
``/fault_manager/events``).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       fault_manager:
         namespace: "robot1"
         service_timeout_sec: 5.0

Logging Configuration
---------------------

Configure the in-memory log buffer that collects ``/rosout`` messages.

.. list-table::
   :header-rows: 1
   :widths: 25 10 15 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``logs.buffer_size``
     - int
     - ``200``
     - Maximum log entries retained per node. Valid range: 1-100000
       (values outside this range are clamped with a warning).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       logs:
         buffer_size: 500

A ``LogProvider`` plugin can replace the default ``/rosout`` backend.
See :doc:`/tutorials/plugin-system` for details.

Performance Tuning
------------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``refresh_interval_ms``
     - int
     - ``30000``
     - Safety-backstop refresh interval (ms). Primary discovery is graph-event
       driven (polled every 100 ms); this timer forces a full refresh only if
       a graph event is missed. Range: 100-60000 (0.1s-60s).
   * - ``discovery.refresh_debounce_ms``
     - int
     - ``1000``
     - Debounce for graph-event-driven refreshes (ms). Under graph churn the
       rclcpp graph event fires many times per second; each refresh runs the
       full discovery pipeline. Coalesces graph events so a refresh runs at most
       once per this interval, bounding per-event allocation. A pending change is
       serviced within this interval plus one 100 ms poll. Range: 0-60000
       (0 disables debouncing). A value outside the range is rejected with a
       warning and the default (1000) is used.
   * - ``entity_cache.capacity``
     - int
     - ``256``
     - Fixed capacity of the thread-safe entity cache object pool, reserved once
       at startup. Steady-state refresh cycles perform zero structural allocations
       in the cache layer as long as the live entity count stays within this
       value. Valid range: 16-1000000 (values outside this range are clamped with
       a warning). Raise it for graphs larger than the default; exceeding the
       reserved capacity is harmless but triggers a one-shot WARN log. Note that a
       refresh that fully turns the entity set over allocates the new slots before
       freeing the absent old ones, so the pool transiently holds up to ~2x the
       steady-state live count - size capacity to about twice the expected peak to
       keep full-turnover ticks allocation-free.

Lower values shorten the worst-case recovery window if a graph event is missed
but increase idle CPU. The default rarely fires on a stable graph because the
graph-event poll handles node up/down events directly.

How long a departed node keeps being listed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once a node has actually left the ROS graph, ``GET /apps`` stops listing it within
roughly one refresh: at most ``discovery.refresh_debounce_ms`` plus the 100 ms graph
poll, or ``refresh_interval_ms``, whichever comes first. Every refresh rebuilds the
entity set from a live read of the graph, so this is the whole of the gateway's share,
and it is the only part of a departure the gateway can be held to. What the gateway
remembers between reads - the names it saw running, described below - decides only
whether an entry the graph still lists after its participant left is exposed; it never
keeps listing a node the graph no longer lists.

Before that point nothing here is promised, and the difference is not small:

* A node that shuts down cleanly unregisters its DDS participant on the way out, and
  the graph drops it almost at once. Measured at 225 ms from the signal to the node
  no longer being listed, polling ``GET /apps`` every 200 ms.
* A node that dies without unregistering - killed outright, or stopped before its
  shutdown could finish - costs the DDS participant lease instead, because remote
  participants have nothing to go on but its silence. Measured at 20.07 s, 19.76 s
  and 19.99 s over three runs, with stock ``rmw_fastrtps_cpp`` and no participant
  profile override.
* How long a node takes to shut down is the node's own business. The gateway cannot
  tell a node that is slow to exit from one that is simply still running, and neither
  can this bound.

So a caller waiting for a specific node to disappear should watch for its absence
rather than assume a deadline. Anything that measures a departure should establish
that the process has actually exited first, and only then hold the gateway to the
figure above.

The graph itself can also keep a node after its process has exited, for as long as the
gateway runs. The DDS RMWs take a participant's node list from discovery messages on an
internal topic, read on a listener thread of their own. When that thread takes a message
only after the participant was removed, the message puts the node back, with an empty
enclave and none of its endpoints, and nothing removes it again. This page calls such an
entry a leftover.

An empty enclave on its own does not make a leftover. A node on the far side of a DDS
router, or a node whose participant was created outside rcl (a micro-ROS agent's, for
example), also reads an empty enclave, and the router's far side resolves no endpoints
either. So the gateway leaves an entry out only for a node it has itself seen running.
Per node name, on every read of the graph:

* an entry with an enclave is listed, and the name counts as seen running;
* an entry without an enclave is dropped when another entry of the same name has one, so a
  restarted node is listed once;
* an entry without an enclave for a name the gateway has seen running is left out when the
  graph resolves no publisher or subscriber for the node (a service or client is a request
  and a reply endpoint, so it counts too);
* an entry without an enclave for a name the gateway has never seen running is listed.

A restarted node listed next to a leftover of its name needs one more step. The DDS RMWs
answer a query for one node's services from the first entry of the name, in participant
order, and that entry can be the leftover, which has no endpoints. For such a node the
gateway takes its services and actions from the writers on each service's reply topic
(``rr/<service>Reply``), which only the running node owns. A read that finds such a node
makes one endpoint query per service an App can list: parameter and other internal services
are skipped.

A leftover is left out of ``GET /apps`` and ``GET /apps/{id}``, out of the Functions
derived from namespaces, and out of the bare-name collision check that would otherwise
rename a live node sharing its name. The read that starts leaving a node out logs:

.. code-block:: text

   [WARN] [ros2_medkit_gateway]: Node '/ns/name' is not exposed: this gateway saw it running, and the ROS graph still lists it after its participant left, with no endpoints

Later reads that keep leaving it out stay quiet. The warning comes back only after a read
that did not leave the node out.

Which names the gateway saw running comes from the reads themselves. The names running on
the previous read are that read's own list, so they cost nothing however many nodes run. A
name that ran on the previous read and does not run on this one - the graph lists no entry
of it, or only entries without an enclave - has departed, and the gateway remembers it until
it runs again or until it is forgotten on a read: the first read that finds no entry of it
more than 10 seconds after the first read that found none. While an entry of it without an
enclave is listed, those 10 seconds do not start. They cover the gap between the
participant's removal and the late message: the time the listener thread takes to process a
message it already holds, measured under a millisecond on an idle host and up to 0.64 s with
its process and eight busy threads sharing two cores.

The name is forgotten only on a read. Reads come from graph changes, the refresh backstop
(``refresh_interval_ms``), the gateway's start and, in ``runtime_only`` mode, a request for an
App or a Function that a refresh removed from the entity cache while the request ran. A late
message that arrives before the read that forgets the name stays hidden, however long after
the 10 seconds it comes: the late message itself changes the graph, and the read that follows
finds its entry. Only a late message that arrives after that read is listed, like a node the
gateway never saw running.

The gateway remembers at most 1024 departed names. Past that it forgets first the names the
graph has listed no entry of for longest, and then the names that departed longest ago; a
forgotten name whose only entries are leftovers is listed again, without a warning. So once
1024 leftovers are hidden, a node that departs with no entry of it left is the first name
forgotten, and a late message for it is listed. Every remembered leftover costs two endpoint
queries on each read, so the cap also bounds that work.

A node that ran on this host and then appears only behind a DDS router, as an entry without
an enclave, is hidden like a leftover if it appears before the gateway forgets the name or
while a leftover of it is still listed.

The startup discovery summary counts peer nodes through the same memory, so in
``runtime_only`` mode, and in ``hybrid`` mode with the runtime layer enabled, it counts what
discovery would list at that moment. When discovery does not read the graph itself - in
``manifest_only`` mode, or in ``hybrid`` mode with ``discovery.runtime.enabled: false`` - the
summary's read is the first, so no node has been seen running: it counts every entry the
graph lists, except an entry without an enclave next to an entry of the same name with one.

The ``param_beacon`` plugin reads the graph itself only when discovery gives it no poll
targets, and then through a memory of its own: a node its reads saw running is not polled
once only a leftover of it is listed. It logs nothing about it.

A node restarted while a leftover of its previous instance is still listed can show no
services, and so no operations, because the graph answers per-node queries from whichever
of the two participants sorts first.

Thread Pools
------------

The gateway runs two thread pools. By default both are bounded to a small fixed
size instead of scaling with the host CPU count, so the gateway's thread
footprint is the same on a 4-core SBC and a 64-core server. A third knob,
``keep_alive_timeout_sec``, bounds how long the HTTP pool keeps a worker parked
on an idle keep-alive connection. Every value is clamped to a working minimum on
read, so a mis-set parameter can never break request serving - and the clamp is
reported, so it cannot quietly change what the gateway runs (see
`Out-of-range values`_).

.. list-table::
   :header-rows: 1
   :widths: 32 8 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.http_thread_pool_size``
     - int
     - ``6``
     - Worker threads in the HTTP request pool (cpp-httplib). Replaces the
       library default of ``max(8, cores - 1)``. Kept at or above
       ``sse.max_clients + data_provider.cold_wait_cap`` so SSE streams and cold
       ``/data`` waits cannot starve every worker (see note below); the gateway
       warns at startup if it is set below that sum. Clamped to ``[1, 1024]``.
   * - ``server.keep_alive_timeout_sec``
     - int
     - ``2``
     - How long (seconds) a request-pool worker stays parked on an idle
       keep-alive connection before freeing it. Replaces the cpp-httplib default
       of ``5``. A shorter value recovers workers from short-lived client
       connections faster (important with a small pool); a longer value favours
       connection reuse. Clamped to ``[1, 3600]``.
   * - ``server.executor_threads``
     - int
     - ``2``
     - Threads in the main rclcpp ``MultiThreadedExecutor``. Replaces rclcpp's
       default (host cores, minimum 2). With two or more threads,
       blocking-RPC responses (operation executions) are dispatched even
       while other gateway callbacks run (see note below). Clamped to
       ``[1, 256]``.

**HTTP pool, keep-alive, and SSE.** Several things hold an HTTP pool worker:
each active SSE stream (fault dashboard, cyclic subscriptions, trigger events -
see `SSE (Server-Sent Events)`_) holds one for its entire lifetime; the data
cold-wait path parks up to ``data_provider.cold_wait_cap`` workers for up to
``topic_sample_timeout_sec`` each; a bulk-data download holds one
for the whole transfer (uncounted by any cap); and on top of that each *recently
used* client connection keeps a worker parked for up to ``keep_alive_timeout_sec``
after its last request. The shipped pool default (``6``) covers the documented
worst case ``sse.max_clients (2) + data_provider.cold_wait_cap (4)``; the gateway
emits a startup warning if ``http_thread_pool_size`` is set below
``sse.max_clients + data_provider.cold_wait_cap``. The short ``keep_alive_timeout_sec``
default (``2`` s) stops a poller that opens several short-lived connections per
cycle (for example hitting ``/apps``, ``/areas`` and ``/functions`` every
iteration) from pinning the pool until the keep-alive timers expire. If you raise
``sse.max_clients``, raise ``cold_wait_cap``, or serve concurrent bulk-data
downloads, raise ``http_thread_pool_size`` to match (and keep
``keep_alive_timeout_sec`` short unless your clients benefit from long-lived
connection reuse).

**Executor threads.** The main executor delivers the gateway node's own
callbacks (timers, graph events, log and fault subscriptions) and the
service-response callbacks that complete operation/action RPC futures. The RPC
response callbacks - the generic service clients behind ``/operations``
executions and the per-action send_goal / get_result / cancel_goal client trio -
run in a shared *reentrant* callback group, so with two or more executor threads
a response is dispatched even while another gateway callback (for example a
discovery refresh pass) is running: the default of ``2`` buys real RPC-response
parallelism. Timers and the SSE-fault / trigger-fault / ``/rosout``
subscriptions stay in the node's default, *mutually-exclusive* group by design -
discovery refresh passes are serialized on purpose, and those subscriptions rely
on in-order delivery. Per-action ``/_action/status`` subscriptions use a
dedicated mutually-exclusive group of their own: ordered among themselves,
decoupled from the default group. A single executor thread remains safe (a
reentrant group does not *require* a second thread), and the blocking wait for
an RPC runs on the cpp-httplib pool thread (a separate server thread), never on
an executor thread, so it cannot deadlock the executor; the fault transport also
uses its own private executor. Raise this beyond ``2`` if many concurrent RPC
responses must be dispatched in parallel or the node's own callback load grows
(for example very frequent graph churn).

Example (more SSE clients needs a larger pool and matching ``sse.max_clients``):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         http_thread_pool_size: 16   # workers for heavy SSE + request load
         keep_alive_timeout_sec: 5   # longer reuse for steady browser clients
         executor_threads: 4
       sse:
         max_clients: 10             # raised together with the HTTP pool

Bulk Data Storage
-----------------

Configure file-based bulk data storage for uploads (calibration files, firmware, etc.).
The ``rosbags`` category is served by the Fault Manager and does not need
configuration.

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``bulk_data.storage_dir``
     - string
     - ``/tmp/ros2_medkit_bulk_data``
     - Base directory for uploaded files. Each entity gets a subdirectory.
   * - ``bulk_data.max_upload_size``
     - int
     - ``104857600``
     - Maximum upload file size in bytes (default: 100 MB). Set to ``0`` for unlimited. A negative value is refused with a warning and the default is used: it would otherwise wrap to a huge unsigned size, which is the same "no limit" a typo should never buy silently.
   * - ``bulk_data.categories``
     - string[]
     - ``[]``
     - Allowed bulk data categories for upload/download (e.g., ``calibration``, ``firmware``).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       bulk_data:
         storage_dir: "/var/ros2_medkit/bulk_data"
         max_upload_size: 52428800   # 50 MB
         categories: ["calibration", "firmware", "logs"]

.. note::

   The ``rosbags`` category is available through the Fault Manager and cannot be
   used for uploads or deletes. It is included in the category list whenever the Fault
   Manager serves rosbags, which is always the case on the ROS 2 node.

.. note::

   The gateway uses native rclcpp APIs for all ROS 2 interactions - no ROS 2 CLI
   dependencies. Topic discovery, sampling, publishing, service calls, and
   action operations are implemented in pure C++ using ros2_medkit_serialization.

.. note::

   Parameters whose value is a credential - ``auth.jwt_secret``,
   ``auth.clients`` and ``aggregation.peer_auth_header`` - are reported by the
   configurations API as ``***`` rather than their value. The parameter is
   still listed, with its real type, so it stays discoverable; only the secret
   is withheld. Set them through the node's configuration, and read them back
   from there rather than over the API.

SSE (Server-Sent Events)
------------------------

Configure limits for SSE-based streaming (fault events and cyclic subscriptions).

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``sse.max_clients``
     - int
     - ``2``
     - Maximum number of concurrent SSE connections (fault stream, cyclic subscription streams, and trigger event streams combined). Each connection pins one ``server.http_thread_pool_size`` worker for its lifetime, so this defaults at or below that pool; raise both together for more concurrent streams. Range: 1-1024, clamped with a warning.
   * - ``sse.max_subscriptions``
     - int
     - ``100``
     - Maximum number of active cyclic subscriptions across all entities. Returns HTTP 503 when this limit is reached. Must be at least ``1``; anything lower is refused with a warning and ``100`` is used, because the cap is compared as an unsigned count and a negative would remove it.
   * - ``sse.max_duration_sec``
     - int
     - ``3600``
     - Maximum allowed subscription duration in seconds. Requests exceeding this are rejected with HTTP 400. Range: 1 to 2147483647 (the value becomes an ``int``); outside it the value is refused with a warning and ``3600`` is used.
   * - ``sse.keepalive_interval_sec``
     - int
     - ``30``
     - How often an idle stream writes a keepalive comment. This is also how long a client that has gone away stays counted against ``sse.max_clients``, because a closed connection is only noticed on the next write: a gateway whose clients come and go faster than this interval can refuse a stream for a slot nobody is using. Lower it where streams are short-lived, at the cost of more traffic on idle ones. Range: 1-3600, clamped with a warning.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       sse:
         max_clients: 2
         max_subscriptions: 100
         max_duration_sec: 3600
         keepalive_interval_sec: 30

Triggers
--------

Configure condition-based push notifications for resource changes.

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``triggers.enabled``
     - bool
     - ``true``
     - Enable/disable the trigger subsystem. When disabled, trigger endpoints
       return HTTP 501.
   * - ``triggers.max_triggers``
     - int
     - ``1000``
     - Maximum number of concurrent triggers across all entities. Returns
       HTTP 503 when this limit is reached. A value below ``1`` or past what an
       ``int`` holds is refused with a warning and ``1000`` is used, because the
       cap is compared as an unsigned count and a negative would remove it.
   * - ``triggers.on_restart_behavior``
     - string
     - ``"reset"``
     - Behavior on gateway restart for persistent triggers. ``"reset"`` clears
       all triggers on restart. ``"restore"`` reloads persistent triggers from
       the storage database.
   * - ``triggers.storage.path``
     - string
     - ``""``
     - Path to SQLite database for persistent trigger storage. When empty
       (default), triggers are stored in-memory only and lost on restart.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       triggers:
         enabled: true
         max_triggers: 1000
         on_restart_behavior: "restore"
         storage:
           path: "/var/lib/ros2_medkit/triggers.db"

A program that hosts the gateway core instead of the ROS 2 node sets the same
values in ``TriggerHostConfig`` (``enabled``, ``max_triggers``,
``on_restart_behavior``, ``storage_path``). It has one more setting, which the
node does not read: ``data_poll_interval_ms`` (default ``1000``, clamped to
``100`` - ``60000`` with a warning) is the period at which a data trigger on an
item a plugin serves reads that item. A trigger has no interval of its own.

Fault-trigger rules
^^^^^^^^^^^^^^^^^^^

A separate engine evaluates threshold rules over plugin-provided data values and
raises faults from them, without a rule file. It only starts when at least one
plugin is loaded; with no plugins these parameters do nothing.

.. list-table::
   :header-rows: 1
   :widths: 32 8 14 46

   * - Parameter
     - Type
     - Default
     - Description
   * - ``fault_triggers.enabled``
     - bool
     - ``true``
     - Master switch for the rule engine.
   * - ``fault_triggers.poll_interval_ms``
     - int
     - ``1000``
     - How often the rules are evaluated. Range: 50 to 2147483647 ms. The floor
       stops the loop busy-spinning; the ceiling is what an ``int`` holds, since
       the value becomes a timer period. Either end is clamped with a warning.
       Read only when the engine actually starts, so with no plugin loaded a
       mis-set value is neither applied nor reported - it does nothing either
       way.
   * - ``fault_triggers.storage.path``
     - string
     - ``""``
     - SQLite file the rules are persisted in. Empty keeps them in memory, so
       they are lost on restart.

Rate Limiting
-------------

Token-bucket-based rate limiting for API requests. Disabled by default.

.. list-table::
   :header-rows: 1
   :widths: 35 10 15 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``rate_limiting.enabled``
     - bool
     - ``false``
     - Enable rate limiting.
   * - ``rate_limiting.global_requests_per_minute``
     - int
     - ``600``
     - Maximum RPM across all clients combined.
   * - ``rate_limiting.client_requests_per_minute``
     - int
     - ``60``
     - Maximum RPM per client IP.
   * - ``rate_limiting.endpoint_limits``
     - string[]
     - ``[]``
     - Per-endpoint overrides as ``"pattern:rpm"`` strings.
       Pattern uses ``*`` as single-segment wildcard
       (e.g., ``"/api/v1/*/operations/*:10"``).
   * - ``rate_limiting.client_cleanup_interval_seconds``
     - int
     - ``300``
     - How often to scan and remove idle client tracking entries (seconds).
   * - ``rate_limiting.client_max_idle_seconds``
     - int
     - ``600``
     - Remove client entries idle longer than this (seconds).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       rate_limiting:
         enabled: true
         global_requests_per_minute: 600
         client_requests_per_minute: 60
         endpoint_limits: ["/api/v1/*/operations/*:10"]

See :doc:`/api/rest` for rate limiting response headers and 429 behavior.

Authentication
--------------

JWT-based authentication with Role-Based Access Control (RBAC). Disabled by
default for local development.

.. list-table::
   :header-rows: 1
   :widths: 35 10 25 30

   * - Parameter
     - Type
     - Default
     - Description
   * - ``auth.enabled``
     - bool
     - ``false``
     - Enable/disable JWT authentication. ``MEDKIT_JWT_SECRET`` turns it on and ``MEDKIT_AUTH_DISABLED=1`` turns it off, whatever this says. Reads back the value in force. With it off, or on with ``auth.require_auth_for`` ``none``, the gateway binds only a loopback host unless ``server.allow_unauthenticated_remote`` is set (:ref:`protected-mode`).
   * - ``auth.jwt_secret``
     - string
     - ``""``
     - JWT signing secret. For HS256: the shared secret string. For RS256: path to the private key file (PEM format), read once at startup, so a rotated key takes effect at the next restart. ``MEDKIT_JWT_SECRET`` overrides it; set but empty, that variable stops the gateway, and with RS256 it is refused. Read back as ``<from MEDKIT_JWT_SECRET>``, ``<set at start>`` or empty, never as the value.
   * - ``auth.jwt_public_key``
     - string
     - ``""``
     - Path to public key file for RS256. Required for RS256, optional for HS256. Read once at startup, so a rotated key takes effect at the next restart.
   * - ``auth.jwt_algorithm``
     - string
     - ``"HS256"``
     - JWT signing algorithm: ``"HS256"`` (symmetric) or ``"RS256"`` (asymmetric).
   * - ``auth.token_expiry_seconds``
     - int
     - ``3600``
     - Access token validity period in seconds (1 hour). A refresh record is kept until its expiry plus this value, and a revoked one minted here until the later of the last access token ``exp`` reserved from it and ``min(expires_at, revoked_at)`` plus this value, so a revocation holds until every access token minted from it has expired. A revocation of a token minted elsewhere is kept until that token's ``exp`` plus this value. Under ``aggregation.forward_auth`` a peer's value must be at least the issuer's, or a revocation the peer records for the issuer's token lapses while that token still verifies.
   * - ``auth.refresh_token_expiry_seconds``
     - int
     - ``86400``
     - Refresh token validity period in seconds (24 hours). Must be >= ``token_expiry_seconds``. It does not bound a revocation recorded here for a token another gateway issued: that record is kept to the token's own ``exp``.
   * - ``auth.max_refresh_tokens_per_client``
     - int
     - ``100``
     - Refresh records kept per client, revoked ones included, 1 to 100000; outside that range the gateway does not start. A login at the cap drops the client's oldest unrevoked refresh token, which then stops refreshing; access tokens minted from it stay valid until they expire. The cap never drops a revoked record; the sweep drops it after its horizon. When revoked records fill the cap, a login answers 429 with ``Retry-After`` until the oldest is swept; that blocks only this client. Revocations of tokens without a login record here (minted elsewhere, or dropped by this cap) have a count of their own, also capped at this value; they never block a login. When they fill it, ``POST /auth/revoke`` answers 503 with ``Retry-After`` and records nothing. A web UI or tool that shares one client id across many sessions needs a value above its session count.
   * - ``auth.require_auth_for``
     - string
     - ``"write"``
     - When to require authentication: ``"none"`` (auth endpoints still available), ``"write"`` (POST/PUT/DELETE only), or ``"all"`` (every request). ``MEDKIT_JWT_SECRET`` forces ``"all"``. Reads back the value in force, ``"none"`` while authentication is off.
   * - ``auth.issuer``
     - string
     - ``"ros2_medkit_gateway"``
     - JWT issuer claim (``iss`` field in tokens).
   * - ``auth.clients``
     - string[]
     - ``[]``
     - Pre-configured clients as ``"client_id:client_secret:role"`` strings. ``MEDKIT_CLIENTS`` replaces the list when ``MEDKIT_JWT_SECRET`` is set. Read back as ``<id>:<sentinel>:<role>``. With authentication on, a secret equal to a placeholder (``<set at start>``, ``<from MEDKIT_CLIENTS>``, ``<from MEDKIT_JWT_SECRET>``) stops the gateway; with it off no client is registered.

.. note::

   The three variables ``MEDKIT_JWT_SECRET``, ``MEDKIT_CLIENTS`` and
   ``MEDKIT_AUTH_DISABLED`` are read by the gateway node and override every
   ``auth.*`` source; see :doc:`/tutorials/authentication`. All ``auth.*``
   parameters and ``aggregation.peer_auth_header`` are read once at start: a
   ``ros2 param set`` or ``ros2 param load`` of them is refused.

.. note::

   RBAC roles and their permissions:

   - **viewer** - Read-only access (GET on areas, components, data, faults)
   - **operator** - Viewer + trigger operations, acknowledge faults, publish data
   - **configurator** - Operator + modify/reset configurations
   - **admin** - Full access including auth management

.. danger::

   The example below uses **placeholder secrets for illustration only**.
   In production, generate secrets with ``openssl rand -base64 32`` and
   never commit them to configuration files. Use environment variable
   substitution or a secrets manager.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       auth:
         enabled: true
         jwt_secret: "CHANGE-ME-use-openssl-rand-base64-32"
         jwt_algorithm: "HS256"
         require_auth_for: "write"
         token_expiry_seconds: 3600
         clients: ["admin:REPLACE_WITH_STRONG_SECRET:admin", "viewer:REPLACE_WITH_STRONG_SECRET:viewer"]

What a token is checked against
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

An access token is accepted when the signature verifies under the configured
secret or public key, the issuer matches, the expiry is in the future, and the
client named in ``sub`` exists in this gateway's ``auth.clients`` and is
enabled. The role it grants is the one **this** gateway's ``auth.clients``
gives that client. The ``role`` claim in the token is not read: any value, or
none, is accepted.

The gateway keeps a record of the refresh tokens it issues, in process memory;
at ``auth.max_refresh_tokens_per_client`` a login drops the client's oldest
unrevoked one. The records are a **denylist**: a record marked revoked refuses
the access tokens minted from it, and a token with no record here is judged on
the checks above. So a token issued before a restart, or by another gateway that shares
this JWT configuration, is accepted.

``POST /auth/revoke`` revokes refresh tokens only; for any other token it gives
the same answer and records nothing. It revokes a refresh token this gateway did
not issue by writing a revoked record for it, when the signature and issuer
verify here and ``sub`` is a client in ``auth.clients``; for any other subject
it records nothing. A record this gateway holds is revoked only by a token
whose ``sub`` is the record's client. Revocation ignores ``exp``, ``iat`` and
``nbf``: a refresh token past its expiry can still have live access tokens,
and an issuer whose clock runs ahead stamps times this gateway has not reached
yet.

Each record has a horizon, and a sweep drops it once the horizon has passed.
Sweeps run at login, at refresh and every 5 minutes.

- An unrevoked record: its expiry plus ``auth.token_expiry_seconds``.
- A revoked record minted here: the later of the last access token ``exp``
  reserved from it and ``min(expires_at, revoked_at)`` plus
  ``auth.token_expiry_seconds``. After a revocation no refresh reserves a new
  access token from the record; a refresh that reserved before it can still
  return a token, and that token is refused. The reserved ``exp`` covers a
  clock that stepped back between a mint and the revocation; the second term
  covers a clock that jumped forward before the revocation and comes back.
  Access-token expiry is wall-clock time, so a clock that jumps forward past a
  record's horizon and then back can end a revocation early.
- A revocation of a token with no login record here (minted by another
  gateway, or dropped by the cap): that token's own ``exp`` plus
  ``auth.token_expiry_seconds``. The issuer may still mint from the token
  until ``exp``. The record and the token's ``exp`` are read against the same
  clock, so a clock difference between gateways does not shorten it.

Revoked login records count toward ``auth.max_refresh_tokens_per_client`` and
the cap never drops one. When they fill it, a login answers 429 with
``Retry-After``, the seconds until the oldest is swept. Such a client blocks
only itself.

Revocations of tokens without a login record form a pool per client, counted
apart from its login records and capped at the same
``auth.max_refresh_tokens_per_client``. They never block a login. Nothing is
dropped to make room: with the pool full, ``POST /auth/revoke`` answers 503
with ``Retry-After``, the seconds until the oldest is swept, and records
nothing.

A stream or download this gateway serves - the fault stream, a cyclic
subscription or trigger stream, a bulk-data or recording download - keeps the
access token it was opened with and checks it again before each keepalive,
event or chunk it writes. Once the token no longer validates (revoked, expired,
or the client disabled), the stream ends at that write. The fault stream writes
a keepalive every ``sse.keepalive_interval_sec``; cyclic subscription and
trigger streams every 15 s, and a cyclic stream also writes a frame per sample.
A request this gateway forwards to a peer completes like any other request.

Records do not survive a restart: **a revoked token is honoured again after a
restart**, for at most ``auth.token_expiry_seconds``. Where a revocation must
survive a restart, disable the client in ``auth.clients``, which is checked on
every request.

See :doc:`/tutorials/authentication` for a complete setup tutorial.

Plugin Framework
----------------

Extend the gateway with custom plugins loaded from shared libraries (``.so``).
Plugins can implement provider interfaces (e.g., ``UpdateProvider``, ``IntrospectionProvider``)
that are automatically detected and wired into the gateway's subsystem managers.

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``plugins``
     - string[]
     - ``[]``
     - List of plugin names to load. Each plugin requires a corresponding ``plugins.<name>.path`` parameter.
   * - ``plugins.<name>.path``
     - string
     - (required)
     - Absolute path to the plugin ``.so`` file. Must exist and have ``.so`` extension.
   * - ``plugins.<name>.<key>``
     - any
     - (none)
     - Plugin config, read at start and passed to ``configure()``. Not served as a ROS parameter.

Plugin loading lifecycle:

1. Shared library is loaded via ``dlopen`` with ``RTLD_NOW | RTLD_LOCAL``
2. API version is checked (must match gateway headers)
3. ``create_plugin()`` factory is called to instantiate the plugin
4. Provider interfaces are queried via ``extern "C"`` functions
5. ``configure()`` is called with per-plugin config
6. ``set_context()`` passes the gateway context to the plugin
7. ``get_routes()`` returns custom REST endpoint definitions as ``vector<PluginRoute>``

Error isolation: if a plugin throws during any lifecycle call, it is disabled
without crashing the gateway. Other plugins continue to operate normally.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

Software Updates
----------------

Configure the software updates system. Updates are disabled by default.
When enabled, a plugin implementing ``UpdateProvider`` is required to provide
the backend functionality (see `Plugin Framework`_ above).

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``updates.enabled``
     - bool
     - ``false``
     - Enable/disable the software updates backend. The ``/updates`` routes are
       always registered and always documented; when this is ``false`` - or when
       no ``UpdateProvider`` plugin is loaded - every one of them answers
       ``501``.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_update_plugin"]
       plugins.my_update_plugin.path: "/opt/ros2_medkit/lib/libmy_update_plugin.so"
       updates:
         enabled: true

Complete Example
----------------

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         host: "0.0.0.0"
         port: 8080
         allow_unauthenticated_remote: true  # no auth.* below, so opt out of protected mode
         tls:
           enabled: false

       refresh_interval_ms: 5000
       topic_sample_timeout_sec: 3.0

       data_provider:
         max_parallel_samples: 30

       cors:
         allowed_origins: ["http://localhost:5173", "https://dashboard.example.com"]
         allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
         allowed_headers: ["Content-Type", "Accept", "Authorization"]
         allow_credentials: true
         max_age_seconds: 86400

       bulk_data:
         storage_dir: "/var/ros2_medkit/bulk_data"
         max_upload_size: 104857600
         categories: ["calibration", "firmware"]

       sse:
         max_clients: 2
         max_subscriptions: 100
         max_duration_sec: 3600

       triggers:
         enabled: true
         max_triggers: 1000
         on_restart_behavior: "reset"

       logs:
         buffer_size: 200

       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

       updates:
         enabled: true

       auth:
         enabled: true
         jwt_secret: "CHANGE-ME-use-openssl-rand-base64-32"
         jwt_algorithm: "HS256"
         require_auth_for: "write"
         clients: ["admin:REPLACE_WITH_STRONG_SECRET:admin"]

       rate_limiting:
         enabled: false

       scripts:
         scripts_dir: "/var/ros2_medkit/scripts"
         allow_uploads: true
         max_concurrent_executions: 5

API Documentation
-----------------

Configure the self-describing OpenAPI capability description endpoint.

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``docs.enabled``
     - bool
     - ``true``
     - Enable/disable ``/docs`` capability description endpoints. When disabled,
       all ``/docs`` endpoints return HTTP 501.

**Build option:** ``ENABLE_SWAGGER_UI``

Set ``-DENABLE_SWAGGER_UI=ON`` during CMake configure to embed Swagger UI assets
in the gateway binary. This provides an interactive API browser at
``/api/v1/swagger-ui``. Requires network access to download assets from unpkg.com
during configure. Disabled by default.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       docs:
         enabled: true

Scripts
-------

Diagnostic scripts: upload, manage, and execute scripts on entities. Set
``scripts.scripts_dir`` to a directory path to enable the feature. When left
empty, all script endpoints return HTTP 501.

.. list-table::
   :header-rows: 1
   :widths: 35 10 15 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``scripts.scripts_dir``
     - string
     - ``""``
     - Directory for storing uploaded scripts. Empty string disables the feature.
   * - ``scripts.allow_uploads``
     - bool
     - ``true``
     - Allow uploading scripts via HTTP. Set to ``false`` for hardened deployments that only use manifest-defined scripts.
   * - ``scripts.max_file_size_mb``
     - int
     - ``10``
     - Maximum uploaded script file size in megabytes.
   * - ``scripts.max_concurrent_executions``
     - int
     - ``5``
     - Maximum number of scripts executing concurrently.
   * - ``scripts.default_timeout_sec``
     - int
     - ``300``
     - Default timeout per execution in seconds (5 minutes).
   * - ``scripts.max_execution_history``
     - int
     - ``100``
     - Maximum number of completed executions to keep in memory. Oldest completed entries are evicted when this limit is exceeded.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       scripts:
         scripts_dir: "/var/ros2_medkit/scripts"
         allow_uploads: true
         max_file_size_mb: 10
         max_concurrent_executions: 5
         default_timeout_sec: 300

Locking
-------

SOVD resource locking (ISO 17978-3, Section 7.17). Clients acquire locks on
components and apps to prevent concurrent modification.

.. list-table::
   :header-rows: 1
   :widths: 40 15 15 30

   * - Parameter
     - Type
     - Default
     - Description
   * - ``locking.enabled``
     - bool
     - ``true``
     - Enable the LockManager and lock endpoints
   * - ``locking.default_max_expiration``
     - int
     - ``3600``
     - Maximum lock TTL in seconds
   * - ``locking.cleanup_interval``
     - int
     - ``30``
     - Seconds between expired lock cleanup sweeps. Values below ``1`` are raised to ``1``. The sweep runs on a thread of the gateway core (``LockReaper``).
   * - ``locking.defaults.components.lock_required_scopes``
     - [string]
     - ``[""]``
     - Collections requiring a lock on components (empty = no requirement)
   * - ``locking.defaults.components.breakable``
     - bool
     - ``true``
     - Whether component locks can be broken
   * - ``locking.defaults.apps.lock_required_scopes``
     - [string]
     - ``[""]``
     - Collections requiring a lock on apps (empty = no requirement)
   * - ``locking.defaults.apps.breakable``
     - bool
     - ``true``
     - Whether app locks can be broken

Per-entity overrides are configured in the manifest ``lock:`` section.
See :doc:`manifest-schema` and :doc:`/api/locking`.

See Also
--------

- :doc:`/tutorials/authentication` - JWT authentication setup
- :doc:`/tutorials/https` - HTTPS configuration
- :doc:`discovery-options` - Discovery and entity mapping options
