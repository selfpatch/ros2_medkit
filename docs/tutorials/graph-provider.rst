Graph Provider Plugin
======================

The ``ros2_medkit_graph_provider`` plugin serves ``x-medkit-graph``: a per-Function
dataflow graph of ROS 2 apps (nodes) and topic connections (edges), enriched with
frequency, latency, and drop-rate metrics. It answers the question an entity tree
alone cannot: *is data actually flowing through this pipeline, and if not, where is
it stuck?*

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Each ``x-medkit-graph`` document is scoped to a single SOVD Function. It models:

- **nodes** - the Function's hosted Apps, each with a ``node_status``
  (``reachable`` / ``unreachable``).
- **edges** - publisher/subscriber topic connections between those Apps, each
  carrying ``frequency_hz``, ``latency_ms``, ``drop_rate_percent``, and a
  ``metrics_status``.
- **pipeline_status** - the Function's aggregate health, rolled up from every edge.

The metrics themselves are **not** measured by the plugin directly (it does not
subscribe to your data topics). They come entirely from a ``/diagnostics``
producer running elsewhere in your system. Getting that producer right is the
single most important prerequisite - read the next section before doing
anything else.

Prerequisites
-------------

The ``/diagnostics`` producer contract
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin subscribes to ``/diagnostics`` (``diagnostic_msgs/msg/DiagnosticArray``)
and keys its per-topic metrics map on ``DiagnosticStatus.name``. **That name must be
the exact, fully-qualified ROS topic name** (leading ``/``, no aliasing, no
shortened form) - byte-for-byte the same string ``ros2 topic list`` would show for
the topic the graph provider should attach the sample to.

Recognized ``DiagnosticStatus.values`` keys:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Key
     - Mapped to
   * - ``frame_rate_msg``
     - ``frequency_hz`` (primary source; see the header-stamp note below)
   * - ``frame_rate_node``
     - ``frequency_hz`` fallback, used only when ``frame_rate_msg`` is present
       but exactly ``0``
   * - ``current_delay_from_realtime_ms``
     - ``latency_ms``
   * - ``expected_frequency``
     - The topic's expected frequency for this sample. When present, it wins
       over any graph-provider-side configuration (global default or
       per-function override) for that topic.
   * - ``drop_rate_percent`` / ``drop_rate``
     - ``drop_rate_percent``
   * - ``total_dropped_frames``
     - Recognized (keeps the sample from being treated as irrelevant) but not
       currently mapped to any output field.

A negative ``frame_rate_msg`` or ``frame_rate_node`` (a "not measured yet"
sentinel some producers emit) is rejected outright and never used.

The header-stamp requirement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``frame_rate_msg`` is usually computed from a message's header timestamp. Message
types with **no header field** cannot be rated that way, so a compliant producer
reports ``frame_rate_msg: 0`` for them and puts the real measured rate in
``frame_rate_node`` instead. The graph provider falls back to ``frame_rate_node``
only in that exact case (``frame_rate_msg`` present and exactly ``0``) - never for
a missing ``frame_rate_msg``, and never overriding a positive ``frame_rate_msg``.

The failure mode, stated plainly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**No ``/diagnostics`` producer publishing for a topic means every edge on that
topic reports ``metrics_status: "pending"`` forever.** It never transitions to
``"active"`` or ``"error"`` on its own - there is no timeout that turns "no data
was ever seen" into an error state, by design (an edge that was never observed is
not the same failure as one that stopped being observed). If you bring up the
graph provider and see every edge stuck on ``pending``, this is almost always the
cause - see :ref:`graph-provider-pending-troubleshooting` below.

Reference producer: ``greenwave_monitor``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``greenwave_monitor`` (package and executable both named ``greenwave_monitor``) is
the reference ``/diagnostics`` producer used by this project's own integration
tests. It resolves its monitored-topic list **once, at startup**, and never
re-resolves it - a topic with no live publisher yet when it starts is silently
skipped for the rest of its lifetime. Always start it strictly after the
publishers of every topic it should monitor are already advertised in the ROS
graph.

.. code-block:: bash

   ros2 run greenwave_monitor greenwave_monitor --ros-args \
     -p gw_monitored_topics:="['/powertrain/engine/temperature']" \
     -p gw_time_check_preset:="header_with_nodetime_fallback"

Parameter shape:

- ``gw_monitored_topics`` (list of strings) - fully-qualified topic names to
  monitor with no per-topic expected-frequency tracking.
- ``gw_frequency_monitored_topics`` (dict) - ``topic -> {expected_frequency,
  tolerance}``. When set for a topic, the ``expected_frequency`` stamped into
  ``/diagnostics`` wins over any graph-provider-side configuration for that
  topic (see :doc:`/config/graph-provider`).
- ``gw_time_check_preset`` (string) - e.g. ``"header_with_nodetime_fallback"``,
  matching greenwave's own example configuration.

``greenwave_monitor`` also publishes ``/diagnostics`` at only ~1 Hz, with the
first useful (non-transient) reading typically arriving a few seconds after
startup - budget your polling accordingly.

Turning the plugin on
----------------------

The graph provider is a standalone ``GatewayPlugin`` - see :doc:`plugin-system`
for the general plugin loading mechanism. ``gateway.launch.py`` auto-detects
``ros2_medkit_graph_provider`` at launch time: if the package is installed
alongside the gateway, the plugin is loaded automatically with no configuration
required. To load it manually (a custom launch setup, or a non-default install
layout):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["graph_provider"]
       plugins.graph_provider.path: "/path/to/lib/ros2_medkit_graph_provider/libros2_medkit_graph_provider_plugin.so"

See :doc:`/config/graph-provider` for the full set of tunable thresholds.

Functions are populated by manifest **and** by runtime discovery - namespaces
turn into Functions automatically (``create_functions_from_namespaces``,
enabled by default). The ``x-medkit-graph`` endpoint is not manifest-only: any
Function, however it was discovered, gets the capability.

The Discovery Path
-------------------

1. **List functions** to find the one you care about:

   .. code-block:: bash

      curl http://localhost:8080/api/v1/functions | jq

2. **Read the Function's detail** and follow its capability href. Every
   Function detail response carries an ``"x-medkit-graph"`` link:

   .. code-block:: bash

      curl http://localhost:8080/api/v1/functions/engine-monitoring | jq '."x-medkit-graph"'
      # "/api/v1/functions/engine-monitoring/x-medkit-graph"

3. **GET the graph** at that href.

A Worked Example
-----------------

The example below matches this project's own end-to-end test topology: a
Function ``engine-monitoring`` hosting an ``engine-temp-sensor`` App (publishes
``/powertrain/engine/temperature`` at 2 Hz) and an ``engine-temp-monitor`` App
(subscribes to it), with a real ``greenwave_monitor`` watching that topic.

.. code-block:: bash

   curl http://localhost:8080/api/v1/functions/engine-monitoring/x-medkit-graph | jq

.. code-block:: json

   {
     "x-medkit-graph": {
       "schema_version": "2.0.0",
       "graph_id": "engine-monitoring-graph",
       "timestamp": "2026-07-20T14:03:11.204Z",
       "scope": {
         "type": "function",
         "entity_id": "engine-monitoring"
       },
       "pipeline_status": "healthy",
       "bottleneck_edge": null,
       "topics": [
         {
           "topic_id": "topic-1",
           "name": "/powertrain/engine/temperature"
         }
       ],
       "nodes": [
         {
           "entity_id": "engine-temp-sensor",
           "node_status": "reachable"
         },
         {
           "entity_id": "engine-temp-monitor",
           "node_status": "reachable"
         }
       ],
       "edges": [
         {
           "edge_id": "edge-1",
           "source": "engine-temp-sensor",
           "target": "engine-temp-monitor",
           "topic_id": "topic-1",
           "transport_type": "unknown",
           "metrics": {
             "source": "/greenwave_monitor",
             "frequency_hz": 2.02,
             "latency_ms": 3.8,
             "drop_rate_percent": 0.0,
             "metrics_status": "active"
           }
         }
       ]
     }
   }

If a second App pair existed in the same Function without a matching
``greenwave_monitor`` entry in ``gw_monitored_topics``, its edge would appear
in the same document with ``"metrics_status": "pending"`` and no
``metrics.source`` key at all - both apps stay ``reachable``, the edge is just
silent. See :ref:`graph-provider-pending-troubleshooting`.

Cyclic Subscription (Live Updates)
-----------------------------------

The plugin registers a cyclic-subscription sampler for the ``x-medkit-graph``
resource, so a client can receive periodic graph snapshots over Server-Sent
Events instead of polling. See :doc:`/api/rest` for the general
cyclic-subscription API; the graph-specific parts are below.

Create the subscription:

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/functions/engine-monitoring/cyclic-subscriptions \
     -H "Content-Type: application/json" \
     -d '{
       "resource": "/api/v1/functions/engine-monitoring/x-medkit-graph",
       "interval": "normal",
       "duration": 300
     }'

.. code-block:: json

   {
     "id": "sub_001",
     "observed_resource": "/api/v1/functions/engine-monitoring/x-medkit-graph",
     "event_source": "/api/v1/functions/engine-monitoring/cyclic-subscriptions/sub_001/events",
     "protocol": "sse",
     "interval": "normal"
   }

Then stream events from ``event_source``. Each frame's payload is the same
``{"x-medkit-graph": {...}}`` wrapper the plain ``GET`` returns:

.. code-block:: bash

   curl -N http://localhost:8080/api/v1/functions/engine-monitoring/cyclic-subscriptions/sub_001/events

.. code-block:: text

   data: {"timestamp":"2026-07-20T14:03:12.204Z","payload":{"x-medkit-graph":{"schema_version":"2.0.0","graph_id":"engine-monitoring-graph","timestamp":"2026-07-20T14:03:12.204Z","scope":{"type":"function","entity_id":"engine-monitoring"},"pipeline_status":"healthy","bottleneck_edge":null,"topics":[{"topic_id":"topic-1","name":"/powertrain/engine/temperature"}],"nodes":[{"entity_id":"engine-temp-sensor","node_status":"reachable"},{"entity_id":"engine-temp-monitor","node_status":"reachable"}],"edges":[{"edge_id":"edge-1","source":"engine-temp-sensor","target":"engine-temp-monitor","topic_id":"topic-1","transport_type":"unknown","metrics":{"source":"/greenwave_monitor","frequency_hz":2.03,"latency_ms":3.7,"drop_rate_percent":0.0,"metrics_status":"active"}}]}}}

Before the producer has warmed up, the same stream carries the edge as
``"metrics_status": "pending"`` with no ``metrics.source`` key - a client
watching the stream will see that transition to ``"active"`` live, without
reconnecting.

Reading the Output
-------------------

``metrics_status`` (per edge)
   - ``pending`` - no ``/diagnostics`` sample has ever been merged for this
     topic. Permanent until real data arrives; see the prerequisites above.
   - ``active`` - a fresh sample was merged within the freshness window
     (see :doc:`/config/graph-provider`). Tracks freshness, not field
     completeness: an edge with a zero-rate warmup sample from the producer
     is still ``active``.
   - ``error`` - a sample was merged at some point but the newest one is older
     than the freshness window. Carries ``"error_reason": "metrics_stale"`` -
     the only reachable value of that field. This is a real, currently-broken
     data flow, distinct from ``pending`` (never observed).

``pipeline_status`` (per graph)
   - ``broken`` - at least one edge is ``error``/``metrics_stale``.
   - ``degraded`` - no edge is in error, but at least one measured edge's
     frequency ratio (measured / expected) is below
     ``degraded_frequency_ratio``, or its drop rate exceeds
     ``drop_rate_percent_threshold``.
   - ``healthy`` - everything else, including a graph where every edge is
     still ``pending``. A Function with no ``/diagnostics`` coverage at all
     reads as ``healthy``, not broken - a pipeline that was never observed is
     not evidence of a broken one.

``bottleneck_edge``
   Present only when ``pipeline_status`` is ``degraded``; the ``edge_id`` of
   the edge with the lowest frequency ratio among all edges that have one.

``topic_id`` / ``edge_id``
   **Positional, not stable.** They are assigned by enumeration order
   (``topic-1``, ``topic-2``, ... and ``edge-1``, ``edge-2``, ...) each time
   the document is built, and will renumber whenever the set of topics or
   edges in the Function changes (a node joins or leaves, a topic starts or
   stops being published). Do not persist them or use them as a stable
   reference across requests - identify an edge by its ``(source, target,
   topic_id)`` within a single response instead.

See Also
--------

- :doc:`/config/graph-provider` - Configuration keys, defaults, and per-function overrides
- :doc:`/api/rest` - Full ``x-medkit-graph`` schema reference
- :doc:`plugin-system` - General plugin loading mechanism
- :doc:`/troubleshooting` - :ref:`graph-provider-pending-troubleshooting`
