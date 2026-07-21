ros2_medkit_graph_provider
===========================

This section contains design documentation for the ros2_medkit_graph_provider plugin.

Overview
--------

The ``ros2_medkit_graph_provider`` package implements a gateway plugin that generates
live dataflow graph documents for SOVD Function entities. It models the ROS 2 topic
graph as a directed graph of nodes (apps) and edges (topic connections), enriched
with frequency, latency, and drop rate metrics from ``/diagnostics``. The graph
document powers the ``x-medkit-graph`` vendor extension endpoint and cyclic
subscription sampler.

Architecture
------------

The following diagram shows the plugin's main components and data flow.

.. plantuml::
   :caption: Graph Provider Plugin Architecture

   @startuml ros2_medkit_graph_provider_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title Graph Provider - Plugin Architecture

   package "ros2_medkit_gateway" {
       interface GatewayPlugin
       interface IntrospectionProvider
       class PluginContext {
           +register_capability()
           +register_sampler()
           +get_entity_snapshot()
       }
   }

   package "ROS 2" {
       class "/diagnostics" as diag_topic <<topic>>
   }

   package "ros2_medkit_graph_provider" {

       class GraphProviderPlugin {
           - topic_metrics_: map<string, TopicMetrics>
           - topic_metrics_order_: deque<string>
           - last_seen_by_app_: map<string, string>
           - config_: ConfigOverrides
           --
           + introspect(input): IntrospectionResult
           + {static} build_graph_document(): json
           --
           - subscribe_to_diagnostics()
           - diagnostics_callback(msg, msg_info)
           - resolve_publisher_source(msg_info): optional<string>
           - build_current_graph(func_id)
           - build_graph_from_entity_cache(func_id)
           - build_state_snapshot(input, timestamp)
           - resolve_config(func_id)
           - load_parameters()
       }

       class TopicMetrics {
           + frequency_hz: optional<double>
           + latency_ms: optional<double>
           + drop_rate_percent: optional<double>
           + expected_frequency_hz: optional<double>
           + source: optional<string>
           + last_update_ns: int64
       }

       class GraphBuildState {
           + topic_metrics: map
           + last_seen_by_app: map
           + now_ns: int64
       }

       class GraphBuildConfig {
           + expected_frequency_hz_default: 30.0
           + degraded_frequency_ratio: 0.5
           + drop_rate_percent_threshold: 5.0
           + freshness_headroom_factor: 3.0
           + freshness_floor_sec: 5.0
       }
   }

   GraphProviderPlugin .up.|> GatewayPlugin
   GraphProviderPlugin .up.|> IntrospectionProvider
   GraphProviderPlugin --> PluginContext
   GraphProviderPlugin --> diag_topic : subscribes
   GraphProviderPlugin *--> TopicMetrics : caches
   GraphProviderPlugin ..> GraphBuildState : builds
   GraphProviderPlugin ..> GraphBuildConfig : configured by

   @enduml

Graph Document Schema
---------------------

The plugin generates a JSON document per Function entity with the following structure:

- **schema_version**: ``"2.0.0"`` - a real, semver contract on the document's shape and
  field semantics. A minor bump is additive/backward-compatible (new optional field, new
  enum value a tolerant client can ignore); a major bump means the shape or the meaning of
  an existing field changed and old parsing logic may break.
- **graph_id**: ``"{function_id}-graph"``
- **timestamp**: ISO 8601 nanosecond timestamp
- **scope**: Function entity that owns the graph
- **pipeline_status**: ``"healthy"``, ``"degraded"``, or ``"broken"``
- **bottleneck_edge**: Edge ID with the lowest frequency ratio (only when degraded)
- **topics**: List of topics with **positional** IDs (``topic-1``, ``topic-2``, ...,
  assigned by enumeration order on every build). They renumber whenever the topic set
  changes and must never be persisted or treated as stable across requests.
- **nodes**: List of app entities with reachability status
- **edges**: Publisher-subscriber connections with per-edge metrics, keyed by
  equally positional ``edge-1``, ``edge-2``, ... IDs

Edge metrics include frequency, latency, drop rate, and a ``metrics_status`` field:

- ``"active"`` - A ``/diagnostics`` sample was merged within the freshness window
- ``"pending"`` - No ``/diagnostics`` sample has ever been merged for this topic
- ``"error"`` - A sample was merged in the past, but the newest one is older than the
  freshness window (``error_reason: "metrics_stale"`` - the only reachable value)

Pipeline Status Logic
---------------------

The overall pipeline status is determined by aggregating edge states:

1. **broken** - At least one edge has ``metrics_status: "error"`` (``metrics_stale`` -
   a real, currently-broken data flow)
2. **degraded** - At least one edge has frequency below the degraded ratio threshold, or drop rate exceeds the threshold
3. **healthy** - Everything else, including a graph where every edge is still ``pending``
   (a pipeline that was never observed is not evidence of a broken one)

When the status is ``"degraded"``, the ``bottleneck_edge`` field identifies the edge
with the lowest frequency-to-expected ratio, helping operators pinpoint the
constraint in the dataflow pipeline.

Data Sources
------------

Diagnostics Subscription
~~~~~~~~~~~~~~~~~~~~~~~~

The plugin subscribes to ``/diagnostics`` and parses ``DiagnosticStatus`` messages
for topic-level metrics, keyed on ``DiagnosticStatus.name`` (the fully-qualified
topic name). Recognized keys:

- ``frame_rate_msg`` - Mapped to ``frequency_hz``; falls back to ``frame_rate_node``
  when ``frame_rate_msg`` is present but exactly ``0`` (header-less message types
  cannot be rated from a header timestamp)
- ``current_delay_from_realtime_ms`` - Mapped to ``latency_ms``
- ``drop_rate_percent`` / ``drop_rate`` - Mapped to ``drop_rate_percent``
- ``expected_frequency`` - Mapped to ``expected_frequency_hz``; wins over any
  graph-provider-side configuration for that topic when present
- ``total_dropped_frames`` - Recognized (keeps the sample from being skipped as
  irrelevant) but not currently mapped to an output field

Each ``DiagnosticArray`` message resolves to exactly one publisher (via publisher
GID matching against ``get_publishers_info_by_topic("/diagnostics")``), stamped as
``metrics.source`` on every topic that message updates. No GID match leaves
``source`` unresolved for that update - never a fabricated name or a vendor
literal; a pending or unresolved edge simply omits the ``source`` key.

A bounded cache (max 512 topics) with FIFO eviction (oldest-inserted topic evicted
first, tracked via an insertion-ordered deque) prevents unbounded memory growth.

Entity Cache
~~~~~~~~~~~~

On every request (HTTP route and cyclic-subscription sampler alike), the plugin
rebuilds the graph fresh from the current entity cache
(``PluginContext::get_entity_snapshot()``) - there is no cross-request graph cache
to go stale. This ensures the response always reflects the latest node and topic
state.

Function Scoping
~~~~~~~~~~~~~~~~

Graph documents are scoped to individual Function entities. The plugin resolves
which apps belong to a function by checking the function's ``hosts`` list against
app IDs and component IDs. Only topics that connect scoped apps appear as edges.
System topics (``/parameter_events``, ``/rosout``, ``/diagnostics``, NITROS topics)
are filtered out.

Design Decisions
----------------

Extracted Plugin
~~~~~~~~~~~~~~~~

The graph provider was extracted from the gateway core into a standalone plugin
package. This follows the same pattern as the linux introspection plugins: the
gateway loads graph_provider as a ``.so`` at runtime, keeping the core gateway
free of diagnostics-specific logic. The plugin can be omitted from deployments
that do not need dataflow visualization.

Static build_graph_document
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``build_graph_document()`` method is static and takes all inputs explicitly
(function_id, IntrospectionInput, GraphBuildState, GraphBuildConfig, timestamp).
This makes the graph generation logic fully testable without instantiating the
plugin or its ROS 2 dependencies.

Per-Function Config Overrides
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin supports per-function configuration overrides for thresholds
(expected frequency, degraded ratio, drop rate). This allows operators to set
different health baselines for different subsystems - for example, a camera
pipeline at 30 Hz vs. a LiDAR pipeline at 10 Hz.

Cyclic Subscription Sampler
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin registers a sampler via ``PluginContext::register_sampler()`` for the
``x-medkit-graph`` resource. This allows clients to create cyclic subscriptions
that receive periodic graph snapshots over SSE, enabling live dashboard updates
without polling the HTTP endpoint.
