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
           +list_all_faults()
       }
   }

   package "ROS 2" {
       class "/diagnostics" as diag_topic <<topic>>
   }

   package "ros2_medkit_graph_provider" {

       class GraphProviderPlugin {
           - graph_cache_: map~<string, json~>
           - topic_metrics_: map~<string, TopicMetrics~>
           - last_seen_by_app_: map~<string, string~>
           - config_: ConfigOverrides
           --
           + introspect(input): IntrospectionResult
           + {static} build_graph_document(): json
           --
           - subscribe_to_diagnostics()
           - diagnostics_callback(msg)
           - get_cached_or_built_graph(func_id)
           - build_graph_from_entity_cache(func_id)
           - collect_stale_topics(func_id)
           - build_state_snapshot()
           - resolve_config(func_id)
           - load_parameters()
       }

       struct TopicMetrics {
           + frequency_hz: optional~<double~>
           + latency_ms: optional~<double~>
           + drop_rate_percent: double
           + expected_frequency_hz: optional~<double~>
       }

       struct GraphBuildState {
           + topic_metrics: map
           + stale_topics: set
           + last_seen_by_app: map
           + diagnostics_seen: bool
       }

       struct GraphBuildConfig {
           + expected_frequency_hz_default: 30.0
           + degraded_frequency_ratio: 0.5
           + drop_rate_percent_threshold: 5.0
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

- **schema_version**: ``"1.0.0"``
- **graph_id**: ``"{function_id}-graph"``
- **timestamp**: ISO 8601 nanosecond timestamp
- **scope**: Function entity that owns the graph
- **pipeline_status**: ``"healthy"``, ``"degraded"``, or ``"broken"``
- **bottleneck_edge**: Edge ID with the lowest frequency ratio (only when degraded)
- **topics**: List of topics with stable IDs
- **nodes**: List of app entities with reachability status
- **edges**: Publisher-subscriber connections with per-edge metrics

Edge metrics include frequency, latency, drop rate, and a status field:

- ``"active"`` - Diagnostics data available, normal operation
- ``"pending"`` - No diagnostics data received yet
- ``"error"`` - Node offline, topic stale, or no data source after diagnostics started

Pipeline Status Logic
---------------------

The overall pipeline status is determined by aggregating edge states:

1. **broken** - At least one edge has ``metrics_status: "error"`` (node offline or topic stale)
2. **degraded** - At least one edge has frequency below the degraded ratio threshold, or drop rate exceeds the threshold
3. **healthy** - All edges are active with acceptable metrics

When the status is ``"degraded"``, the ``bottleneck_edge`` field identifies the edge
with the lowest frequency-to-expected ratio, helping operators pinpoint the
constraint in the dataflow pipeline.

Data Sources
------------

Diagnostics Subscription
~~~~~~~~~~~~~~~~~~~~~~~~

The plugin subscribes to ``/diagnostics`` and parses ``DiagnosticStatus`` messages
for topic-level metrics. Recognized keys:

- ``frame_rate_msg`` - Mapped to ``frequency_hz``
- ``current_delay_from_realtime_ms`` - Mapped to ``latency_ms``
- ``drop_rate_percent`` / ``drop_rate`` - Mapped to ``drop_rate_percent``
- ``expected_frequency`` - Mapped to ``expected_frequency_hz``

A bounded cache (max 512 topics) with LRU eviction prevents unbounded memory growth.

Fault Manager Integration
~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin queries the fault manager via ``PluginContext::list_all_faults()`` to
detect stale topics. A topic is considered stale when there is a confirmed critical
fault whose fault code matches the topic name (after normalization). Stale topics
cause their edges to be marked as ``"error"`` with reason ``"topic_stale"``.

Entity Cache
~~~~~~~~~~~~

On HTTP requests, the plugin rebuilds the graph from the current entity cache
(``PluginContext::get_entity_snapshot()``) rather than serving the potentially stale
introspection-pipeline cache. This ensures the HTTP endpoint always reflects the
latest node and topic state.

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
