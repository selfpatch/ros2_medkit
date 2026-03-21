ros2_medkit_param_beacon
=========================

This section contains design documentation for the ros2_medkit_param_beacon plugin.

Overview
--------

The ``ros2_medkit_param_beacon`` package implements a gateway discovery plugin that
collects beacon hints from ROS 2 nodes via their parameter servers. Nodes declare
discovery metadata as ROS 2 parameters under a configurable prefix
(default ``ros2_medkit.discovery``), and the plugin polls these parameters periodically
to build ``BeaconHint`` structs.

This plugin implements both the ``GatewayPlugin`` and ``IntrospectionProvider`` interfaces.
It uses the shared ``ros2_medkit_beacon_common`` library for hint storage, validation,
entity mapping, and response building. See the
:doc:`beacon common design <../ros2_medkit_beacon_common/index>` for details on those
components.

How It Works
------------

1. During each poll cycle, the plugin retrieves the current list of ROS 2 nodes
   from the gateway's entity cache
2. For each node, it creates (or reuses) an ``AsyncParametersClient`` and fetches
   all parameters matching the configured prefix
3. Parameters are parsed into a ``BeaconHint``: ``entity_id``, ``stable_id``,
   ``function_ids``, ``metadata.*`` keys, etc.
4. Validated hints are stored in the ``BeaconHintStore``
5. On each ``introspect()`` call, the store is snapshot and mapped to an
   ``IntrospectionResult`` via ``BeaconEntityMapper``

Key Design Points
-----------------

Polling Model
~~~~~~~~~~~~~

Unlike the topic beacon (push-based), the parameter beacon uses a pull model.
A background thread polls nodes at a configurable interval (default 5 seconds).
This is appropriate for parameters because they change infrequently and nodes
do not need to actively publish discovery information - they only need to set
their parameters once at startup.

Client Management
~~~~~~~~~~~~~~~~~

The plugin maintains a cache of ``AsyncParametersClient`` instances keyed by node
FQN. Clients for nodes that disappear from the graph are evicted after a
configurable timeout. A lock ordering protocol (``nodes_mutex_`` then
``clients_mutex_`` then ``param_ops_mutex_``) prevents deadlocks between the
poll thread and the introspection callback.

Backoff and Budget
~~~~~~~~~~~~~~~~~~

Nodes that fail to respond (timeout, unavailable) accumulate a backoff counter.
Subsequent poll cycles skip backed-off nodes with exponentially increasing skip
counts. A per-cycle time budget (default 10 seconds) prevents a few slow nodes
from starving the rest of the poll targets. The start offset rotates each cycle
so that all nodes eventually get polled even under budget pressure.

Injectable Client Factory
~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin constructor accepts an optional ``ParameterClientFactory`` for
dependency injection. Unit tests provide a mock factory that returns
``ParameterClientInterface`` stubs, enabling full testing without a running
ROS 2 graph.

Integration with Gateway
~~~~~~~~~~~~~~~~~~~~~~~~

The plugin registers the ``x-medkit-beacon`` vendor extension capability on Apps
and exposes HTTP endpoints for querying individual beacon metadata. Route
registration and capability handling follow the standard gateway plugin pattern
via ``PluginContext``.
