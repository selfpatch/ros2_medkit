ros2_medkit_topic_beacon
=========================

This section contains design documentation for the ros2_medkit_topic_beacon plugin.

Overview
--------

The ``ros2_medkit_topic_beacon`` package implements a gateway discovery plugin that
collects beacon hints from ROS 2 nodes via a shared discovery topic. Nodes publish
``MedkitDiscoveryHint`` messages to a configurable topic
(default ``/ros2_medkit/discovery``), and the plugin subscribes to receive them in
real time.

This plugin implements both the ``GatewayPlugin`` and ``IntrospectionProvider`` interfaces.
It uses the shared ``ros2_medkit_beacon_common`` library for hint storage, validation,
entity mapping, and response building. See the
:doc:`beacon common design <../ros2_medkit_beacon_common/index>` for details on those
components.

How It Works
------------

1. On ``set_context()``, the plugin creates a ROS 2 subscription to the discovery topic
2. Each incoming ``MedkitDiscoveryHint`` message is converted to a ``BeaconHint``
3. The hint passes through the rate limiter and validator before being stored
4. On each ``introspect()`` call, the store is snapshot and mapped to an
   ``IntrospectionResult`` via ``BeaconEntityMapper``

Key Design Points
-----------------

Push Model
~~~~~~~~~~

Unlike the parameter beacon (pull-based polling), the topic beacon is push-based.
Nodes actively publish discovery hints at their own cadence. This provides lower
latency for discovery updates and avoids the overhead of per-node parameter queries,
but requires nodes to include the ``ros2_medkit_msgs`` dependency and actively
publish beacons.

Rate Limiting
~~~~~~~~~~~~~

A ``TokenBucket`` rate limiter (default 100 messages/second) protects the gateway
from beacon floods. The bucket refills continuously and drops excess messages with
a single log warning. The rate limiter is thread-safe, as the DDS callback may fire
from any executor thread.

Timestamp Back-Projection
~~~~~~~~~~~~~~~~~~~~~~~~~

The topic beacon converts the message header stamp (if non-zero) to a
``steady_clock`` time point by computing the message age relative to the current
wall-clock time and subtracting from ``steady_clock::now()``. This maps ROS time
into the monotonic domain used by the ``BeaconHintStore`` for TTL computation,
avoiding clock-jump artifacts.

Integration with Gateway
~~~~~~~~~~~~~~~~~~~~~~~~

The plugin registers the ``x-medkit-beacon`` vendor extension capability on Apps
and exposes HTTP endpoints for querying individual beacon metadata. Route
registration and capability handling follow the standard gateway plugin pattern
via ``PluginContext``.
