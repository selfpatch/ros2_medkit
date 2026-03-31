Aggregation Configuration
=========================

This reference describes all aggregation-related configuration options for
multi-instance peer aggregation in ros2_medkit_gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Aggregation allows multiple gateway instances to federate their entity trees
into a single unified API. A primary gateway merges entities from peer gateways
and transparently forwards requests for remote entities.

All aggregation parameters are under the ``aggregation`` key in
``gateway_params.yaml`` or can be set via ROS 2 parameters.

Quick Start
-----------

.. code-block:: bash

   # Enable aggregation with a static peer
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p aggregation.enabled:=true \
       -p aggregation.peer_urls:="['http://192.168.1.10:8080']" \
       -p aggregation.peer_names:="['peer_b']"

Or in ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         peer_urls: ["http://192.168.1.10:8080"]
         peer_names: ["peer_b"]

Core Parameters
---------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.enabled``
     - bool
     - ``false``
     - Master switch for peer aggregation. When disabled, the gateway operates
       in standalone mode with no peer communication.
   * - ``aggregation.timeout_ms``
     - int
     - ``2000``
     - HTTP timeout in milliseconds for all peer communication: health checks,
       entity fetching, and request forwarding. Increase for high-latency
       networks.

mDNS Discovery Parameters
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.announce``
     - bool
     - ``false``
     - Broadcast this gateway's presence via mDNS. Other gateways on the local
       network can discover this instance automatically. Opt-in to avoid
       surprising network behavior.
   * - ``aggregation.discover``
     - bool
     - ``false``
     - Browse for peer gateways via mDNS. When a new peer is found, it is
       automatically added to the peer list. Opt-in to avoid surprising
       network behavior.
   * - ``aggregation.mdns_service``
     - string
     - ``"_medkit._tcp.local"``
     - mDNS service type used for announcement and browsing. All gateways in
       the same aggregation cluster must use the same service type.
   * - ``aggregation.mdns_name``
     - string
     - ``""``
     - mDNS instance name for announcement and self-discovery filtering.
       Defaults to the system hostname (via ``gethostname()``). Must be unique
       per gateway instance. Set explicitly when running multiple gateways on
       the same host - otherwise they share the same hostname and filter each
       other out as "self".

Static Peers
------------

Static peers are configured as parallel arrays: ``peer_urls[i]`` pairs with
``peer_names[i]``. Both arrays must have the same length. Empty-string entries
are ignored.

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.peer_urls``
     - string[]
     - ``[""]``
     - List of peer gateway base URLs (e.g.,
       ``["http://192.168.1.10:8080", "http://192.168.1.11:8080"]``).
       Each URL must include the scheme and port.
   * - ``aggregation.peer_names``
     - string[]
     - ``[""]``
     - List of human-readable names for peers (e.g.,
       ``["arm_controller", "base_platform"]``).
       Used as prefix for collision resolution (e.g., ``peername__entity_id``)
       and in the routing table. Must be unique across all peers.

Scenario Examples
-----------------

Star Topology
~~~~~~~~~~~~~

One primary gateway aggregates from three subsystem gateways:

.. code-block:: yaml

   # Primary gateway (host-A, port 8080)
   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         timeout_ms: 3000
         announce: false
         discover: false  # Use static peers only
         peer_urls: ["http://192.168.1.10:8080", "http://192.168.1.11:8080", "http://192.168.1.12:8080"]
         peer_names: ["arm_controller", "base_platform", "sensor_array"]

The leaf gateways do not need aggregation enabled - they serve their own
entities independently. Only the primary gateway needs aggregation.

Chain Topology
~~~~~~~~~~~~~~

Gateway A aggregates from B, which aggregates from C:

.. code-block:: yaml

   # Gateway A (top-level aggregator)
   ros2_medkit_gateway:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peer_urls: ["http://gateway-b:8080"]
         peer_names: ["subsystem_b"]

.. code-block:: yaml

   # Gateway B (mid-level aggregator)
   ros2_medkit_gateway:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peer_urls: ["http://gateway-c:8080"]
         peer_names: ["subsystem_c"]

.. code-block:: yaml

   # Gateway C (leaf - no aggregation needed)
   ros2_medkit_gateway:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: false

Gateway A sees entities from A + B + C. Gateway B sees entities from B + C.

mDNS-Only Discovery
~~~~~~~~~~~~~~~~~~~~

Fully automatic peer discovery with no static configuration:

.. code-block:: yaml

   # All gateways use the same config
   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         announce: true
         discover: true
         mdns_service: "_medkit._tcp.local"
         # No static peers - all discovery via mDNS

All gateways on the same network segment automatically find each other. When a
gateway starts, it announces itself and discovers existing peers. When a gateway
stops, it sends an mDNS goodbye and peers remove it automatically.

.. note::

   mDNS requires multicast network support. Docker containers using bridge
   networking may not support mDNS - use static peers or host networking
   instead.

Mixed Static + mDNS
~~~~~~~~~~~~~~~~~~~~

Combine static peers for known infrastructure with mDNS for dynamic discovery:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       aggregation:
         enabled: true
         announce: true
         discover: true
         # Always connect to the base platform
         peer_urls: ["http://base-platform:8080"]
         peer_names: ["base_platform"]
         # Additional peers discovered via mDNS at runtime

.. note::

   When authentication is enabled, the gateway forwards the client's
   ``Authorization`` header to peer gateways. Ensure all peers use
   the same JWT configuration. This means peer gateways receive
   client credentials - only configure peers you trust.

Entity Merge Behavior
---------------------

When aggregation is enabled, entities from peers are merged with local entities:

- **Areas and Functions**: Merged by ID. If both local and remote have the same
  ID, they become one entity. This is appropriate because Areas and Functions
  represent logical groupings that often span hosts.

- **Components and Apps**: Prefixed on collision. If a remote entity has the
  same ID as a local one, the remote entity's ID is prefixed with
  ``peername__`` (double underscore). For example, Component ``lidar`` from peer
  ``arm`` becomes ``arm__lidar``.

Requests for remote entities are transparently forwarded to the owning peer.
The routing table maps entity IDs to peer names.

See :doc:`../design/ros2_medkit_gateway/aggregation` for detailed merge logic
and architecture diagrams.

Health and Partial Results
--------------------------

If a peer is unreachable during a fan-out request (e.g., ``GET /api/v1/components``),
the response body includes:

- ``x-medkit.partial: true`` in the JSON response body
- ``x-medkit.failed_peers`` listing which peers failed

This allows clients to detect degraded responses and take appropriate action.
Individual entity requests for remote entities return ``502 Bad Gateway`` if the
owning peer is unreachable.

Migration Notes
---------------

The entity model has been simplified. Synthetic/heuristic Area and Component
creation from ROS 2 namespaces has been removed:

- **Areas** come from manifest only. Runtime discovery never creates Areas.
- **Components** come from ``HostInfoProvider`` (single host-level Component)
  or manifest. Runtime discovery never creates Components.
- **Functions** are created from namespace grouping (``create_functions_from_namespaces``
  defaults to ``true``).
- **Apps** are created from ROS 2 nodes with ``source: "heuristic"``.

The removed parameters are: ``create_synthetic_areas``,
``create_synthetic_components``, ``grouping_strategy``,
``synthetic_component_name_pattern``, ``topic_only_policy``,
``min_topics_for_component``, ``allow_heuristic_areas``, and
``allow_heuristic_components``.
