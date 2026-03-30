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
       -p aggregation.peers:="[{url: 'http://192.168.1.10:8080', name: 'peer_b'}]"

Or in ``gateway_params.yaml``:

.. code-block:: yaml

   gateway_node:
     ros__parameters:
       aggregation:
         enabled: true
         peers:
           - url: "http://192.168.1.10:8080"
             name: "peer_b"

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

Static Peers
------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 10 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``aggregation.peers``
     - list
     - ``[]``
     - List of statically configured peer gateways. Each entry has ``url``
       (base URL including port) and ``name`` (human-readable identifier).
       Static peers are always present regardless of mDNS discovery.

Each peer entry:

.. list-table::
   :header-rows: 1
   :widths: 20 10 70

   * - Field
     - Type
     - Description
   * - ``url``
     - string
     - Base URL of the peer gateway (e.g., ``http://192.168.1.10:8080``).
       Must include the scheme and port.
   * - ``name``
     - string
     - Human-readable name for the peer. Used as prefix for collision
       resolution (e.g., ``peername__entity_id``) and in the routing table.
       Must be unique across all peers.

Scenario Examples
-----------------

Star Topology
~~~~~~~~~~~~~

One primary gateway aggregates from three subsystem gateways:

.. code-block:: yaml

   # Primary gateway (host-A, port 8080)
   gateway_node:
     ros__parameters:
       aggregation:
         enabled: true
         timeout_ms: 3000
         announce: false
         discover: false  # Use static peers only
         peers:
           - url: "http://192.168.1.10:8080"
             name: "arm_controller"
           - url: "http://192.168.1.11:8080"
             name: "base_platform"
           - url: "http://192.168.1.12:8080"
             name: "sensor_array"

The leaf gateways do not need aggregation enabled - they serve their own
entities independently. Only the primary gateway needs aggregation.

Chain Topology
~~~~~~~~~~~~~~

Gateway A aggregates from B, which aggregates from C:

.. code-block:: yaml

   # Gateway A (top-level aggregator)
   gateway_node:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peers:
           - url: "http://gateway-b:8080"
             name: "subsystem_b"

.. code-block:: yaml

   # Gateway B (mid-level aggregator)
   gateway_node:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peers:
           - url: "http://gateway-c:8080"
             name: "subsystem_c"

.. code-block:: yaml

   # Gateway C (leaf - no aggregation needed)
   gateway_node:
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
   gateway_node:
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

   gateway_node:
     ros__parameters:
       aggregation:
         enabled: true
         announce: true
         discover: true
         peers:
           # Always connect to the base platform
           - url: "http://base-platform:8080"
             name: "base_platform"
         # Additional peers discovered via mDNS at runtime

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
the response includes:

- ``X-Medkit-Partial: true`` header
- A ``partial_errors`` field listing which peers failed

This allows clients to detect degraded responses and take appropriate action.
Individual entity requests for remote entities return ``502 Bad Gateway`` if the
owning peer is unreachable.
