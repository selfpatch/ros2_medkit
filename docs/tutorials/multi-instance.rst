Multi-Instance Aggregation
==========================

This tutorial walks through setting up multiple ros2_medkit_gateway instances
and aggregating their entity trees into a single unified API.

.. contents:: Table of Contents
   :local:
   :depth: 2

Prerequisites
-------------

- ros2_medkit built and installed (``colcon build && source install/setup.bash``)
- Two or more terminals available
- Familiarity with the :doc:`../getting_started` guide

What You Will Learn
-------------------

- How to run two gateways on different ports
- How to configure static peer connections
- How to enable mDNS auto-discovery
- What the merged API looks like
- How to handle peer failures
- How to set up chain topologies

Step 1: Start Two Gateways
--------------------------

Open two terminals. In each, source the ROS 2 workspace:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

**Terminal 1 - Gateway A (port 8080):**

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p server.port:=8080 \
       -r __node:=gateway_a \
       -r __ns:=/subsystem_a

**Terminal 2 - Gateway B (port 8081):**

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p server.port:=8081 \
       -r __node:=gateway_b \
       -r __ns:=/subsystem_b

Verify both gateways are healthy:

.. code-block:: bash

   curl http://localhost:8080/api/v1/health
   curl http://localhost:8081/api/v1/health

Each gateway should report ``{"status": "healthy"}``.

Step 2: Configure Static Peers
------------------------------

To make Gateway A aggregate entities from Gateway B, create a config file:

.. code-block:: yaml
   :caption: gateway_a_params.yaml

   gateway_node:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peers:
           - url: "http://localhost:8081"
             name: "subsystem_b"

Restart Gateway A with the config:

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       --params-file gateway_a_params.yaml \
       -r __node:=gateway_a

Now Gateway A merges entities from both itself and Gateway B.

Step 3: Explore the Merged API
------------------------------

**List all components (merged from both gateways):**

.. code-block:: bash

   curl -s http://localhost:8080/api/v1/components | jq

The response includes components from both gateways. Components from Gateway B
have ``"source": "peer:subsystem_b"`` in their metadata.

If both gateways have a component with the same ID (e.g., both hosts are named
``robot``), the remote component gets a prefixed ID:

.. code-block:: json

   {
     "items": [
       {"id": "robot", "source": "runtime"},
       {"id": "subsystem_b__robot", "source": "peer:subsystem_b"}
     ]
   }

**List all areas (merged by ID):**

.. code-block:: bash

   curl -s http://localhost:8080/api/v1/areas | jq

Areas are merged by ID - if both gateways discover a ``root`` area, only one
``root`` appears in the response.

**Access data from a remote entity:**

.. code-block:: bash

   # If subsystem_b has a component "arm_controller"
   curl -s http://localhost:8080/api/v1/components/arm_controller/data | jq

The request is transparently forwarded to Gateway B. The client does not need
to know which gateway owns the entity.

**List all functions:**

.. code-block:: bash

   curl -s http://localhost:8080/api/v1/functions | jq

Functions are merged by ID with combined ``hosts`` lists. A ``navigation``
function that exists on both gateways appears once, listing hosts from both.

Step 4: Enable mDNS Auto-Discovery
-----------------------------------

Instead of listing peers statically, use mDNS to discover gateways
automatically on the local network.

**Gateway A config with mDNS:**

.. code-block:: yaml
   :caption: gateway_a_mdns.yaml

   gateway_node:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         announce: true
         discover: true

**Gateway B config with mDNS:**

.. code-block:: yaml
   :caption: gateway_b_mdns.yaml

   gateway_node:
     ros__parameters:
       server:
         port: 8081
       aggregation:
         enabled: true
         announce: true
         discover: true

Start both gateways with their configs:

.. code-block:: bash

   # Terminal 1
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       --params-file gateway_a_mdns.yaml -r __node:=gateway_a

   # Terminal 2
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       --params-file gateway_b_mdns.yaml -r __node:=gateway_b

After a few seconds, each gateway discovers the other via mDNS. Check the
health endpoint to see discovered peers:

.. code-block:: bash

   curl -s http://localhost:8080/api/v1/health | jq '.peers'

.. note::

   mDNS requires multicast network support. If using Docker, ensure containers
   share a network that supports multicast, or use host networking. For
   bridge-networked containers, use static peers instead.

Step 5: Handle Peer Failures
-----------------------------

When a peer goes down, the gateway handles it gracefully:

1. **Health checks detect failure**: The health check interval
   (default: 10 seconds) detects the peer is unreachable and marks it
   unhealthy.

2. **Collection requests return partial results**: Global endpoints like
   ``GET /api/v1/components`` return local entities plus a
   ``X-Medkit-Partial: true`` header when some peers are unreachable.

3. **Entity-specific requests return 502**: If a request targets a remote
   entity whose peer is down, the gateway returns ``502 Bad Gateway``.

Test this by stopping Gateway B and querying Gateway A:

.. code-block:: bash

   # Stop Gateway B (Ctrl+C in Terminal 2)

   # Wait for health check interval, then:
   curl -s http://localhost:8080/api/v1/components | jq
   # Returns only local components + partial flag

   # Try accessing a remote entity:
   curl -s http://localhost:8080/api/v1/components/subsystem_b__some_entity/data
   # Returns 502 Bad Gateway

When Gateway B comes back online, it is automatically re-included after the
next successful health check.

Step 6: Chain Topology
----------------------

For hierarchical systems, gateways can be chained. Gateway A aggregates from
B, which aggregates from C:

.. code-block:: yaml
   :caption: gateway_a_chain.yaml

   gateway_node:
     ros__parameters:
       server:
         port: 8080
       aggregation:
         enabled: true
         peers:
           - url: "http://localhost:8081"
             name: "mid_level"

.. code-block:: yaml
   :caption: gateway_b_chain.yaml

   gateway_node:
     ros__parameters:
       server:
         port: 8081
       aggregation:
         enabled: true
         peers:
           - url: "http://localhost:8082"
             name: "leaf_system"

.. code-block:: yaml
   :caption: gateway_c_chain.yaml

   gateway_node:
     ros__parameters:
       server:
         port: 8082
       aggregation:
         enabled: false

Start all three gateways, then query the top-level:

.. code-block:: bash

   curl -s http://localhost:8080/api/v1/components | jq

Gateway A returns components from all three levels. Requests for entities on
Gateway C are forwarded through B to C.

Summary
-------

- **Static peers**: List known gateways in ``aggregation.peers`` for
  deterministic connections.
- **mDNS discovery**: Set ``aggregation.announce`` and ``aggregation.discover``
  to ``true`` for zero-configuration peer discovery.
- **Entity merging**: Areas and Functions merge by ID. Components and Apps get
  peer-name prefixes on collision.
- **Transparent forwarding**: Requests for remote entities are forwarded to the
  owning peer. Clients interact with a single API endpoint.
- **Graceful degradation**: Unhealthy peers are excluded from fan-out. Partial
  results are clearly marked.

Next Steps
----------

- :doc:`../config/aggregation` - Full configuration reference
- :doc:`docker` - Deploy aggregated gateways in Docker containers
- :doc:`authentication` - Secure peer-to-peer communication with JWT
