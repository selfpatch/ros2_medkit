Attach to an Existing ROS 2 Stack
=================================

ros2_medkit discovers a running ROS 2 graph at runtime and builds the SOVD
entity tree with no manifest and no changes to your nodes. This guide covers
the common case of pointing the gateway at a stack you did not launch yourself
(for example Nav2, MoveIt, or Autoware), including the cross-process ROS 2
setup that any external node needs.

.. contents:: Table of Contents
   :local:
   :depth: 2

The short version
-----------------

If you install ros2_medkit from a binary release into the **same ROS 2
environment** as your stack, it already shares the RMW, domain and distro.
Source it and run the gateway; it discovers the live graph:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   ros2 run ros2_medkit_gateway gateway_node
   # SOVD entity tree + faults at http://localhost:8080/api/v1/

Everything below is for the case where medkit runs in a different environment
than your stack (a separate container, image, or install).

Match the ROS 2 environment
---------------------------

medkit talks to your stack over DDS like any other ROS 2 process. Four things
must line up, or the gateway sees an empty graph:

- **Distro.** ROS 2 distros do not interoperate over DDS. Run the gateway on
  the same distro as the target stack (Humble with Humble, Jazzy with Jazzy).
- **RMW implementation.** Both sides must use the same RMW. If your stack runs
  CycloneDDS, set it for the gateway too:

  .. code-block:: bash

     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

  The RMW you select must be installed in the gateway's environment. A binary
  install pulls in the default RMW; install the matching ``ros-<distro>-rmw-*``
  package for any other implementation.
- **Domain ID.** ``ROS_DOMAIN_ID`` must match the stack (default is 0).
- **Discovery reachability.** The processes must reach each other's DDS
  discovery: same host, a shared network, or - for containers - a shared
  network namespace (see :ref:`attach-containers`).

Run the gateway against the graph
---------------------------------

Runtime discovery is the default; no manifest is required. The gateway builds
the SOVD tree from whatever is on the graph:

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node
   # nodes -> SOVD Apps
   curl -s http://localhost:8080/api/v1/apps | jq '.items | length'
   # whole-system faults (from /diagnostics, /rosout, aborted actions via the bridges)
   curl -s http://localhost:8080/api/v1/faults | jq

.. _attach-containers:

Containers
----------

When the gateway runs as a container attaching to a stack on the host, give it
access to the host's DDS traffic and match the environment:

.. code-block:: bash

   # With --network host the gateway shares the host's ports directly
   # (no -p needed); reach the API at http://localhost:8080.
   docker run --rm \
     --network host \
     -e ROS_DOMAIN_ID=0 \
     -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

If the stack itself runs in a container, share its network namespace instead of
the host network:

.. code-block:: bash

   docker run --rm --network container:<stack_container> \
     -e ROS_DOMAIN_ID=0 \
     -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

.. note::

   On hosts with a small ``net.core.rmem_max``, CycloneDDS may warn about the
   socket receive buffer. Basic operation is unaffected; if a large-throughput
   stack needs it, raise ``net.core.rmem_max`` or point ``CYCLONEDDS_URI`` at a
   tuned ``cyclonedds.xml``.

Web UI
------

The web UI is a separate service served from its own origin, so the gateway
must allow that origin via CORS. Set ``cors.allowed_origins`` to your UI
origin(s) in the gateway params. See :doc:`web-ui` and :doc:`docker`. Restrict
origins and enable authentication (:doc:`authentication`) for production.

Verify
------

.. code-block:: bash

   curl -s http://localhost:8080/api/v1/health           # -> healthy
   curl -s http://localhost:8080/api/v1/apps  | jq '.items | length'   # your nodes
   curl -s http://localhost:8080/api/v1/faults | jq '.items | length'   # active faults

Troubleshooting
---------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Symptom
     - Likely cause
   * - Empty entity tree / no apps discovered
     - RMW, ``ROS_DOMAIN_ID`` or distro mismatch, or discovery not reachable
       (wrong network / namespace).
   * - Gateway loads but the stack uses CycloneDDS
     - ``RMW_IMPLEMENTATION`` not set to ``rmw_cyclonedds_cpp`` (the image
       default is FastDDS).
   * - Web UI shows "Failed to fetch"
     - CORS: the gateway is not allowing the UI's origin.
   * - Cannot reach the gateway from another host
     - Bind host is loopback; set ``server.host`` to ``0.0.0.0`` and expose the
       port.
