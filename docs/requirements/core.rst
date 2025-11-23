Core & Topology
===============

.. req:: SOVDServer
   :id: REQ_SOVD_001
   :status: open
   :tags: P

   **Goal:** Entry point / aggregation

   **ROS 2 Equivalent:** Gateway node + graph introspection

   **Implementation Notes:** A 'sovd_gateway' node collects metadata via NodeGraphInterface and exposes a REST API.

.. req:: Area
   :id: REQ_SOVD_002
   :status: open
   :tags: A

   **Goal:** Logical view / topology

   **ROS 2 Equivalent:** Namespaces (e.g. /powertrain, /rear)

   **Implementation Notes:** Establish a naming convention for namespaces and optional tags via parameters.

.. req:: Component
   :id: REQ_SOVD_003
   :status: open
   :tags: A

   **Goal:** HW/SW unit

   **ROS 2 Equivalent:** Node / composable nodes / device namespace

   **Implementation Notes:** Map 1:1 to a node or a group of nodes in one container; an ECU/HPC is a group of nodes.

.. req:: App
   :id: REQ_SOVD_004
   :status: open
   :tags: A

   **Goal:** Application running on a component

   **ROS 2 Equivalent:** Set of nodes / lifecycle application

   **Implementation Notes:** Represent as a launch description or a supervising 'super-node' managing child nodes.

.. req:: Function
   :id: REQ_SOVD_005
   :status: open
   :tags: P

   **Goal:** Functional view across components

   **ROS 2 Equivalent:** Aggregator node (topics / services / actions)

   **Implementation Notes:** Introduce dedicated aggregator nodes that collect from multiple topics and expose a single interface.

.. req:: GET /version-info
   :id: REQ_SOVD_006
   :status: open
   :tags: P

   **Goal:** Version information

   **ROS 2 Equivalent:** Package manifests and build metadata

   **Implementation Notes:** A GetVersionInfo service reads package.xml, binary --version output and build_info parameters.

.. req:: GET /{any}/docs
   :id: REQ_SOVD_007
   :status: open
   :tags: P

   **Goal:** Online capabilities / documentation

   **ROS 2 Equivalent:** Introspection of topics / services / actions / parameters

   **Implementation Notes:** Generate OpenAPI (or similar) from .msg / .srv / .action definitions plus QoS metadata.

.. req:: GET /{entity-collection}
   :id: REQ_SOVD_008
   :status: open
   :tags: P

   **Goal:** List entities

   **ROS 2 Equivalent:** ros2 node list + custom entity index

   **Implementation Notes:** The gateway builds indices of areas, components, apps and functions.

.. req:: GET /areas/{id}/subareas
   :id: REQ_SOVD_009
   :status: open
   :tags: A

   **Goal:** Area hierarchy

   **ROS 2 Equivalent:** Sub-namespaces

   **Implementation Notes:** Traverse the namespace tree and expose child namespaces as sub-areas.

.. req:: GET /components/{id}/subcomponents
   :id: REQ_SOVD_010
   :status: open
   :tags: A

   **Goal:** In-process SW hierarchy

   **ROS 2 Equivalent:** Composable nodes

   **Implementation Notes:** Use component containers to represent subcomponents within a single process.

.. req:: GET /areas/{id}/contains
   :id: REQ_SOVD_011
   :status: open
   :tags: A

   **Goal:** Area contents

   **ROS 2 Equivalent:** Namespace contents

   **Implementation Notes:** List nodes and topics that live under the given namespace.

.. req:: GET /components/{id}/hosts
   :id: REQ_SOVD_012
   :status: open
   :tags: A

   **Goal:** Apps hosted on a component

   **ROS 2 Equivalent:** Nodes hosted in a process

   **Implementation Notes:** Map each component to the list of nodes running in its process or container.

.. req:: GET /components/{id}/depends-on
   :id: REQ_SOVD_013
   :status: open
   :tags: A

   **Goal:** Component dependencies

   **ROS 2 Equivalent:** Graph edges (pub/sub, services)

   **Implementation Notes:** Analyze the ROS 2 graph (publishers/subscribers, clients/servers) to determine dependencies.

.. req:: GET /apps/{id}/depends-on
   :id: REQ_SOVD_014
   :status: open
   :tags: A

   **Goal:** Application dependencies

   **ROS 2 Equivalent:** Topics / services / actions used by the app

   **Implementation Notes:** Collect interfaces used by all nodes that belong to the app.

.. req:: GET /
   :id: REQ_SOVD_015
   :status: open
   :tags: A

   **Goal:** System capabilities

   **ROS 2 Equivalent:** Global list of interfaces

   **Implementation Notes:** Aggregate all topics, services and actions from NodeGraphInterface into a single capability view.

.. req:: tags=... (query)
   :id: REQ_SOVD_016
   :status: open
   :tags: P

   **Goal:** Filtering by tags

   **ROS 2 Equivalent:** Parameters / metadata

   **Implementation Notes:** Use a convention like component.tags=[...] in parameters and filter entities in the gateway.

