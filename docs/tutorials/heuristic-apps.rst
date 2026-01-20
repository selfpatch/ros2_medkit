Heuristic Runtime Discovery
===========================

This tutorial explains how to use heuristic runtime discovery to expose
ROS 2 nodes as SOVD Apps, with synthetic Components grouping them logically.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

By default, ros2_medkit gateway uses **heuristic discovery** to map ROS 2 graph
entities to SOVD entities:

- **ROS 2 nodes** → **Apps** (software applications)
- **Namespaces** → **Areas** (logical groupings)
- **Apps grouped by namespace** → **Synthetic Components**
- **Topics, services, actions** → **Data, Operations**

This approach requires no configuration and works out of the box with any
ROS 2 system.

When to Use Heuristic Discovery
-------------------------------

Heuristic discovery is ideal when:

✅ You want to explore a ROS 2 system without prior setup
✅ Entity IDs can be derived from node names
✅ Your system doesn't require stable IDs across restarts
✅ You're prototyping or debugging

Consider using :doc:`manifest-discovery` when:

❌ You need stable, semantic IDs (e.g., ``front-lidar`` instead of ``scan_node``)
❌ You need to define entities that don't exist at runtime
❌ You need offline detection of failed components

Quick Start
-----------

Launch the gateway with default settings:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py

The gateway automatically discovers all nodes and maps them to SOVD entities.

Query available Apps:

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps | jq

Example response:

.. code-block:: json

   {
     "items": [
       {
         "id": "lidar_driver",
         "name": "lidar_driver",
         "namespace_path": "/perception",
         "area": "perception",
         "component": "perception",
         "source": "heuristic"
       },
       {
         "id": "camera_node",
         "name": "camera_node",
         "namespace_path": "/perception",
         "area": "perception",
         "component": "perception",
         "source": "heuristic"
       }
     ]
   }

Understanding the Entity Hierarchy
----------------------------------

With heuristic discovery, the SOVD hierarchy is built as follows:

.. code-block:: text

   Area: "perception"                    ← from namespace /perception
   └── Component: "perception"           ← synthetic, groups apps in this area
       ├── App: "lidar_driver"           ← from node /perception/lidar_driver
       │   ├── Data: "scan"              ← published topics
       │   └── Operations: ...           ← services/actions
       └── App: "camera_node"            ← from node /perception/camera_node
           └── Data: "image_raw"

Configuration Options
---------------------

All options are under ``discovery.runtime`` in the gateway parameters:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       discovery:
         mode: "runtime_only"  # or "hybrid"

         runtime:
           create_synthetic_components: true
           grouping_strategy: "namespace"
           synthetic_component_name_pattern: "{area}"

create_synthetic_components
^^^^^^^^^^^^^^^^^^^^^^^^^^^

When ``true`` (default), the gateway creates synthetic Components to group Apps:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components
   # Returns: [{"id": "perception", "source": "synthetic", ...}]

   curl http://localhost:8080/api/v1/components/perception/apps
   # Returns: [{"id": "lidar_driver"}, {"id": "camera_node"}]

When ``false``, each node is its own Component (no grouping):

.. code-block:: bash

   curl http://localhost:8080/api/v1/components
   # Returns: [{"id": "lidar_driver"}, {"id": "camera_node"}]

grouping_strategy
^^^^^^^^^^^^^^^^^

Controls how Apps are grouped into Components:

- ``namespace`` (default): Group by first namespace segment
- ``none``: Each app is its own component

Handling Topic-Only Namespaces
------------------------------

Some systems (like Isaac Sim) publish topics without creating ROS 2 nodes.
The ``topic_only_policy`` controls how these are handled:

.. code-block:: yaml

   discovery:
     runtime:
       topic_only_policy: "create_component"
       min_topics_for_component: 2

Policies:

- ``create_component`` (default): Create a Component for topic namespaces
- ``create_area_only``: Create only the Area, no Component
- ``ignore``: Skip topic-only namespaces entirely

Example: Filtering Noise
^^^^^^^^^^^^^^^^^^^^^^^^

To ignore orphaned topics from crashed processes:

.. code-block:: yaml

   discovery:
     runtime:
       topic_only_policy: "ignore"

Or require multiple topics before creating a component:

.. code-block:: yaml

   discovery:
     runtime:
       topic_only_policy: "create_component"
       min_topics_for_component: 3  # Need 3+ topics

API Endpoints
-------------

Apps Endpoints
^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Endpoint
     - Description
   * - ``GET /apps``
     - List all discovered Apps
   * - ``GET /apps/{app_id}``
     - Get specific App details
   * - ``GET /components/{id}/apps``
     - List Apps in a synthetic Component

Components Endpoints
^^^^^^^^^^^^^^^^^^^^

With synthetic components, the ``/components`` endpoint returns
grouped entities:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components | jq '.items[] | {id, source}'

.. code-block:: json

   {"id": "perception", "source": "synthetic"}
   {"id": "navigation", "source": "synthetic"}
   {"id": "isaac_sim", "source": "topic"}

The ``source`` field indicates how the component was discovered:

- ``synthetic``: Grouped from multiple nodes
- ``node``: Direct node-to-component mapping (legacy mode)
- ``topic``: From topic-only namespace

Migrating to Manifest Discovery
-------------------------------

When you need more control, consider migrating to hybrid mode.
See :doc:`migration-to-manifest` for a step-by-step guide.

Key differences:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Feature
     - Heuristic
     - Manifest
   * - Setup required
     - None
     - YAML manifest file
   * - Entity IDs
     - Derived from node names
     - Custom, semantic IDs
   * - Offline detection
     - No
     - Yes (failed components)
   * - Stable across restarts
     - No (depends on node names)
     - Yes (defined in manifest)

See Also
--------

- :doc:`/config/discovery-options` - Full configuration reference
- :doc:`manifest-discovery` - Manifest-based discovery
- :doc:`migration-to-manifest` - Migration guide
