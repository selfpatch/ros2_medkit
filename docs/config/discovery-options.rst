Discovery Options Reference
===========================

.. contents::
   :local:
   :depth: 2

This document describes configuration options for the gateway's discovery system.
The discovery system maps ROS 2 graph entities (nodes, topics, services, actions)
to SOVD entities (areas, components, apps).

Discovery Modes
---------------

The gateway supports three discovery modes, controlled by the ``discovery.mode`` parameter:

.. list-table:: Discovery Modes
   :header-rows: 1
   :widths: 20 80

   * - Mode
     - Description
   * - ``runtime_only``
     - Use ROS 2 graph introspection only. Nodes are discovered at runtime
       and mapped to SOVD entities using heuristic rules. (default)
   * - ``manifest_only``
     - Only expose entities declared in the manifest file. Runtime discovery
       is disabled.
   * - ``hybrid``
     - Manifest defines the structure, runtime links to discovered nodes.

Runtime Discovery Options
-------------------------

When using ``runtime_only`` or ``hybrid`` mode, the following options control
how ROS 2 nodes are mapped to SOVD entities.

Synthetic Components
^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   discovery:
     runtime:
       create_synthetic_components: true
       grouping_strategy: "namespace"
       synthetic_component_name_pattern: "{area}"

When ``create_synthetic_components`` is true:

- Components are created as logical groupings of Apps
- ``grouping_strategy: "namespace"`` groups nodes by their first namespace segment
- ``synthetic_component_name_pattern`` defines the component ID format

Topic-Only Namespaces
^^^^^^^^^^^^^^^^^^^^^

Some ROS 2 systems have topics published to namespaces without any associated nodes.
This is common with:

- Isaac Sim and other simulators
- External bridges (MQTT, Zenoh, ROS 1)
- Dead/orphaned topics from crashed processes

The ``topic_only_policy`` controls how these namespaces are handled:

.. code-block:: yaml

   discovery:
     runtime:
       topic_only_policy: "create_component"
       min_topics_for_component: 1

.. list-table:: Topic-Only Policies
   :header-rows: 1
   :widths: 25 75

   * - Policy
     - Description
   * - ``ignore``
     - Don't create any entities for topic-only namespaces.
       Use this to reduce noise from orphaned topics.
   * - ``create_component``
     - Create a Component with ``source: "topic"`` for each topic-only
       namespace. (default)
   * - ``create_area_only``
     - Only create the Area, but don't create a Component.
       Useful when you want the namespace visible but not as a component.

The ``min_topics_for_component`` parameter (default: 1) sets the minimum number
of topics required before creating a component. This can filter out namespaces
with only a few stray topics.

Merge Pipeline (Hybrid Mode)
-----------------------------

In hybrid mode, the gateway uses a layered merge pipeline to combine entities
from multiple sources. Three layers contribute entities independently:

- **ManifestLayer** - entities declared in the YAML manifest (highest priority for identity/hierarchy)
- **RuntimeLayer** - entities discovered from the ROS 2 graph (highest priority for live data/status)
- **PluginLayer** - entities contributed by ``IntrospectionProvider`` plugins

Each layer's contribution is merged per **field-group** with configurable policies.

Field Groups
^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Field Group
     - Contents
   * - ``identity``
     - id, name, translation_id, description, tags
   * - ``hierarchy``
     - area, component_id, parent references, depends_on, hosts
   * - ``live_data``
     - topics, services, actions
   * - ``status``
     - is_online, bound_fqn
   * - ``metadata``
     - source, x-medkit extensions, custom metadata fields

Merge Policies
^^^^^^^^^^^^^^

Each layer declares a policy per field-group:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Policy
     - Behavior
   * - ``authoritative``
     - This layer's value wins over lower-priority layers.
   * - ``enrichment``
     - Fill empty fields only; don't override existing values.
   * - ``fallback``
     - Use only if no other layer provides the value.

**Defaults:**

- Manifest: ``authoritative`` for identity/hierarchy/metadata, ``enrichment`` for live_data, ``fallback`` for status
- Runtime: ``authoritative`` for live_data/status, ``enrichment`` for metadata, ``fallback`` for identity/hierarchy

Override per-layer policies in ``gateway_params.yaml``. Empty string means
"use layer default". Policy values are **case-sensitive** and must be lowercase
(``authoritative``, ``enrichment``, ``fallback``):

.. code-block:: yaml

   discovery:
     merge_pipeline:
       layers:
         manifest:
           identity: ""          # authoritative (default)
           hierarchy: ""         # authoritative (default)
           live_data: ""         # enrichment (default)
           status: ""            # fallback (default)
           metadata: ""          # authoritative (default)
         runtime:
           identity: ""          # fallback (default)
           hierarchy: ""         # fallback (default)
           live_data: ""         # authoritative (default)
           status: ""            # authoritative (default)
           metadata: ""          # enrichment (default)

Gap-Fill
^^^^^^^^

In hybrid mode, the runtime layer can create heuristic entities for namespaces
not covered by the manifest. Gap-fill controls what the runtime layer is allowed
to create:

.. code-block:: yaml

   discovery:
     merge_pipeline:
       gap_fill:
         allow_heuristic_areas: true
         allow_heuristic_components: true
         allow_heuristic_apps: true
         allow_heuristic_functions: false
         # namespace_blacklist: ["/rosout"]
         # namespace_whitelist: []

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``allow_heuristic_areas``
     - ``true``
     - Create areas from namespaces not in manifest.
   * - ``allow_heuristic_components``
     - ``true``
     - Create synthetic components for unmanifested namespaces.
   * - ``allow_heuristic_apps``
     - ``true``
     - Create apps for nodes without manifest ``ros_binding``.
   * - ``allow_heuristic_functions``
     - ``false``
     - Create heuristic functions (not recommended).
   * - ``namespace_blacklist``
     - ``[]``
     - Namespaces excluded from gap-fill (e.g., ``["/rosout"]``).
   * - ``namespace_whitelist``
     - ``[]``
     - If non-empty, only these namespaces are eligible for gap-fill.

Health Endpoint
^^^^^^^^^^^^^^^

``GET /api/v1/health`` includes a ``discovery`` section in hybrid mode with
pipeline stats, linking results, and merge warnings:

.. code-block:: json

   {
     "status": "healthy",
     "discovery": {
       "mode": "hybrid",
       "strategy": "hybrid",
       "pipeline": {
         "layers": ["manifest", "runtime", "plugin"],
         "total_entities": 6,
         "enriched_count": 5,
         "conflict_count": 0,
         "conflicts": [],
         "id_collisions": 0,
         "filtered_by_gap_fill": 0
       },
       "linking": {
         "linked_count": 5,
         "orphan_count": 1,
         "binding_conflicts": 0
       }
     }
   }

Configuration Example
---------------------

Complete YAML configuration for runtime discovery:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       discovery:
         mode: "runtime_only"

         runtime:
           # Group Apps into Components by namespace
           create_synthetic_components: true
           grouping_strategy: "namespace"
           synthetic_component_name_pattern: "{area}"

           # Handle topic-only namespaces
           topic_only_policy: "create_component"
           min_topics_for_component: 2  # Require at least 2 topics

         # Merge pipeline (hybrid mode only)
         merge_pipeline:
           gap_fill:
             allow_heuristic_areas: true
             allow_heuristic_components: true
             allow_heuristic_apps: true
             namespace_blacklist: ["/rosout"]

Command Line Override
---------------------

Override discovery options via command line:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py \
     discovery.runtime.topic_only_policy:="ignore" \
     discovery.runtime.min_topics_for_component:=3

Beacon Discovery Plugin (TopicBeaconPlugin)
--------------------------------------------

The ``ros2_medkit_topic_beacon`` plugin enriches discovered entities with
push-based metadata from ROS 2 nodes. Nodes publish
``ros2_medkit_msgs/msg/MedkitDiscoveryHint`` messages to
``/ros2_medkit/discovery`` and the plugin injects the data into the merge
pipeline as an ``ENRICHMENT`` layer.

Configuration
^^^^^^^^^^^^^

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["topic_beacon"]
       plugins.topic_beacon.path: "/path/to/libtopic_beacon_plugin.so"

       # ROS 2 topic to subscribe for beacon hints
       # Default: "/ros2_medkit/discovery"
       plugins.topic_beacon.topic: "/ros2_medkit/discovery"

       # Soft TTL: hints older than this are marked STALE (seconds)
       # Default: 10.0
       plugins.topic_beacon.beacon_ttl_sec: 10.0

       # Hard expiry: hints older than this are removed (seconds)
       # Default: 300.0
       plugins.topic_beacon.beacon_expiry_sec: 300.0

       # Allow plugin to introduce entirely new entities not seen by other layers
       # When false (recommended), only known entities are enriched
       # Default: false
       plugins.topic_beacon.allow_new_entities: false

       # Verify process is alive by checking /proc/<pid>
       # Default: true
       plugins.topic_beacon.check_process_alive: true

       # Maximum number of hints to keep in memory
       # Default: 10000
       plugins.topic_beacon.max_hints: 10000

       # Rate limit for incoming beacon messages
       # Default: 100
       plugins.topic_beacon.max_messages_per_second: 100

Beacon Lifecycle
^^^^^^^^^^^^^^^^

Each hint follows a soft TTL lifecycle based on the ``stamp`` field:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - State
     - Description
   * - **ACTIVE**
     - Hint is within ``beacon_ttl_sec``. Enrichment is applied normally.
   * - **STALE**
     - Hint is past TTL but within ``beacon_expiry_sec``.
       Enrichment is still applied; the ``x-medkit-beacon`` endpoint
       includes a ``stale`` flag in its response.
   * - **EXPIRED**
     - Hint is past ``beacon_expiry_sec``. It is removed from the store
       and no longer contributes to enrichment.

The ``x-medkit-beacon`` vendor endpoint exposes current beacon state:

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/my_sensor/x-medkit-beacon

**Example Response:**

.. code-block:: json

   {
     "entity_id": "my_sensor",
     "status": "active",
     "display_name": "Temperature Sensor",
     "function_ids": ["thermal_monitoring"],
     "process_id": 12345,
     "process_name": "component_container",
     "hostname": "robot-01",
     "last_seen": "2026-03-13T10:30:00.123Z",
     "stale": false
   }

See Also
--------

- :doc:`manifest-schema` - Manifest-based configuration
- :doc:`/tutorials/heuristic-apps` - Tutorial on runtime discovery
- :doc:`/tutorials/manifest-discovery` - Hybrid mode tutorial
- :doc:`/tutorials/plugin-system` - Plugin layer integration
