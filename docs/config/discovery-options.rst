Discovery Options Reference
===========================

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

Merge Pipeline Options (Hybrid Mode)
-------------------------------------

When using ``hybrid`` mode, the merge pipeline controls how entities from
different discovery layers are combined. The ``merge_pipeline`` section
configures gap-fill behavior for runtime-discovered entities.

Gap-Fill Configuration
^^^^^^^^^^^^^^^^^^^^^^

In hybrid mode, the manifest is the source of truth. Runtime (heuristic) discovery
fills gaps - entities that exist at runtime but aren't in the manifest. Gap-fill
controls restrict what the runtime layer is allowed to create:

.. code-block:: yaml

   discovery:
     merge_pipeline:
       gap_fill:
         allow_heuristic_areas: true
         allow_heuristic_components: true
         allow_heuristic_apps: true
         allow_heuristic_functions: false
         namespace_whitelist: []
         namespace_blacklist: []

.. list-table:: Gap-Fill Options
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``allow_heuristic_areas``
     - ``true``
     - Allow runtime layer to create Area entities not in the manifest
   * - ``allow_heuristic_components``
     - ``true``
     - Allow runtime layer to create Component entities not in the manifest
   * - ``allow_heuristic_apps``
     - ``true``
     - Allow runtime layer to create App entities not in the manifest
   * - ``allow_heuristic_functions``
     - ``false``
     - Allow runtime layer to create Function entities (rarely useful at runtime)
   * - ``namespace_whitelist``
     - ``[]``
     - If non-empty, only allow gap-fill from these ROS 2 namespaces (Areas and Components only)
   * - ``namespace_blacklist``
     - ``[]``
     - Exclude gap-fill from these ROS 2 namespaces (Areas and Components only)

When both whitelist and blacklist are empty, all namespaces are eligible for gap-fill.
If whitelist is non-empty, only listed namespaces pass. Blacklist is always applied.

Namespace matching uses path-segment boundaries: ``/nav`` matches ``/nav`` and ``/nav/sub``
but NOT ``/navigation``. Both lists only filter Areas and Components (which carry
``namespace_path``). Apps and Functions are not namespace-filtered.


Merge Policies
^^^^^^^^^^^^^^

Each discovery layer declares a ``MergePolicy`` per field group. When two layers
provide the same entity (matched by ID), policies determine which values win:

.. list-table:: Merge Policies
   :header-rows: 1
   :widths: 25 75

   * - Policy
     - Description
   * - ``AUTHORITATIVE``
     - This layer's value wins over lower-priority layers.
       If two AUTHORITATIVE layers conflict, a warning is logged and the
       higher-priority (earlier) layer wins.
   * - ``ENRICHMENT``
     - Fill empty fields from this layer. Non-empty target values are preserved.
       Two ENRICHMENT layers merge additively (collections are unioned).
   * - ``FALLBACK``
     - Use only if no other layer provides the value. Two FALLBACK layers
       merge additively (first non-empty fills gaps).

**Built-in layer policies:**

- **ManifestLayer** (priority 1): IDENTITY, HIERARCHY, METADATA are AUTHORITATIVE.
  LIVE_DATA is ENRICHMENT (runtime topics/services take precedence).
  STATUS is FALLBACK (manifest cannot know online state).
- **RuntimeLayer** (priority 2): LIVE_DATA and STATUS are AUTHORITATIVE.
  METADATA is ENRICHMENT. IDENTITY and HIERARCHY are FALLBACK.
- **PluginLayer** (priority 3+): All field groups ENRICHMENT

Configuration Example
---------------------

Complete YAML configuration for runtime discovery:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       discovery:
         mode: "runtime_only"

         runtime:
           # Map nodes to Apps
           expose_nodes_as_apps: true

           # Group Apps into Components by namespace
           create_synthetic_components: true
           grouping_strategy: "namespace"
           synthetic_component_name_pattern: "{area}"

           # Handle topic-only namespaces
           topic_only_policy: "create_component"
           min_topics_for_component: 2  # Require at least 2 topics

         # Note: merge_pipeline settings only apply when mode is "hybrid"
         merge_pipeline:
           gap_fill:
             allow_heuristic_areas: true
             allow_heuristic_components: true
             allow_heuristic_apps: true
             allow_heuristic_functions: false
             namespace_whitelist: []
             namespace_blacklist: ["/rosout", "/parameter_events"]

Command Line Override
---------------------

Override discovery options via command line:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py \
     discovery.runtime.topic_only_policy:="ignore" \
     discovery.runtime.min_topics_for_component:=3

See Also
--------

- :doc:`manifest-schema` - Manifest-based configuration
- :doc:`/tutorials/heuristic-apps` - Tutorial on runtime discovery
- :doc:`/tutorials/manifest-discovery` - Hybrid mode tutorial
- :doc:`/tutorials/plugin-system` - Plugin layer integration
