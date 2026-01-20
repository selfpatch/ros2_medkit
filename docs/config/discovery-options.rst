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
