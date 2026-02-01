Configuration Reference
=======================

This section contains configuration references for ros2_medkit.

.. toctree::
   :maxdepth: 2

   server
   discovery-options
   manifest-schema
   fault-manager
   diagnostic-bridge

Server Configuration
--------------------

:doc:`server`
   REST server settings including network binding, TLS/HTTPS, CORS,
   data access tuning, and performance options.

Discovery Options
-----------------

:doc:`discovery-options`
   Configuration reference for runtime discovery options.
   Controls how ROS 2 nodes, topics, and services are mapped to SOVD entities.

Manifest Configuration
----------------------

:doc:`manifest-schema`
   Complete YAML schema reference for SOVD system manifests.
   Defines areas, components, apps, and functions for your ROS 2 system.

Fault Manager
-------------

:doc:`fault-manager`
   FaultManager node configuration: storage, debounce thresholds, snapshot
   capture, rosbag recording, and correlation settings.

Diagnostic Bridge
-----------------

:doc:`diagnostic-bridge`
   Diagnostic bridge configuration for converting standard ROS 2 diagnostics
   to fault events. Includes custom fault code mappings.
