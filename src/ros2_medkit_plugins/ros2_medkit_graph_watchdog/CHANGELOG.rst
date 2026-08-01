^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_graph_watchdog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Gateway plugin detecting silent faults in the ROS 2 graph, with a central reliability gate (bringup-quiesce warmup + lifecycle gating), a ``GET /x-medkit-watchdog`` status route, and four detectors: ``qos_mismatch`` (``GRAPH_QOS_MISMATCH``), ``orphan`` (``GRAPH_ORPHAN``), ``param_drift`` (``GRAPH_PARAM_DRIFT``) and ``lifecycle_expectation`` (``GRAPH_NODE_INACTIVE``)
