^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_graph_provider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ``x-medkit-graph`` pipeline health rework: ``pipeline_status`` no longer misreports ``"broken"`` for edges with no ``/diagnostics`` coverage at all (the previous status model conflated "never observed" with "actively broken"). ``error_reason`` is now freshness-based and its only reachable value is ``metrics_stale``; ``node_offline``, ``topic_stale``, and ``no_data_source`` are gone. ``metrics.source`` is now the actual resolved ``/diagnostics`` publisher node name (resolved per message via publisher GID matching against ``/diagnostics``), omitted rather than hardcoded to ``"greenwave_monitor"`` when it cannot be resolved. Per-function threshold overrides (``plugins.graph_provider.function_overrides.<function_id>.*``) now take effect. ``schema_version`` bumped to ``"2.0.0"`` (`#545 <https://github.com/selfpatch/ros2_medkit/issues/545>`_)

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Migrated ``GraphProviderPlugin`` to the ``get_routes()`` plugin API and fixed a route-separator bug
* Added shutdown guards and ``noexcept`` destructors that reset rclcpp resources before member destruction, preventing teardown SIGSEGV
* Added post-shutdown guard unit tests; dropped the cpp-httplib source install from the Dockerfile (now vendored via ``ros2_medkit_cmake``)
* Build: adopt the centralized ``ROS2MedkitWarnings`` cmake module
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - extracted from ``ros2_medkit_gateway`` package
* ``GraphProviderPlugin`` for ROS 2 graph-based entity introspection
* Standalone external plugin package with independent build and test
* Locking support via ``PluginContext`` API
* Contributors: @bburda
