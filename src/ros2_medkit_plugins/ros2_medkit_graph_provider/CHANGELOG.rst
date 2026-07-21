^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_graph_provider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ``x-medkit-graph`` pipeline health rework: ``pipeline_status`` no longer misreports ``"broken"`` for edges with no ``/diagnostics`` coverage at all (the previous status model conflated "never observed" with "actively broken"). ``error_reason`` is now freshness-based and its only reachable value is ``metrics_stale``; ``node_offline``, ``topic_stale``, and ``no_data_source`` are gone. ``metrics.source`` is now the actual resolved ``/diagnostics`` publisher node name (resolved per message via publisher GID matching against ``/diagnostics``), omitted rather than hardcoded to ``"greenwave_monitor"`` when it cannot be resolved. Per-function threshold overrides (``plugins.graph_provider.function_overrides.<function_id>.*``) now take effect. ``schema_version`` bumped to ``"2.0.0"`` (`#545 <https://github.com/selfpatch/ros2_medkit/issues/545>`_)
* Robustness hardening for ``x-medkit-graph`` health signals: freshness and staleness are now computed against a monotonic clock, so a backward wall-clock step (e.g. an NTP correction) can no longer make a dead topic read as fresh; a new ``stale_grace_sec`` setting (default ``2.0``, per-function overridable) debounces a single late ``/diagnostics`` sample so ``pipeline_status`` does not flap to ``"broken"`` and back; a new ``multi_publisher_rate`` setting (default ``"annotate"``, per-function overridable) defends against a duplicate or leftover publisher inflating a topic's summed arrival rate and masking a slow pipeline as healthy, backed by new per-edge ``metrics.publisher_count`` and ``metrics.rate_ambiguous`` fields; ``metrics.source`` is now cleared - not retained - whenever the most recent sample's publisher cannot be resolved, matching its documented latest-wins contract; and invalid ``plugins.graph_provider.*`` configuration, including per-function override field-name typos and wrong-typed values, now logs a warning and falls back to the previous valid value instead of failing silently (`#545 <https://github.com/selfpatch/ros2_medkit/issues/545>`_)

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
