^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_graph_provider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
