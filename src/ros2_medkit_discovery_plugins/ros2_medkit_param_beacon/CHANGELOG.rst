^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_param_beacon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Migrated ``ParameterBeaconPlugin`` to the ``get_routes()`` plugin API
* Added shutdown guards and ``noexcept`` destructors that reset rclcpp resources before member destruction, preventing teardown SIGSEGV; the graph poll now swallows ``rcl`` "context invalid" during shutdown
* Added post-shutdown guard unit tests
* Build: adopt the centralized ``ROS2MedkitWarnings`` cmake module
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - parameter-based beacon discovery plugin
* ``ParameterBeaconPlugin`` with pull-based parameter reading for entity enrichment
* ``x-medkit-param-beacon`` vendor extension REST endpoint
* Poll target discovery from ROS graph in non-hybrid mode
* ``ParameterClientInterface`` for testable parameter access
* Contributors: @bburda
