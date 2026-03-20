^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_integration_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-03-20)
------------------
* Integration tests for SOVD resource locking (acquire, release, extend, fault clear with locks, expiration, parent propagation)
* Integration tests for SOVD script execution endpoints (all formats, params, output, failure, lifecycle)
* Integration tests for graph provider plugin (external plugin loading, entity introspection)
* Integration tests for beacon discovery plugins (topic beacon, parameter beacon)
* Integration tests for OpenAPI/docs endpoint
* Integration tests for logging endpoints (``/logs``, ``/logs/configuration``)
* Integration tests for linux introspection plugins (launch_testing and Docker-based)
* Port isolation per integration test via CMake-assigned unique ports
* ``ROS_DOMAIN_ID`` isolation for integration tests
* Build: use shared cmake modules from ``ros2_medkit_cmake`` package
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Refactored integration test suite into dedicated ``ros2_medkit_integration_tests`` package (`#227 <https://github.com/selfpatch/ros2_medkit/pull/227>`_)
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda
