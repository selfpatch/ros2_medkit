^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_diagnostic_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-03-20)
------------------
* Build: use shared cmake modules from ``ros2_medkit_cmake`` package
* Build: auto-detect ccache, centralized clang-tidy configuration
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* Bridge node converting standard ROS 2 /diagnostics to FaultManager fault reports
* Severity mapping:

  * OK -> PASSED event (fault condition cleared)
  * WARN -> WARN severity FAILED event
  * ERROR -> ERROR severity FAILED event
  * STALE -> CRITICAL severity FAILED event

* Auto-generated fault codes from diagnostic names (UPPER_SNAKE_CASE)
* Custom name_to_code mappings via ROS parameters
* Stateless design: always sends PASSED for OK status (handles restarts)
* Contributors: Michal Faferek
