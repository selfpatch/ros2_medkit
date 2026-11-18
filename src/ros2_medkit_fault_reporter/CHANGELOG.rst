^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_reporter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2026-11-18)
------------------
* Delivery of a report is confirmed, and a report without a response is sent again. ``fault_reporter.delivery.ack_timeout_sec`` sets how long a response is awaited, a queued report is dropped after ``fault_reporter.delivery.max_attempts`` (10) tries, and a new report is dropped when ``fault_reporter.delivery.max_pending`` (1024) reports are queued. A report made while the service is not ready is dropped. When a reporter is destroyed, the ROS context takes its pending reports and keeps sending them (`#764 <https://github.com/selfpatch/ros2_medkit/pull/764>`_)
* **Breaking (ABI):** the member layout of ``FaultReporter`` in ``fault_reporter.hpp`` changed (``filter_`` is gone; ``delivery_config_``, ``delivery_`` and ``core_`` are new), so a binary built against 0.7.0 must be rebuilt. The reporting logic is split into a ROS-free core (`#739 <https://github.com/selfpatch/ros2_medkit/pull/739>`_)
* Contributors: @bburda

0.7.0 (2026-08-27)
------------------
* ``FaultReporter`` can now be constructed from an ``rclcpp_lifecycle::LifecycleNode`` (and from a plain ``rclcpp::Node`` reference or explicit node interfaces), enabling use inside lifecycle nodes (`#556 <https://github.com/selfpatch/ros2_medkit/pull/556>`_, `#555 <https://github.com/selfpatch/ros2_medkit/issues/555>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), and every test takes a DDS domain at run time from the shared allocator (`#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* Contributors: @bburda, @mfaferek93, @zeerekahmad

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* ``LocalFilter`` now debounces ``PASSED`` events with the same threshold / window filtering as ``FAILED``, preventing rapid CONFIRMED/CLEARED status cycling from triggering unbounded snapshot recapture (`#308 <https://github.com/selfpatch/ros2_medkit/issues/308>`_)
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules
* Contributors: @bburda, @mfaferek93

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
* FaultReporter client library with simple API:

  * report(fault_code, severity, description) - report FAILED events
  * report_passed(fault_code) - report fault condition cleared
  * High-severity faults (ERROR, CRITICAL) bypass local filtering

* LocalFilter for per-fault-code threshold/window filtering:

  * Configurable threshold (default: 3 reports) and time window (default: 10s)
  * Prevents flooding FaultManager with duplicate reports
  * PASSED events always forwarded (bypass filtering)

* Configuration via ROS parameters (filter_threshold, filter_window_sec)
* Thread-safe implementation with mutex-protected config access
* Contributors: Bartosz Burda, Michal Faferek
