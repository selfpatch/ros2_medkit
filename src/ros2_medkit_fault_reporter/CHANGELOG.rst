^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_reporter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
