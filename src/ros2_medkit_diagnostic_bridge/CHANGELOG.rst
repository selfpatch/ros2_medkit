^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_diagnostic_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
