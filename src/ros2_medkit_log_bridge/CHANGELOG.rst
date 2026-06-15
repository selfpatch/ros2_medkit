^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_log_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial release: promote ``/rosout`` log entries (WARN/ERROR/FATAL) to
  FaultManager faults, attributed to the originating node via a per-source
  FaultReporter, with auto-generated stable fault codes.
