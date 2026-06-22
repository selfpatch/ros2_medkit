^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_log_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-22)
------------------
* Initial release: promote ``/rosout`` log entries (WARN/ERROR/FATAL) to
  FaultManager faults, attributed to the originating node via a per-source
  FaultReporter, with auto-generated stable fault codes (`#422 <https://github.com/selfpatch/ros2_medkit/pull/422>`_)
* Ships a default configuration so the bridge starts out of the box (`#449 <https://github.com/selfpatch/ros2_medkit/pull/449>`_)
* Skips the medkit stack's own nodes by default, matching on the raw logger name (`#460 <https://github.com/selfpatch/ros2_medkit/pull/460>`_)
* Contributors: @mfaferek93
