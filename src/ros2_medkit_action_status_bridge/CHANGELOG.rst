^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_action_status_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial release: generic action-status bridge. Watches every
  ``/<action>/_action/status`` topic and turns ABORTED goals into FaultManager
  faults (``<PREFIX>_<ACTION>_ABORTED``), heals on SUCCEEDED, with per-goal
  dedup. Captures the terminal action verdict that the diagnostic and log
  bridges cannot see.
