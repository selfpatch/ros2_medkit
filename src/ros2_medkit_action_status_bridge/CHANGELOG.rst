^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_action_status_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial release: generic action-status bridge. Watches every
  ``/<action>/_action/status`` topic and turns ABORTED goals into FaultManager
  faults (``<PREFIX>_<ACTION>_ABORTED``), heals on a non-failing terminal state.
  Captures the terminal action verdict that the diagnostic and log bridges
  cannot see.
* Fault state is per-ACTION, derived from the whole ``GoalStatusArray`` on each
  message: a fault is raised only on the ``healthy -> failed`` transition and
  healed only on ``failed -> healthy``, so it is order-independent and resilient
  to dropped terminal messages. Per-goal dedup now only suppresses duplicate log
  lines.
* ``canceled_is_fault`` emits a status-aware ``<PREFIX>_<ACTION>_CANCELED`` code;
  such a fault heals on the next non-failing terminal or when the canceled goal
  ages out of the retained status array.
* Vanished actions (lifecycle deactivate, one-shot nodes) are pruned on rescan.
* Parameters are range-checked/normalized at load; an incompatible action status
  QoS is now warned about instead of silently dropping faults.
