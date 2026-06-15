# ros2_medkit_action_status_bridge

Generic drop-in bridge that turns terminal ROS 2 action goal states into
structured medkit faults. It watches the `/<action>/_action/status`
(`action_msgs/msg/GoalStatusArray`) topic that **every** action server
publishes, so it works across Nav2, MoveIt 2, ros2_control's controller
actions, and any custom action with **no per-project code**.

## Why this matters

For the core ROS 2 league the authoritative "the goal failed" verdict lives on
the action-result channel, not on `/diagnostics` or `/rosout`:

- Nav2 `NavigateToPose` -> `GoalStatus = ABORTED`
- MoveIt `MoveGroup` -> `GoalStatus = ABORTED` (with `MoveItErrorCode` in the result)
- ros2_control controller actions -> aborted goals

Neither the diagnostic bridge nor the log bridge sees this. This bridge does,
generically, by observing the goal-status topic.

## What it does

- Discovers every action on the graph by scanning for `*/_action/status`
  topics (re-scanned on a timer to catch actions that appear later).
- `ABORTED (6)` -> fault (`SEVERITY_ERROR` by default).
- `CANCELED (5)` -> fault only if `canceled_is_fault` (off by default; cancel is
  usually intentional).
- `SUCCEEDED (4)` -> `PASSED` to heal the action's ABORTED code (if enabled).
- `fault_code` is `<PREFIX>_<ACTION>_ABORTED`, e.g.
  `ACTION_NAVIGATE_TO_POSE_ABORTED`. `source_id` is the action name.
- Per-goal dedup keyed on `goal_id` so a latched terminal status is reported
  once, not on every status publication.

## Scope: the terminal verdict, not the reason

This bridge delivers the generic "it aborted" event. The action-specific
*reason* (e.g. `MoveItErrorCode.val = -26 START_STATE_IN_COLLISION`, Nav2
`error_code` on Iron/Jazzy) lives in the action result message and is a separate
enrichment concern (a future action-result reader using runtime message
introspection / dynmsg). Per-project plugins are only needed for human-readable
labels, not to surface the fault.

## Run it

```bash
ros2 launch ros2_medkit_action_status_bridge action_status_bridge.launch.py
```

## Configuration (`config/action_status_bridge.yaml`)

| Param | Default | Meaning |
|-------|---------|---------|
| `aborted_severity` | `2` (ERROR) | severity of an aborted goal |
| `canceled_is_fault` | `false` | treat CANCELED as a fault |
| `heal_on_succeeded` | `true` | send PASSED on a successful goal |
| `rescan_period_sec` | `2.0` | how often to look for new actions |
| `code_prefix` | `ACTION` | prefix for generated codes |
| `exclude_actions` | `[]` | action-name substrings to skip |
| `include_only_actions` | `[]` | if set, only watch these |
| `dedup_capacity` | `4096` | remembered goal/status keys |
