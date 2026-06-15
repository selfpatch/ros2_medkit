# ros2_medkit_log_bridge

Drop-in bridge that promotes ROS 2 `/rosout` log entries to structured medkit
faults, attributing each fault to the node that logged it. No changes to the
user's nodes are required.

It is a compatibility adapter, the same category as
`ros2_medkit_diagnostic_bridge`. Native `ros2_medkit_fault_reporter`
instrumentation stays the canonical path for code you control; this bridge is
the fallback for nodes that only log.

## What it does

Subscribes to `/rosout` (`rcl_interfaces/msg/Log`) and forwards entries at or
above a severity floor to the FaultManager:

| Log level | medkit severity |
|-----------|-----------------|
| DEBUG (10) / INFO (20) | dropped |
| WARN (30) | `SEVERITY_WARN` |
| ERROR (40) | `SEVERITY_ERROR` |
| FATAL (50) | `SEVERITY_CRITICAL` |

- `source_id` of each fault is the originating node (the `Log.name` field), via
  a per-node `FaultReporter`, so faults attribute correctly and each node gets
  its own local debounce.
- `fault_code` is auto-generated as `<PREFIX>_<NODE>_<HASH>`, where the hash is
  taken over a normalized message template (numbers / hex / paths stripped) so
  the same logical message maps to the same code across occurrences.

## WARN as PREFAILED

The bridge forwards WARN immediately. Whether a WARN shows as `PREFAILED`
(suspected, kept out of the confirmed-fault list) or `CONFIRMED` is decided by
the FaultManager's `confirmation_threshold`, not the bridge. For the
visible-but-quiet behaviour, launch the FaultManager with
`confirmation_threshold:=-2` or lower (or an entity threshold for `LOG_*`
codes). With the shipped default (`-1`), every WARN confirms on first
occurrence.

## Hard limitations (by construction)

- Only sees logs that reach `/rosout` via rclcpp from a still-alive node.
  Console-only loggers (e.g. some Micro XRCE-DDS / non-rclcpp loggers) are
  invisible.
- A node that crashes hard may not flush its final log to `/rosout`, so the
  terminating ERROR can be missed. Process-death detection belongs to a
  separate liveliness bridge, not here.

## Run it

```bash
# next to an existing stack + the medkit gateway/fault_manager
ros2 launch ros2_medkit_log_bridge log_bridge.launch.py
```

## Configuration (`config/log_bridge.yaml`)

| Param | Default | Meaning |
|-------|---------|---------|
| `rosout_topic` | `/rosout` | log topic to subscribe |
| `severity_floor` | `30` (WARN) | minimum level promoted; raise to `40` on chatty / constrained targets |
| `code_prefix` | `LOG` | prefix for generated fault codes |
| `exclude_nodes` | `[]` | node-name substrings to skip |
| `include_only_nodes` | `[]` | if set, only promote these nodes |
