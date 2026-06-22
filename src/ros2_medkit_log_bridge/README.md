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

- `source_id` of each fault is the originating node's fully-qualified name. It
  is derived from `Log.name` by taking the first dotted segment (a `Log.name`
  may carry a sub-logger suffix, e.g. `controller_manager.resource_manager`, and
  node names cannot contain `.`) and prefixing `/`, giving e.g.
  `/controller_manager`. The gateway discovers entities by node FQN, so this is
  the form that lets a fault (and its snapshots / rosbag) associate with the
  entity in the SOVD tree. Each node gets its own per-node `FaultReporter` and
  therefore its own client-side debounce.
- `fault_code` is auto-generated as `<PREFIX>_<NODE>_<HASH>`. `<HASH>` is a fixed
  FNV-1a 32-bit digest (8 lowercase hex) of a normalized message template
  (numbers / hex / paths stripped, isolated single-letter tokens dropped) so the
  same logical message maps to the same code across occurrences. `<NODE>` is the
  upper-snake of `source_id`. The 8-hex hash is never truncated; if the 64-char
  cap is hit the node part is trimmed instead.

> Namespaced-node limitation: `Log.name` encodes a node's namespace with the same
> `.` separator as a sub-logger suffix, so the two are indistinguishable from the
> string alone. `source_id` takes the first dotted segment, which is right for a
> non-namespaced node with a sub-logger but collapses a namespaced node
> (`robot1.planner_server` -> `/robot1`) to its namespace, so same-named nodes in
> different namespaces share one code. Multi-robot fleets typically isolate robots
> by `ROS_DOMAIN_ID` (one gateway per robot, federated by peer aggregation), which
> sidesteps this.

## Forwarding, the LocalFilter, and confirmation

Two independent debounces sit between a log line and a confirmed fault:

1. Per-node `FaultReporter` `LocalFilter` (client-side). WARN is held until
   `default_threshold` (3) occurrences within `default_window_sec` (10s).
   ERROR/FATAL have severity `>= bypass_severity` (2) and bypass the filter,
   forwarding immediately.
2. Bridge `report_cooldown_sec` cooldown, applied only to `ERROR`/`FATAL` (the
   levels that bypass the LocalFilter). It forwards the first occurrence of a
   `(fault_code, severity)` immediately and suppresses that same pair for
   `report_cooldown_sec` (default 5s, `0.0` disables), bounding a flood. `WARN`
   is never cooled here (that would starve its LocalFilter threshold counting),
   and keying on severity means a `WARN` never suppresses a same-message `ERROR`
   escalation.

Whether a forwarded fault then shows as `PREFAILED` (suspected) or `CONFIRMED`
is a separate, gateway-side decision driven by the FaultManager's
`confirmation_threshold` - not by this bridge and not by the client-side
LocalFilter. For visible-but-quiet WARNs, launch the FaultManager with a low
`confirmation_threshold` (or an entity threshold for `LOG_*` codes).

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
| `severity_floor` | `30` (WARN) | minimum level promoted; raise to `40` on chatty / constrained targets. Clamped to `[0, 50]` at load (a value out of range is corrected with a warning) |
| `code_prefix` | `LOG` | prefix for generated fault codes; normalized to `[A-Z0-9_]` at load |
| `exclude_nodes` | `[]` | node-FQN substrings to skip |
| `include_only_nodes` | `[]` | if set, only promote nodes whose FQN matches |
| `max_tracked_nodes` | `512` | cap on per-node reporters; least-recently-used nodes evicted past this |
| `report_cooldown_sec` | `5.0` | per-fault_code forward debounce; `0.0` disables |
| `exclude_medkit_stack` | `true` | skip medkit's own infrastructure nodes (`fault_manager`, gateway, the other bridges) so their logs do not feed back as faults; set `false` to debug medkit's own logs |

`exclude_nodes` / `include_only_nodes` match as **unanchored substrings**
against the node FQN: `planner` matches `/planner_server` and
`/robot1/planner_server`. Use a longer, more specific substring (e.g.
`/planner_server`) to avoid accidental matches.

`exclude_medkit_stack` likewise matches **unanchored substrings** (on the raw
logger name, so a namespaced `robot1.fault_manager` is still caught). A user
node whose name contains one of these tokens - e.g. `fault_manager` or
`diagnostic_bridge` - is therefore also skipped; set `exclude_medkit_stack:
false` (and use `exclude_nodes` for the real medkit nodes) if that collides
with your naming.
