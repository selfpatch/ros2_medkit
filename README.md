# ros2_medkit

[![CI](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/selfpatch/ros2_medkit/branch/main/graph/badge.svg)](https://codecov.io/gh/selfpatch/ros2_medkit)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2 Jazzy | Humble | Lyrical](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble%20%7C%20Lyrical-blue)](https://docs.ros.org/en/jazzy/)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/6CXPMApAyq)

<p align="center">
  <img src="docs/_static/images/medkit_hero.gif" alt="A Nav2 NavigateToPose goal aborts: stock ROS 2 diagnostics leave you a log line lost in the noise, while ros2_medkit turns it into one structured fault over REST" width="820">
</p>

<p align="center">
  <b>A structured diagnostic model for ROS 2 robots, over REST.</b><br>
  Drop it next to the stack you already run - no code changes - and a failure that ROS 2
  diagnostics leaves as a log line becomes one structured fault: a fault code on a SOVD entity
  tree, with lifecycle, history, and the state captured at the moment it happened.
</p>

<p align="center">
  Structured fault codes · SOVD entity tree · Fault lifecycle · Freeze-frame + black-box capture · <a href="https://github.com/selfpatch/ros2_medkit_mcp">AI via MCP</a>
</p>

## Drop it into the stack you already run

### Nav2: a navigation goal that quietly aborts

You run Nav2. A `NavigateToPose` goal aborts - the planner gives up, the robot is wedged in a
corner - and the only trace is a line buried in a log you would have to SSH in to read.

Start ros2_medkit next to it (no changes to Nav2). The aborted goal becomes a fault:

```bash
curl http://localhost:8080/api/v1/apps/bt_navigator/faults
# → ACTION_NAVIGATE_TO_POSE_ABORTED  severity=ERROR  source=/bt_navigator  status=CONFIRMED
#   + a black-box rosbag of the seconds around the failure
```

It works because an aborted action goal is often the *only* failure signal Nav2 emits, and
ros2_medkit's action bridge turns that into a structured fault on the `bt_navigator` entity -
no instrumentation, no callbacks added to Nav2.

### MoveIt: a motion that fails to execute

You run MoveIt. A `MoveGroup` goal aborts - no valid plan, or the controller rejects the
trajectory. Same story: the same action bridge surfaces the aborted move as a fault on the
`move_group` entity, with the freeze-frame of what the arm was doing, without touching MoveIt.

```bash
curl http://localhost:8080/api/v1/apps/move_group/faults
# → the aborted MoveGroup goal, as a structured fault with its snapshot
```

**The point:** ros2_medkit reads the signals your stack already emits - aborted actions,
`/rosout` errors, `/diagnostics` - through drop-in bridges. You add nothing to Nav2, MoveIt,
or your own nodes; you get a remote, queryable, time-traveled fault instead of a log line.

## vs. standard ROS 2 diagnostics

`diagnostic_updater` + `/diagnostics` + `diagnostic_aggregator` + `rqt_robot_monitor` report
current node health to a desktop GUI. ros2_medkit turns that into a queryable, remote,
time-traveled, actionable fault - and it **consumes `/diagnostics` too**, so it is additive,
not a rip-and-replace.

| | ROS 2 diagnostics | ros2_medkit |
|---|---|---|
| Access | `/diagnostics` topic + rqt GUI (local desktop) | SOVD REST API (remote; any tool / dashboard / agent) |
| Instrumentation | required (`diagnostic_updater` in node code) | works without it - drop-in bridges (`/rosout`, action status, `/diagnostics` passthrough) |
| State | live OK / WARN / ERROR / STALE | fault lifecycle (debounce, confirm, heal) |
| Moment of failure | none | freeze-frame snapshot + black-box rosbag |
| History | none (live only) | persisted, queryable |
| Model | key/value pairs | structured fault codes + SOVD entity model |
| Scope | ROS only | ROS today; PLC / ECU on one API |
| Agent access | none | MCP adapter |

## Run it in 5 minutes

**Install** (ROS 2 Jazzy, Humble, or Lyrical; Pixi and other options in the
[installation docs](https://selfpatch.github.io/ros2_medkit/installation.html)):

```bash
source /opt/ros/jazzy/setup.bash   # or humble / lyrical
git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
cd ros2_medkit
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

**Run it next to your robot** - one command starts the gateway, the fault manager and the
drop-in bridges:

```bash
ros2 launch ros2_medkit_gateway bringup.launch.py
# REST API on http://localhost:8080/api/v1/
```

**See a fault** (or just let a node log an `ERROR` / an action abort, as above):

```bash
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'DEMO_FAULT', event_type: 0, severity: 2, description: 'hello medkit', source_id: '/your_node'}"

curl http://localhost:8080/api/v1/faults   # the fault appears, with its black-box
```

For a guided walkthrough, see the
[Getting Started tutorial](https://selfpatch.github.io/ros2_medkit/getting_started.html).

## Beyond faults

Faults are the front door; the same REST surface exposes the whole ROS 2 graph - discovery,
live topic data, service/action calls, parameters, bulk data, SSE subscriptions, triggers,
locking, scripts, software updates and JWT/RBAC auth (OpenAPI 3.1.0 + Swagger UI at
`/api/v1/docs`). It models your robot as a SOVD **entity tree** (areas -> components -> apps,
with cross-cutting functions) so the same concepts carry across robots, vehicles and embedded
systems. See the [full documentation](https://selfpatch.github.io/ros2_medkit/) and the
[roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html).

## Documentation & community

- 📖 [Documentation](https://selfpatch.github.io/ros2_medkit/) · 🗺️ [Roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html) · 🧩 [Postman collection](postman/)
- 💬 [Discord](https://discord.gg/6CXPMApAyq) · 🐛 [Issues](https://github.com/selfpatch/ros2_medkit/issues) · 💡 [Discussions](https://github.com/selfpatch/ros2_medkit/discussions)
- 🤝 Contributing: see [CONTRIBUTING.md](CONTRIBUTING.md) and [good first issues](https://github.com/selfpatch/ros2_medkit/labels/good%20first%20issue)
- 🔒 Security: responsible disclosure in [SECURITY.md](SECURITY.md)

## License

Apache License 2.0 - see [LICENSE](LICENSE).

---

<p align="center">
  Made with ❤️ by the <a href="https://github.com/selfpatch">selfpatch</a> community ·
  <a href="https://discord.gg/6CXPMApAyq">Join us on Discord</a>
</p>
