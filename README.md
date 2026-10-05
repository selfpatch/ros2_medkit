<!-- The logo, header and diagram live in the selfpatch/.github repository (profile/design/ros2_medkit.html), so this repository carries no image binaries. Change them there, and keep each image's alt text here in step with its words. -->

<a href="https://selfpatch.ai"><picture><source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/selfpatch/.github/main/profile/assets/ros2_medkit/logo-dark.png"><img src="https://raw.githubusercontent.com/selfpatch/.github/main/profile/assets/ros2_medkit/logo-light.png" alt="selfpatch.ai" height="30"></picture></a>

<picture><source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/selfpatch/.github/main/profile/assets/ros2_medkit/header-dark.png"><img src="https://raw.githubusercontent.com/selfpatch/.github/main/profile/assets/ros2_medkit/header-light.png" alt="ros2_medkit: see what broke on any ROS 2 robot, from one REST API." width="100%"></picture>

[![CI](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/selfpatch/ros2_medkit/branch/main/graph/badge.svg)](https://codecov.io/gh/selfpatch/ros2_medkit)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2 Jazzy | Humble | Lyrical](https://img.shields.io/badge/ROS%202-Jazzy%20%7C%20Humble%20%7C%20Lyrical-blue)](https://docs.ros.org/en/jazzy/)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/6CXPMApAyq)

ros2_medkit gives your ROS 2 robot a diagnostics REST API. It finds every node, topic, service and
action by itself, and turns failures - error logs, failed actions, `/diagnostics` - into clear
faults: what broke, where, how bad, with a snapshot and a rosbag of the moment it happened.
No changes to your code.

<a name="run-it-in-5-minutes"></a>

## Quick start

1. Get [Docker](https://docs.docker.com/get-started/get-docker/), if you don't have it yet.

2. While your robot is running, start ros2_medkit on its computer:

   ```bash
   docker run --rm --network host --ipc host \
     -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
     -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
     ros2 launch ros2_medkit_gateway bringup.launch.py
   ```

   On Humble or Lyrical, change `jazzy` to `humble` or `lyrical`.

3. See what it found:

   ```bash
   curl localhost:8080/api/v1/apps     # every node on your robot
   curl localhost:8080/api/v1/faults   # everything that has failed
   ```

   Or browse the whole API at [localhost:8080/api/v1/docs](http://localhost:8080/api/v1/docs).

> [!TIP]
> That's it. ros2_medkit is now watching your robot, and every new failure shows up as a fault.

**No robot at hand?** The sensor demo runs on any laptop and lets you break things on purpose
(it needs `curl` and `jq`):

```bash
git clone https://github.com/selfpatch/selfpatch_demos.git
cd selfpatch_demos/demos/sensor_diagnostics
./run-demo.sh     # web UI on http://localhost:3000
./inject-nan.sh   # break a sensor, then watch the fault appear
```

<details>
<summary>What a fault looks like</summary>

<br>

After a Nav2 goal is aborted:

```jsonc
// GET /api/v1/faults
{
  "items": [
    { "fault_code": "ACTION_NAVIGATE_TO_POSE_ABORTED",
      "severity_label": "ERROR", "status": "CONFIRMED",
      "reporting_sources": ["/bt_navigator"] }
  ],
  "x-medkit": { "count": 1 }
}
```

Each fault also keeps a snapshot of the moment it happened and a rosbag of the seconds around it:

```bash
curl localhost:8080/api/v1/apps/bt_navigator/faults/ACTION_NAVIGATE_TO_POSE_ABORTED
curl -O -J localhost:8080/api/v1/apps/bt_navigator/bulk-data/rosbags/ACTION_NAVIGATE_TO_POSE_ABORTED
```

</details>

<details>
<summary>Watch faults live, or add the web UI</summary>

<br>

```bash
# Faults as they happen
curl -N localhost:8080/api/v1/faults/stream

# A dashboard in the browser: open http://localhost:3000, click Connect, enter http://localhost:8080
docker run -p 3000:80 ghcr.io/selfpatch/ros2_medkit_web_ui:latest
```

See the [web UI tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/web-ui.html) and the
[Postman collection](postman/).

</details>

<details>
<summary>Install without Docker</summary>

<br>

ros2_medkit is on its way into the official ROS packages. Once it reaches your distro:

```bash
sudo apt install ros-jazzy-ros2-medkit-gateway   # or ros-humble- / ros-lyrical-
ros2 launch ros2_medkit_gateway bringup.launch.py
```

To build from source or use Pixi, follow the
[installation guide](https://selfpatch.github.io/ros2_medkit/installation.html).

</details>

<details>
<summary>Troubleshooting</summary>

<br>

- Robot missing? Run ros2_medkit on the robot's computer, or one on the same network, from a
  terminal where your ROS 2 setup works.
- Opening it from another computer? Add `server_host:=0.0.0.0` to the end of the command in
  step 2, on a network you trust, and use the robot's IP address instead of `localhost`.
- No faults from `/diagnostics`? Add `enable_diagnostic_bridge:=true` to the end of the command
  in step 2.
- Still stuck? See [troubleshooting](https://selfpatch.github.io/ros2_medkit/troubleshooting.html)
  or ask on [Discord](https://discord.gg/6CXPMApAyq).

</details>

## How it works

<picture><source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/selfpatch/.github/main/profile/assets/ros2_medkit/how-it-works-dark.png"><img src="https://raw.githubusercontent.com/selfpatch/.github/main/profile/assets/ros2_medkit/how-it-works-light.png" alt="How it works. Your robot: error logs, failed actions, /diagnostics, and your own nodes through FaultReporter. ros2_medkit listens to logs, actions and diagnostics, records each fault with its history, a snapshot and a rosbag, and serves it all on one REST API, with every node it found. You read it from a browser, curl, the web UI or an AI assistant." width="100%"></picture>

ros2_medkit listens to what your robot already publishes, so it works without code changes. For
more control, report faults from your own nodes with the
[`FaultReporter`](src/ros2_medkit_fault_reporter) client, see the
[integration tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/integration.html).
The API follows SOVD (ISO 17978), the diagnostics standard from the automotive world.

## Features

- Finds every node, topic, service and action, with no config
- Turns error logs, failed actions and `/diagnostics` into faults
- Confirms each fault, keeps its history and clears it once it's fixed
- Saves a snapshot and a rosbag of the moment each fault happened
- Reads any topic as JSON, calls services and actions, and reads and sets parameters
- Streams faults as they happen
- Software updates through a plugin, and sign-in with per-user permissions
- Rosbags open in whatever tool you already use to browse robot data

<a name="vs-standard-ros-2-diagnostics"></a>

## Compared with standard ROS 2 diagnostics

ros2_medkit builds on them. It reads `/diagnostics` too, so keep your `diagnostic_updater` code.

| | Standard ROS 2 diagnostics | ros2_medkit |
|---|---|---|
| Where you see it | A desktop app next to the robot | A REST API, from anywhere |
| Code changes | Each node reports its own health | None needed |
| What you get | OK, WARN, ERROR or STALE, right now | A fault that is confirmed, tracked and cleared once fixed |
| When it breaks | Nothing is saved | A snapshot and a rosbag |
| History | None | Saved, so you can look back |
| Covers | ROS only | ROS today, PLCs and vehicle controllers through the same API |
| AI assistants | No | Yes, through MCP |

## Ecosystem

- [ros2_medkit_web_ui](https://github.com/selfpatch/ros2_medkit_web_ui) - see your robot and its faults in the browser
- [ros2_medkit_mcp](https://github.com/selfpatch/ros2_medkit_mcp) - let an AI assistant look into your robot
- [ros2_medkit_clients](https://github.com/selfpatch/ros2_medkit_clients) - Python and TypeScript clients
- [selfpatch_demos](https://github.com/selfpatch/selfpatch_demos) - ready-made demo robots, from a mobile robot to an arm

## Documentation

- [Documentation](https://selfpatch.github.io/ros2_medkit/) and the [step-by-step tutorial](https://selfpatch.github.io/ros2_medkit/getting_started.html)
- [REST API reference](https://selfpatch.github.io/ros2_medkit/api/rest.html) and the [Postman collection](postman/)
- [Roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html)

## Contributing

Contributions are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md), pick a
[good first issue](https://github.com/selfpatch/ros2_medkit/labels/good%20first%20issue), or ask on
[Discord](https://discord.gg/6CXPMApAyq) and in [Discussions](https://github.com/selfpatch/ros2_medkit/discussions).
Report security issues privately, see [SECURITY.md](SECURITY.md).

## License

Apache License 2.0, see [LICENSE](LICENSE). ros2_medkit is built and maintained by
[selfpatch.ai](https://selfpatch.ai).
