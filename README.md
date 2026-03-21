# ros2_medkit

[![CI](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/selfpatch/ros2_medkit/branch/main/graph/badge.svg)](https://codecov.io/gh/selfpatch/ros2_medkit)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS 2 Rolling](https://img.shields.io/badge/ROS%202-Rolling-orange)](https://docs.ros.org/en/rolling/)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/6CXPMApAyq)
[![Quality Level 3](https://img.shields.io/badge/Quality-Level%203-yellow)](QUALITY_DECLARATION.md)

<p align="center">
  <img src="hero-full.gif" alt="Robots break. Now you'll know why." width="600">
</p>

<p align="center">
  <b>Structured diagnostics for ROS 2 robots.</b><br>
  When your robot fails, find out why - in minutes, not hours.
</p>

<p align="center">
  Fault management · Live introspection · REST API · <a href="https://github.com/selfpatch/ros2_medkit_mcp">AI via MCP</a>
</p>

## The problem

When a robot breaks in the field, you SSH in, run `ros2 node list`, grep through logs, and try to reconstruct what happened. It works for one robot on your desk. It does not work for 20 robots at a customer site, at 2 AM, when you cannot reproduce the issue.

ros2_medkit gives your ROS 2 system a **diagnostic REST API** so you can inspect what is running, what failed, and why, without SSH and without custom tooling.

## 🚀 Quick Start

**Try the full demo** (Docker, no ROS 2 needed):

```bash
git clone https://github.com/selfpatch/selfpatch_demos.git
cd selfpatch_demos/demos/turtlebot3_integration
./run-demo.sh --headless
# → API: http://localhost:8080/api/v1/  Web UI: http://localhost:3000
```

Open `http://localhost:3000` in your browser. You will see a TurtleBot3 with Nav2, organized into a browsable entity tree with live faults, topic data, and parameter access.

**Build from source** (ROS 2 Jazzy, Humble, or Rolling):

```bash
git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
cd ros2_medkit
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
ros2 launch ros2_medkit_gateway gateway.launch.py
# → http://localhost:8080/api/v1/areas
```

For API examples, see our [Postman collection](postman/).

### Experimental: Pixi

[Pixi](https://pixi.sh) provides a reproducible, lockfile-based environment
without requiring a system-wide ROS 2 installation (Linux x86_64 only).
This is experimental; the standard ROS 2 toolchain (rosdep + colcon) remains the primary method.

```bash
curl -fsSL https://pixi.sh/install.sh | bash
pixi install -e jazzy     # or: pixi install -e humble
pixi run -e jazzy build
pixi run -e jazzy test
pixi run -e jazzy smoke   # verify gateway starts
```

See [installation docs](https://selfpatch.github.io/ros2_medkit/installation.html#experimental-pixi)
for details. Feedback welcome on [#265](https://github.com/selfpatch/ros2_medkit/issues/265).

## What you get

**Start here: Faults.** Your robot has 47 nodes. Something throws an error.
Instead of grepping logs, you query `GET /api/v1/faults` and get a structured list
with fault codes, timestamps, affected entities, environment snapshots, and history.
Clear faults, subscribe to new ones via SSE, correlate them across components.

Beyond faults, medkit exposes the full ROS 2 graph through REST:

| | What it does |
|---|---|
| **Discovery** | Automatically finds running nodes, topics, services, and actions |
| **Data** | Read and write topic data via REST |
| **Operations** | Call services and actions with execution tracking |
| **Configurations** | Read, write, and reset node parameters |
| **Bulk Data** | Upload/download files (calibration, firmware, rosbags) |
| **Subscriptions** | Stream live data and fault events via SSE |
| **Triggers** | Condition-based push notifications for resource changes |
| **Locking** | Resource locking for safe concurrent access |
| **Scripts** | Upload and execute diagnostic scripts on entities |
| **Software Updates** | Async prepare/execute lifecycle with pluggable backends |
| **Authentication** | JWT-based RBAC (viewer, operator, configurator, admin) |
| **Logs** | Log entries and configuration |

On the [roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html): entity lifecycle control, mode management, communication logs.

## How it organizes your robot

medkit models your system as an **entity tree** with four levels:

```
Areas          Components         Apps (nodes)
─────          ──────────         ────────────
base       ┬─ motor_controller ┬─ left_wheel_driver
           │                   └─ right_wheel_driver
           └─ battery_monitor  └─ bms_node

navigation ┬─ lidar_driver     └─ rplidar_node
           └─ nav_stack        ┬─ nav2_controller
                               ├─ nav2_planner
                               └─ nav2_bt_navigator
```

A small robot might have a single area. A large robot can use areas to separate physical domains:

```
areas/
├── base/
│   └── components/
│       ├── motor_controller/   → apps: left_wheel, right_wheel
│       └── battery_monitor/    → apps: bms_node
├── arm/
│   └── components/
│       ├── joint_controller/   → apps: joint_1..joint_6
│       └── gripper/            → apps: gripper_driver
├── navigation/
│   └── components/
│       ├── lidar_driver/       → apps: rplidar_node
│       ├── camera_driver/      → apps: realsense_node
│       └── nav_stack/          → apps: controller, planner, bt_navigator
└── safety/
    └── components/
        ├── emergency_stop/     → apps: estop_monitor
        └── collision_detect/   → apps: collision_checker
```

**Functions** cut across the tree. A function like `localization` might depend on apps from both `navigation` and `base`, giving you a capability-oriented view alongside the physical hierarchy.

This entity model follows the **SOVD (Service-Oriented Vehicle Diagnostics)** standard, so the same concepts work across robots, vehicles, and embedded systems.

## 📋 Requirements

- **OS:** Ubuntu 24.04 LTS (Jazzy / Rolling) or Ubuntu 22.04 LTS (Humble)
- **ROS 2:** Jazzy Jalisco, Humble Hawksbill, or Rolling (experimental)
- **Compiler:** GCC 11+ (C++17 support)
- **Build System:** colcon + ament_cmake

## 📚 Documentation

- 📖 [Full Documentation](https://selfpatch.github.io/ros2_medkit/)
- 🗺️ [Roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html)
- 📋 [GitHub Milestones](https://github.com/selfpatch/ros2_medkit/milestones)

## 💬 Community

- **💬 Discord** - [Join our server](https://discord.gg/6CXPMApAyq) for discussions, help, and announcements
- **🐛 Issues** - [Report bugs or request features](https://github.com/selfpatch/ros2_medkit/issues)
- **💡 Discussions** - [GitHub Discussions](https://github.com/selfpatch/ros2_medkit/discussions) for Q&A and ideas

## 🤝 Contributing

Contributions are welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for build instructions, testing, pre-commit hooks, CI/CD details, and code coverage.

Quick version:

```bash
pip install pre-commit && pre-commit install && pre-commit install --hook-type pre-push
colcon build --symlink-install
source install/setup.bash
./scripts/test.sh          # unit tests
./scripts/test.sh all      # everything
```

Check out [good first issues](https://github.com/selfpatch/ros2_medkit/labels/good%20first%20issue) for places to start.

## 🔒 Security

If you discover a security vulnerability, please follow the responsible disclosure process in [SECURITY.md](SECURITY.md).

## 📄 License

Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

<p align="center">
  Made with ❤️ by the <a href="https://github.com/selfpatch">selfpatch</a> community
  <br>
  <a href="https://discord.gg/6CXPMApAyq">💬 Join us on Discord</a>
</p>
