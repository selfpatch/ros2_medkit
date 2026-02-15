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
  <b>Automotive-grade diagnostics for ROS 2 robots.</b><br>
  When your robot fails, find out why — in minutes, not hours.
</p>

<p align="center">
  Fault correlation · Black-box recording · REST API · <a href="https://github.com/selfpatch/ros2_medkit_mcp">AI via MCP</a>
</p>

## 🚀 Quick Start

**Try the full demo** (Docker, no ROS 2 needed):

```bash
git clone https://github.com/selfpatch/selfpatch_demos.git
cd selfpatch_demos/demos/turtlebot3_integration
./run-demo.sh --headless
# → API: http://localhost:8080/api/v1/  Web UI: http://localhost:3000
```

**Build from source** (ROS 2 Jazzy, Humble, or Rolling):

```bash
git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
cd ros2_medkit
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
ros2 launch ros2_medkit_gateway gateway.launch.py
# → http://localhost:8080/api/v1/areas
```

For more examples, see our [Postman collection](postman/).

## ✨ Features

- **🔍 Runtime Discovery** — Automatically discover what is actually running on your robot
- **🏗️ Entity Tree Model** — Organize diagnostics as Area → Component → Function → App
- **🔗 SOVD Compatible** — Align with Service-Oriented Vehicle Diagnostics standards
- **🌐 REST API Gateway** — HTTP interface for integration with external tools and UIs
- **📊 Health Modeling** — Track health state per entity for fleet-level observability
- **🔧 Easy Integration** — Works with existing ROS 2 nodes out of the box (Jazzy, Humble & Rolling)
- **📦 Bulk Data Management** — Upload, download, list, and delete bulk data files (calibration, firmware, etc.)

## 📖 Overview

ros2_medkit models a robot as a **diagnostic entity tree**:

| Entity | Description | Example |
|--------|-------------|---------|
| **Area** | Physical or logical domain | `base`, `arm`, `safety`, `navigation` |
| **Component** | Hardware or software component within an area | `motor_controller`, `lidar_driver` |
| **Function** | Capability provided by one or more components | `localization`, `obstacle_detection` |
| **App** | Deployable software unit | node, container, process |

Compatible with the **SOVD (Service-Oriented Vehicle Diagnostics)** model — same concepts across robots, vehicles, and embedded systems.

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

We'd love to have you join our community!

- **💬 Discord** — [Join our server](https://discord.gg/6CXPMApAyq) for discussions, help, and announcements
- **🐛 Issues** — [Report bugs or request features](https://github.com/selfpatch/ros2_medkit/issues)
- **💡 Discussions** — [GitHub Discussions](https://github.com/selfpatch/ros2_medkit/discussions) for Q&A and ideas

---

## 🛠️ Development

This section is for contributors and developers who want to build and test ros2_medkit locally.

### Pre-commit Hooks

This project uses [pre-commit](https://pre-commit.com/) to automatically run
`clang-format`, `flake8`, and other checks on staged files before each commit.

```bash
pip install pre-commit
pre-commit install
```

To run all hooks against every file (useful after first setup):

```bash
pre-commit run --all-files
```

### Installing Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Building

```bash
colcon build --symlink-install
```

### Testing

Run all tests:

```bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

Run linters:

```bash
source install/setup.bash
colcon test --ctest-args -L linters
colcon test-result --verbose
```

Run only unit tests (everything except integration):

```bash
source install/setup.bash
colcon test --ctest-args -E test_integration
colcon test-result --verbose
```

Run only integration tests:

```bash
source install/setup.bash
colcon test --ctest-args -R test_integration
colcon test-result --verbose
```

### Code Coverage

To generate code coverage reports locally:

1. Build with coverage flags enabled:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
```

2. Run tests:

```bash
source install/setup.bash
colcon test --ctest-args -LE linter
```

3. Generate coverage report:

```bash
lcov --capture --directory build --output-file coverage.raw.info --ignore-errors mismatch,negative
lcov --extract coverage.raw.info '*/ros2_medkit/src/*/src/*' '*/ros2_medkit/src/*/include/*' --output-file coverage.info --ignore-errors unused
lcov --list coverage.info
```

4. (Optional) Generate HTML report:

```bash
genhtml coverage.info --output-directory coverage_html
```

Then open `coverage_html/index.html` in your browser.

### CI/CD

All pull requests and pushes to main are automatically built and tested using GitHub Actions.
The CI workflow runs a build matrix across **ROS 2 Jazzy** (Ubuntu 24.04), **ROS 2 Humble** (Ubuntu 22.04), and **ROS 2 Rolling** (Ubuntu 24.04, allow-failure) and consists of the following jobs:

**build-and-test** (matrix: Jazzy + Humble + Rolling):

- Full build and unit/integration tests on all distros
- Rolling jobs are allowed to fail (best-effort forward-compatibility)
- Code linting and formatting checks (clang-format, clang-tidy) — Jazzy only

**coverage** (Jazzy only):

- Builds with coverage instrumentation (Debug mode)
- Runs unit tests only (for stable coverage metrics)
- Generates lcov coverage report (available as artifact)
- Uploads coverage to Codecov (only on push to main)

After every run the workflow uploads test results and coverage reports as artifacts for debugging and review.

---

## 🤝 Contributing

Contributions are welcome! Whether it's bug reports, feature requests, documentation improvements, or code contributions — we appreciate your help.

1. Read our [Contributing Guidelines](CONTRIBUTING.md)
2. Check out [good first issues](https://github.com/selfpatch/ros2_medkit/labels/good%20first%20issue) for beginners
3. Follow our [Code of Conduct](CODE_OF_CONDUCT.md)

## 🔒 Security

If you discover a security vulnerability, please follow the responsible disclosure process in [SECURITY.md](SECURITY.md).

## 📄 License

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.

---

<p align="center">
  Made with ❤️ by the <a href="https://github.com/selfpatch">selfpatch</a> community
  <br>
  <a href="https://discord.gg/6CXPMApAyq">💬 Join us on Discord</a>
</p>
