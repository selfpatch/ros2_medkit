# ros2_medkit

[![CI](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/selfpatch/ros2_medkit/branch/main/graph/badge.svg)](https://codecov.io/gh/selfpatch/ros2_medkit)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/6CXPMApAyq)

**Modern, SOVD-compatible diagnostics for ROS 2 robots** — built around an entity tree
(Area / Component / Function / App) for runtime discovery, health modeling, and troubleshooting.

---

## ✨ Features

- **🔍 Runtime Discovery** — Automatically discover what is actually running on your robot
- **🏗️ Entity Tree Model** — Organize diagnostics as Area → Component → Function → App
- **🔗 SOVD Compatible** — Align with Service-Oriented Vehicle Diagnostics standards
- **🌐 REST API Gateway** — HTTP interface for integration with external tools and UIs
- **📊 Health Modeling** — Track health state per entity for fleet-level observability
- **🔧 Easy Integration** — Works with existing ROS 2 Jazzy nodes out of the box

## 📖 Overview

ros2_medkit provides **modern diagnostics for ROS 2–based systems**.

Instead of hardcoding knowledge about every node, topic, or ECU, ros2_medkit models a robot
as a **diagnostic entity tree**:

| Entity | Description | Example |
|--------|-------------|---------|
| **Area** | Physical or logical domain | `base`, `arm`, `safety`, `navigation` |
| **Component** | Hardware or software component within an area | `motor_controller`, `lidar_driver` |
| **Function** | Capability provided by one or more components | `localization`, `obstacle_detection` |
| **App** | Deployable software unit | node, container, process |

The goal is to make this tree **compatible with the SOVD (Service-Oriented Vehicle Diagnostics) model**,
so the same concepts can be used across robots, vehicles, and other embedded systems.

## 📋 Requirements

- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy Jalisco
- **Compiler:** GCC 13+ (C++17 support)
- **Build System:** colcon + ament_cmake

## 🚀 Quick Start

### 1. Clone and Install Dependencies

```bash
mkdir -p ~/ros2_medkit_ws/src
cd ~/ros2_medkit_ws/src
git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
cd ~/ros2_medkit_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **Note:** If you cloned without `--recurse-submodules`, run:
> `git submodule update --init --recursive`

### 2. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the Gateway

```bash
ros2 launch ros2_medkit_gateway gateway.launch.py
```

### 4. Explore the API

Open your browser at `http://localhost:8080/api/v1/areas` or use curl:

```bash
curl http://localhost:8080/api/v1/areas
```

For more examples, see our [Postman collection](postman/).

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
The CI workflow runs on Ubuntu 24.04 with ROS 2 Jazzy and consists of two parallel jobs:

**build-and-test:**

- Code linting and formatting checks (clang-format, clang-tidy)
- Unit tests and integration tests with demo automotive nodes

**coverage:**

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
