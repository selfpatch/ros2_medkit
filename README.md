# ros2_medkit

[![CI](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/ros2_medkit/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)

Modern, SOVD-compatible diagnostics for ROS 2 robots, built around an entity tree
(Area / Component / Function / App) for runtime discovery, health modeling, and troubleshooting.

## What is ros2_medkit?

ros2_medkit is an experiment in **modern diagnostics for ROS 2–based systems**.

Instead of hardcoding knowledge about every node, topic, or ECU, ros2_medkit models a robot
as a **diagnostic entity tree**:

- **Area** – physical or logical domain (e.g. `base`, `arm`, `safety`, `navigation`)
- **Component** – hardware or software component within an area
- **Function** – capability provided by one or more components
- **App** – deployable software unit (node, container, process)

The goal is to make this tree **compatible with the SOVD (Service-Oriented Vehicle Diagnostics) model**,
so the same concepts can be used across robots, vehicles, and other embedded systems.

## Status

> **Early prototype / work in progress**
>
> This is an open source project exploring diagnostic patterns for ROS 2.
> APIs, architecture, and naming may change as the project evolves.

See the [Roadmap](https://selfpatch.github.io/ros2_medkit/roadmap.html) for planned
milestones, or check [GitHub Milestones](https://github.com/selfpatch/ros2_medkit/milestones)
for current progress.

## Target Use Cases

- Runtime discovery of what is actually running on the robot
- Health state modeled per Area / Component / Function / App
- Better remote troubleshooting and fleet-level observability for ROS 2 robots

## Development

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

### CI/CD

All pull requests and pushes to main are automatically built and tested using GitHub Actions.
The CI workflow runs on Ubuntu 24.04 with ROS 2 Jazzy, executes a single `colcon test` to cover:

- Code linting and formatting checks
- Unit tests
- Integration tests with demo automotive nodes

After every run the workflow always calls `colcon test-result --verbose` and uploads the generated logs/results as artifacts for debugging.

## Contributing

Contributions and early feedback are welcome! Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

By contributing, you agree to follow the [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Security

If you discover a security vulnerability, please follow the process in [`SECURITY.md`](SECURITY.md).

## License

This project is licensed under the Apache License 2.0. See the [`LICENSE`](LICENSE) file for details.
