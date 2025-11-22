# ros2_medkit

[![CI](https://github.com/bburda/ros2_medkit/actions/workflows/ci.yml/badge.svg)](https://github.com/bburda/ros2_medkit/actions/workflows/ci.yml)

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
> This is a personal open source project to explore diagnostic patterns for ROS 2.
> APIs, architecture, and naming may change at any time.

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

```bash
colcon test
colcon test-result --verbose
```

### CI/CD

All pull requests are automatically built and tested using GitHub Actions.
The CI workflow runs on Ubuntu 24.04 with ROS 2 Jazzy.

## Contributing

Contributions and early feedback are welcome! Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

By contributing, you agree to follow the [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Security

If you discover a security vulnerability, please follow the process in [`SECURITY.md`](SECURITY.md).

## License

This project is licensed under the Apache License 2.0. See the [`LICENSE`](LICENSE) file for details.
